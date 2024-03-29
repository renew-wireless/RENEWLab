classdef mimo_driver < handle
    % This class serves as an interface between matlab and
    % iris python driver. Matlab calls the methods here and iris_class calls
    % iris' python functions.
    
    properties
        sdr_params;
        % pyhton object array. This array decribes 1 Iris board or 
        % a collection of Iris boards that belong to the same entity.E.g., a BS.
        mimo_obj;

        bs_serial_ids;
        ue_serial_ids;
        ref_serial_ids;
        n_bs_ant = 0;
        n_ue_ant = 0;
        bs_channels = 'A';
        ue_channels = 'A';
        hub_serial='';
        sample_rate = 0;
        tx_freq = 0;
        rx_freq = 0;
        tx_gain = 0;
        tx_gain_ue = 0;
        rx_gain = 0;
        trig_offset = 0;
    end

    methods
        function obj = mimo_driver(sdr_params)
            if nargin > 0
                obj.sdr_params = sdr_params;
                
                obj.bs_serial_ids = sdr_params.bs_id;
                obj.ue_serial_ids = sdr_params.ue_id;
                obj.ref_serial_ids = sdr_params.ref_id;
                if isfield(sdr_params, 'hub_id')
                    obj.hub_serial = sdr_params.hub_id;
                end
                obj.sample_rate = sdr_params.sample_rate;
                obj.tx_freq = sdr_params.txfreq;
                obj.rx_freq = sdr_params.rxfreq;
                obj.tx_gain = sdr_params.txgain;
                obj.tx_gain_ue = sdr_params.tx_gain_ue;
                obj.rx_gain = sdr_params.rxgain;

                obj.trig_offset = sdr_params.trig_offset;

                n_bs_sdrs = length(obj.bs_serial_ids);
                n_ue_sdrs = length(obj.ue_serial_ids);
                n_ref_sdrs = length(obj.ref_serial_ids);

                obj.n_bs_ant = n_bs_sdrs * length(sdr_params.bs_ant);
                obj.n_ue_ant = n_ue_sdrs * length(sdr_params.ue_ant);
                obj.bs_channels = sdr_params.bs_ant;
                obj.ue_channels = sdr_params.ue_ant;

                % Base Station
                bs_id_str = cell(1, n_bs_sdrs);
                for i = 1:n_bs_sdrs
                    bs_id_str(1, i) = {convertStringsToChars(obj.bs_serial_ids(i))};
                end
                bs_id_list = py.list(bs_id_str);

                % UEs
                ue_id_str = cell(1, n_ue_sdrs);
                for i = 1:n_ue_sdrs
                    ue_id_str(1, i) = {convertStringsToChars(obj.ue_serial_ids(i))};
                end
                ue_id_list = py.list(ue_id_str);

                % Reference Node (overkill... it will always be at most one)
                ref_id_str = cell(1, n_ref_sdrs);
                for i = 1:n_ref_sdrs
                    ref_id_str(1, i) = {convertStringsToChars(obj.ref_serial_ids(i))};
                end
                ref_id_list = py.list(ref_id_str);

                hub_id_str = convertStringsToChars(obj.hub_serial);
                obj.mimo_obj = py.mimo_driver.MIMODriver( pyargs( ...
                    'hub_serial', hub_id_str, ...
                        'bs_serials', bs_id_list, ...
                        'ue_serials', ue_id_list, ...
                        'ref_serials', ref_id_list, ...
                        'rate', obj.sample_rate, ...
                        'tx_freq', obj.tx_freq, 'rx_freq', obj.rx_freq, ...
                        'tx_gain', obj.tx_gain, 'tx_gain_ue', obj.tx_gain_ue, 'rx_gain', obj.rx_gain, ...
                        'bs_channels', obj.bs_channels , 'ue_channels', obj.ue_channels, ...
                        'trig_offset', obj.trig_offset) );

            end
        end

        function [data, numGoodFrames, numRxSyms] = mimo_txrx(obj, tx_data_mat, n_frames, n_samps_pad, mode, bs_sched, ue_sched)

            switch mode
                case 'uplink'
                    results = obj.mimo_obj.txrx_uplink(py.numpy.array(real(tx_data_mat)), py.numpy.array(imag(tx_data_mat)), py.int(n_frames), py.int(n_samps_pad));
                case 'downlink'
                    results = obj.mimo_obj.txrx_downlink(py.numpy.array(real(tx_data_mat)), py.numpy.array(imag(tx_data_mat)), py.int(n_frames), py.int(n_samps_pad));
                case 'dl-sounding'
                    results = obj.mimo_obj.txrx_dl_sound(py.numpy.array(real(tx_data_mat)), py.numpy.array(imag(tx_data_mat)), py.int(n_frames), py.int(n_samps_pad));
                case 'ul-refnode-as-ue'
                    results = obj.mimo_obj.txrx_refnode(py.numpy.array(real(tx_data_mat)), py.numpy.array(imag(tx_data_mat)), py.int(n_frames), bs_sched, ue_sched, py.int(n_samps_pad));
                otherwise
                    disp('Invalid Mode!')
            end

            result_cell = cell(results);
            data_py = result_cell{1,1};
            if result_cell{1, 2} == 0
                data = [];
                numGoodFrames = 0;
                numRxSyms = 0;
                disp('*** Unsucessful uplink receive. Try different gain settings! *** ');

            else
                % Data shape: (# good frames, # BS ant or # UEs, # numRxSyms, # number samps)
                %ngoodframes = double(py.numpy.int_(data_py.data.shape(1)))
                numGoodFrames = double(py.numpy.int_(results(2)));
                nx = double(py.numpy.int_(data_py.data.shape(2)));
                nrecvSyms = double(py.numpy.int_(data_py.data.shape(3)));
                nsamples = double(py.numpy.int_(data_py.data.shape(4)));
                data = zeros(numGoodFrames, nx, nrecvSyms, nsamples);

                P1 = cellfun(@cell, cell(data_py.tolist), 'Uniform',false);
                %P1 = vertcat(P1{:});
                for igf = 1:numGoodFrames
                    for ix = 1:nx   % Number of bs ant if uplink or number of ues if downlink
                        for ism = 1:nrecvSyms
                            data(igf, ix, ism, :) = double( py.array.array( 'd',py.numpy.nditer( py.numpy.real(P1{igf}{ix}{ism}) ) ) ) + ...
                                                  1i*double( py.array.array( 'd',py.numpy.nditer( py.numpy.imag(P1{igf}{ix}{ism}) ) ) );
                            %figure(100); plot(abs(squeeze(data(igf, ibs, ifr, :)))); hold on;
                        end
                    end
                end

            end
            numRxSyms = double(py.numpy.int_(results(3)));
            %numGoodFrames = double(py.numpy.int_(results(2)));
        end


        function [txg, rxg, max_num_beac, valid] = mimo_set_opt_gains(obj, n_frames)
            results = obj.mimo_obj.set_opt_gains(py.int(n_frames));
            txg = double(py.numpy.int_(results(1)));
            rxg = double(py.numpy.int_(results(2)));
            max_num_beac = double(py.numpy.int_(results(3)));
            valid = double(py.numpy.bool_(results(4)));
        end

        function mimo_update_sdr_params(obj, param, param_value, is_bs)
            obj.mimo_obj.update_sdr_params(param, param_value, is_bs);
        end

        function mimo_close(obj)
            obj.mimo_obj.close();
        end
    end
end
