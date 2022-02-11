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
        hub_serial='';
        sample_rate = 0;
        tx_freq = 0;
        rx_freq = 0;
        tx_gain = 0;
        rx_gain = 0;
    end

    methods
        function obj = mimo_driver(sdr_params)
            if nargin > 0
                obj.sdr_params = sdr_params;
                
                obj.bs_serial_ids = sdr_params.bs_id;
                obj.ue_serial_ids = sdr_params.ue_id;
                if isfield(sdr_params, 'hub_id')
                    obj.hub_serial = sdr_params.hub_id;
                end
                obj.sample_rate = sdr_params.sample_rate;
                obj.tx_freq = sdr_params.txfreq;
                obj.rx_freq = sdr_params.rxfreq;
                obj.tx_gain = sdr_params.txgain;
                obj.rx_gain = sdr_params.rxgain;

                n_bs_sdrs = length(obj.bs_serial_ids);
                n_ue_sdrs = length(obj.ue_serial_ids);

                bs_id_str = cell(1, n_bs_sdrs);
                for i = 1:n_bs_sdrs
                    bs_id_str(1, i) = {convertStringsToChars(obj.bs_serial_ids(i))};
                end
                bs_id_list = py.list(bs_id_str);

                ue_id_str = cell(1, n_ue_sdrs);
                for i = 1:n_ue_sdrs
                    ue_id_str(1, i) = {convertStringsToChars(obj.ue_serial_ids(i))};
                end
                ue_id_list = py.list(ue_id_str);

                hub_id_str = convertStringsToChars(obj.hub_serial);
                obj.mimo_obj = py.mimo_driver.MIMODriver( pyargs( ...
                    'hub_serial', hub_id_str,...
                        'bs_serials', bs_id_list, ...
                        'ue_serials', ue_id_list, ...
                        'rate', obj.sample_rate,...
                        'tx_freq', obj.tx_freq, 'rx_freq', obj.rx_freq,...
                        'tx_gain', obj.tx_gain, 'rx_gain', obj.rx_gain) );
                
            end
        end

        function data = mimo_txrx_uplink(obj, tx_data_mat, n_frames, n_samps_pad)
            results = obj.mimo_obj.txrx_uplink(py.numpy.array(tx_data_mat), py.int(n_frames), py.int(n_samps_pad));
            data = [];
            result_cell = cell(results);
            if result_cell{1, 2} == 0
                disp('Unsucessful uplink receive. Try different gain settings!');
            else
                data = double( py.array.array( 'd',py.numpy.nditer( py.numpy.real(results(1)) ) ) ) + ...
                    1i*double( py.array.array( 'd',py.numpy.nditer( py.numpy.imag(results(1)) ) ) );
            end
        end

        function data = mimo_txrx_downlink(obj, tx_data_mat, n_frames, n_samps_pad)
            results = obj.mimo_obj.txrx_downlink(py.numpy.array(tx_data_mat), py.int(n_frames), py.int(n_samps_pad));
            data = [];
            if results == py.NoneType
                disp('Unsucessful downlink receive. Try different gain settings!');
            else
                result_cell = cell(results);
                if result_cell{1, 2} == 0
                    disp('Unsucessful downlink receive. Try different gain settings!');
                else
                    data = double( py.array.array( 'd',py.numpy.nditer( py.numpy.real(results(1)) ) ) ) + ...
                        1i*double( py.array.array( 'd',py.numpy.nditer( py.numpy.imag(results(1)) ) ) );
                end
            end
        end

        function mimo_close(obj)
            obj.mimo_obj.close();
        end
    end
end
