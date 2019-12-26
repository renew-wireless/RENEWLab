%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%	Author(s): C. Nicolas Barati nicobarati@rice.edu 
%		Rahman Doost-Mohamamdy: doost@rice.edu
%
%---------------------------------------------------------------------
% Copyright (c) 2018-2019, Rice University
% RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
% ---------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef iris_py < handle
    % This class serves as an interface between matlab and
    % iris python driver. Matlab calls the methods here and iris_class calls
    % iris' python functions.
    
    properties
        sdr_params;
        % pyhton object array. This array decribes 1 Iris board or 
        % a collection of Iris boards that belong to the same entity.E.g., a BS.
        py_obj_array = {};
        use_hub = 0;
        py_obj_hub;
        % Parameters to feed python (pun very intended!)
        serial_ids = [];
        n_sdrs = 0;         % number of Iris boards in a chain
		sample_rate = 0;
		tx_freq = 0;
		rx_freq = 0;
		bw = 0;
		tx_gain = 0;
		rx_gain = 0;
		n_samp = 0;
        tdd_sched = "";
        n_zpad_samp = 0;    %number of zero-padding samples
        n_frame = 10;
        is_bs = 1;
    end
    
    methods
        function obj = iris_py(sdr_params, hub_id) % User must put 0 in hub_params in case hub is not used!
            if nargin > 0
                if ~isempty(hub_id)
                    disp('Using hub with ID: ');
                    disp(hub_id)
                    id_str = convertStringsToChars(hub_id);
                    obj.py_obj_hub = py.hub_py.Hub_py( pyargs('serial_id',id_str));
                    obj.use_hub = 1;
                end
                
                obj.sdr_params = sdr_params;
                
                obj.serial_ids = sdr_params.id; 
                obj.n_sdrs = sdr_params.n_sdrs;
                obj.sample_rate = sdr_params.sample_rate;
                obj.tx_freq = sdr_params.txfreq;
                obj.rx_freq = sdr_params.rxfreq;
                obj.tx_gain = sdr_params.txgain;
                obj.rx_gain = sdr_params.rxgain;
                obj.n_samp = sdr_params.n_samp;
                obj.n_frame = sdr_params.n_frame;
                obj.tdd_sched = sdr_params.tdd_sched; % This is an array
                obj.n_zpad_samp = sdr_params.n_zpad_samp;
                
                for ipy=1:obj.n_sdrs
                    id_str = convertStringsToChars(obj.serial_ids(ipy));
                    py_obj = py.iris_py.Iris_py( pyargs('serial_id',id_str,...
                        'tx_freq', obj.tx_freq, 'rx_freq', obj.rx_freq,...
                        'tx_gain',obj.tx_gain,'rx_gain',obj.rx_gain,...
                        'sample_rate',obj.sample_rate, 'n_samp', obj.n_samp) );
                    
                    obj.py_obj_array(ipy,:) = {py_obj};
                end
            end
        end

        function sdr_set_n_frame(obj, n_frame)
            obj.n_frame = n_frame;
        end
        
        % NB: Need to redefine py_obj as array to be parsed in all the
        % functions that follow
        function sdrtrigger(obj)
            if ~obj.use_hub
                obj.py_obj_array{1}.set_trigger(); % The trigger is set only on the fist node
            else
                obj.py_obj_hub.set_trigger();
                disp('triggering FAROS hub...')
            end
        end
        
         function sdrsync(obj)
            if ~obj.use_hub
                obj.py_obj_array{1}.sync_delays();
            else
                obj.py_obj_hub.sync_delays();
            end
         end
         
         function sdr_setcorr(obj)
             for ipy = 1:obj.n_sdrs
                 obj.py_obj_array{ipy}.set_corr(); 
             end
         end
         
         function sdr_unsetcorr(obj)
             for ipy = 1:obj.n_sdrs
                 obj.py_obj_array{ipy}.unset_corr(); 
             end
         end
         
         function sdr_configgainctrl(obj)
             for ipy = 1:obj.n_sdrs
                 obj.py_obj_array{ipy}.config_gain_ctrl(); 
             end
         end
         
         function sdr_setupbeacon(obj)
             % Assume Beacon only from the first Iris board in the BS array
             for ipy = 1:obj.n_sdrs
                 obj.py_obj_array{ipy}.burn_beacon();
             end
         end
         
         function set_tddconfig(obj, is_bs, tdd_sched)
             obj.is_bs = is_bs;
             sched  = convertStringsToChars(tdd_sched);
             for ipy = 1:obj.n_sdrs
                 obj.py_obj_array{ipy}.config_sdr_tdd( pyargs('tdd_sched', sched, ...
                     'is_bs', is_bs, 'prefix_len', obj.n_zpad_samp, 'max_frames', 1));
             end
         end

         function set_tddconfig_single(obj, is_bs, tdd_sched, index)
             sched  = convertStringsToChars(tdd_sched);
             obj.py_obj_array{index}.config_sdr_tdd( pyargs('tdd_sched', sched, ...
                 'is_bs', is_bs, 'prefix_len', obj.n_zpad_samp, 'max_frames', 1));
         end
         
         function sdrrxsetup(obj)
            for ipy = 1:obj.n_sdrs
                obj.py_obj_array{ipy}.setup_stream_rx();
            end
         end
        
         function sdr_activate_rx(obj)
           for ipy=1:obj.n_sdrs
               obj.py_obj_array{ipy}.activate_stream_rx();
           end
         end
        
        %Assume same data is Tx-ed from all memebers of the array
        function sdrtx(obj, data)
            for ipy = 1:obj.n_sdrs
                obj.py_obj_array{ipy}.burn_data( pyargs('data_r', real(data), 'data_i', imag(data)) );
            end
        end

        function sdrtx_single(obj, data, index)
            obj.py_obj_array{index}.burn_data( pyargs('data_r', real(data), 'data_i', imag(data)) );
        end
        
        % Read n_frame x n_samp data
        function [data, len] = sdrrx(obj, n_samp)
            data_raw = zeros(obj.n_sdrs, obj.n_frame * n_samp);  % Change this to max frame!
            
            for jf=1:obj.n_frame
                %trigger base station
                if obj.is_bs
                    if ~obj.use_hub
                        obj.py_obj_array{1}.set_trigger(); % The trigger is set only on the fist node
                    else
                        obj.py_obj_hub.set_trigger();
                        disp('triggering FAROS hub...')
                    end
                end
                
                for ipy = 1:obj.n_sdrs
                    rcv_data = obj.py_obj_array{ipy}.recv_stream_tdd();
                    data_raw(ipy, (jf-1)*n_samp + 1: jf*n_samp) = double( py.array.array( 'd',py.numpy.nditer( py.numpy.real(rcv_data) ) ) ) + ...
                        1i*double( py.array.array( 'd',py.numpy.nditer( py.numpy.imag(rcv_data) ) ) );
                end
            end
            data = obj.get_best_frame(data_raw.', n_samp);
            len = length(data);
        end
        
        
        function sdr_close(obj)
            for ipy = 1:obj.n_sdrs
                obj.py_obj_array{ipy}.close();
                delete(obj.py_obj_array{ipy});
            end
        end
        
        function [data] = get_best_frame(obj, data_frame, n_samp)
            % FD LTS
            lts_f = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 ...
                -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 1 -1 -1 ...
                1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1].';
            % TD LTS
            lts_t = ifft(lts_f, 64);
            % Add CP and a second TD LTS
            lts_cp = lts_t(33:end);
            lts_t = [lts_cp; lts_t; lts_t];
            
            % Correlation through filtering
            lts_corr_dr = zeros(size(data_frame));
            unos = ones(size(lts_t));
            a = 1;
            for nc =1:size(data_frame,2)
                v0 = filter(flipud(conj(lts_t)),a,data_frame(:,nc));
                v1 = filter(unos,a,abs(data_frame(:,nc)).^2);
                lts_corr_dr(:,nc) = (abs(v0).^2)./v1; % normalized correlation
            end
            % Sum accross antennas
            lts_corr_sum = sum(lts_corr_dr,2);
            % zero non-peaks:
            lts_corr_sum(lts_corr_sum < 0.2) = 0.001;
            
            % Assume peak in the first 500 samples
            lts_corr_frm = reshape(lts_corr_sum, [], obj.n_frame);
            
            if obj.n_sdrs == 1 && length(lts_corr_frm >= 300)
                lts_corr_frm = lts_corr_frm(1:300,:);
            elseif (obj.n_sdrs > 1) && length(lts_corr_frm >= 420)
                lts_corr_frm = lts_corr_frm(1:420,:);
            end
            save my_data.mat lts_corr_dr lts_corr_frm
            
            % Avg corr value per frame
            frm_avg_corr = sum(lts_corr_frm,1)./obj.n_sdrs
            % Take index of maximum corr. value
            [max_corr, m_idx] = max(frm_avg_corr);
           
            % Reshape data frame to n_samp-by-n_antenna-by-n_frame
            data_split  = zeros(obj.n_samp, obj.n_sdrs,obj.n_frame);
            for nf = 1:obj.n_frame
                strt_idx = (nf-1)*obj.n_samp +1;
                end_idx = nf*obj.n_samp;
                data_split(:,:,nf) = data_frame(strt_idx :end_idx ,:);
            end
            %data_frame = reshape(data_frame,n_samp, [], obj.n_frame );        
    
            % Return the frame with the highest value 
            data = data_split(:,:,m_idx).';
            fprintf('Returning frame number %d with max mean correlation = %f \n',m_idx,max_corr);
        end
        
    end
end

