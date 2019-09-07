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
        
        % Parameters to feed python (pun very intended!)
        serial_ids;
        n_chain;         % number of Iris boards in a chain
		sample_rate;
		tx_freq;q
		rx_freq;
		bw;
		tx_gain;
		rx_gain;
		n_samp;
        tdd_sched;
        n_zpad_samp;    %number of zero-padding samples
        n_frame = 10;
        
    end
    
    methods
        function obj = iris_py(sdr_params)
            if nargin > 0
                obj.sdr_params = sdr_params;
                
                obj.serial_ids = sdr_params.id; 
                obj.n_chain = sdr_params.n_chain;
                obj.sample_rate = sdr_params.sample_rate;
                obj.tx_freq = sdr_params.txfreq;
                obj.rx_freq = sdr_params.rxfreq;
                obj.tx_gain = sdr_params.txgain;
                obj.rx_gain = sdr_params.rxgain;
                obj.n_samp = sdr_params.n_samp;
                obj.n_frame = sdr_params.n_frame;
                obj.tdd_sched = sdr_params.tdd_sched; % This is an array
                obj.n_zpad_samp = sdr_params.n_zpad_samp;
                
                for ipy=1:obj.n_chain
                    id_str = convertStringsToChars( obj.serial_ids(ipy))
                    py_obj = py.iris_py.Iris_py( pyargs('serial_id',id_str,...
                        'tx_freq', obj.tx_freq, 'rx_freq', obj.rx_freq,...
                        'tx_gain',obj.tx_gain,'rx_gain',obj.rx_gain,...
                        'sample_rate',obj.sample_rate, 'n_samp',...
                        obj.n_samp,'n_zpad_samp',obj.n_zpad_samp) );%,...
                        %'max_frames',obj.n_frame) ); 
                    
                    obj.py_obj_array(ipy,:) = {py_obj};
                end
            end
        end
        % NB: Need to redefine py_obj as array to be parsed in all the
        % functions that follow
        function sdrtrigger(obj, trig)
            obj.py_obj_array{1}.set_trigger(pyargs('trig',trig)); % The trigger is set only on the fist node
        end
        
         function sdrsync(obj, is_bs)
            obj.py_obj_array{1}.sync_delays(pyargs('is_bs', is_bs)); 
         end
         
         function sdr_setcorr(obj)
            obj.py_obj_array{1}.set_corr(); 
         end
         
         function sdr_txgainctrl(obj)
            obj.py_obj_array{1}.tx_gain_ctrl(); 
         end
         
         function sdr_txbeacon(obj, prefix_len)
             % Assume Beacon only from the first Iris board in the BS array
             obj.py_obj_array{1}.burn_beacon( pyargs('prefix_len', prefix_len) );
         end
        
         
         function set_config(obj, chained_tx_rx, is_bs)
             if chained_tx_rx
                sched  = convertStringsToChars((obj.tdd_sched));
                obj.py_obj_array{1}.config_sdr_tdd_chained(pyargs('tdd_sched', sched));        
             else
                sched  = convertStringsToChars(obj.tdd_sched(1));
                obj.py_obj_array{1}.config_sdr_tdd( pyargs('tdd_sched', sched, ...
                    'is_bs', is_bs));
                if (obj.n_chain > 1)
                   sched2  = convertStringsToChars(obj.tdd_sched(2));
                   for ipy = 2:obj.n_chain
                       obj.py_obj_array{ipy}.config_sdr_tdd( pyargs('tdd_sched',sched2, ...
                           'is_bs', is_bs));
                   end
                end
             end
             
         end
        
        function sdrrxsetup(obj)
            for ipy = 1:obj.n_chain
                obj.py_obj_array{ipy}.setup_stream_rx();
            end
        end
        
        function sdr_activate_rx(obj)
           for ipy=1:obj.n_chain
               obj.py_obj_array{ipy}.activate_stream_rx();
           end
        end
        
        %Assume same data is Tx-ed from all memebers of the array
        function sdrtx(obj, data)
            re = real(data).';   
            im = imag(data).';
            for ipy = 1:obj.n_chain
                obj.py_obj_array{ipy}.burn_data( pyargs('data_r', re, 'data_i', im) );
            end
        end
        
        % Read n_frame x n_samp data
        function [data, len] = sdrrx(obj, n_samp)
            data_raw = zeros(obj.n_chain, obj.n_frame*n_samp);  % Change this to max frame!
            
            for jf=1:obj.n_frame
                %trigger beacon TX
                trig = 1;
                obj.py_obj_array{1}.set_trigger(pyargs('trig',trig));
                for ipy = 1:obj.n_chain
                    rcv_data = obj.py_obj_array{ipy}.recv_stream_tdd();
                    data_raw(ipy, (jf-1)*n_samp + 1: jf*n_samp) = double( py.array.array( 'd',py.numpy.nditer( py.numpy.real(rcv_data) ) ) ) + ...
                        1i*double( py.array.array( 'd',py.numpy.nditer( py.numpy.imag(rcv_data) ) ) );
                end
            end
            data = obj.get_best_frame(data_raw.', n_samp);
            len = length(data);
        end
        
        
        function sdr_close(obj)
            for ipy = 1:obj.n_chain
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
            
            if obj.n_chain == 1 && length(lts_corr_frm >= 300)
                lts_corr_frm = lts_corr_frm(1:300,:);
            elseif (obj.n_chain > 1) && length(lts_corr_frm >= 420)
                lts_corr_frm = lts_corr_frm(1:420,:);
            end
            save my_data.mat lts_corr_dr lts_corr_frm
            
            % Avg corr value per frame
            frm_avg_corr = sum(lts_corr_frm,1)./obj.n_chain
            % Take index of maximum corr. value
            [max_corr, m_idx] = max(frm_avg_corr);
           
            % Reshape data frame to n_samp-by-n_antenna-by-n_frame
            data_split  = zeros(obj.n_samp, obj.n_chain,obj.n_frame);
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

