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
        py_obj;         %pyhton object
        
        % Parameters to feed python (pun very intended!)
        serial_id;
		sample_rate;
		tx_freq;
		rx_freq;
		bw;
		tx_gain;
		rx_gain;
		n_samp;
        tdd_sched;
        n_zpad_samp;    %number of zero-padding samples
        
    end
    
    methods
        function obj = iris_py(sdr_params)
            if nargin > 0
                obj.sdr_params = sdr_params;
                
                obj.serial_id = sdr_params.id;
                obj.sample_rate = sdr_params.sample_rate;
                obj.tx_freq = sdr_params.txfreq;
                obj.rx_freq = sdr_params.rxfreq;
                obj.tx_gain = sdr_params.txgain;
                obj.rx_gain = sdr_params.rxgain;
                obj.n_samp = sdr_params.n_samp;
                obj.tdd_sched = sdr_params.tdd_sched;
                obj.n_zpad_samp = sdr_params.n_zpad_samp;
        
                obj.py_obj = py.iris_py.Iris_py( pyargs('serial_id',obj.serial_id,...
                    'tx_freq', obj.tx_freq, 'rx_freq', obj.rx_freq,...
                    'tx_gain',obj.tx_gain,'rx_gain',obj.rx_gain,...
                    'sample_rate',obj.sample_rate, 'n_samp',obj.n_samp,'n_zpad_samp',obj.n_zpad_samp) ); 
            end
        end
        
        function sdrtrigger(obj, trig)
            obj.py_obj.set_trigger(pyargs('trig',trig)); 
        end
        
         function sdrsync(obj, is_bs)
            obj.py_obj.sync_delays(pyargs('is_bs', is_bs)); 
         end
         
         function sdr_setcorr(obj)
            obj.py_obj.set_corr(); 
         end
         
         function sdr_txgainctrl(obj)
            obj.py_obj.tx_gain_ctrl(); 
         end
         
         function sdr_txbeacon(obj, prefix_len)
             obj.py_obj.burn_beacon( pyargs('prefix_len', prefix_len) );
         end
        

         function set_config(obj, chained_mode, is_bs, trigger_out)
             if chained_mode
                obj.py_obj.config_sdr_tdd_chained(pyargs('tdd_sched', obj.tdd_sched));        
             else
                obj.py_obj.config_sdr_tdd( pyargs('tdd_sched', obj.tdd_sched, 'is_bs', is_bs, 'trigger_out', trigger_out));
                
             end
             
         end
        
        function sdrrxsetup(obj)
           obj.py_obj.setup_stream_rx();
        end
        
        function sdr_activate_rx(obj)
           obj.py_obj.activate_stream_rx();
        end
        
        function sdrtx(obj, data)
            re = real(data);   
            im = imag(data);
            obj.py_obj.burn_data( pyargs('data_r', re, 'data_i', im) );
        end
        
        function [data, len] = sdrrx(obj)
            rcv_data = obj.py_obj.recv_stream_tdd();
            data = double( py.array.array( 'd',py.numpy.nditer( py.numpy.real(rcv_data) ) ) ) + ...
                1i*double( py.array.array( 'd',py.numpy.nditer( py.numpy.imag(rcv_data) ) ) );
            len = length(data);
        end
        
        
        function sdr_close(obj)
            obj.py_obj.close();
            delete(obj.py_obj);
        end
        
    end
end

