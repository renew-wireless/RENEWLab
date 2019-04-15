%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%	Author(s): C. Nicolas Barati nicobarati@rice.edu 
%		Rahman Doost-Mohamamdy: doost@rice.edu
%
%---------------------------------------------------------------------
% Copyright (c) 2018-2019, Rice University
% RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
% Original code copyright Mango Communications, Inc.
% Distributed under the WARP License http://warpproject.org/license
% ---------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
close all;

[version, executable, isloaded] = pyversion;
if ~isloaded
    pyversion /usr/bin/python
    py.print() %weird bug where py isn't loaded in an external script
end

% Params:

% Waveform params
N_OFDM_SYM              = 0;         % Number of OFDM symbols for burst, it needs to be less than 47
TX_SCALE                = 1;         % Scale for Tx waveform ([0:1])

CP_LEN                  = 16;                                     % Cyclic prefix length
N_LTS_SYM               = 20;                                      % Number of 
N_STS_SYM               = 0;                                      % Number of STS Symbols (taken as N_SC + CP_LEN units)
N_ZPAD_PRE              = 70;                                     % Zero-padding prefix for Iris
N_ZPAD_POST             = N_ZPAD_PRE -14;                         % Zero-padding postfix for Iris

%% Define the pilot

% LTS for fine CFO and channel estimation
lts_f = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 ...
    1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1];
lts_t = ifft(lts_f, 64); %time domain

preamble =repmat(lts_t,1,10);




%% Iris Tx 
% Need to be done once for burst! Just burn the data onto the FPGAs RAM
% Scale the Tx vector to +/- 1

% Construct the full time-domain OFDM waveform, no CP
tx_vec = [zeros(1,N_ZPAD_PRE) preamble zeros(1,N_ZPAD_POST)];
tx_vec_iris = tx_vec.';
tx_vec_iris = TX_SCALE .* tx_vec_iris ./ max(abs(tx_vec_iris));

%% Init Iris nodes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set up the Iris nodes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 % Create a two Iris node objects:
    b_sched = "PGGGGGRG";               % BS schedule
    u_sched = "GGGGGGPG";               % UE schedule
    n_samp = length(tx_vec_iris);
    % Create a vector of node objects
    sdr_params = struct(...
        'id', "RF3C000007", ...
        'n_chain',1, ...
        'txfreq', 2.6e9, ...
        'rxfreq', 2.6e9, ...
        'txgain', 42, ...
        'rxgain', 30, ...
        'sample_rate', 5e6, ...
        'n_samp', n_samp, ...     % number of samples per frame time.
        'tdd_sched', b_sched, ...     % number of zero-paddes samples
        'n_zpad_samp', (N_ZPAD_PRE + N_ZPAD_POST) ...
        );
    
    sdr_params(2) = sdr_params(1);
    sdr_params(2).id =  "RF3C000053";
    sdr_params(2).rxfreq = 2.6e9;
    sdr_params(2).txfreq = 2.6e9;
    sdr_params(2).tdd_sched = u_sched;
    node_bs = iris_py(sdr_params(1));
    node_ue = iris_py(sdr_params(2));



trig = 1;

node_ue.sdrsync(0);

node_ue.sdrrxsetup();
node_bs.sdrrxsetup();
chained_mode = 0;
node_bs.set_config(chained_mode,1,0);
node_ue.set_config(chained_mode,0,0);


node_bs.sdr_txbeacon(N_ZPAD_PRE);
node_ue.sdrtx(tx_vec_iris);

node_bs.sdr_activate_rx();

node_ue.sdr_setcorr()
 
node_bs.sdrtrigger(trig);

%% Iris Rx
% Only UL data:

[rx_vec_iris, data0_len] = node_bs.sdrrx(n_samp);

node_bs.sdr_close();
node_ue.sdr_close();

fprintf('Matlab script: Length of the received vector: \tUE:%d\n', data0_len);

rx_vec_iris = rx_vec_iris.';
raw_rx_dec = rx_vec_iris(:,1).';
rx_vec_air = raw_rx_dec;


%% Rx signal plot
figure; clf;
subplot(2,1,1);
plot(real(rx_vec_air), 'b');
axis([0 length(rx_vec_air) -TX_SCALE TX_SCALE])
grid on;
title('Rx Waveform (I)');

subplot(2,1,2);
plot(imag(rx_vec_air), 'r');
axis([0 length(rx_vec_air) -TX_SCALE TX_SCALE])
grid on;
title('Rx Waveform (Q)');
