%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%	Author(s): C. Nicolas Barati nicobarati@rice.edu 
%		Rahman Doost-Mohamamdy: doost@rice.edu
%
%---------------------------------------------------------------------
% Original code copyright Mango Communications, Inc.
% Distributed under the WARP License http://warpproject.org/license
% Copyright (c) 2018-2019, Rice University
% RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
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
WRITE_PNG_FILES         = 0;           % Enable writing plots to PNG

%Iris params:
USE_HUB                 = 0;
TX_FRQ                  = 3.6e9;    
RX_FRQ                  = TX_FRQ;
TX_GN                   = 70;
RX_GN                   = 50;
SMPL_RT                 = 5e6;  
N_FRM                   = 1;
bs_ids = string.empty();
bs_sched = string.empty();


% Waveform params

% OFDM params
SC_IND_PILOTS           = [8 22 44 58];                           % Pilot subcarrier indices
SC_IND_DATA             = [2:7 9:21 23:27 39:43 45:57 59:64];     % Data subcarrier indices
SC_IND_DATA_PILOT       = [2:27 39:64]';
N_SC                    = 64;                                     % Number of subcarriers
CP_LEN                  = 16;                                     % Cyclic prefix length
N_SYM_SAMP              = N_SC + CP_LEN;                          % Number of samples that will go over the air
N_ZPAD_PRE              = 100;                                     % Zero-padding prefix for Iris
N_ZPAD_POST             = 76;                         % Zero-padding postfix for Iris
N_OFDM_SYM = floor((4096 - N_ZPAD_PRE - N_ZPAD_POST) / N_SYM_SAMP);% Number of OFDM symbols for burst, it needs to be less than 47

% Rx processing params
FFT_OFFSET                    = 16;          % Number of CP samples to use in FFT (on average)


%% Define the preamble
% LTS for fine CFO and channel estimation
lts_f = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 ...
    1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1];
lts_t = ifft(lts_f, 64); %time domain

%% Init Iris nodes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set up the Iris experiment
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create BS Hub and UE objects. Note: BS object is a collection of Iris
% nodes.

if USE_HUB
    hub_id = "FH4A000002";
else
    hub_id = [];
end

bs_ids = ["RF3E000208", "RF3E000636", "RF3E000632", "RF3E000568", "RF3E000558", "RF3E000633", "RF3E000566", "RF3E000635"];

bs_sched = ["PGRG", "RGPG"];  % All BS schedule, Ref Schedule
N_BS_NODE = length(bs_ids);
REF_ANT = floor(N_BS_NODE/2);
sched_id = ones(1, N_BS_NODE);
sched_id(REF_ANT) = 2;

lts = [lts_t(49:64) lts_t];
N_BS = N_BS_NODE - 1;
DATA_REP = floor(N_OFDM_SYM / N_BS);
data_len = N_BS * DATA_REP * N_SYM_SAMP;
preamble = zeros(N_BS_NODE, data_len);
for jp = 1:N_BS
    nid = jp + (jp >= REF_ANT);
    for rp = 1:DATA_REP
        start_index = ((rp - 1) + (jp - 1) * DATA_REP) * N_SYM_SAMP;
        preamble(nid, start_index + 1: start_index + N_SYM_SAMP) = lts;
        preamble(REF_ANT, start_index + 1: start_index + N_SYM_SAMP) = lts;
    end
end

n_samp = N_ZPAD_PRE + data_len + N_ZPAD_POST;
payload_rx = zeros(N_BS_NODE, data_len);
rx_fft = zeros(N_BS, N_SC);
rx_fft_ref = zeros(N_BS, N_SC);
cal_mat = zeros(N_BS, N_SC);

% Iris nodes' parameters
bs_sdr_params = struct(...
    'id', bs_ids, ...
    'n_sdrs', N_BS_NODE, ...
    'txfreq', TX_FRQ, ...
    'rxfreq', RX_FRQ, ...
    'txgain', TX_GN, ...
    'rxgain', RX_GN, ...
    'sample_rate', SMPL_RT, ...
    'n_samp', n_samp, ...          % number of samples per frame time.
    'n_frame', 1, ...
    'tdd_sched', bs_sched, ...     % number of zero-paddes samples
    'n_zpad_samp', N_ZPAD_PRE ...
    );

node_bs = iris_py(bs_sdr_params,hub_id);        % initialize BS

node_bs.sdrsync();                 % synchronize delays only for BS
node_bs.sdrrxsetup();

node_bs.set_tddconfig_single(1, sched_id); % configure the BS: schedule etc.
for i=1:N_BS_NODE
    tx_data = [zeros(1, N_ZPAD_PRE) preamble(i, :) zeros(1, N_ZPAD_POST)];
    node_bs.sdrtx_single(tx_data, i);  % Burn data to the UE RAM
end
node_bs.sdr_activate_rx();   % activate reading stream
tic
[rx_vec_iris, data0_len] = node_bs.sdrrx(n_samp); % read data
toc
a = 1;
unos = ones(size(conj(lts)));
for ibs = 1:N_BS_NODE
    % Correlation through filtering
    v0 = filter(fliplr(conj(preamble(REF_ANT, end - DATA_REP * N_SYM_SAMP: end))), a, rx_vec_iris(ibs, end - DATA_REP * N_SYM_SAMP: end));
    v1 = filter(unos, a, abs(rx_vec_iris(ibs, end - DATA_REP * N_SYM_SAMP: end)) .^ 2);
    lts_corr = (abs(v0) .^ 2) ./ v1; % normalized correlation

    % position of the last peak
    [~, max_idx] = max(abs(lts_corr));
    rx_data_start = max_idx + (n_samp - DATA_REP * N_SYM_SAMP) - data_len
    if rx_data_start < 0
       display('bad receive!');
       break;
    end
    payload_rx(ibs, :) = rx_vec_iris(ibs, rx_data_start: rx_data_start + data_len - 1);

    for sid = 1:N_BS
        for rid = 1:DATA_REP
            % nid = sid + (sid >= REF_ANT);
            start_index = CP_LEN + ((rid - 1) + (sid - 1) * DATA_REP) * N_SYM_SAMP;
            if (ibs == REF_ANT)
                rx_fft_ref(sid, :) = rx_fft_ref(sid, :) + fft(payload_rx(ibs, 1 + start_index : start_index + N_SC), N_SC);
            else
                rx_fft(sid, :) = rx_fft(sid, :) + fft(payload_rx(ibs, 1 + start_index : start_index + N_SC), N_SC);
            end
        end
    end
end

for sid = 1:N_BS
    cal_mat(sid, :) = (rx_fft_ref(sid, :) / DATA_REP) ./ (rx_fft(sid, :) / (N_BS * DATA_REP));
end

node_bs.sdr_close();

cf = 0;
cf = cf + 1;
figure(cf);clf;

for i = 1:N_BS
    subplot(N_BS, 1, i);
    plot(-32:1:31, abs(cal_mat(i, :)));
    axis([-40 40 0 5])
    grid on;
    title('Calibration MAGNITUDE');
end

cf = cf + 1;
figure(cf);clf;

for i = 1:N_BS
    subplot(N_BS, 1, i);
    plot(-32:1:31, angle(cal_mat(i, :)));
    axis([-40 40 -pi pi])
    grid on;
    title('Calibration ANGLE');
end

