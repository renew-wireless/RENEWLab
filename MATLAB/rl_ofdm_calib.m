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
CHANNEL                 = 11;          % Channel to tune Tx and Rx radios
SIM_MOD                 = 0;    
N_BS_NODE = 6;
N_UE = 1;


nt                      = 1;
nsnr                    = 1;
TX_SCALE                = 0.5;         % Scale for Tx waveform ([0:1])
chan_type               = "iris";


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


ber_SIM = zeros(nt,nsnr);           % BER
berr_th = zeros(nsnr,1);            % Theoretical BER
fprintf("Channel type: %s \n",chan_type);


% Waveform params
N_OFDM_SYM              = 46;         % Number of OFDM symbols for burst, it needs to be less than 47
MOD_ORDER               = 16;           % Modulation order (2/4/16/64 = BSPK/QPSK/16-QAM/64-QAM)

% OFDM params
SC_IND_PILOTS           = [8 22 44 58];                           % Pilot subcarrier indices
SC_IND_DATA             = [2:7 9:21 23:27 39:43 45:57 59:64];     % Data subcarrier indices
SC_IND_DATA_PILOT       = [2:27 39:64]';
N_SC                    = 64;                                     % Number of subcarriers
CP_LEN                  = 16;                                     % Cyclic prefix length
N_DATA_SYMS             = N_OFDM_SYM * length(SC_IND_DATA);       % Number of data symbols (one per data-bearing subcarrier per OFDM symbol)
N_LTS_SYM               = 2;                                      % Number of 
N_SYM_SAMP              = N_SC + CP_LEN;                          % Number of samples that will go over the air
N_ZPAD_PRE              = 90;                                     % Zero-padding prefix for Iris
N_ZPAD_POST             = N_ZPAD_PRE -14;                         % Zero-padding postfix for Iris

% Rx processing params
FFT_OFFSET                    = 16;          % Number of CP samples to use in FFT (on average)


%% Define the preamble
% LTS for fine CFO and channel estimation
lts_f = [0 0 0 0 0 0 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0].';
lts_t = ifft(lts_f, 64); %time domain

%% Init Iris nodes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set up the Iris experiment
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create BS Hub and UE objects. Note: BS object is a collection of Iris
% nodes.

if USE_HUB
    % Using chains of different size requires some internal
    % calibration on the BS. This functionality will be added later.
    % For now, we use only the 4-node chains:

    bs_ids = ["RF3E000134", "RF3E000191", "RF3E000171", "RF3E000105",...
        "RF3E000053", "RF3E000177", "RF3E000192", "RF3E000117",...
        "RF3E000183", "RF3E000152", "RF3E000123", "RF3E000178", "RF3E000113", "RF3E000176", "RF3E000132", "RF3E000108", ...
        "RF3E000143", "RF3E000160", "RF3E000025", "RF3E000034",...
        "RF3E000189", "RF3E000024", "RF3E000139", "RF3E000032", "RF3E000154", "RF3E000182", "RF3E000038", "RF3E000137", ...
        "RF3E000103", "RF3E000180", "RF3E000181", "RF3E000188"];

    hub_id = "FH4A000001";

else
    bs_ids = ["0328", "0339", "0268", "0282", "0344", "0233"];
end

bs_sched = ["PGRG", "RGPG"];  % All BS schedule, Ref Schedule
REF_ANT = (N_BS_NODE/2);
sched_id = ones(1, N_BS_NODE);
sched_id(REF_ANT) = 2;

preamble_common = [lts_t(49:64); lts_t];
l_pre = length(preamble_common);
N_BS = N_BS_NODE - 1;
data_len = N_BS * l_pre;
preamble = zeros(data_len, N_BS_NODE);
for jp = 1:N_BS
    ibs = jp + (jp >= REF_ANT);
    preamble((jp-1)*l_pre + 1: (jp-1)*l_pre+l_pre, ibs) = preamble_common;
    preamble((jp-1)*l_pre + 1: (jp-1)*l_pre+l_pre, REF_ANT) = preamble_common;
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

if USE_HUB
    node_bs = iris_py(bs_sdr_params, hub_id);
else
    node_bs = iris_py(bs_sdr_params,[]);        % initialize BS
end

node_bs.sdrsync();                 % synchronize delays only for BS
node_bs.sdrrxsetup();

node_bs.set_tddconfig_single(1, sched_id); % configure the BS: schedule etc.
for i=1:N_BS_NODE
    tx_data = [zeros(N_ZPAD_PRE, 1); preamble(:, i); zeros(N_ZPAD_POST, 1)];
    node_bs.sdrtx_single(tx_data, i);  % Burn data to the UE RAM
end
node_bs.sdr_activate_rx();   % activate reading stream
[rx_vec_iris, data0_len] = node_bs.sdrrx(n_samp); % read data

a = 1;
unos = ones(size(preamble_common'));
for ibs = 1:N_BS_NODE
    % Correlation through filtering
    v0 = filter(fliplr(preamble(:, REF_ANT)), a, rx_vec_iris(ibs, :));
    v1 = filter(unos, a, abs(rx_vec_iris(ibs, :)) .^ 2);
    lts_corr = (abs(v0) .^ 2) ./ v1; % normalized correlation

    % position of the last peak
    [~, max_idx] = max(lts_corr);
    rx_data_start = max_idx + 1 - data_len;
    payload_rx(ibs, :) = rx_vec_iris(ibs, rx_data_start: max_idx );

    for sid = 1:N_BS
        nid = sid + (sid >= REF_ANT);
        if (ibs == REF_ANT)
            rx_fft_ref(sid, :) = fft(payload_rx(ibs, 1 + CP_LEN + (sid - 1) * N_SYM_SAMP : sid * N_SYM_SAMP), N_SC);
        else
            rx_fft(sid, :) = rx_fft(sid, :) + fft(payload_rx(ibs, 1 + CP_LEN + (sid - 1) * N_SYM_SAMP : sid * N_SYM_SAMP), N_SC);
        end
    end
end

for sid = 1:N_BS
    cal_mat(sid, :) = rx_fft_ref(sid, :) ./ (rx_fft(sid, :) / N_BS);
end

cf = 0;

cf = cf + 1;
figure(cf);clf;

for i = 1:N_BS
    subplot(N_BS, 1, i);
    plot(-32:1:31, abs(cal_mat(i, :)));
    axis([-40 40 0 4])
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

