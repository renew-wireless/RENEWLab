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
TX_FRQ                  = 2.5e9;    
RX_FRQ                  = TX_FRQ;
TX_GN                   = 80;
RX_GN                   = 60;
SMPL_RT                 = 5e6;  
N_FRM                   = 1;
bs_ids = string.empty();
bs_sched = string.empty();


% Waveform params
TX_SCALE                = 0.5;         % Scale for Tx waveform ([0:1])

% OFDM params
SC_IND_PILOTS           = [8 22 44 58];                           % Pilot subcarrier indices
SC_IND_DATA             = [2:7 9:21 23:27 39:43 45:57 59:64];     % Data subcarrier indices
SC_IND_DATA_PILOT       = [2:27 39:64]';
N_SC                    = 64;                                     % Number of subcarriers
CP_LEN                  = 16;                                     % Cyclic prefix length
N_SYM_SAMP              = N_SC + CP_LEN;                          % Number of samples that will go over the air
N_SAMP                  = 4096;                                   % N_ZPAD_PRE + data_len + N_ZPAD_POST;
N_ZPAD_PRE              = 100;                                    % Zero-padding prefix for Iris
N_ZPAD_POST             = 76;                                     % Zero-padding postfix for Iris
N_OFDM_SYM              = floor((N_SAMP - N_ZPAD_PRE - N_ZPAD_POST) / N_SYM_SAMP);  % Number of OFDM symbols for burst, it needs to be less than 47
N_PILOTS_SYMS           = 2;
N_DATA_SYMS             = (N_OFDM_SYM - N_PILOTS_SYMS);       % Number of data symbols (one per data-bearing subcarrier per OFDM symbol)
N_DATA_SC               = N_DATA_SYMS * length(SC_IND_DATA);
MOD_ORDER               = 16;           % Modulation order (2/4/16/64 = BSPK/QPSK/16-QAM/64-QAM)

% Rx processing params
FFT_OFFSET                    = 16;          % Number of CP samples to use in FFT (on average)
RECIP_PLOT = 0;
PILOT_PLOT = 0;
DOWNLINK_PLOT = 1;

%% Define the preamble
% LTS for fine CFO and channel estimation
lts_f = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 ...
    1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1];
lts_t = ifft(lts_f, 64); %time domain
lts = [lts_t(49:64) lts_t];

%% Generate a payload of random integers
tx_data = randi(MOD_ORDER, 1, N_DATA_SC) - 1;

tx_syms = mod_sym(tx_data, MOD_ORDER);
% Reshape the symbol vector to a matrix with one column per OFDM symbol
tx_syms_mat = reshape(tx_syms, N_DATA_SYMS , length(SC_IND_DATA));

% Define the pilot tone values as BPSK symbols
pilots = [1 1 -1 1];

% Repeat the pilots across all OFDM symbols
pilots_mat = repmat(pilots, N_DATA_SYMS, 1);

% Construct the precoding input matrix
precoding_in_mat = zeros(N_OFDM_SYM, N_SC);

% Insert the data and pilot values; other subcarriers will remain at 0
for i = 1:N_PILOTS_SYMS
    precoding_in_mat(i, :) = lts_f;
end
precoding_in_mat(N_PILOTS_SYMS + 1:end, SC_IND_DATA)   = tx_syms_mat;
precoding_in_mat(N_PILOTS_SYMS + 1:end, SC_IND_PILOTS) = pilots_mat;


%% Init Iris nodes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set up the Iris experiment
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create BS Hub and UE objects. Note: BS object is a collection of Iris
% nodes.

if USE_HUB
    hub_id = "FH4A000001";
else
    hub_id = [];
end

bs_ids = ["RF3E000189", "RF3E000024", "RF3E000139", "RF3E000032", "RF3E000154", "RF3E000182", "RF3E000038", "RF3E000137"];
ue_ids= ["RF3E000157"];

%bs_sched = ["PGRG", "RGPG"];  % All BS schedule, Ref Schedule
bs_sched = ["PGRG", "RGPG", "BGRG", "BGPG"];  % BS schedule
ue_sched = ["GGPG", "GGRG"];  % UE schedule

N_BS_NODE = length(bs_ids);
N_UE_NODE = length(ue_ids);


REF_ANT = floor(N_BS_NODE/2);
sched_id = ones(1, N_BS_NODE);
sched_id(REF_ANT) = 2;

% reciprocity calibration tx pilots
N_BS = N_BS_NODE - 1;
DATA_REP = floor(N_OFDM_SYM / N_BS);
data_len = N_BS * DATA_REP * N_SYM_SAMP;
recip_tx = zeros(N_BS_NODE, data_len);
for jp = 1:N_BS
    nid = jp + (jp >= REF_ANT);
    for rp = 1:DATA_REP
        start_index = ((rp - 1) + (jp - 1) * DATA_REP) * N_SYM_SAMP;
        recip_tx(nid, start_index + 1: start_index + N_SYM_SAMP) = lts;
        recip_tx(REF_ANT, start_index + 1: start_index + N_SYM_SAMP) = lts;
    end
end

% uplink tx piltos
UE_DATA_REP = floor(N_OFDM_SYM / N_UE_NODE);
ue_pilot_len = UE_DATA_REP * N_SYM_SAMP;
ue_pilots_len = N_UE_NODE * ue_pilot_len;
uplink_pilot_tx = zeros(N_UE_NODE, ue_pilots_len);
for jp = 1:N_UE_NODE
    for rp = 1:UE_DATA_REP
        start_index = ((rp - 1) + (jp - 1) * UE_DATA_REP) * N_SYM_SAMP;
        uplink_pilot_tx(jp, start_index + 1: start_index + N_SYM_SAMP) = lts;
    end
end

recip_rx = zeros(N_BS_NODE, data_len);
rx_fft = zeros(N_BS, N_SC);
rx_fft_ref = zeros(N_BS, N_SC);
cal_mat = ones(N_BS_NODE, N_SC);

% Iris nodes' parameters
bs_sdr_params = struct(...
    'id', bs_ids, ...
    'n_sdrs', N_BS_NODE, ...
    'txfreq', TX_FRQ, ...
    'rxfreq', RX_FRQ, ...
    'txgain', TX_GN, ...
    'rxgain', RX_GN, ...
    'sample_rate', SMPL_RT, ...
    'n_samp', N_SAMP, ...          % number of samples per frame time.
    'n_frame', 1, ...
    'tdd_sched', bs_sched, ...     % number of zero-paddes samples
    'n_zpad_samp', N_ZPAD_PRE ...
    );


ue_sdr_params = bs_sdr_params;
ue_sdr_params.id =  ue_ids;
ue_sdr_params.n_sdrs = N_UE_NODE;
ue_sdr_params.tdd_sched = ue_sched;
    
node_bs = iris_py(bs_sdr_params, hub_id);% initialize BS
node_ue = iris_py(ue_sdr_params, []);    % initialize UE

node_bs.sdrsync();                 % synchronize delays only for BS
node_bs.sdrrxsetup();

node_ue.sdr_configgainctrl();
node_ue.sdrrxsetup();
node_bs.sdr_setupbeacon();   % Burn beacon to the BS RAM

%% Reciprocity Calibration
recip_postfix_len = N_SAMP - data_len - N_ZPAD_PRE;
for i=1:N_BS_NODE
    node_bs.set_tddconfig_single(1, bs_sched(sched_id(i)), i); % configure the BS: schedule etc.
    tx_data = [zeros(1, N_ZPAD_PRE) recip_tx(i, :) zeros(1, recip_postfix_len)];
    node_bs.sdrtx_single(tx_data, i);  % Burn data to the UE RAM
end
node_bs.sdr_activate_rx();   % activate reading stream
[rx_vec_iris, ~] = node_bs.sdrrx(N_SAMP); % read data

a = 1;
unos = ones(size(conj(lts)));
for ibs = 1:N_BS_NODE
    % Correlation through filtering
    v0 = filter(fliplr(conj(recip_tx(REF_ANT, end - DATA_REP * N_SYM_SAMP: end))), a, rx_vec_iris(ibs, end - DATA_REP * N_SYM_SAMP: end));
    v1 = filter(unos, a, abs(rx_vec_iris(ibs, end - DATA_REP * N_SYM_SAMP: end)) .^ 2);
    lts_corr = (abs(v0) .^ 2) ./ v1; % normalized correlation

    % position of the last peak
    [~, max_idx] = max(abs(lts_corr));
    rx_data_start = max_idx + (N_SAMP - DATA_REP * N_SYM_SAMP) - data_len
    if rx_data_start < 0
       display('bad receive!');
       break;
    end
    recip_rx(ibs, :) = rx_vec_iris(ibs, rx_data_start: rx_data_start + data_len - 1);

    for sid = 1:N_BS
        for rid = 1:DATA_REP
            % nid = sid + (sid >= REF_ANT);
            start_index = CP_LEN + ((rid - 1) + (sid - 1) * DATA_REP) * N_SYM_SAMP;
            if (ibs == REF_ANT)
                rx_fft_ref(sid, :) = rx_fft_ref(sid, :) + fft(recip_rx(ibs, 1 + start_index : start_index + N_SC), N_SC);
            else
                rx_fft(sid, :) = rx_fft(sid, :) + fft(recip_rx(ibs, 1 + start_index : start_index + N_SC), N_SC);
            end
        end
    end
end

for sid = 1:N_BS
    nid = sid + (sid >= REF_ANT);
    cal_mat(nid, :) = (rx_fft_ref(sid, :) / DATA_REP) ./ (rx_fft(sid, :) / (N_BS * DATA_REP));
end

cf = 0;
if RECIP_PLOT
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
end

%% Uplink Pilot Collection and Channel Estimation
%node_bs.set_n_frame(10);
bs_sched_id = 3;
node_bs.set_tddconfig(1, bs_sched(bs_sched_id)); % configure the BS: schedule etc.

%node_bs.set_n_frame(10);
ue_sched_id = 1;
node_ue.set_tddconfig(0, ue_sched(ue_sched_id)); % configure the BS: schedule etc.

pilot_postfix_len = N_SAMP - data_len - N_ZPAD_PRE;
for i=1:N_UE_NODE
    tx_data = [zeros(1, N_ZPAD_PRE) uplink_pilot_tx(i, :) zeros(1, pilot_postfix_len)];
    node_ue.sdrtx_single(tx_data, i);       % Burn data to the UE RAM
end

node_ue.sdr_setcorr();              % activate correlator
[rx_vec_pilot, data0_len] = node_bs.sdrrx(N_SAMP); % read data

uplink_pilot_rx = zeros(N_BS_NODE, N_UE_NODE, ue_pilot_len);
uplink_pilot_csi = zeros(N_BS_NODE, N_UE_NODE, N_SC);

for ibs =1:N_BS_NODE
    v0 = filter(fliplr(conj(uplink_pilot_tx(1 : UE_DATA_REP * N_SYM_SAMP))), a, rx_vec_pilot(ibs, :));
    v1 = filter(unos, a, abs(rx_vec_pilot(ibs, :)) .^ 2);
    m_filt = (abs(v0) .^ 2) ./ v1; % normalized correlation
    [~, max_idx] = max(abs(m_filt));
    % In case of bad correlatons:
    pilot_data_start = max_idx + 1 - ue_pilots_len
    if pilot_data_start < 0
       display('bad receive!');
       break;
    end
    for iue = 1:N_UE_NODE
        uplink_pilot_rx(ibs, iue, :) = rx_vec_pilot(ibs, pilot_data_start: pilot_data_start + ue_pilot_len - 1);
        pilot_data_start = pilot_data_start + ue_pilot_len;
        rx_fft = zeros(UE_DATA_REP, N_SC);
        for irp = 1:UE_DATA_REP
            start = (irp - 1) * N_SYM_SAMP + CP_LEN;
            rx_fft(irp, :) = fft(uplink_pilot_rx(ibs, iue, start + 1: start + N_SC));
        end
        uplink_pilot_csi(ibs, iue, :) = mean(rx_fft) .* lts_f; % mean across reps
    end
end

if PILOT_PLOT
    cf = cf + 1;
    figure(cf);clf;

    for i = 1:N_BS_NODE
        subplot(N_BS_NODE, 1, i);
        plot(real(rx_vec_pilot(i, :)));
        axis([0 N_SAMP -.1 .1])
        grid on;
        title('Received Pilots (Real)');
    end
end

%% Downlink CSI Calculation and Zeroforcing
downlink_pilot_csi = zeros(N_BS_NODE, N_UE_NODE, N_SC);
ifft_in_mat = zeros(N_BS_NODE, N_OFDM_SYM, N_SC);
for isc =1:N_SC
    downlink_pilot_csi(:, :, isc) = diag(squeeze(cal_mat(:, isc))) * squeeze(uplink_pilot_csi(:, :, isc));
    downlink_beam_weights = pinv(squeeze(downlink_pilot_csi(:, :, isc)));
    for isym = 1:N_OFDM_SYM
        ifft_in_mat(:, isym, isc) = precoding_in_mat(isym, isc) * downlink_beam_weights;
    end
end

%% IFFT

%Perform the IFFT
tx_payload_mat = zeros(N_BS_NODE, N_SYM_SAMP, N_OFDM_SYM);
for ibs = 1:N_BS_NODE
    for isym = 1:N_OFDM_SYM
        tx_sym = squeeze(ifft(ifft_in_mat(ibs, isym, :)));
        tx_payload_mat(ibs, :, isym) = [tx_sym(end - CP_LEN + 1: end); tx_sym];
    end
end


% Reshape to a vector
tx_payload_vec = reshape(tx_payload_mat, N_BS_NODE, numel(tx_payload_mat(1, :, :)));

bs_sched_id = 4;
node_bs.set_tddconfig(1, bs_sched(bs_sched_id)); % configure the BS: schedule etc.

ue_sched_id = 2;
node_ue.set_tddconfig(0, ue_sched(ue_sched_id)); % configure the BS: schedule etc.

% Construct the full time-domain OFDM waveform
donwlink_postfix_len = N_SAMP - N_ZPAD_PRE - N_OFDM_SYM * N_SYM_SAMP;
for i=1:N_BS_NODE
    tx_data = [zeros(1, N_ZPAD_PRE) tx_payload_vec(i, :) zeros(1, donwlink_postfix_len)];
    tx_vec_iris = TX_SCALE .* tx_data ./ max(abs(tx_data));
    node_bs.sdrtx_single(tx_vec_iris, i);       % Burn data to the UE RAM
end

node_ue.sdr_setcorr();              % activate correlator
node_bs.sdrtrigger();
[rx_vec_downlink, ~] = node_ue.sdrrx(N_SAMP); % read data

if DOWNLINK_PLOT
    cf = cf + 1;
    figure(cf);clf;

    for i = 1:N_UE_NODE
        subplot(N_UE_NODE, 1, i);
        plot(real(rx_vec_downlink(i, :)));
        axis([0 N_SAMP -.1 .1])
        grid on;
        title('Received Pilots (Real)');
    end
end

node_bs.sdr_close();
node_ue.sdr_close();



