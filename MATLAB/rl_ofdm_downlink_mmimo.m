%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%	Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
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
USE_HUB                 = 1;
WIRED_UE                 = 1;
TX_FRQ                  = 2.5e9;    
RX_FRQ                  = TX_FRQ;
TX_GN                   = 40;
RX_GN                   = 20;
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
N_OFDM_SYMS              = floor((N_SAMP - N_ZPAD_PRE - N_ZPAD_POST) / N_SYM_SAMP);  % Number of OFDM symbols for burst, it needs to be less than 47
N_PILOTS_SYMS           = 2;
N_DATA_SYMS             = (N_OFDM_SYMS - N_PILOTS_SYMS);       % Number of data symbols (one per data-bearing subcarrier per OFDM symbol)
N_DATA_SC               = N_DATA_SYMS * length(SC_IND_DATA);
MOD_ORDER               = 4;           % Modulation order (2/4/16/64 = BSPK/QPSK/16-QAM/64-QAM)

% Rx processing params
FFT_OFFSET                    = 16;          % Number of CP samples to use in FFT (on average)
RECIP_PLOT = 1;
PILOT_PLOT = 1;
DOWNLINK_PLOT = 1;

%% Define the preamble
% LTS for fine CFO and channel estimation
lts_f = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 ...
    1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1];
lts_t = ifft(lts_f, 64); %time domain
lts = [lts_t(49:64) lts_t];


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

bs_ids = ["RF3E000183", "RF3E000152", "RF3E000123", "RF3E000178", "RF3E000113", "RF3E000176", "RF3E000132", "RF3E000108"];
ue_ids= ["RF3E000103", "RF3E000180"];

%bs_sched = ["PGRG", "RGPG"];  % All BS schedule, Ref Schedule
bs_sched = ["PGRG", "RGPG", "BGRG", "BGPG"];  % BS schedule
ue_sched = ["GGPG", "GGRG"];  % UE schedule

N_BS_NODE = length(bs_ids);
N_UE_NODE = length(ue_ids);


REF_ANT = floor(N_BS_NODE/2);
sched_id = ones(1, N_BS_NODE);
sched_id(REF_ANT) = 2;

%% Generate a payload of random integers
tx_data = randi(MOD_ORDER, N_UE_NODE, N_DATA_SC) - 1;

tx_syms = mod_sym(tx_data, MOD_ORDER);
% Reshape the symbol vector to a matrix with one column per OFDM symbol
tx_syms_mat = reshape(tx_syms, N_UE_NODE, length(SC_IND_DATA), N_DATA_SYMS);

% Define the pilot tone values as BPSK symbols
pilots = [1 1 -1 1].';

% Repeat the pilots across all OFDM symbols
pilots_mat = zeros(N_UE_NODE, length(SC_IND_PILOTS), N_DATA_SYMS);
for i=1:N_UE_NODE
    pilots_mat(i,:,:) = repmat(pilots, 1, N_DATA_SYMS);
end

% Construct the precoding input matrix
precoding_in_mat = zeros(N_UE_NODE, N_SC, N_OFDM_SYMS);

% Insert the data and pilot values; other subcarriers will remain at 0
for i = 1:N_PILOTS_SYMS
    precoding_in_mat(:, :, i) = repmat(lts_f, N_UE_NODE, 1);
end
precoding_in_mat(:, SC_IND_DATA, N_PILOTS_SYMS + 1:end)   = tx_syms_mat;
precoding_in_mat(:, SC_IND_PILOTS, N_PILOTS_SYMS + 1:end) = pilots_mat;

% reciprocity calibration tx pilots
N_BS = N_BS_NODE - 1;
DATA_REP = floor(N_OFDM_SYMS / N_BS);
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
UE_DATA_REP = floor(N_OFDM_SYMS / N_UE_NODE);
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
    'n_frame', N_FRM, ...
    'tdd_sched', bs_sched, ...     % number of zero-paddes samples
    'n_zpad_samp', N_ZPAD_PRE ...
    );


ue_sdr_params = bs_sdr_params;
ue_sdr_params.id =  ue_ids;
ue_sdr_params.n_sdrs = N_UE_NODE;
ue_sdr_params.tdd_sched = ue_sched;
    
node_bs = iris_py(bs_sdr_params, hub_id);% initialize BS
node_ue = iris_py(ue_sdr_params, []);    % initialize UE

node_bs.sdrsync();                 % Synchronize delays only for BS
node_bs.sdrrxsetup();
node_bs.sdr_setupbeacon();   % Burn beacon to the BS RAM

node_ue.sdr_configgainctrl();
node_ue.sdrrxsetup();



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


%% Uplink Pilot Collection and Channel Estimation
%node_bs.set_n_frame(10);
bs_sched_id = 3;
node_bs.set_tddconfig(1, bs_sched(bs_sched_id)); % configure the BS: schedule etc.

%node_bs.set_n_frame(10);
ue_sched_id = 1;
node_ue.set_tddconfig(WIRED_UE, ue_sched(ue_sched_id)); % configure the BS: schedule etc.

pilot_postfix_len = N_SAMP - data_len - N_ZPAD_PRE;
for i=1:N_UE_NODE
    tx_data = [zeros(1, N_ZPAD_PRE) uplink_pilot_tx(i, :) zeros(1, pilot_postfix_len)];
    node_ue.sdrtx_single(tx_data, i);       % Burn data to the UE RAM
end

if ~WIRED_UE
    node_ue.sdr_setcorr();              % activate correlator
end
[rx_vec_pilot, data0_len] = node_bs.sdrrx(N_SAMP); % read data
if ~WIRED_UE
    node_ue.sdr_unsetcorr();              % activate correlator
end

uplink_pilot_rx = zeros(N_BS_NODE, N_UE_NODE, ue_pilot_len);
uplink_pilot_csi = zeros(N_BS_NODE, N_UE_NODE, N_SC);

for ibs =1:N_BS_NODE
    v0 = filter(fliplr(conj(uplink_pilot_tx(1 : UE_DATA_REP * N_SYM_SAMP))), a, rx_vec_pilot(ibs, :));
    v1 = filter(unos, a, abs(rx_vec_pilot(ibs, :)) .^ 2);
    m_filt = (abs(v0) .^ 2) ./ v1; % normalized correlation
    [~, max_idx] = max(abs(m_filt));
    % In case of bad correlatons:
    pilot_data_start = max_idx + 1 - ue_pilots_len;
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

%% Downlink CSI Calculation and Zeroforcing
downlink_pilot_csi = zeros(N_BS_NODE, N_UE_NODE, N_SC);
ifft_in_mat = zeros(N_BS_NODE, N_SC, N_OFDM_SYMS);
for isc =1:N_SC
    downlink_pilot_csi(:, :, isc) = diag(squeeze(cal_mat(:, isc))) * squeeze(uplink_pilot_csi(:, :, isc));
    downlink_beam_weights = pinv(squeeze(downlink_pilot_csi(:, :, isc)));
    for isym = 1:N_OFDM_SYMS
        ifft_in_mat(:, isc, isym) = downlink_beam_weights.' * precoding_in_mat(:, isc, isym);
    end
end

%% IFFT

%Perform the IFFT
tx_payload_mat = zeros(N_BS_NODE, N_SYM_SAMP, N_OFDM_SYMS);
for ibs = 1:N_BS_NODE
    for isym = 1:N_OFDM_SYMS
        tx_sym = squeeze(ifft(ifft_in_mat(ibs, :, isym)));
        tx_payload_mat(ibs, :, isym) = [tx_sym(end - CP_LEN + 1: end) tx_sym].';
    end
end


% Reshape to a vector
tx_payload_vec = reshape(tx_payload_mat, N_BS_NODE, numel(tx_payload_mat(1, :, :)));

bs_sched_id = 4;
node_bs.set_tddconfig(1, bs_sched(bs_sched_id)); % configure the BS: schedule etc.

ue_sched_id = 2;
node_ue.set_tddconfig(WIRED_UE, ue_sched(ue_sched_id)); % configure the BS: schedule etc.

% Write beamformed signal to all antennas
donwlink_postfix_len = N_SAMP - N_ZPAD_PRE - N_OFDM_SYMS * N_SYM_SAMP;
for i=1:N_BS_NODE
    tx_data = [zeros(1, N_ZPAD_PRE) tx_payload_vec(i, :) zeros(1, donwlink_postfix_len)];
    tx_vec_iris = TX_SCALE .* tx_data ./ max(abs(tx_data));
    node_bs.sdrtx_single(tx_vec_iris, i);       % Burn data to the UE RAM
end

if ~WIRED_UE
    node_ue.sdr_setcorr();              % activate correlator
end

% Transmit beamformed signal from all antennas and receive at UEs
node_ue.sdr_activate_rx();   % activate reading stream
node_bs.sdrtrigger();
[rx_vec_downlink, ~] = node_ue.sdrrx(N_SAMP); % read data

if ~WIRED_UE
    node_ue.sdr_unsetcorr();              % activate correlator
end

node_bs.sdr_close();
node_ue.sdr_close();


%% Process downlink receive signal
dl_data_start = zeros(1, N_UE_NODE);
for iue=1:N_UE_NODE
    v0 = filter(fliplr(conj(lts)), a, rx_vec_downlink(iue, :));
    v1 = filter(unos, a, abs(rx_vec_downlink(iue, :)) .^ 2);
    m_filt = (abs(v0) .^ 2) ./ v1; % normalized correlation
    [~, max_idx_dl] = max(abs(m_filt));
    % In case of bad correlatons:
    dl_data_start(iue) = max_idx_dl + 1 - N_PILOTS_SYMS * length(lts);
    if dl_data_start(iue) < 0
       disp('bad dl data!');
    end
end

N_RX_OFDM_SYMS = min(N_OFDM_SYMS, floor((N_SAMP - dl_data_start(1))/N_SYM_SAMP));
rx_dl_vec = zeros(N_UE_NODE, N_RX_OFDM_SYMS * N_SYM_SAMP);
for iue=1:N_UE_NODE
    end_idx = min(4096, dl_data_start(iue) + N_RX_OFDM_SYMS * N_SYM_SAMP - 1);
    rx_dl_vec(iue, :) = rx_vec_downlink(iue, dl_data_start(iue): end_idx);
end
rx_dl_mat = reshape(rx_dl_vec, N_UE_NODE, N_SYM_SAMP, N_RX_OFDM_SYMS);
if(CP_LEN > 0)
    rx_dl_mat = rx_dl_mat(:, CP_LEN+1:end, :);
end
rx_dl_f_mat = fft(rx_dl_mat, N_SC, 2);
rx_lts_f = zeros(N_UE_NODE, N_SC);
for p=1:N_PILOTS_SYMS
    rx_lts_f = rx_lts_f + repmat(lts_f, N_UE_NODE, 1).*rx_dl_f_mat(:, :, p);
end
dl_syms_f_mat = rx_dl_f_mat(:, :, N_PILOTS_SYMS+1:end);
rx_H_est = rx_lts_f/N_PILOTS_SYMS;

N_RX_DATA_OFDM_SYMS = N_RX_OFDM_SYMS - N_PILOTS_SYMS;
dl_syms_eq_mat = zeros(N_UE_NODE, N_SC, N_RX_DATA_OFDM_SYMS);
for i=1:N_RX_DATA_OFDM_SYMS
    dl_syms_eq_mat(:,:,i) = squeeze(dl_syms_f_mat(:,:,i))./rx_H_est;
end

pilots_eq_mat = dl_syms_eq_mat(:,SC_IND_PILOTS,:);
%pilots_eq_mat_comp = zeros(size(pilots_eq_mat));
%for iue=1:N_UE_NODE
%    pilots_eq_mat_comp(iue, :, :) = pilots_eq_mat(iue, :, :).*pilots_mat(iue, :, :);
%end
pilots_eq_mat_comp = pilots_eq_mat.*pilots_mat;

pilot_dl_phase_err = squeeze(angle(mean(pilots_eq_mat_comp,2)));

pilot_dl_phase_corr = zeros(N_UE_NODE, N_SC, N_RX_DATA_OFDM_SYMS);
for i=1:N_SC
    pilot_dl_phase_corr(:,i,:) = exp(-1i*pilot_dl_phase_err);
end
  
% Apply the pilot phase correction per symbol
dl_syms_eq_pc_mat = dl_syms_eq_mat.* pilot_dl_phase_corr;
payload_dl_syms_mat = dl_syms_eq_pc_mat(:, SC_IND_DATA, :);
payload_dl_syms_mat = reshape(payload_dl_syms_mat, N_UE_NODE, numel(payload_dl_syms_mat(1,:,:)));


%% Plotting
cf = 0;

% Reciprocal Calibration Vectors Plots
if RECIP_PLOT
    cf = cf + 1;
    figure(cf);clf;

    for i = 1:N_BS_NODE
        subplot(N_BS_NODE, 1, i);
        plot(-32:1:31, abs(cal_mat(i, :)));
        axis([-40 40 0 5])
        grid on;
        title('Calibration MAGNITUDE');
    end

    cf = cf + 1;
    figure(cf);clf;

    for i = 1:N_BS_NODE
        subplot(N_BS_NODE, 1, i);
        plot(-32:1:31, angle(cal_mat(i, :)));
        axis([-40 40 -pi pi])
        grid on;
        title('Calibration ANGLE');
    end
end

% Uplink Pilot Rx Vectors Plots
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

% Downlink Rx Vector and Constellation Plots
if DOWNLINK_PLOT
    cf = cf + 1;
    figure(cf);clf;

    for i = 1:N_UE_NODE
        subplot(N_UE_NODE, 1, i);
        plot(real(rx_vec_downlink(i, :)));
        xline(dl_data_start(i),'--r')
        axis([0 N_SAMP -.1 .1])
        grid on;
        title('Received Pilots (Real)');
    end
    % DL
    cf = cf + 1;
    figure(cf); clf;
    
    for i = 1:N_UE_NODE
        subplot(N_UE_NODE,1,i)
        plot(payload_dl_syms_mat(i, :),'ro','MarkerSize',1);
        axis square; axis(1.5*[-1 1 -1 1]);
        grid on;
        hold on;

        plot(tx_syms(:),'bo');
        title('Downlink Tx and Rx Constellations')
        legend('Rx','Tx');
    end
end




