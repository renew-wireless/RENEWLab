%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Downlink Single-User massive MIMO script.
%   Script does the following:
%   1) Reciprocity calibration
%   2) Implicit sounding: Uplink pilot transmission
%   3) Downlink beamforming (use beamweights computed in step2)
%   4) Compute stats and plot
%
%---------------------------------------------------------------------
% Original code copyright Mango Communications, Inc.
% Distributed under the WARP License http://warpproject.org/license
% Copyright (c) 2018-2019, Rice University
% RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
% ---------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
close all;

[version, executable, isloaded] = pyversion;
if ~isloaded
    pyversion /usr/bin/python
    py.print() %weird bug where py isn't loaded in an external script
end
py.importlib.import_module('iris_py')

% Params:
WRITE_PNG_FILES    = 0;                                      % Enable writing plots to PNG

%Iris params:
USE_HUB                 = 1;
WIRED_UE                = 0;
TX_FRQ                    = 3.6e9;    
RX_FRQ                    = TX_FRQ;
TX_GN                      = 80;
RX_GN                      = 65;
SMPL_RT                   = 5e6;  
N_FRM                      = 1;
bs_ids                       = string.empty();
bs_sched                  = string.empty();
ue_sched                  = string.empty();

% Waveform params
TX_SCALE                = 1;                                      % Scale for Tx waveform ([0:1])

% OFDM params
SC_IND_PILOTS         = [8 22 44 58];                    % Pilot subcarrier indices
SC_IND_DATA           = [2:7 9:21 23:27 39:43 45:57 59:64];     % Data subcarrier indices
SC_IND_DATA_PILOT = [2:27 39:64]';
N_SC                       = 64;                                    % Number of subcarriers
CP_LEN                    = 16;                                    % Cyclic prefix length
N_SYM_SAMP           = N_SC + CP_LEN;                % Number of samples that will go over the air
N_SAMP                   = 4096;                               % N_ZPAD_PRE + data_len + N_ZPAD_POST;
N_ZPAD_PRE            = 160;                                 % Zero-padding prefix for Iris
N_ZPAD_POST          = 160;                                 % Zero-padding postfix for Iris
N_OFDM_SYMS         = floor((N_SAMP - N_ZPAD_PRE - N_ZPAD_POST) / N_SYM_SAMP);  % Number of OFDM symbols for burst, it needs to be less than 47
N_PILOTS_SYMS        = 2;
N_DATA_SYMS          = (N_OFDM_SYMS - N_PILOTS_SYMS);       % Number of data symbols (one per data-bearing subcarrier per OFDM symbol)
N_DATA_SC               = N_DATA_SYMS * length(SC_IND_DATA);
MOD_ORDER             = 4;                                    % Modulation order (2/4/16/64 = BSPK/QPSK/16-QAM/64-QAM)

% Rx processing params         
RECIP_PLOT              = 0;
PILOT_PLOT              = 1;
DOWNLINK_PLOT      = 1;
AUTO_OFFSET          = 1;

timing_offset            = 8;
FFT_OFFSET            = 0;                                      % Number of CP samples to use in FFT (on average)

%% Define the preamble
% LTS for fine CFO and channel estimation
lts_f = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 ...
    1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1];
lts_t = ifft(lts_f, 64); %time domain
lts = [lts_t(49:64) lts_t];
lts_lcp = [lts_t(33:64) lts_t lts_t]; % 2.5 LTS

%% Init Iris nodes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set up the Iris experiment
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create BS Hub and UE objects. Note: BS object is a collection of Iris
% nodes.

if USE_HUB
    hub_id = "FH4B000019";
else
    hub_id = [];
end

% Last node in list is calibration node!
%bs_ids = ["RF3E000300", "RF3E000510", "RF3E000484", "RF3E000460", "RF3E000496", "RF3E000476", "RF3E000537", "RF3E000089"];
bs_ids = ["RF3E000347", "RF3E000564", "RF3E000569", "RF3E000639", "RF3E000605", "RF3E000600", "RF3E000611", "RF3E000089"];
ue_ids= ["RF3E000241"];

beacon_node = 0; % set 0 to make all nodes send beacon

ref_calib_sched =  "RGPG";
bs_calib_sched = "PGRG";

bs_ul_sched_beacon = "BGRG";
bs_ul_sched = "GGRG";
ue_ul_sched = "GGPG";

bs_dl_sched_beacon = "BGPG";
bs_dl_sched = "GGPG";
ue_dl_sched = "GGRG";

N_BS_NODE = length(bs_ids);
N_UE_NODE = 1;

REF_ANT = 8; %ceil(N_BS_NODE/2);
bs_index = 1:N_BS_NODE;
bs_index(REF_ANT) = [];

% Manually measured offsets for the start of data
rx_cal_data_start = 168 * ones(1, N_BS_NODE);
rx_cal_data_start(REF_ANT) = 292;
pilot_data_start = 150 * ones(1, N_BS_NODE);
dl_data_start = 305;
if WIRED_UE
  pilot_data_start(:) = 228;
  dl_data_start(:) = 228;
  rx_cal_data_start(:) = 228;
end


%% Generate a payload of random integers
tx_data = randi(MOD_ORDER, 1, N_DATA_SC) - 1;

tx_syms = mod_sym(tx_data, MOD_ORDER);
% Reshape the symbol vector to a matrix with one column per OFDM symbol
tx_syms_mat = reshape(tx_syms, length(SC_IND_DATA), N_DATA_SYMS);

% Define the pilot tone values as BPSK symbols
pilots = [1 1 -1 1].';

% Repeat the pilots across all OFDM symbols
pilots_mat = repmat(pilots, 1, N_DATA_SYMS);

% Construct the precoding input matrix
precoding_in_mat = zeros(N_SC, N_OFDM_SYMS);

% Insert the data and pilot values; other subcarriers will remain at 0
for i = 1:N_PILOTS_SYMS
    precoding_in_mat(:, i) = lts_f;
end
precoding_in_mat(SC_IND_DATA, N_PILOTS_SYMS + 1:end)   = tx_syms_mat;
precoding_in_mat(SC_IND_PILOTS, N_PILOTS_SYMS + 1:end) = pilots_mat;

% reciprocity calibration tx pilots
N_BS = N_BS_NODE - 1;
DATA_REP = floor(N_OFDM_SYMS / N_BS);
lts_rep = repmat(lts, 1, DATA_REP);
lts_rep_len = DATA_REP * N_SYM_SAMP;
data_len = N_BS * lts_rep_len;
recip_tx = zeros(N_BS_NODE, data_len);
for nid = 1:N_BS
    first = (nid - 1) * lts_rep_len + 1;
    last = nid * lts_rep_len;
    recip_tx(nid + (nid >= REF_ANT), first:last) = lts_rep;
    recip_tx(REF_ANT, first:last) = lts_rep;
end

% uplink tx piltos
UE_DATA_REP = N_OFDM_SYMS;
ue_pilot_len = UE_DATA_REP * N_SYM_SAMP;
uplink_pilot_tx = zeros(1, ue_pilot_len);
for rp = 1:UE_DATA_REP
    start_index = (rp - 1) * N_SYM_SAMP;
    uplink_pilot_tx(start_index + 1: start_index + N_SYM_SAMP) = lts;
end

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
ue_sdr_params.is_bs = 0;
    
node_bs = iris_py(bs_sdr_params, hub_id);% initialize BS
node_ue = iris_py(ue_sdr_params, []);    % initialize UE

node_bs.sdrsync();                 % Synchronize delays only for BS
node_bs.sdrrxsetup();
node_bs.sdr_setupbeacon();   % Burn beacon to the BS RAM

node_ue.sdr_configgainctrl();
node_ue.sdrrxsetup();


%% Step 1: Reciprocity Calibration
recip_postfix_len = N_SAMP - data_len - N_ZPAD_PRE;
schedule = bs_calib_sched;
for i=1:N_BS_NODE
    if i == REF_ANT
        schedule = ref_calib_sched;
    end
    node_bs.set_tddconfig_single(1, schedule, i); % configure the BS: schedule etc.
    tx_signal = [zeros(1, N_ZPAD_PRE) recip_tx(i, :) zeros(1, recip_postfix_len)];
    node_bs.sdrtx_single(tx_signal, i);  % Burn data to the UE RAM
end
node_bs.sdr_activate_rx();   % activate reading stream
[rx_vec_iris, ~] = node_bs.sdrrx(N_SAMP); % read data

a = 1;
unos = ones(size(conj(lts)));
% default offsets measured with prefix and postfix = 160
if AUTO_OFFSET
    for ibs = 1:N_BS_NODE
        % Correlation through filtering
        v0 = filter(fliplr(conj(recip_tx(REF_ANT, end - DATA_REP * N_SYM_SAMP: end))), a, rx_vec_iris(ibs, end - DATA_REP * N_SYM_SAMP: end));
        v1 = filter(unos, a, abs(rx_vec_iris(ibs, end - DATA_REP * N_SYM_SAMP: end)) .^ 2);
        lts_corr = (abs(v0) .^ 2) ./ v1; % normalized correlation

        % position of the last peak
        [~, max_idx] = max(abs(lts_corr));
        rx_cal_data_start(ibs) = max_idx + (N_SAMP - DATA_REP * N_SYM_SAMP) - data_len;
        if rx_cal_data_start(ibs) < 0
           display('bad receive!');
           break;
        end
    end
end

% recip_rx = zeros(N_BS_NODE, data_len);
rx_fft = zeros(N_BS, N_SC);
rx_fft_ref = zeros(N_BS, N_SC);
cal_mat = zeros(N_BS_NODE, N_SC);
first = rx_cal_data_start(REF_ANT);
last = rx_cal_data_start(REF_ANT) + data_len - 1;
recip_rx_ref = rx_vec_iris(REF_ANT, first:last);
recip_rx_ref_mat = reshape(recip_rx_ref, N_SYM_SAMP, DATA_REP, N_BS);
recip_ref_fft_mat = fft(recip_rx_ref_mat(CP_LEN+1:N_SYM_SAMP, :, :), N_SC, 1);

recip_rx_bs = rx_vec_iris(bs_index, 1:lts_rep_len);
recip_rx_bs_mat = reshape(recip_rx_bs, N_BS, N_SYM_SAMP, DATA_REP);
recip_bs_fft_mat = fft(recip_rx_bs_mat(:, CP_LEN+1:N_SYM_SAMP, :), N_SC, 2);

for i = 1:DATA_REP
    rx_fft = rx_fft + squeeze(recip_bs_fft_mat(:, :, i));
    rx_fft_ref = rx_fft_ref + squeeze(recip_ref_fft_mat(:, i, :)).';
end

for sid = 1:N_BS
    nid = sid + (sid >= REF_ANT);
    % cal_mat(nid, :) = (rx_fft_ref(sid, :) / DATA_REP) ./ (rx_fft(sid, :) / (N_BS * DATA_REP));
    cal_mat(nid, :) = (rx_fft_ref(sid, :) / DATA_REP) ./ (rx_fft(sid, :) / DATA_REP);
end


%% Step 2: Uplink Pilot Collection and Channel Estimation
node_bs.sdr_set_n_frame(100);
schedule = bs_ul_sched;
if beacon_node == 0
    schedule = bs_ul_sched_beacon;
end
node_bs.set_tddconfig(1, schedule); % configure the BS: schedule etc.
if beacon_node > 0
    node_bs.set_tddconfig_single(1, bs_ul_sched_beacon, beacon_node);
end

node_ue.set_tddconfig(WIRED_UE, ue_ul_sched); % configure the BS: schedule etc.

tx_signal = [zeros(1, N_ZPAD_PRE) uplink_pilot_tx zeros(1, N_ZPAD_POST)];
node_ue.sdrtx_single(tx_signal, 1);       % Burn data to the UE RAM

if ~WIRED_UE
    node_ue.sdr_setcorr();              % activate correlator
end

[rx_vec_pilot_all, data_len_all] = node_bs.sdrrx(N_SAMP, 0); % read data

pilot_rep = 1;
rx_vec_pilot = node_bs.get_best_frame(rx_vec_pilot_all, N_SAMP);
data0_len = length(rx_vec_pilot);

if ~WIRED_UE
    node_ue.sdr_unsetcorr();              % activate correlator
end

if AUTO_OFFSET
    for ibs =1:N_BS_NODE
        lts_corr = abs(conv(conj(fliplr(lts_t)), sign(rx_vec_pilot(ibs, :))));
        lts_peaks = find(lts_corr > 0.8*max(lts_corr));
        [LTS1, LTS2] = meshgrid(lts_peaks,lts_peaks);
        [lts_second_peak_index,y] = find(LTS2-LTS1 == length(lts));  % use size of lts+cp (80)
        % Stop if no valid correlation peak was found
        if(isempty(lts_second_peak_index))
            fprintf('UPLINK PILOT COLLECTION: No LTS Correlation Peaks Found!\n');
            break;
        end
        offset = lts_peaks(lts_second_peak_index(1)) - (2*length(lts));  % Get sSecond peak
        pilot_data_start(ibs) = offset;

        %v0 = filter(fliplr(conj(uplink_pilot_tx)), a, rx_vec_pilot(ibs, :));
        %v1 = filter(unos, a, abs(rx_vec_pilot(ibs, :)) .^ 2);
        %m_filt = (abs(v0) .^ 2) ./ v1; % normalized correlation
        %[~, max_idx] = max(abs(m_filt));
        %%In case of bad correlatons:
        %pilot_data_start(ibs) = max_idx + 1 - ue_pilot_len;
        if pilot_data_start(ibs) < 0
           display('Uplink Pilots: Bad receive! Exit Now!');
           return;
        end
    end
end

uplink_pilot_rx = zeros(N_BS_NODE, ue_pilot_len);
uplink_pilot_csi = zeros(N_BS_NODE, N_SC);
for ibs =1:N_BS_NODE
    uplink_pilot_rx(ibs, :) = rx_vec_pilot(ibs, pilot_data_start(ibs): pilot_data_start(ibs) + ue_pilot_len - 1);
    pilot_data_start(ibs) = pilot_data_start(ibs) + ue_pilot_len;
    rx_fft = zeros(UE_DATA_REP, N_SC);
    for irp = 1:UE_DATA_REP
        start = (irp - 1) * N_SYM_SAMP + CP_LEN;
        rx_fft(irp, :) = fft(uplink_pilot_rx(ibs, start + 1: start + N_SC));
    end
    uplink_pilot_csi(ibs, :) = mean(rx_fft) .* lts_f; % mean across reps
end


%% Step 3: Downlink Transmission
% CSI Calculation and Zeroforcing
downlink_pilot_csi = zeros(N_BS_NODE, N_SC);
ifft_in_mat = zeros(N_BS_NODE, N_SC, N_OFDM_SYMS);
for isc =1:N_SC
    downlink_pilot_csi(:, isc) = diag(squeeze(cal_mat(:, isc))) * squeeze(uplink_pilot_csi(:, isc));
    downlink_beam_weights = pinv(squeeze(downlink_pilot_csi(:, isc)));
    for isym = 1:N_OFDM_SYMS
        ifft_in_mat(:, isc, isym) = downlink_beam_weights.' * precoding_in_mat(isc, isym);
    end
end

% IFFT
tx_payload_mat = zeros(N_BS_NODE, N_SYM_SAMP, N_DATA_SYMS);
tx_pilot_mat = zeros(N_BS_NODE, length(lts_t)*2.5);
for ibs = 1:N_BS_NODE
    
    pilot1 = squeeze(ifft(ifft_in_mat(ibs, :, 1)));
    pilot2 = squeeze(ifft(ifft_in_mat(ibs, :, 2)));
    tx_pilot_mat(ibs, :) = [pilot2(33:64) pilot1 pilot2];
    
    for isym = N_PILOTS_SYMS+1:N_OFDM_SYMS
        tx_sym = squeeze(ifft(ifft_in_mat(ibs, :, isym)));
        tx_payload_mat(ibs, :, isym-N_PILOTS_SYMS) = [tx_sym(end - CP_LEN + 1: end) tx_sym].';
    end
end

% Reshape to a vector
tx_payload_vec = reshape(tx_payload_mat, N_BS_NODE, numel(tx_payload_mat(1, :, :)));
tx_payload_vec = [tx_pilot_mat tx_payload_vec];

% configure the BS: schedule
schedule = bs_dl_sched;
if beacon_node == 0
    schedule = bs_dl_sched_beacon;
end
node_bs.set_tddconfig(1, schedule); % configure the BS: schedule etc.
if beacon_node > 0
    node_bs.set_tddconfig_single(1, bs_dl_sched_beacon, beacon_node);
end

% configure the UE: schedule
node_ue.set_tddconfig(WIRED_UE, ue_dl_sched); 

% Write beamformed signal to all antennas
donwlink_postfix_len = N_SAMP - N_ZPAD_PRE - N_OFDM_SYMS * N_SYM_SAMP;
for i=1:N_BS_NODE
    tx_signal = [zeros(1, N_ZPAD_PRE) tx_payload_vec(i, :) zeros(1, donwlink_postfix_len)];
    tx_vec_iris = TX_SCALE .* tx_signal ./ max(abs(tx_signal));
    node_bs.sdrtx_single(tx_vec_iris, i);       % Burn data to the UE RAM
end

if ~WIRED_UE
    node_ue.sdr_setcorr();              % activate correlator
end

% Transmit beamformed signal from all antennas and receive at UEs
bad_pilot = true;
bad_cnt = 0;
bad_cnt_max = 1000;
while bad_pilot
    rx_vec_dl = zeros(N_UE_NODE, N_SAMP);
    node_ue.sdr_activate_rx();   % activate reading stream
    node_bs.sdrtrigger();
    [rx_dl, ~] = node_ue.uesdrrx(N_SAMP); % read data
    rx_vec_dl(:, :) = rx_dl.';
    rx_vec_downlink = zeros(N_UE_NODE, N_SAMP);
    
    all_ue_rx = 0;
    for j=1:N_UE_NODE
        if (sum(abs(rx_vec_dl(j,:))) > 0) % successful receive
            all_ue_rx = all_ue_rx + 1;
            fprintf('Downlink Beacon Successful at UE %d \n', j);
        else
            fprintf('WARNING: NO Downlink Beacon Detected at UE %d \n', j);
        end
    end
    if (all_ue_rx == N_UE_NODE)
        rx_vec_downlink = rx_vec_dl(:, :);
    end

    % Process downlink receive signal
    if AUTO_OFFSET  
        % Correlation
        %lts_rep_dl = repmat(lts_t, 1, N_PILOTS_SYMS);  % tx_vec_iris
        lts_corr = abs(conv(conj(fliplr(lts_t)), sign(rx_vec_downlink)));
        %figure; plot(lts_corr);
        lts_peaks = find(lts_corr > 0.8*max(lts_corr));
        [LTS1, LTS2] = meshgrid(lts_peaks,lts_peaks);
        [lts_second_peak_index,y] = find(LTS2-LTS1 == length(lts_t));  % use size of lts_t
        % Stop if no valid correlation peak was found
        if(isempty(lts_second_peak_index))
            bad_cnt = bad_cnt+ 1;
            fprintf('DOWNLINK TRAINING: No LTS Correlation Peaks Found! Count: %d \n', bad_cnt);
            if bad_cnt == 1000
                fprintf('Bad correlation exceeded max number of tries (%d). Exit now! \n', bad_cnt_max);
                return;
            end
        else
            offset = lts_peaks(lts_second_peak_index(1)) + 1;
            dl_data_start = offset;
            dl_pilot_start = offset-(2.5*length(lts_t));
            fprintf('CORR. PEAK AT: %d, PILOT STARTS AT: %d \n', offset-1, dl_pilot_start);
            stop = 1;
            bad_pilot = false;
        end
        
        % Another correlation method (similar performance to code above)...
        if 0 
            lts_rep_dl = repmat(lts_t, 1, N_PILOTS_SYMS);
            unos = ones(size(lts_rep_dl));
            v0 = filter(fliplr(conj(lts_rep_dl)), a, rx_vec_downlink);  
            v1 = filter(unos, a, abs(rx_vec_downlink) .^ 2);
            m_filt = (abs(v0) .^ 2) ./ v1; % normalized correlation
            [~, max_idx_dl] = max(abs(m_filt));
            %% In case of bad correlatons:
            dl_data_start = max_idx_dl + 1; %max_idx_dl + 1 - N_PILOTS_SYMS * length(lts);
            fprintf('CORR. ALT. PEAK AT: %d \n', max_idx_dl);
            if dl_data_start < 0
               disp('bad dl data!');
            end
        end        
    end
end

% Deactivate correlator and cleanup
if ~WIRED_UE
    node_ue.sdr_unsetcorr();              
end
node_bs.sdr_close();
node_ue.sdr_close();


%% Step 4: Process Received Data (Downlink)
% Pilots
rx_dl_pilot_vec = rx_vec_downlink(1, dl_pilot_start - timing_offset: dl_pilot_start + 2*length(lts) - 1 - timing_offset);
rx_lts1 = rx_dl_pilot_vec(-64  + -FFT_OFFSET + [97:160]);
rx_lts2 = rx_dl_pilot_vec(-FFT_OFFSET + [97:160]);
rx_lts1_f = fft(rx_lts1);
rx_lts2_f = fft(rx_lts2);
rx_H_est = (lts_f).* (rx_lts1_f + rx_lts2_f) / 2;

% Data
N_RX_DATA_SYMS = min(N_DATA_SYMS, floor((N_SAMP - dl_data_start)/N_SYM_SAMP));
rx_dl_data_vec = zeros(N_RX_DATA_SYMS * N_SYM_SAMP);
end_idx = min(4096, dl_data_start + N_RX_DATA_SYMS * N_SYM_SAMP - 1);
rx_dl_data_vec = rx_vec_downlink(1, dl_data_start - timing_offset: end_idx - timing_offset);
rx_dl_data_mat = reshape(rx_dl_data_vec, N_SYM_SAMP, N_RX_DATA_SYMS );
if(CP_LEN > 0)
    rx_dl_data_mat = rx_dl_data_mat(CP_LEN+1-FFT_OFFSET:end-FFT_OFFSET, :);
end
rx_dl_f_mat = fft(rx_dl_data_mat, N_SC, 1);
N_RX_DATA_OFDM_SYMS = N_RX_DATA_SYMS;

dl_syms_eq_mat = zeros(N_SC, N_RX_DATA_OFDM_SYMS);
for i=1:N_RX_DATA_OFDM_SYMS
    dl_syms_eq_mat(:,i) = squeeze(rx_dl_f_mat(:,i).')./rx_H_est;
end

pilots_eq_mat = dl_syms_eq_mat(SC_IND_PILOTS,:);
pilots_eq_mat_comp = pilots_eq_mat.*repmat(pilots, 1, N_RX_DATA_OFDM_SYMS);

pilot_dl_phase_err = squeeze(angle(mean(pilots_eq_mat_comp,1)));

pilot_dl_phase_corr = zeros(N_SC, N_RX_DATA_OFDM_SYMS);
for i=1:N_SC
    pilot_dl_phase_corr(i,:) = exp(-1i*pilot_dl_phase_err);
end
  
% Apply the pilot phase correction per symbol
dl_syms_eq_pc_mat = dl_syms_eq_mat.* pilot_dl_phase_corr;
payload_dl_syms_mat = dl_syms_eq_pc_mat(SC_IND_DATA, :);


%% Step 5: Demodulate and Print Stats
rx_syms = reshape(payload_dl_syms_mat, 1, N_DATA_SC);

rx_data = demod_sym(rx_syms ,MOD_ORDER);

% EVM & SNR
% Do yourselves. Calculate EVM and effective SNR:
evm_mat = abs(payload_dl_syms_mat - tx_syms_mat).^2;
aevms = mean(evm_mat(:)); % needs to be a scalar
snr = 10*log10(1./aevms); % calculate in dB scale.

sym_errs = sum(tx_data ~= rx_data);
bit_errs = length(find(dec2bin(bitxor(tx_data, rx_data),8) == '1'));
rx_evm   = sqrt(sum((real(rx_syms) - real(tx_syms)).^2 + (imag(rx_syms) - imag(tx_syms)).^2)/(length(SC_IND_DATA) * N_DATA_SYMS));

fprintf('\nResults:\n');
fprintf('Num Bytes:  \t  %d\n', N_DATA_SC * log2(MOD_ORDER) / 8);
fprintf('Sym Errors:  \t %d (of %d total symbols)\n', sym_errs, N_DATA_SC);
fprintf('Bit Errors: \t %d (of %d total bits)\n', bit_errs, N_DATA_SC * log2(MOD_ORDER));
fprintf('EVM: \t %f, SNR: %f \n', aevms, snr);


%% Step 6: Plotting
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

    plot(real(rx_vec_pilot.'));
    %axis([0 N_SAMP -1 1])
    grid on;
    title('Received Pilots (Real)');
end

% Downlink Rx Vector and Constellation Plots
if DOWNLINK_PLOT
    fst_clr = [0, 0.4470, 0.7410];
    sec_clr = [0.8500, 0.3250, 0.0980];
    cf = cf + 1;
    figure(cf);clf;

    plot(real(rx_vec_downlink));
    xline(dl_data_start,'--r')
    axis([0 N_SAMP -1 1])
    grid on;
    title('Received Data (Real)');
    % DL
    cf = cf + 1;
    figure(cf); clf;
    
    plot(payload_dl_syms_mat(:),'ro','MarkerSize',1);
    axis square; axis(1.5*[-1 1 -1 1]);
    grid on;
    hold on;

    plot(tx_syms(:),'bo');
    title('Downlink Tx and Rx Constellations')
    legend('Rx','Tx');


    % EVM & SNR
    cf = cf + 1;
    figure(cf); clf;
    subplot(2,1,1)
    plot(100*evm_mat(:),'o','MarkerSize',1)
    axis tight
    hold on
    plot([1 length(evm_mat(:))], 100*[aevms, aevms],'color', sec_clr,'LineWidth',4)
    myAxis = axis;
    h = text(round(.05*length(evm_mat(:))), 100*aevms+ .1*(myAxis(4)-myAxis(3)), sprintf('Effective SNR: %.1f dB', snr));
    set(h,'Color',[1 0 0])
    set(h,'FontWeight','bold')
    set(h,'FontSize',10)
    set(h,'EdgeColor',[1 0 0])
    set(h,'BackgroundColor',[1 1 1])
    hold off
    xlabel('Data Symbol Index')
    ylabel('EVM (%)');
    legend('Per-Symbol EVM','Average EVM','Location','NorthWest');
    title('EVM vs. Data Symbol Index')
    grid on

    subplot(2,1,2)
    imagesc(1:N_DATA_SYMS, (SC_IND_DATA - N_SC/2), 100*fftshift(evm_mat,1))

    grid on
    xlabel('OFDM Symbol Index')
    ylabel('Subcarrier Index')
    title('EVM vs. (Subcarrier & OFDM Symbol)')
    h = colorbar;
    set(get(h,'title'),'string','EVM (%)');
    myAxis = caxis();
    if (myAxis(2)-myAxis(1)) < 5
        caxis([myAxis(1), myAxis(1)+5])
    end
end
