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
WRITE_PNG_FILES    = 0;                                       % Enable writing plots to PNG

%Iris params:
USE_HUB                 = 0.8;
WIRED_UE                = 0;                                  % Enable if UE is directly connected to the BS (wired)
TX_FRQ                  = 3.6e9;    
RX_FRQ                  = TX_FRQ;
TX_GN                   = 70;
RX_GN                   = 70;
SMPL_RT                 = 5e6;  
N_FRM                   = 10;
bs_ids                  = string.empty();
ue_ids                  = string.empty();
bs_sched                = string.empty();
ue_sched                = string.empty();

% Waveform params
TX_SCALE                = 1;                                  % Scale for Tx waveform ([0:1])

% OFDM params
SC_IND_PILOTS           = [8 22 44 58];                       % Pilot subcarrier indices
SC_IND_DATA             = [2:7 9:21 23:27 39:43 45:57 59:64]; % Data subcarrier indices
SC_IND_DATA_PILOT       = [2:27 39:64]';
N_SC                    = 64;                                 % Number of subcarriers
CP_LEN                  = 16;                                 % Cyclic prefix length
N_SYM_SAMP              = N_SC + CP_LEN;                      % Number of samples that will go over the air
N_SAMP                  = 4096;                               % N_ZPAD_PRE + data_len + N_ZPAD_POST;
N_ZPAD_PRE              = 128;                                % Zero-padding prefix for Iris
N_ZPAD_POST             = 128;                                % Zero-padding postfix for Iris
N_OFDM_SYMS             = floor((N_SAMP - N_ZPAD_PRE - N_ZPAD_POST) / N_SYM_SAMP);  % Number of OFDM symbols for burst, it needs to be less than 47
N_PILOTS_SYMS           = 2;
N_DATA_SYMS             = (N_OFDM_SYMS - N_PILOTS_SYMS);      % Number of data symbols (one per data-bearing subcarrier per OFDM symbol)
N_DATA_SC               = N_DATA_SYMS * length(SC_IND_DATA);
N_LTS_SYM               = N_PILOTS_SYMS;
MOD_ORDER               = 4;                                  % Modulation order (2/4/16/64 = BSPK/QPSK/16-QAM/64-QAM)

% Rx processing params         
RECIP_PLOT              = 0;
PILOT_PLOT              = 1;
DOWNLINK_PLOT           = 1;
AUTO_OFFSET             = 1;
FFT_OFFSET              = 0;                                      % Number of CP samples to use in FFT (on average)

%% Define the preamble
% LTS for fine CFO and channel estimation
lts_f = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 ...
    1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1];
lts_t = ifft(lts_f, N_SC); %time domain
preamble = [lts_t(33:64), repmat(lts_t,1,N_LTS_SYM)];  % 2.5 LTS

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

chain2 = ["RF3E000087", "RF3E000084", "RF3E000107", "RF3E000110", "RF3E000086", "RF3E000162", "RF3E000127", "RF3E000597"];
chain3 = ["RF3E000346", "RF3E000543", "RF3E000594", "RF3E000404", "RF3E000616", "RF3E000622", "RF3E000601", "RF3E000602"];
chain4 = ["RF3E000146", "RF3E000122", "RF3E000150", "RF3E000128", "RF3E000168", "RF3E000136", "RF3E000213", "RF3E000142"];
chain5 = ["RF3E000356", "RF3E000546", "RF3E000620", "RF3E000609", "RF3E000604", "RF3E000612", "RF3E000640", "RF3E000551"];
chain6 = ["RF3E000208", "RF3E000636", "RF3E000632", "RF3E000568", "RF3E000558", "RF3E000633", "RF3E000566", "RF3E000635"];
bs_ids = [chain3];
ue_ids= ["RF3E000164"]; % "RF3E000392"
N_BS_NODE = length(bs_ids);
N_UE_NODE = length(ue_ids);

% Sounding Schedule
% Max number of symbols per frame is 256
ue_sched = "GG";
bs_sched = "GG" + repelem('G',[2*N_BS_NODE]);
bs_sched = repmat(bs_sched, 1, N_BS_NODE);
bs_sched{1}(1) = 'B';                       % Only the first antenna sends beacon
for bs_node_idx = 1:N_BS_NODE
    ue_sched = ue_sched + "RG";
    bs_sched{bs_node_idx}(2*bs_node_idx+1:2*bs_node_idx+2) = "PG";
    disp("BS " + bs_sched{bs_node_idx});
    if bs_node_idx == N_BS_NODE
        disp("UE " + ue_sched);
    end
end

% Construct signal
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

node_bs.sdrsync();                  % Synchronize delays only for BS
node_ue.sdr_configgainctrl();
node_ue.sdrrxsetup();
node_bs.sdrrxsetup();


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Step 0: Sample Offset Calibration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if(0)
% Sample offset calibration. (FIXME - do it at the beginning)
samp_offset_array = corr_pks - corr_pks(1,1);
rx_mat_calibrated_tmp = zeros(size(rx_vec_pilot));

for ibs =1:N_BS_NODE
    curr_offset = samp_offset_array(ibs);
    
    if curr_offset < 0
        rx_mat_calibrated_tmp(ibs, 1+abs(curr_offset):end) = rx_vec_pilot(ibs, 1:end-abs(curr_offset));
    elseif  curr_offset > 0
        rx_mat_calibrated_tmp(ibs, 1:end-curr_offset) = rx_vec_pilot(ibs, 1+curr_offset:end);
    else
        rx_mat_calibrated_tmp(ibs, :) = rx_vec_pilot(ibs, :);
    end
end
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Step 1: Explicit Channel Sounding
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Construct the full time-domain OFDM waveform
preamble_all = repmat(preamble, 1, floor((N_SAMP-N_ZPAD_PRE-N_ZPAD_POST)/length(preamble)));
sounding_vec = [zeros(1, N_ZPAD_PRE) preamble_all zeros(1, N_ZPAD_POST)];
% Scale the Tx vector to +/- 1
sounding_vec_iris = TX_SCALE .* sounding_vec ./ max(abs(sounding_vec));
n_samp_sounding = length(sounding_vec_iris);
node_ue.set_num_samps(n_samp_sounding);     % In case the number of samples changes for sounding
node_bs.set_num_samps(n_samp_sounding);
numRxPilots = ue_sched.count("R");

% Configure TDD Sched and write data to BS boards
node_ue.set_tddconfig(WIRED_UE, ue_sdr_params.tdd_sched); % configure the UE: schedule etc.
node_bs.set_tddconfig(1, bs_sdr_params.tdd_sched);        % configure the BS: schedule etc.
node_bs.sdr_setupbeacon_single();                         % Burn beacon to the BS(1) RAM
node_bs.sdrtx(sounding_vec_iris);                         % Burn same pilot to all BS boards RAM

% Read sounding at the UE
max_try = 1;
good_signal = 0;
pilot_data_cell = cell(max_try);
beacon_array = zeros(N_UE_NODE, max_try);
for i = 1:max_try
    disp("TRY # " + i);
    if ~WIRED_UE
        node_ue.sdr_setcorr();                                    % activate correlator
    end
    node_ue.sdr_activate_rx();                                    % activate reading stream
    node_bs.sdrtrigger();
    [rx_vec_iris, data_len] = node_ue.uesdrrx(n_samp_sounding);  % read data
    pilot_data_cell{i} = rx_vec_iris;
    beacon_array(:, i) = node_ue.sdr_gettriggers();
    if ~WIRED_UE
        node_ue.sdr_unsetcorr();                                  % activate correlator
    end
end
if sum(beacon_array, 'all') == 0
    disp('UE(s) did not receive trigger beacon during sounding phase!');
    node_bs.sdr_close();
    node_ue.sdr_close();
    return;
end

% Process downlink (sounding) pilots
corr_pks = zeros(N_UE_NODE, numRxPilots);
pilot_start = zeros(N_UE_NODE, numRxPilots);
pilot_data = zeros(N_UE_NODE, numRxPilots, length(preamble));
for iue = 1:N_UE_NODE
    bidx = find(beacon_array(iue, :) > 0);
    if bidx
        per_ue_rx_data = pilot_data_cell{bidx(1)};
        %figure; plot(abs(per_ue_rx_data));
        for isym = 1:numRxPilots
            startIdx = (isym-1)*n_samp_sounding + 1;
            endIdx = (isym-1)*n_samp_sounding + n_samp_sounding;

            % Correlation
            per_sym_rx_data = per_ue_rx_data(startIdx:endIdx);
            lts_corr = abs(conv(conj(fliplr(lts_t)), sign(per_sym_rx_data)));
            %figure; plot(lts_corr);
            lts_peaks = find(lts_corr > 0.8*max(lts_corr));
            [LTS1, LTS2] = meshgrid(lts_peaks, lts_peaks);
            [lts_second_peak_index,y] = find(LTS2-LTS1 == length(lts_t));
            % Stop if no valid correlation peak was found
            if(isempty(lts_second_peak_index))
                fprintf('DOWNLINK PILOT COLLECTION: NO LTS Correlation Peaks Found!\n');
                return;
            end
            corr_pks(iue, isym) = lts_peaks(max(lts_second_peak_index));  % Get last peak
            pilot_start(iue, isym) = corr_pks(iue, isym) - length(preamble) + 1;

            % Check if invalid correlation peak
            if pilot_start(iue, isym) < 0
                fprintf('DOWNLINK PILOT COLLECTION: BAD LTS Correlation!\n');
                pilot_start(iue, isym) = 0;
            end

            pilot_range = pilot_start(iue, isym): pilot_start(iue, isym) + length(preamble) - 1;
            pilot_data(iue, isym, :) = per_sym_rx_data(pilot_range);
        end
    end
end

% Channel Estimation: Calculate channel estimate from average of 2 training symbols
rx_lts1 = pilot_data(:,:,-64+-FFT_OFFSET + [97:160]);
rx_lts2 = pilot_data(:,:,-FFT_OFFSET + [97:160]);
rx_lts1_f = fft(rx_lts1);
rx_lts2_f = fft(rx_lts2);
downlink_pilot_csi = zeros(N_SC, N_BS_NODE, N_UE_NODE);
for iue = 1:N_UE_NODE
    for isym = 1:numRxPilots
        lts1 = squeeze(rx_lts1_f(iue, isym,:));
        lts2 = squeeze(rx_lts2_f(iue, isym,:));
        downlink_pilot_csi(:, isym, iue) = ((lts1 + lts2) / N_LTS_SYM) .* lts_f.';
    end
end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Step 3: Downlink Transmission
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CSI Calculation and Precoding (Zeroforcing)
ifft_in_mat = zeros(N_BS_NODE, N_SC, N_OFDM_SYMS);
for isc =1:N_SC
    downlink_beam_weights = pinv(squeeze(downlink_pilot_csi(isc, :, :)));
    for isym = 1:N_OFDM_SYMS
        ifft_in_mat(:, isc, isym) = downlink_beam_weights.' * precoding_in_mat(:, isc, isym);
    end
end

% IFFT
tx_payload_mat = zeros(N_BS_NODE, N_SYM_SAMP, N_DATA_SYMS);
tx_pilot_mat = zeros(N_BS_NODE, length(preamble));
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

% Rewrite TDD Schedule
ue_sdr_params.tdd_sched = "BGPG";
bs_sdr_params.tdd_sched = "GGRG";
node_ue.set_tddconfig(WIRED_UE, ue_sdr_params.tdd_sched); % configure the UE: schedule etc.
node_bs.set_tddconfig(1, bs_sdr_params.tdd_sched);        % configure the BS: schedule etc.
node_bs.sdr_setupbeacon_single();                         % Burn beacon to the BS(1) RAM

% Write beamformed signal to all antennas
donwlink_postfix_len = N_SAMP - N_ZPAD_PRE - N_OFDM_SYMS * N_SYM_SAMP;
for i=1:N_BS_NODE
    tx_data_signal = [zeros(1, N_ZPAD_PRE) tx_payload_vec(i, :) zeros(1, donwlink_postfix_len)];
    tx_vec_iris = TX_SCALE .* tx_data_signal ./ max(abs(tx_data_signal));
    node_bs.sdrtx_single(tx_vec_iris, i);       % Burn data to the UE RAM
end

% Transmit beamformed signal from all antennas and receive at UEs
max_try = 10;
good_signal = 0;
dl_data_cell = cell(max_try);
beacon_array = zeros(N_UE_NODE, max_try);
for i=1:max_try
    if ~WIRED_UE
        node_ue.sdr_setcorr();              % activate correlator
    end

    node_ue.sdr_activate_rx();   % activate reading stream

    node_bs.sdrtrigger();
    [rx_vec_downlink, ~] = node_ue.uesdrrx(N_SAMP); % read data
    dl_data_cell{i} = rx_vec_downlink;
    beacon_array(:, i) = node_ue.sdr_gettriggers();
    if ~WIRED_UE
        node_ue.sdr_unsetcorr();              % activate correlator
    end
end
if sum(beacon_array, 'all') == 0
    disp('UE(s) did not receive trigger beacon during sounding phase!');
    node_bs.sdr_close();
    node_ue.sdr_close();
    return;
end

% Process downlink data
corr_pks = zeros(1,N_UE_NODE);
data_start = zeros(1,N_UE_NODE);
preamble_start = zeros(1,N_UE_NODE);
rx_vec_downlink = zeros(N_UE_NODE, N_SAMP);
for iue = 1:N_UE_NODE
    bidx = find(beacon_array(iue, :) > 0);
    if bidx
        per_ue_rx_data = dl_data_cell{bidx(1)};
        %figure; plot(abs(per_ue_rx_data));
        lts_corr = abs(conv(conj(fliplr(lts_t)), sign(per_ue_rx_data)));
        %figure; plot(lts_corr);
        lts_peaks = find(lts_corr > 0.8*max(lts_corr));
        [LTS1, LTS2] = meshgrid(lts_peaks, lts_peaks);
        [lts_second_peak_index,y] = find(LTS2-LTS1 == length(lts_t));
        % Stop if no valid correlation peak was found
        if(isempty(lts_second_peak_index))
            fprintf('DOWNLINK DATA COLLECTION: NO LTS Correlation Peaks Found!\n');
            return;
        end
        corr_pks(iue) = lts_peaks(max(lts_second_peak_index));  % Get last peak
        data_start(iue) = corr_pks(iue) + 1;
        preamble_start(iue) = corr_pks(iue) - length(preamble) + 1;

        % Check if invalid correlation peak
        if pilot_start(iue, isym) < 0
            fprintf('DOWNLINK DATA COLLECTION: BAD LTS Correlation!\n');
            pilot_start(iue, isym) = 0;
        end

        pilot_range = pilot_start(iue, isym): pilot_start(iue, isym) + length(preamble) - 1;
        rx_vec_downlink(iue, :) = per_sym_rx_data(pilot_range);

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
%rx_dl_pilot_vec = rx_vec_downlink(1, dl_pilot_start - timing_offset: dl_pilot_start + 2*length(lts) - 1 - timing_offset);
rx_dl_pilot_vec = rx_vec_downlink(1, dl_pilot_start+ 2*length(lts): dl_pilot_start + 2*length(lts)  + 2*length(lts) - 1);
rx_lts1 = rx_dl_pilot_vec(-64  + -FFT_OFFSET + [97:160]);
rx_lts2 = rx_dl_pilot_vec(-FFT_OFFSET + [97:160]);
rx_lts1_f = fft(rx_lts1);
rx_lts2_f = fft(rx_lts2);
rx_H_est = (lts_f).* (rx_lts1_f + rx_lts2_f) / 2;

% Data
N_RX_DATA_SYMS = min(N_DATA_SYMS, floor((N_SAMP - dl_data_start)/N_SYM_SAMP));
rx_dl_data_vec = zeros(N_RX_DATA_SYMS * N_SYM_SAMP);
end_idx = min(4096, dl_data_start + N_RX_DATA_SYMS * N_SYM_SAMP - 1);
%rx_dl_data_vec = rx_vec_downlink(1, dl_data_start - timing_offset: end_idx - timing_offset);
rx_dl_data_vec = rx_vec_downlink(1, dl_data_start: end_idx);
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
N_DATA_SC_RX = N_RX_DATA_OFDM_SYMS * length(SC_IND_DATA);

if N_DATA_SC_RX ~= N_DATA_SC
    disp('Missing Data. Exit now!');
    return;
end
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
