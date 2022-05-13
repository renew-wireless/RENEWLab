%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%	Author(s): C. Nicolas Barati nicobarati@rice.edu 
%		Rahman Doost-Mohamamdy: doost@rice.edu
%
%
% Single-shot transmission from N_UE clients to N_BS_NODE base station
% radios (UE stands for User Equipment). We define two modes:
% OTA (Over-the-air) and SIM_MODE (simulation).
% In simulation mode we simply use a Rayleigh channel whereas the OTA mode
% relies on the Iris hardware for transmission and reception.
% In both cases the clients transmit an OFDM signal that resembles a
% typical 802.11 WLAN waveform. If the transmission is OTA, then the user
% specifies a schedule that tells all clients when to transmit their frame
% The base station initiates the schedule by sending a beacon signal that
% synchronizes clients. After that, all clients will transmit
% simultaneously. We implement a frame structure that allows the base
% station to capture clean (non-overlaping) training sequences for
% equalization and demultiplexing of the concurrent data streams.
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
    py.print() % weird bug where py isn't loaded in an external script
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
WRITE_PNG_FILES         = 0;                % Enable writing plots to PNG
SIM_MODE                = 0;
DEBUG                   = 0;
PLOT                    = 0;

if SIM_MODE
    TX_SCALE                = 1;            % Scale for Tx waveform ([0:1])
    N_BS_NODE               = 2;            % Number of SDRs (Matlab scripts only using antenna A)
    N_UE                    = 1;
    SNR_db                  = 10;
    N_FRM                   = 1;
    bs_ids                  = ones(1, N_BS_NODE);
    ue_ids                  = ones(1, N_UE);

else 
    %Iris params:
    TX_SCALE                = 0.8;          % Scale for Tx waveform ([0:1])
    ANT_BS                  = 'AB';         % Options: {A, B, AB}
    ANT_UE                  = 'A';          % Currently, only support single antenna
    USE_HUB                 = 1;
    TX_FRQ                  = 3.58e9;
    RX_FRQ                  = TX_FRQ;
    TX_GN                   = 80;
    RX_GN                   = 70;
    SMPL_RT                 = 5e6;
    N_FRM                   = 1;
    bs_ids                  = string.empty();
    ue_ids                  = string.empty();
    ue_scheds               = string.empty();

    if USE_HUB
        % Using chains of different size requires some internal
        % calibration on the BS. This functionality will be added later.
        % For now, we use only the 4-node chains:
        bs_ids = ["RF3E000356","RF3E000620"];%,"RF3E000609","RF3E000604","RF3E000612","RF3E000640","RF3E000551"];
        hub_id = ["FH4B000019"];
    else
        bs_ids = ["RF3E000246","RF3E000490","RF3E000749","RF3E000697","RF3E000724","RF3E000740","RF3E000532"];
        hub_id = [];
    end
    ue_ids= ["RF3E000392"];%, "RF3D000016"];

    N_BS_NODE               = length(bs_ids);                   % Number of nodes at the BS
    N_BS_ANT                = length(bs_ids) * length(ANT_BS);  % Number of antennas at the BS
    N_UE                    = length(ue_ids) * length(ANT_UE);  % Number of UE nodes (assume it is the same as number of antennas)
end

% MIMO params
MIMO_ALG                = 'ZF';      % MIMO ALGORITHM: ZF or Conjugate 

% Waveform params
N_OFDM_SYM              = 40;          % Number of OFDM symbols for burst, it needs to be less than 47
N_LTS_SYM               = 2;           % Number of preamble symbols
N_DATA_SYM              = N_OFDM_SYM - N_LTS_SYM;
MOD_ORDER               = 16;          % Modulation order (2/4/16/64 = BSPK/QPSK/16-QAM/64-QAM)

% OFDM params
SC_IND_PILOTS           = [8 22 44 58];                           % Pilot subcarrier indices
SC_IND_DATA             = [2:7 9:21 23:27 39:43 45:57 59:64];     % Data subcarrier indices
SC_IND_DATA_PILOT       = [2:27 39:64]';
N_SC                    = 64;                                     % Number of subcarriers
CP_LEN                  = 16;                                     % Cyclic prefix length
CP_LEN_LTS              = 32;
N_DATA_SC               = N_DATA_SYM * length(SC_IND_DATA);
N_SYM_SAMP              = N_SC + CP_LEN;                          % Number of samples that will go over the air
N_ZPAD_PRE              = 100;                                    % Zero-padding prefix for Iris
N_ZPAD_POST             = nan;                                    % Zero-padding postfix for Iris
MAX_NUM_SAMPS           = 4096;                                   % DO NOT MODIFY: Max number of samples in FPGA buffer.

% Rx processing params
FFT_OFFSET                    = 16;          % Number of CP samples to use in FFT (on average)
DO_APPLY_PHASE_ERR_CORRECTION = 1;           % Enable Residual CFO estimation/correction

%% Define the preamble
% LTS for fine CFO and channel estimation
lts_f = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 ...
    1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1].';
lts_t = ifft(lts_f, N_SC); %time domain
preamble_common = [lts_t(33:64); repmat(lts_t,N_LTS_SYM,1)];



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           BUILD SIGNALS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% SOUNDING: Build signals for explicit downlink training
post_samps_sound = zeros(MAX_NUM_SAMPS-N_ZPAD_PRE-length(preamble_common), 1);
tx_vec_train = [zeros(N_ZPAD_PRE, 1); preamble_common; post_samps_sound];


%%%%% DATA: For downlink beamformed signals
%% Generate a payload of random integers
tx_data = randi(MOD_ORDER, N_UE, N_DATA_SC) - 1;
tx_syms = mod_sym(tx_data, MOD_ORDER);

% Reshape the symbol vector to a matrix with one column per OFDM symbol
tx_syms_mat = reshape(tx_syms, N_UE, length(SC_IND_DATA), N_DATA_SYM);

% Define the pilot tone values as BPSK symbols
pilots = [1 1 -1 1].';

% Repeat the pilots across all OFDM symbols
pilots_mat = repmat(pilots, N_UE, N_DATA_SYM);
pilots_mat = reshape(pilots_mat, length(pilots), N_UE, []);
pilots_mat = permute(pilots_mat, [2,1,3]);

% Construct the precoding input matrix
precoding_in_mat = zeros(N_UE, N_SC, N_OFDM_SYM);

% Insert the data and pilot values; other subcarriers will remain at 0
for i = 1:N_LTS_SYM
    precoding_in_mat(:, :, i) = repmat(lts_f,1,N_UE).';
end
precoding_in_mat(:, SC_IND_DATA, N_LTS_SYM + 1:end)   = tx_syms_mat;
precoding_in_mat(:, SC_IND_PILOTS, N_LTS_SYM + 1:end) = pilots_mat;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                       SOUNDING (DL TRAINING)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if SIM_MODE
    disp("Running: RAYLEIGH FADING CHANNEL SIMULATION MODE");

    % TODO: this part has not been tested
    rx_vec_iris = zeros(N_UE, N_BS_NODE, length(tx_vec_train));
    h_train_all = zeros(N_UE, N_BS_NODE, length(tx_vec_train));

    for bsidx = 1:N_BS_ANT
        % Rayleigh Fading Channel
        tx_var = mean(mean(abs(tx_vec_train).^2 )) * (64/48);
        nvar =  tx_var / 10^(0.1*SNR_db); % noise variance per data sample
        % noise vector
        W_ul = sqrt(nvar/2) * (randn(1, length(tx_vec_train)) + ...
            1i*randn(1, length(tx_vec_train)) );
        hvar = 1;
        if N_UE == 1
            H_ul_tmp = sqrt(hvar/2).*randn(1, length(tx_vec_train)) + 1i*randn(1, length(tx_vec_train));
            H_ul_tmp = sqrt(abs(H_ul_tmp).^2);
            H_ul_tmp = smoothdata(H_ul_tmp, 2, 'movmean',15);
            y0 = H_ul_tmp.*tx_vec_train.';
            h_train_all(1, bsidx, :) = H_ul_tmp;

        else
            H_ul_tmp = sqrt(hvar/2).*randn(1, N_UE) + 1i*randn(1, N_UE);
            y0 = H_ul_tmp*tx_vec_train.';
        end
        
        y = y0 + W_ul;
        rx_vec_iris(1, bsidx, :) = y;   % Modify for UE > 1
        numGoodFrames = 1;
    end

else

    fprintf('Running: HARDWARE MODE \n');
    fprintf('=============================== \n');
    fprintf('Initiate Sounding Process \n');
    % Iris nodes' parameters
    sdr_params = struct(...
        'bs_id', bs_ids, ...
        'ue_id', ue_ids, ...
        'bs_ant', ANT_BS, ...
        'ue_ant', ANT_UE, ...
        'txfreq', TX_FRQ, ...
        'rxfreq', RX_FRQ, ...
        'txgain', TX_GN, ...
        'rxgain', RX_GN, ...
        'sample_rate', SMPL_RT);

    mimo_handle = mimo_driver(sdr_params);
    [rx_vec_iris, numGoodFrames, numRxSyms] = mimo_handle.mimo_txrx_dl_sound(tx_vec_train, N_FRM, N_ZPAD_PRE);
    if isempty(rx_vec_iris)
        error("Driver returned empty array. No good data received by base station");
    end
    assert(size(rx_vec_iris,3) == N_BS_ANT) 
end

fprintf('=============================== \n');
fprintf('Channel Estimation and Beamweight Calculation \n');

preamble_pk = nan(N_BS_ANT, N_UE);
H = zeros(N_UE, N_BS_ANT, N_SC);
for iue = 1:N_UE
    for ibs = 1:N_BS_ANT

        % Data shape: (# good frames, # UEs, # numRxSyms, # number samps)
        curr_vec = squeeze(rx_vec_iris(1, iue, ibs, :));
        lts_corr = abs(conv(conj(fliplr(lts_t.')), sign(curr_vec.')));

        if DEBUG
            figure; subplot(2,1,1); plot(abs(curr_vec)); subplot(2,1,2); plot(lts_corr);
        end

        lts_peaks = find(lts_corr > 0.8*max(lts_corr));
        [LTS1, LTS2] = meshgrid(lts_peaks,lts_peaks);
        [lts_second_peak_index2,y] = find(LTS2-LTS1 == length(lts_t));

        if(isempty(lts_second_peak_index2))
            fprintf('SOUNDING: NO correlation peak from BS antenna %d at UE %d. Exit now! \n', ibs, iue);
            return;
        else
            if length(lts_second_peak_index2) > 1
                preamble_pk(ibs, iue) = lts_peaks(lts_second_peak_index2(2));
            else
                preamble_pk(ibs, iue) = lts_peaks(lts_second_peak_index2(1));
            end

            % Check if valid...
            pk_tmp = preamble_pk(ibs, iue);
            lts_ind = pk_tmp - length(preamble_common);
            if lts_ind <= 0
                fprintf('INVALID correlation peak from BS antenna %d at UE %d. Exit now! \n', ibs, iue);
                return;
            else
                fprintf('LTS Index: %d \n', lts_ind);
            end

            % Re-extract LTS for channel estimate
            rx_lts = curr_vec(lts_ind : lts_ind+159);
            rx_lts1 = rx_lts(-64+-FFT_OFFSET + [97:160]);
            rx_lts2 = rx_lts(-FFT_OFFSET + [97:160]);

            % Received LTSs
            rx_lts1_f = fft(rx_lts1);
            rx_lts2_f = fft(rx_lts2);

            % Calculate channel estimate from average of 2 training symbols: 
            rx_H_est = mean([rx_lts1_f./lts_f   rx_lts2_f./ lts_f], 2);
            H(iue, ibs, :) = rx_H_est;
        end
    end
end

% Beamweight calculation
W = zeros(N_BS_ANT, N_UE, N_SC);
if strcmp(MIMO_ALG, 'ZF')
    for isc =1:N_SC
        currH = squeeze(H(:, :, isc));
        currH(isnan(currH)) = 0;   % Remove nan 
        currH(isinf(currH)) = 0;   % Remove inf... I believe pinv() in R2021b no longer breaks with non-finite vals.
        W(:, :, isc) = pinv(currH);
    end
else
    fprintf("Only Zero-Forcing (ZF) is Currently Supported");
    return;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                       DOWNLINK BEAMFORMING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% No need to send explicit feedback to base station, simply use the computed
% weights
fprintf('=============================== \n');
fprintf('Downlink Beamforming \n');


if SIM_MODE
    % TODO

else

    % Apply precoding weights to data (in freq. domain)
    ifft_in_mat = zeros(N_BS_ANT, N_SC, N_OFDM_SYM);
    for isc = 1:N_SC
        for isym = 1:N_OFDM_SYM
            ifft_in_mat(:, isc, isym) = W(:, :, isc) * precoding_in_mat(:, isc, isym);
        end
    end

    % IFFT
    tx_payload_mat = zeros(N_BS_ANT, N_SYM_SAMP, N_DATA_SYM);
    tx_pilot_mat = zeros(N_BS_ANT, length(lts_t)*2.5);

    for ibs = 1:N_BS_ANT
        pilot1 = squeeze(ifft(ifft_in_mat(ibs, :, 1)));
        pilot2 = squeeze(ifft(ifft_in_mat(ibs, :, 2)));
        tx_pilot_mat(ibs, :) = [pilot2(33:64) pilot1 pilot2];
        
        for isym = N_LTS_SYM+1:N_OFDM_SYM
            tx_sym = squeeze(ifft(ifft_in_mat(ibs, :, isym)));
            tx_payload_mat(ibs, :, isym - N_LTS_SYM) = [tx_sym(end - CP_LEN + 1: end) tx_sym].';
        end
    end

    % Reshape to a vector
    tx_payload_vec = reshape(tx_payload_mat, N_BS_ANT, numel(tx_payload_mat(1, :, :)));
    tx_payload_vec = [tx_pilot_mat tx_payload_vec];

    % NOTE: Add preamble to one antenna for sync (workaround to offset from beamformed preamble)
    %syncSeq = zeros(size(tx_payload_vec,1), length(lts_lcp));
    %syncSeq(1, :) = lts_lcp;
    %tx_payload_vec = [syncSeq tx_payload_vec];

    [rx_vec_iris_tmp, numGoodFrames, ~] = mimo_handle.mimo_txrx_downlink(tx_payload_vec, N_FRM, N_ZPAD_PRE);
    mimo_handle.mimo_close();

    if isempty(rx_vec_iris)
        error("Driver returned empty array. No good data received by UE");
        exit(0);
    end

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                        PROCESS DOWNLINK DATA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
return;

igf = numGoodFrames;
preamble_pk = zeros(1, N_UE);
for iue = 1:N_UE

    %%%%% Find Preamble
    curr_vec = squeeze(rx_vec_iris_tmp(igf, iue, 1, :)); % Dimensions: (nGoodFrames,nUE,numRxSyms,numSamps)
    lts_corr = abs(conv(conj(fliplr(lts_t)), sign(curr_vec)));
    lts_peaks = find(lts_corr > 0.8*max(lts_corr));
    [LTS1, LTS2] = meshgrid(lts_peaks,lts_peaks);
    [lts_second_peak_index,y] = find(LTS2-LTS1 == length(lts_t));

    if DEBUG
        figure; subplot(2,1,1); plot(abs(curr_vec)); subplot(2,1,2); plot(lts_corr);
    end

    % Stop if no valid correlation peak was found
    if(isempty(lts_second_peak_index2))
        fprintf('SOUNDING: NO correlation peak at UE %d. Exit now! \n', iue);
        return;
    end

    if length(lts_second_peak_index2) > 1
        preamble_pk(iue) = lts_peaks(lts_second_peak_index2(2));
    else
        preamble_pk(iue) = lts_peaks(lts_second_peak_index2(1));
    end

    % Check if valid...
    pk_tmp = preamble_pk(iue); 
    lts_ind = pk_tmp - length(preamble_common);

    if lts_ind <= 0
        fprintf('INVALID correlation peak at UE %d. Exit now! \n', iue);
        return;
    else
        fprintf('LTS Index: %d \n', lts_ind);
    end

    % Re-extract LTS for channel estimate
    rx_lts = curr_vec(lts_ind : lts_ind+159);
    rx_lts1 = rx_lts(-64+-FFT_OFFSET + [97:160]);
    rx_lts2 = rx_lts(-FFT_OFFSET + [97:160]);
    rx_lts1_f = fft(rx_lts1);
    rx_lts2_f = fft(rx_lts2);

    % Calculate channel estimate from average of 2 training symbols: 
    rx_H_est = mean([rx_lts1_f./lts_f   rx_lts2_f./ lts_f], 2);
    H(iue, ibs, :) = rx_H_est;




    %%%%% Retrieve data and apply corrections
    N_RX_DATA_SYMS = min(N_DATA_SYMS, floor((N_SAMP - dl_data_start)/N_SYM_SAMP));
    rx_dl_data_vec = zeros(N_RX_DATA_SYMS * N_SYM_SAMP);
    end_idx = min(4096, dl_data_start + N_RX_DATA_SYMS * N_SYM_SAMP - 1);
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


    %%%%% Demodulate
    N_DATA_SC_RX = N_RX_DATA_OFDM_SYMS * length(SC_IND_DATA);

    if N_DATA_SC_RX ~= N_DATA_SC
        disp('Missing Data. Exit now!');
        return;
    end
    rx_syms = reshape(payload_dl_syms_mat, 1, N_DATA_SC);
    rx_data = demod_sym(rx_syms ,MOD_ORDER);

    
    %%%%% Calculate EVM & SNR
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

end


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