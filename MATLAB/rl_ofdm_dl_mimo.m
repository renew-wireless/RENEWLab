%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%	Author(s): C. Nicolas Barati nicobarati@rice.edu
%		       Rahman Doost-Mohamamdy: doost@rice.edu
%
% - Supports Downlink MISO transmissions from multiple antennas in the
%   mMIMO base station to one single-antenna user.
% - Relies on explicit feedback (i.e., downlink pilots for beamforming
%   weight computation)
% - Supports transmission from one or two antennas per Iris board in the
%   base station (each Iris board has two antennas)
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


%[version, executable, isloaded] = pyversion;
pe = pyenv;
%disp(pe);
if pe.Status == 'NotLoaded'
    pyversion /usr/bin/python3
    py.print() %weird bug where py isn't loaded in an external script
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global DEBUG;
global APPLY_CFO_CORRECTION;

DEBUG                   = 0;
WRITE_PNG_FILES         = 0;                % Enable writing plots to PNG
SIM_MODE                = 0;
APPLY_CFO_CORRECTION    = 1;

PILOT_PLOT              = 0;
CONST_PLOT              = 0;
CHANNEL_PLOT            = 0;
DOWNLINK_PLOT           = 0;
EVM_SNR_PLOT            = 0;

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
    TX_SCALE                = 1;            % Scale for Tx waveform ([0:1])
    ANT_BS                  = 'A';         % Options: {A, AB}
    ANT_UE                  = 'A';          % Currently, only support single antenna UE, i.e., A
    USE_HUB                 = 1;
    TX_FRQ                  = 3.5475e9;
    RX_FRQ                  = TX_FRQ;
    TX_GN                   = 81;
    TX_GN_BF                = 81;           % BS gain during DL BF transmission
    TX_GN_UE                = [81, 81];
    RX_GN                   = 65;
    SMPL_RT                 = 5e6;
    N_FRM                   = 1;            % Not tested with N_FRM > 1
    frm_idx                 = 1;            % Hardcoded to select frame_index == 1
    bs_ids                  = string.empty();
    ue_ids                  = string.empty();
    ue_scheds               = string.empty();
    TX_ADVANCE              = 400;          % !!!! IMPORTANT: DO NOT MODIFY - POWDER default is 400, RENEW(Rice) default is 235!!!!

    if USE_HUB
        % Using chains of different size requires some internal
        % calibration on the BS. This functionality will be added later.
        % For now, we use only the 4-node chains:
        chain1A = ["RF3E000731","RF3E000747","RF3E000734"];                    % Chain1A
        chain1B = ["RF3E000654"];%,"RF3E000458"];%,"RF3E000463","RF3E000424"]; % Chain1B
        chain2A = ["RF3E000053","RF3E000192","RF3E000117"];                    % Chain2A
        chain2B = ["RF3E000257","RF3E000430","RF3E000311","RF3E000565"];       % Chain2B
        chain3A = ["RF3E000686","RF3E000574","RF3E000595","RF3E000585"];       % Chain3
        chain4A = ["RF3E000722","RF3E000494","RF3E000592","RF3E000333"];       % Chain4
        chain5A = ["RF3E000748","RF3E000492"];                                 % Chain5A
        chain5B = ["RF3E000708","RF3E000437","RF3E000090"];                    % Chain5B
        bs_ids = [chain1B, chain1A, chain2B];
        bs_ids = ["RF3E000356","RF3E000546","RF3E000620","RF3E000609"];%,"RF3E000604","RF3E000612","RF3E000640","RF3E000551"];
                    %"RF3E000208","RF3E000636","RF3E000632","RF3E000568","RF3E000558","RF3E000633","RF3E000566","RF3E000635"];
                    %,"RF3E000136","RF3E000213","RF3E000142", ...
        hub_id = ["FH4B000019"];%["FH4B000003"];
    else
        bs_ids = ["RF3E000654","RF3E000458","RF3E000463","RF3E000424"];
        hub_id = [];
    end
    ue_ids = ["RF3E000392"];%["RF3E000706"];
    ref_ids= [];

    N_BS_NODE               = length(bs_ids);                   % Number of nodes at the BS
    N_BS_ANT                = length(bs_ids) * length(ANT_BS);  % Number of antennas at the BS
    N_UE                    = length(ue_ids) * length(ANT_UE);  % Number of UE nodes (assume it is the same as number of antennas)
end

% MIMO params
MIMO_ALG                = 'ZF';      % MIMO ALGORITHM: ZF or Conjugate 

% Waveform params
N_OFDM_SYM              = 24;          % Number of OFDM symbols for burst, it needs to be less than 47
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
FFT_OFFSET                    = 0;          % Number of CP samples to use in FFT (on average)
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
    fprintf('======== Initialization ======= \n');
    fprintf('=============================== \n');
    % Iris nodes' parameters
    sdr_params = struct(...
        'bs_id', bs_ids, ...
        'ue_id', ue_ids, ...
        'ref_id', ref_ids, ...
        'hub_id', hub_id,...
        'bs_ant', ANT_BS, ...
        'ue_ant', ANT_UE, ...
        'txfreq', TX_FRQ, ...
        'rxfreq', RX_FRQ, ...
        'txgain', TX_GN, ...
        'tx_gain_ue', TX_GN_UE, ...
        'rxgain', RX_GN, ...
        'sample_rate', SMPL_RT, ...
        'trig_offset', TX_ADVANCE);

    mimo_handle = mimo_driver(sdr_params);

    fprintf(' =========================== \n ======== Sounding ========= \n =========================== \n');
    % Scale the Tx vector to +/- 1
    tx_vec_train = TX_SCALE .* tx_vec_train ./ max(abs(tx_vec_train));
    tx_mat_train = repmat(tx_vec_train.', N_BS_ANT,1);
    [rx_vec_iris_sound, numGoodFrames, numRxSyms] = mimo_handle.mimo_txrx(tx_mat_train, N_FRM, N_ZPAD_PRE, 'dl-sounding', '[]', '[]');
    rx_vec_iris_sound_tmp = rx_vec_iris_sound;
    if isempty(rx_vec_iris_sound)
        mimo_handle.mimo_close();
        error("Driver returned empty array. No good data received by base station");
    end
    assert(size(rx_vec_iris_sound,3) == N_BS_ANT);
end


fprintf('=============================== \n');
fprintf('Channel Estimation and Beamweight Calculation \n');
% Channel estimation
clear peaks;
cfo_est_mat = nan(N_BS_ANT, N_UE);
[H, H_tmp, peaks, cfo_est_mat, err_flag] = channel_estimation_fun(rx_vec_iris_sound, N_BS_ANT, N_UE, N_SC, lts_t, lts_f, preamble_common, FFT_OFFSET, 'sounding', frm_idx, cfo_est_mat);
if err_flag
    mimo_handle.mimo_close();
    error();
end

% Beamweight calculation
W = zeros(N_BS_ANT, N_UE, N_SC);
Wtmp = zeros(N_BS_ANT, N_UE, N_SC);
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
fprintf(' =========================== \n ======== Downlink BF ========= \n =========================== \n');

% Update TX gain
is_bs = 1;   % Is base station node
param = 1;   % txgain == 1 [TODO, change to string, txgain=1, rxgain=2]
mimo_handle.mimo_update_sdr_params(param, TX_GN_BF, is_bs);

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
    ifft_in_mat(isnan(ifft_in_mat)) = 0;

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

    % Reshape to a vector and append zeros to fill out remaining of buffer (not necessary)
    % Total frame length 4096 = N_SAMPS
    tx_payload_vec = reshape(tx_payload_mat, N_BS_ANT, numel(tx_payload_mat(1, :, :)));
    frame_length = length(tx_payload_mat(1,:)) + length(tx_pilot_mat(1,:));
    N_ZPAD_POST = MAX_NUM_SAMPS - frame_length - N_ZPAD_PRE;
    tx_payload = [zeros(N_BS_ANT, N_ZPAD_PRE) tx_pilot_mat tx_payload_vec zeros(N_BS_ANT, N_ZPAD_POST)];
    % Scale the Tx vector to +/- 1
    tx_payload = TX_SCALE .* tx_payload ./ max(max(abs(tx_payload)));

    N_DATA_SAMP = N_SYM_SAMP * N_DATA_SYM;
    N_SAMPS = N_ZPAD_PRE + length(tx_pilot_mat(1,:)) + (N_SYM_SAMP * N_DATA_SYM) + N_ZPAD_POST;
    assert(N_SAMPS == MAX_NUM_SAMPS);

    [tx_payload_cal] = time_offset_cal(peaks, squeeze(tx_payload), N_BS_ANT, lts_t);
    [rx_vec_iris_tmp, numGoodFrames, ~] = mimo_handle.mimo_txrx(tx_payload_cal, N_FRM, N_ZPAD_PRE, 'downlink', '[]', '[]');
    mimo_handle.mimo_close();

    if isempty(rx_vec_iris_tmp)
        error("Driver returned empty array. No good data received by UE");
        exit(0);
    end

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                        PROCESS DOWNLINK DATA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Find Preambles (downlink beamforming mode: 'dl-bf')
% Replace number of BS with number of streams for channel estimation in DL BF transmission
N_STREAMS = 1;
cfo_est_vec = mean(cfo_est_mat,1); % Average across base station antennas, use one single CFO est. value per UE.
[H, ~, preamble_pk, ~, err_flag] = channel_estimation_fun(rx_vec_iris_tmp, N_STREAMS, N_UE, N_SC, lts_t, lts_f, preamble_common, FFT_OFFSET, 'dl-bf', frm_idx, cfo_est_vec);
if err_flag
    mimo_handle.mimo_close();
    error();
end
rx_H_est = squeeze(H);

for iue = 1:N_UE
    peak = preamble_pk(1, iue);
    dl_data_start = peak + 1;

    curr_vec = squeeze(rx_vec_iris_tmp(frm_idx, iue, 1, :));
    %%%%% Retrieve data and apply corrections
    %data_samples = dl_data_start + N_DATA_SAMP;
    rx_vec_downlink = curr_vec;
    N_RX_DATA_SYMS = min(N_DATA_SYM, floor((length(curr_vec) - dl_data_start) / N_SYM_SAMP));
    end_idx = min(N_SAMPS, dl_data_start + N_RX_DATA_SYMS * N_SYM_SAMP - 1);
    rx_dl_data_vec = rx_vec_downlink(dl_data_start: end_idx);
    rx_dl_data_mat = reshape(rx_dl_data_vec, N_SYM_SAMP, N_RX_DATA_SYMS);

    if(CP_LEN > 0)
        rx_dl_data_mat = rx_dl_data_mat(CP_LEN+1-FFT_OFFSET:end-FFT_OFFSET, :);
    end
    rx_dl_f_mat = fft(rx_dl_data_mat, N_SC, 1);
    N_RX_DATA_OFDM_SYMS = N_RX_DATA_SYMS;

    dl_syms_eq_mat = zeros(N_SC, N_RX_DATA_OFDM_SYMS);
    for i=1:N_RX_DATA_OFDM_SYMS
        dl_syms_eq_mat(:,i) = squeeze(rx_dl_f_mat(:,i))./rx_H_est;
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
    evm_mat = abs(payload_dl_syms_mat - squeeze(tx_syms_mat(iue, :, :))).^2;
    aevms = mean(evm_mat(:)); % needs to be a scalar
    snr = 10*log10(1./aevms); % calculate in dB scale.

    sym_errs = sum(tx_data ~= rx_data);
    bit_errs = length(find(dec2bin(bitxor(tx_data, rx_data),8) == '1'));
    rx_evm   = sqrt(sum((real(rx_syms) - real(tx_syms)).^2 + (imag(rx_syms) - imag(tx_syms)).^2)/(length(SC_IND_DATA) * N_DATA_SYM));

    fprintf('\nResults:\n');
    fprintf('Num Bytes:  \t  %d\n', N_DATA_SC * log2(MOD_ORDER) / 8);
    fprintf('Sym Errors:  \t %d (of %d total symbols)\n', sym_errs, N_DATA_SC);
    fprintf('Bit Errors: \t %d (of %d total bits)\n', bit_errs, N_DATA_SC * log2(MOD_ORDER));
    fprintf('EVM: \t %f%%, SNR: %f \n', 100*aevms, snr);

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                               PLOTTER
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for iue = 1:N_UE
    cf = 0 + 10^iue;
    fst_clr = [0, 0.4470, 0.7410];
    sec_clr = [0.8500, 0.3250, 0.0980];

    if PILOT_PLOT
        cf = cf + 1;
        figure(cf); clf;
        numRows = 5;
        numCols = 5;
        for ibs = 1:N_BS_ANT
            subplot(numRows,numCols,ibs);
            plot(abs(squeeze(rx_vec_iris_sound(1, iue, ibs, :))));
        end
    end


    if CHANNEL_PLOT
        cf = cf + 1;
        figure(cf); clf;
        x = (20/N_SC) * (-(N_SC/2):(N_SC/2 - 1));
        rx_H_est_plot = repmat(complex(NaN,NaN),1,length(rx_H_est));
        rx_H_est_plot(SC_IND_DATA) = rx_H_est(SC_IND_DATA);
        rx_H_est_plot(SC_IND_PILOTS) = rx_H_est(SC_IND_PILOTS);
        bar(x, fftshift(abs(rx_H_est_plot)),1,'LineWidth', 1);
        axis([min(x) max(x) 0 1.1*max(abs(rx_H_est_plot))])
        grid on;
        title('Channel Estimates (Magnitude)')
        xlabel('Baseband Frequency (MHz)')
    end

    if CONST_PLOT
        cf = cf + 1;
        figure(cf); clf;

        plot(payload_dl_syms_mat(:),'o','MarkerSize',2, 'color', sec_clr);
        axis square; axis(1.5*[-1 1 -1 1]);
        xlabel('Inphase')
        ylabel('Quadrature')
        grid on;
        hold on;

        plot(squeeze(tx_syms_mat(1,:,:)),'*', 'MarkerSize',16, 'LineWidth',2, 'color', fst_clr);
        title('Tx and Rx Constellations')
        legend('Rx','Tx','Location','EastOutside');
    end

    % Downlink Rx Vector and Constellation Plots
    if DOWNLINK_PLOT
        cf = cf + 1;
        figure(cf);clf;

        subplot(2,1,1);
        plot(real(squeeze(rx_vec_iris_tmp(1,iue,1,:))));
        xline(dl_data_start,'--r')
        axis([0 N_SAMPS -1 1])
        grid on;
        title('Downlink Data - Received Data (Real)');

        subplot(2,1,2);
        plot(lts_corr);
        axis([0 N_SAMPS -1 6])
        grid on;
        title('Downlink Data - LTS Correlation');
    end

    if EVM_SNR_PLOT
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
        imagesc(1:N_DATA_SYM, (SC_IND_DATA - N_SC/2), 100*fftshift(evm_mat,1))

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
end


function [H, rx_H_est, preamble_pk, cfo_est_mat, err_flag] = channel_estimation_fun(data_vec, N_BS_ANT, N_UE, N_SC, lts_t, lts_f, preamble_common, FFT_OFFSET, mode, frm_idx, cfo_est_mat)
    global DEBUG;
    H = [];
    rx_H_est = [];
    err_flag = 0;
    fprintf("==== Channel Estimation Stage: %s ==== \n", mode);
    % We expect as many pilots as there are number of BS antennas
    preamble_pk = nan(N_BS_ANT, N_UE);
    H = zeros(N_UE, N_BS_ANT, N_SC);
    for iue = 1:N_UE
        for ibs = 1:N_BS_ANT

            if strcmp(mode, 'sounding')
                % Data shape: (# good frames, # UEs, numRxSyms==N_BS_ANT, n_samps)
                curr_vec = squeeze(data_vec(frm_idx, iue, ibs, :));
                cfo_est_apply = nan;  % dummy
            elseif strcmp(mode, 'calibration')
                assert(N_UE == 1);
                % Data shape: (# good frames, n_bs_antenna, numRxSyms==1, n_samps)
                curr_vec = squeeze(data_vec(frm_idx, ibs, 1, :));
                cfo_est_apply = nan;  % dummy
            elseif strcmp(mode, 'dl-bf')
                % Data shape: (# good frames, # UEs, numRxSyms==1, numSamps)
                curr_vec = squeeze(data_vec(frm_idx, iue, 1, :));
                cfo_est_apply = cfo_est_mat(iue); % This is actually a vector during dl-bf
                %curr_vec = curr_vec.';
            end

            lts_corr = abs(conv(conj(fliplr(lts_t.')), sign(curr_vec.')));
            lts_peaks = find(lts_corr > 0.8*max(lts_corr));
            [LTS1, LTS2] = meshgrid(lts_peaks,lts_peaks);
            [lts_second_peak_index,y] = find(LTS2-LTS1 == length(lts_t));

            if 1
                figure; subplot(2,1,1); plot(abs(curr_vec)); subplot(2,1,2); plot(lts_corr); title(sprintf('%s UE %d, BS %d',mode,iue,ibs));
            end

            % Stop if no valid correlation peak was found
            if(isempty(lts_second_peak_index))
                fprintf('%s: NO correlation peak from BS antenna %d at UE %d. Exit now! \n', mode, ibs, iue);
                err_flag = 1;
                return;
            else
                if length(lts_second_peak_index) > 1
                    preamble_pk(ibs, iue) = lts_peaks(lts_second_peak_index(2));
                else
                    preamble_pk(ibs, iue) = lts_peaks(lts_second_peak_index(1));
                end

                % Check if valid...
                pk_tmp = preamble_pk(ibs, iue);
                lts_ind = pk_tmp - length(preamble_common) + 1;

                if lts_ind <= 0
                    fprintf('INVALID correlation peak from BS antenna %d at UE %d. Exit now! \n', ibs, iue);
		            err_flag = 1;
                    return;
                else
                    fprintf('LTS Index: %d \n', lts_ind);
                end

                % Measure CFO and apply correction
                [curr_vec, cfo_est] = meas_and_apply_cfo_corr(curr_vec, lts_ind, FFT_OFFSET, mode, cfo_est_apply);
                cfo_est_mat(ibs, iue) = cfo_est;

                % Re-extract LTS for channel estimate
                rx_lts = curr_vec(lts_ind : lts_ind+159);
                rx_lts1 = rx_lts(-64+-FFT_OFFSET + [97:160]);
                rx_lts2 = rx_lts(-FFT_OFFSET + [97:160]);

                % Received LTSs
                rx_lts1_f = fft(rx_lts1);
                rx_lts2_f = fft(rx_lts2);

                % Calculate channel estimate from average of 2 training symbols:
                %rx_H_est = mean([rx_lts1_f./lts_f   rx_lts2_f./ lts_f], 2);
                %rx_H_est = (lts_f.') .* (rx_lts1_f + rx_lts2_f) / 2;
                rx_H_est = lts_f .* (rx_lts1_f + rx_lts2_f) / 2;
                rx_H_est_sound = rx_H_est;
		        H(iue, ibs, :) = rx_H_est;
            end
        end
    end
end


function [cal_data_vec] = time_offset_cal(corr_peaks, data, N_BS_ANT, lts_t)

    % Sample offset calibration
    samp_offset_array = corr_peaks - corr_peaks(1);
    rx_mat_calibrated_tmp = zeros(size(data));

    for ibs =1:N_BS_ANT
        curr_offset = samp_offset_array(ibs);
	    if curr_offset < 0
            rx_mat_calibrated_tmp(ibs, 1+abs(curr_offset):end) = data(ibs, 1:end-abs(curr_offset));
        elseif  curr_offset > 0
            rx_mat_calibrated_tmp(ibs, 1:end-curr_offset) = data(ibs, 1+curr_offset:end);
        else
            rx_mat_calibrated_tmp(ibs, :) = data(ibs, :);
        end

        % VALIDATION!
	    curr_vec = rx_mat_calibrated_tmp(ibs, :);
        lts_corr = abs(conv(conj(fliplr(lts_t.')), sign(curr_vec.')));
        lts_peaks = find(lts_corr > 0.8*max(lts_corr));
        [LTS1, LTS2] = meshgrid(lts_peaks,lts_peaks);
        [lts_second_peak_index,y] = find(LTS2-LTS1 == length(lts_t));

        if(isempty(lts_second_peak_index))
            fprintf('%s: NO correlation peak from BS antenna %d. Exit now! \n', 'VERIFY', ibs);
        else
            if length(lts_second_peak_index) > 1
                preamble_pk(ibs) = lts_peaks(lts_second_peak_index(2));
            else
                preamble_pk(ibs) = lts_peaks(lts_second_peak_index(1));
            end

            pk_tmp = preamble_pk(ibs);
            lts_ind = pk_tmp - 160 + 1;
	    end
	    fprintf("LTS INDEX VERIFY: %d (Offset: %d) \n", lts_ind, curr_offset);
    end

    cal_data_vec = rx_mat_calibrated_tmp;
end


function [rx_dec_cfo_corr, rx_cfo_est_lts] = meas_and_apply_cfo_corr(rx_vec_iris, lts_ind, FFT_OFFSET, mode, cfo_est_mat)   

    % Estimate during downlink sound, and apply during downlink BF
    global APPLY_CFO_CORRECTION;

    %Extract LTS (not yet CFO corrected)
    rx_lts = rx_vec_iris(lts_ind : lts_ind+159);
    rx_lts1 = rx_lts(-64+-FFT_OFFSET + [97:160]);
    rx_lts2 = rx_lts(-FFT_OFFSET + [97:160]);
    %Calculate coarse CFO est
    rx_cfo_est_lts = mean(unwrap(angle(rx_lts2 .* conj(rx_lts1))));

    if(APPLY_CFO_CORRECTION)
        if strcmp(mode, 'dl-bf')
            % Apply the average CFO estimate we computed during the sounding process
            disp("APPLY CFO CORRECTION...");
            rx_cfo_est_lts = cfo_est_mat;
        else
            % Apply the CFO estimate we just computed
            rx_cfo_est_lts = rx_cfo_est_lts/(2*pi*64);
        end
    else
        rx_cfo_est_lts = 0;
    end

    % Apply CFO correction to raw Rx waveform
    rx_cfo_corr_t = exp(-1i*2*pi*rx_cfo_est_lts*[0:length(rx_vec_iris)-1]);
    rx_dec_cfo_corr = rx_vec_iris .* rx_cfo_corr_t.';
end
