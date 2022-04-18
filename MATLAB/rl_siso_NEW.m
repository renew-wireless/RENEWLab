%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%	Author(s): C. Nicolas Barati nicobarati@rice.edu 
%		Rahman Doost-Mohamamdy: doost@rice.edu
%
% Multiple iterations of a single-shot transmissions from one client or UE
% to one base station radio (UE stands for User Equipment).
% The script explores Bit Error Rate (BER) as a function of Signal-to-Noise
% Ratio (SNR) and therefore iterates over different SNR values (sim_SNR_db
% variable). Within e   ach iteration, only a single frame transmission takes
% place.
%
% We define two modes: OTA (Over-the-air) and SIM_MOD (simulation).
% In simulation mode we simply use a Rayleigh channel whereas the OTA mode
% relies on the Iris hardware for transmission and reception.
% In both cases the client transmits an OFDM signal that resembles a
% typical 802.11 WLAN waveform. If the transmission is OTA, then the user
% specifies a schedule that tells the client when to transmit its frame
% The base station initiates the schedule by sending a beacon signal that
% synchronizes the client. After that, the client will simply transmit its
% frame.
%
%---------------------------------------------------------------------
% Original code copyright Mango Communications, Inc.
% Distributed under the WARP License http://warpproject.org/license
% Copyright (c) 2018-2019, Rice University
% RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
% ---------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
close all;

[version, executable, isloaded] = pyversion;
if ~isloaded
    pyversion /usr/bin/python
    py.print() %weird bug where py isn't loaded in an external script
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
WRITE_PNG_FILES         = 0;            % Enable writing plots to PNG
PLOT                    = 0;
FIND_OPTIMAL_GAINS      = 0;            % Evaluates different TX/RX gain combinations and returns the combination that yields the largest number of detected beacons
SIM_MODE                = 0;            % Enable for AWGN sim, disable to run hardware

%Iris params:
N_BS_NODE               = 1;
N_UE                    = 1;
TX_FRQ                  = 3.6e9;
RX_FRQ                  = TX_FRQ;
TX_GN                   = 80;
RX_GN                   = 60;
SMPL_RT                 = 5e6;
TX_SCALE                = 1;            % Scale for Tx waveform ([0:1])
N_FRM                   = 3;
          
bs_ids = string.empty();
bs_sched = string.empty();
ue_ids = string.empty();
ue_sched = string.empty();


% Waveform params
N_OFDM_SYM              = 46;         % Number of OFDM symbols for burst, it needs to be less than 47
MOD_ORDER               = 16;           % Modulation order (2/4/16/64 = BSPK/QPSK/16-QAM/64-QAM)

% OFDM params
SC_IND_PILOTS           = [8 22 44 58];                           % Pilot subcarrier indices
SC_IND_DATA             = [2:7 9:21 23:27 39:43 45:57 59:64];     % Data subcarrier indices
N_SC                    = 64;                                     % Number of subcarriers
CP_LEN                  = 16;                                     % Cyclic prefix length
N_DATA_SYMS             = N_OFDM_SYM * length(SC_IND_DATA);       % Number of data symbols (one per data-bearing subcarrier per OFDM symbol)
N_LTS_SYM               = 2;                                      % Number of 
N_SYM_SAMP              = N_SC + CP_LEN;                          % Number of samples that will go over the air
N_ZPAD_PRE              = 90;                                     % Zero-padding prefix for Iris
N_ZPAD_POST             = N_ZPAD_PRE - 14;                         % Zero-padding postfix for Iris

% Rx processing params
FFT_OFFSET                    = 0;          % Number of CP samples to use in FFT (on average)
DO_APPLY_PHASE_ERR_CORRECTION = 1;           % Enable Residual CFO estimation/correction

%% Define the preamble

% LTS for fine CFO and channel estimation
lts_f = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 ...
    1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1];
lts_t = ifft(lts_f, 64); %time domain

preamble = [lts_t(33:64) lts_t lts_t];

%% Generate a payload of random integers
tx_data = randi(MOD_ORDER, 1, N_DATA_SYMS) - 1;

tx_syms = mod_sym(tx_data, MOD_ORDER);

% Reshape the symbol vector to a matrix with one column per OFDM symbol
tx_syms_mat = reshape(tx_syms, length(SC_IND_DATA), N_OFDM_SYM);

% Define the pilot tone values as BPSK symbols
pilots = [1 1 -1 1].';

% Repeat the pilots across all OFDM symbols
pilots_mat = repmat(pilots, 1, N_OFDM_SYM);

%% Time Domain

% Do yourselves: construct the TD input matrix
fdd_mat = zeros(N_SC, N_OFDM_SYM);

% Insert the data and pilot values; other subcarriers will remain at 0
fdd_mat(SC_IND_DATA, :)   = tx_syms_mat;
fdd_mat(SC_IND_PILOTS, :) = pilots_mat;

% Do yourselves: get TD samples:
tdd_tx_payload_mat = ifft(fdd_mat, N_SC, 1);

% Insert the cyclic prefix
if(CP_LEN > 0)
    % Do yourselves: Insert CP
    tx_cp = tdd_tx_payload_mat((end-CP_LEN+1 : end), :);
    tdd_tx_payload_mat = [tx_cp; tdd_tx_payload_mat];
end

% Reshape to a vector
tx_payload_vec = reshape(tdd_tx_payload_mat, 1, numel(tdd_tx_payload_mat));


% Construct the full time-domain OFDM waveform
tx_vec = [zeros(1,N_ZPAD_PRE) preamble tx_payload_vec zeros(1,N_ZPAD_POST)];

% Leftover from zero padding:
tx_vec_iris = tx_vec.';
% Scale the Tx vector to +/- 1
tx_vec_iris = TX_SCALE .* tx_vec_iris ./ max(abs(tx_vec_iris));

if SIM_MODE
    disp("Running: AWGN SIMULATION MODE");
    % AWGN only
    snr = 20;
    tx_var = mean(mean(abs(tx_vec_iris).^2 )) * (64/48);
    nvar =  tx_var / 10^(0.1*snr); % noise variance per data sample
        
    H_ul = ones(size(tx_vec_iris.'));
    % noise vector
    W_ul = sqrt(nvar/2).* (randn(N_BS_NODE, length(tx_vec_iris)) + ...
        1i*randn(N_BS_NODE, length(tx_vec_iris)) );
    
    % output vector
    rx_vec_iris_tmp = H_ul.*tx_vec_iris.' + W_ul;
    rx_vec_iris_tmp = rx_vec_iris_tmp.';

    numGoodFrames = 1;  % Don't care about number of frames in SIM MODE
else
    % Set up the Iris experiment
    disp("Running: HARDWARE MODE");

    % Create two Iris node objects:
    bs_ids = ["RF3E000146"];
    ue_ids = ["RF3D000016"];

    % Iris nodes' parameters
    sdr_params = struct(...
        'bs_id', bs_ids, ...
        'ue_id', ue_ids,...
        'txfreq', TX_FRQ, ...
        'rxfreq', RX_FRQ, ...
        'txgain', TX_GN, ...
        'rxgain', RX_GN, ...
        'sample_rate', SMPL_RT);


    mimo_handle = mimo_driver(sdr_params);

    if FIND_OPTIMAL_GAINS
        [txg_opt, rxg_opt, max_num_beacon, valid] = mimo_handle.mimo_set_opt_gains(N_FRM);
        fprintf(" !!! Optimal Gain Setting: TX %d, RX %d, max_num_beacon %d !!! \n", txg_opt, rxg_opt, max_num_beacon);
        if ~valid
            printf("[ERROR] Could NOT find an adequate gain setting (no beacons detected). Check setup!");
            return;
        end
        mimo_handle.mimo_update_sdr_param('txgain', txg_opt);
        mimo_handle.mimo_update_sdr_param('rxgain', rxg_opt);
    end

    [rx_vec_iris_tmp, numGoodFrames, ~] = mimo_handle.mimo_txrx_uplink(tx_vec_iris, N_FRM, N_ZPAD_PRE);
    mimo_handle.mimo_close();

end

if (isempty(rx_vec_iris_tmp))
    return;
end

% Process data
for frm_idx = 1:numGoodFrames
    fprintf(' =============================== \n');
    fprintf('Frame #%d Out of %d Triggered Frames \n', frm_idx, numGoodFrames);

    if numGoodFrames == 1
        rx_vec_iris = squeeze(rx_vec_iris_tmp);
    else
        rx_vec_iris = squeeze(rx_vec_iris_tmp(frm_idx, 1, 1, :));
    end

    %figure; plot(abs(rx_vec_iris))
    %% Correlate for LTS
    % Complex cross correlation of Rx waveform with time-domain LTS
    a = 1;
    unos = ones(size(preamble.'))';
    v0 = filter(flipud(preamble'),a,rx_vec_iris);
    v1 = filter(unos,a,abs(rx_vec_iris).^2);
    m_filt = (abs(v0).^2)./v1; % normalized correlation
    lts_corr = m_filt;
    [rho_max, ipos] = max(lts_corr);

    payload_ind = ipos + 1;
    lts_ind = payload_ind - N_LTS_SYM*(N_SC + CP_LEN);

    if lts_ind < 1
        lts_ind = 1;
    end

    % Re-extract LTS for channel estimate
    rx_lts = rx_vec_iris(lts_ind : lts_ind+159);
    rx_lts1 = rx_lts(-64+-FFT_OFFSET + [97:160]);
    rx_lts2 = rx_lts(-FFT_OFFSET + [97:160]);

    % Received LTSs
    % Do yourselves: take the FD pilots:
    rx_lts1_f = fft(rx_lts1);
    rx_lts2_f = fft(rx_lts2);

    % Do yourselves: Calculate channel estimate from average of 2 training symbols: 
    rx_H_est = mean([rx_lts1_f./lts_f.'   rx_lts2_f./ lts_f.'], 2); 

    %% Rx payload processing
    % Extract the payload samples (integer number of OFDM symbols following preamble)
    if( (length(rx_vec_iris) - payload_ind ) > (N_SYM_SAMP * N_OFDM_SYM) )
        payload_vec = rx_vec_iris(payload_ind : payload_ind + (N_SYM_SAMP * N_OFDM_SYM));
    else
        payload_vec = rx_vec_iris(payload_ind : end);
    end

    missed_samps = (N_SC+CP_LEN) * N_OFDM_SYM - length(payload_vec); %sometimes it's below 0.

    if (missed_samps > 0)
        payload_vec = [payload_vec zeros(1, missed_samps)];
    elseif (missed_samps < 0)
        payload_vec = payload_vec(1:end+missed_samps);
    end


    payload_mat = reshape(payload_vec, (N_SC+CP_LEN), N_OFDM_SYM);

    % Remove the cyclic prefix, keeping FFT_OFFSET samples of CP (on average)
    payload_mat_noCP = payload_mat(CP_LEN-FFT_OFFSET+[1:N_SC], :);

    % Do yourselves: bring to frequency domain:
    syms_f_mat = fft(payload_mat_noCP, N_SC, 1);

    % Do yourselves: Equalize.
    syms_eq_mat = syms_f_mat ./ repmat(rx_H_est, 1, N_OFDM_SYM);

    %% Calculate phase correction
    if DO_APPLY_PHASE_ERR_CORRECTION
        % Extract the pilots and calculate per-symbol phase error
        pilots_f_mat = syms_eq_mat(SC_IND_PILOTS, :);
        pilots_f_mat_comp = pilots_f_mat.*pilots_mat;
        pilot_phase_err = angle(mean(pilots_f_mat_comp));
    else
        % Define an empty phase correction vector (used by plotting code below)
        pilot_phase_err = zeros(1, N_OFDM_SYM);
    end
    pilot_phase_err_corr = repmat(pilot_phase_err, N_SC, 1);
    pilot_phase_corr = exp(-1i*(pilot_phase_err_corr));

    %% Apply phase correction
    % Apply the pilot phase correction per symbol
    syms_eq_pc_mat = syms_eq_mat .* pilot_phase_corr;
    payload_syms_mat = syms_eq_pc_mat(SC_IND_DATA, :);

    %% Demodulate
    rx_syms = reshape(payload_syms_mat, 1, N_DATA_SYMS);

    rx_data = demod_sym(rx_syms ,MOD_ORDER);

    bit_errs = length(find(dec2bin(bitxor(tx_data, rx_data),8) == '1'));

    ber_SIM = bit_errs/(N_DATA_SYMS * log2(MOD_ORDER));


    % EVM & SNR
    % Do yourselves. Calculate EVM and effective SNR:
    evm_mat = abs(payload_syms_mat - tx_syms_mat).^2;
    aevms = mean(evm_mat(:)); % needs to be a scalar
    snr = 10*log10(1./aevms); % calculate in dB scale.


    %% Plot Results

    if PLOT
        cf = 0;
        fst_clr = [0, 0.4470, 0.7410];
        sec_clr = [0.8500, 0.3250, 0.0980];
        % Tx signal
        cf = cf + 1;
        figure(cf); clf;

        subplot(2,1,1);
        plot(real(tx_vec_iris));
        axis([0 length(tx_vec_iris) -TX_SCALE TX_SCALE])
        grid on;
        title('Tx Waveform (I)');

        subplot(2,1,2);
        plot(imag(tx_vec_iris), 'color' , sec_clr );
        axis([0 length(tx_vec_iris) -TX_SCALE TX_SCALE])
        grid on;
        title('Tx Waveform (Q)');

        if(WRITE_PNG_FILES)
            print(gcf,sprintf('wl_ofdm_plots_%s_txIQ', example_mode_string), '-dpng', '-r96', '-painters')
        end

        % Rx signal
        cf = cf + 1;
        figure(cf); clf;
        subplot(2,1,1);
        plot(real(rx_vec_iris));
        axis([0 length(rx_vec_iris) -TX_SCALE TX_SCALE])
        grid on;
        title('Rx Waveform (I)');

        subplot(2,1,2);
        plot(imag(rx_vec_iris), 'color', sec_clr);
        axis([0 length(rx_vec_iris) -TX_SCALE TX_SCALE])
        grid on;
        title('Rx Waveform (Q)');

        if(WRITE_PNG_FILES)
            print(gcf,sprintf('wl_ofdm_plots_%s_rxIQ', example_mode_string), '-dpng', '-r96', '-painters')
        end

        % Rx LTS correlation
        cf = cf + 1;
        figure(cf); clf;
        lts_to_plot = lts_corr;
        plot(lts_to_plot, '.-b', 'LineWidth', 1);
        hold on;
        grid on;
        title('LTS Correlation')
        xlabel('Sample Index')
        myAxis = axis();
        axis([1, 1000, myAxis(3), myAxis(4)])

        if(WRITE_PNG_FILES)
            print(gcf,sprintf('wl_ofdm_plots_%s_ltsCorr', example_mode_string), '-dpng', '-r96', '-painters')
        end

        % Channel Estimates
        cf = cf + 1;
        figure(cf); clf;
        rx_H_est_plot = repmat(complex(NaN,NaN),1,length(rx_H_est));
        rx_H_est_plot(SC_IND_DATA) = rx_H_est(SC_IND_DATA);
        rx_H_est_plot(SC_IND_PILOTS) = rx_H_est(SC_IND_PILOTS);

        x = (20/N_SC) * (-(N_SC/2):(N_SC/2 - 1));

        figure(cf); clf;
        bar(x, fftshift(abs(rx_H_est_plot)),1,'LineWidth', 1);
        axis([min(x) max(x) 0 1.1*max(abs(rx_H_est_plot))])
        grid on;
        title('Channel Estimates (Magnitude)')
        xlabel('Baseband Frequency (MHz)')

        if(WRITE_PNG_FILES)
            print(gcf,sprintf('wl_ofdm_plots_%s_chanEst', example_mode_string), '-dpng', '-r96', '-painters')
        end

        % Symbol constellation
        cf = cf + 1;
        figure(cf); clf;

        plot(payload_syms_mat(:),'o','MarkerSize',2, 'color', sec_clr);
        axis square; axis(1.5*[-1 1 -1 1]);
        xlabel('Inphase')
        ylabel('Quadrature')
        grid on;
        hold on;

        plot(tx_syms_mat(:),'*', 'MarkerSize',16, 'LineWidth',2, 'color', fst_clr);
        title('Tx and Rx Constellations')
        legend('Rx','Tx','Location','EastOutside');

        if(WRITE_PNG_FILES)
            print(gcf,sprintf('wl_ofdm_plots_%s_constellations', example_mode_string), '-dpng', '-r96', '-painters')
        end


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
        imagesc(1:N_OFDM_SYM, (SC_IND_DATA - N_SC/2), 100*fftshift(evm_mat,1))

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



        if(WRITE_PNG_FILES)
            print(gcf,sprintf('wl_ofdm_plots_%s_evm', example_mode_string), '-dpng', '-r96', '-painters')
        end
    end

    %% Calculate Rx stats

    sym_errs = sum(tx_data ~= rx_data);
    bit_errs = length(find(dec2bin(bitxor(tx_data, rx_data),8) == '1'));
    rx_evm   = sqrt(sum((real(rx_syms) - real(tx_syms)).^2 + (imag(rx_syms) - imag(tx_syms)).^2)/(length(SC_IND_DATA) * N_OFDM_SYM));

    fprintf('\n Frame %d Results:\n', frm_idx);
    fprintf('Num Bytes:   %d\n', N_DATA_SYMS * log2(MOD_ORDER) / 8);
    fprintf('Sym Errors:  %d (of %d total symbols)\n', sym_errs, N_DATA_SYMS);
    fprintf('Bit Errors:  %d (of %d total bits)\n', bit_errs, N_DATA_SYMS * log2(MOD_ORDER));
end