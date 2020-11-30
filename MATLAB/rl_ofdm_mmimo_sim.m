%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%    Massive-MIMO Uplink and Downlink Beamforming Simulation
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

NUM_ITER = 10000;
NUM_USER = 4;
evm_snr_corr = -999*ones(4, NUM_USER, NUM_ITER);   % pilot snr, evm, bit error rate

for iter=1:NUM_ITER
    fprintf('Progress: %.1f%% \n', 100*(iter/NUM_ITER));
    % Waveform params
    N_OFDM_SYMS             = 24;           % Number of OFDM symbols
    MOD_ORDER               = 16;           % Modulation order (2/4/16/64 = BSPK/QPSK/16-QAM/64-QAM)
    TX_SCALE                = 1.0;          % Scale for Tx waveform ([0:1])

    % OFDM params
    SC_IND_PILOTS           = [8 22 44 58];                           % Pilot subcarrier indices
    SC_IND_DATA             = [2:7 9:21 23:27 39:43 45:57 59:64];     % Data subcarrier indices
    N_SC                    = 64;                                     % Number of subcarriers
    CP_LEN                  = 16;                                     % Cyclic prefix length
    N_DATA_SYMS             = N_OFDM_SYMS * length(SC_IND_DATA);      % Number of data symbols (one per data-bearing subcarrier per OFDM symbol)
    SYM_LEN                 = N_SC + CP_LEN;
    SC_ZER0                 = [1 28:38];                              % Indices of subcarriers with no data
    N_SC_ZERO               = length(SC_ZER0);

    % Rx processing params
    FFT_OFFSET                    = 16;         % Number of CP samples to use in FFT (on average)
    LTS_CORR_THRESH               = 0.8;        % Normalized threshold for LTS correlation
    DO_APPLY_CFO_CORRECTION       = 0;          % Enable CFO estimation/correction
    DO_APPLY_SFO_CORRECTION       = 0;          % Enable SFO estimation/correction
    DO_APPLY_PHASE_ERR_CORRECTION = 0;          % Enable Residual CFO estimation/correction

    SAMP_FREQ               = 20e6;
    TRIGGER_OFFSET_TOL_NS   = 3000;             % Trigger time offset toleration between Tx and Rx that can be accomodated
    N_BEGIN_ZERO_PAD        = 100;
    N_END_ZERI_PAD          = 100;

    % Massive-MIMO params
    N_UE                    = 4;
    N_BS_ANT                = 64;               % N_BS_ANT >> N_UE
    N_UPLINK_SYMBOLS        = N_OFDM_SYMS;
    N_0                     = 0.035; %1e-2;
    H_var                   = 1;
    SAVE_RX_DATA            = 0;

    % LTS for CFO and channel estimation
    lts_f = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1];
    lts_t = ifft(lts_f, 64);

    preamble = [lts_t(33:64) lts_t lts_t];

    %% Generate a payload of random integers
    tx_ul_data = randi(MOD_ORDER, N_UE, N_DATA_SYMS) - 1;

    % Functions for data -> complex symbol mapping (like qammod, avoids comm toolbox requirement)
    % These anonymous functions implement the modulation mapping from IEEE 802.11-2012 Section 18.3.5.8

    modvec_bpsk   =  (1/sqrt(2))  .* [-1 1];
    modvec_16qam  =  (1/sqrt(10)) .* [-3 -1 +3 +1];
    modvec_64qam  =  (1/sqrt(43)) .* [-7 -5 -1 -3 +7 +5 +1 +3];

    mod_fcn_bpsk  = @(x) complex(modvec_bpsk(1+x),0);
    mod_fcn_qpsk  = @(x) complex(modvec_bpsk(1+bitshift(x, -1)), modvec_bpsk(1+mod(x, 2)));
    mod_fcn_16qam = @(x) complex(modvec_16qam(1+bitshift(x, -2)), modvec_16qam(1+mod(x,4)));
    mod_fcn_64qam = @(x) complex(modvec_64qam(1+bitshift(x, -3)), modvec_64qam(1+mod(x,8)));


    % Map the data values on to complex symbols
    switch MOD_ORDER
        case 2         % BPSK
            tx_ul_syms = arrayfun(mod_fcn_bpsk, tx_ul_data);
        case 4         % QPSK
            tx_ul_syms = arrayfun(mod_fcn_qpsk, tx_ul_data);
        case 16        % 16-QAM
            tx_ul_syms = arrayfun(mod_fcn_16qam, tx_ul_data);
        case 64        % 64-QAM
            tx_ul_syms = arrayfun(mod_fcn_64qam, tx_ul_data);
        otherwise
            fprintf('Invalid MOD_ORDER (%d)!  Must be in [2, 4, 16, 64]\n', MOD_ORDER);
            return;
    end


    % Reshape the symbol vector to a matrix with one column per OFDM symbol
    tx_syms_mat = reshape(tx_ul_syms, N_UE, length(SC_IND_DATA), N_OFDM_SYMS);

    % Define the pilot tone values as BPSK symbols
    pt_pilots = [1 1 -1 1].';

    % Repeat the pilots across all OFDM symbols
    pt_pilots_mat = zeros(N_UE, 4, N_OFDM_SYMS);
    for i=1:N_UE
        pt_pilots_mat(i,:,:) = repmat(pt_pilots, 1, N_OFDM_SYMS);
    end

    %% IFFT

    % Construct the IFFT input matrix
    ifft_in_mat = zeros(N_UE, N_SC, N_OFDM_SYMS);

    % Insert the data and pilot values; other subcarriers will remain at 0
    ifft_in_mat(:, SC_IND_DATA, :)   = tx_syms_mat;
    ifft_in_mat(:, SC_IND_PILOTS, :) = pt_pilots_mat;

    %Perform the IFFT
    tx_payload_mat = ifft(ifft_in_mat, N_SC, 2);

    % Insert the cyclic prefix
    if(CP_LEN > 0)
        tx_cp = tx_payload_mat(:, (end-CP_LEN+1 : end), :);
        tx_payload_mat = cat(2, tx_cp, tx_payload_mat); %[tx_cp; tx_payload_mat];
    end

    % Reshape to a vector
    tx_payload_vec = reshape(tx_payload_mat, N_UE, numel(tx_payload_mat(1,:,:)));
    tx_pilot_vec = zeros(N_UE, SYM_LEN * (N_UE+1)); % additional pilot as noise
    for i=1:N_UE
        tx_pilot_vec(i, (i-1)*SYM_LEN+1:i*SYM_LEN) = [lts_t(64-CP_LEN+1:64) lts_t];
    end

    % Construct the full time-domain OFDM waveform
    tx_vec = [tx_pilot_vec tx_payload_vec];
    tx_vec_air = TX_SCALE .* tx_vec ./ repmat(max(abs(tx_vec),[],2), 1, size(tx_vec, 2));

    %% Uplink
    % Rayleight + AWGN:
    rng('shuffle');
    Z_mat = sqrt(N_0/2) * ( randn(N_BS_ANT,length(tx_vec_air) ) + 1i*randn(N_BS_ANT,length(tx_vec_air) ) );     % UL noise matrix
    H = sqrt(H_var/2) .* ( randn(N_BS_ANT, N_UE) + 1i*randn(N_BS_ANT, N_UE) );                                  % Spatial Channel Matrix
    rx_vec_air = H*tx_vec_air + Z_mat;
    %rx_vec_air = rx_vec_air./repmat(max(abs(rx_vec_air'))', 1, size(rx_vec_air, 2));

    rx_pilot_vec = zeros(N_BS_ANT, N_SC, N_UE);
    Z_mat_tmp = zeros(N_BS_ANT, N_SC, N_UE); 
    for i=1:N_UE
        rx_pilot_vec(:, :, i) = rx_vec_air(:, (i-1)*SYM_LEN+CP_LEN+1:i*SYM_LEN);
        Z_mat_tmp(:, :, i) = Z_mat(:, (i-1)*SYM_LEN+CP_LEN+1:i*SYM_LEN);
    end


    % =================================================
    % ====                  OBCH                   ====
    % =================================================
    pwr_signal = zeros(N_BS_ANT,N_UE);
    pwr_noise = zeros(N_BS_ANT,N_UE);
    pwr_signal_lin = zeros(N_BS_ANT,N_UE);
    pwr_noise_lin = zeros(N_BS_ANT,N_UE);
    for ant=1:N_BS_ANT
        for ue=1:N_UE
            pwr_lin = bandpower(rx_pilot_vec(ant, :, ue));
            pwr_db = 10*log10(pwr_lin);
            pwr_signal_lin(ant, ue) = pwr_lin;
            pwr_signal(ant, ue) = pwr_db;


            pwr_lin_n = bandpower(Z_mat_tmp(ant, :, ue));
            pwr_db_n = 10*log10(pwr_lin_n);
            pwr_noise_lin(ant, ue) = pwr_lin_n;
            pwr_noise(ant, ue) = pwr_db_n;
        end
    end

    snr_obch = pwr_signal - pwr_noise;
    snr_obch_avg = 10 * log10(mean(pwr_signal_lin) ./ mean(pwr_noise_lin));
    % =================================================
    % =================================================

    lts_f_mat = zeros(N_BS_ANT, N_SC, N_UE);
    for i = 1:N_UE
        lts_f_mat(:, :, i) = repmat(lts_f, N_BS_ANT, 1);
    end
    csi_mat = fft(rx_pilot_vec, N_SC, 2).*lts_f_mat;

    rx_payload_vec=rx_vec_air(:, (N_UE+1)*SYM_LEN+1:end);
    rx_payload_mat = reshape(rx_payload_vec, N_BS_ANT, SYM_LEN, N_OFDM_SYMS);%first two are preamble
    rx_payload_mat_noCP = rx_payload_mat(:, CP_LEN+1:end, :);
    fft_out_mat=fft(rx_payload_mat_noCP, N_SC, 2); %N_BS_ANT * N_SC * N_OFDM_SYMS+2

    precoding_mat = zeros(N_BS_ANT, N_SC, N_UE);
    demult_mat = zeros(N_UE, N_SC, N_OFDM_SYMS);
    sc_csi_mat = zeros(N_BS_ANT, N_UE);
    for j=1:N_SC
        sc_csi_mat = squeeze(csi_mat(:, j, :));
        zf_mat=pinv(sc_csi_mat);
        demult_mat(:, j, :) = zf_mat*squeeze(fft_out_mat(:, j, :));
        precoding_mat(:, j, :) = zf_mat.';
    end

    pilots_f_mat = demult_mat(:,SC_IND_PILOTS,:);
    pilots_f_mat_comp = pilots_f_mat.*pt_pilots_mat;
    pilot_phase_err = squeeze(angle(mean(pilots_f_mat_comp,2)));

    pilot_phase_corr = zeros(N_UE, N_SC, N_OFDM_SYMS);
    for i=1:N_SC
        pilot_phase_corr(:,i,:) = exp(-1i*pilot_phase_err);
    end

    % Apply the pilot phase correction per symbol
    demult_pc_mat = demult_mat.* pilot_phase_corr;
    payload_syms_mat = demult_pc_mat(:, SC_IND_DATA, :);
    payload_syms_mat = reshape(payload_syms_mat, N_UE, numel(payload_syms_mat(1,:,:)));

    %% Downlink
    tx_mult_mat = zeros(N_BS_ANT, N_SC, N_OFDM_SYMS+2);
    for i=1:N_SC
        lts_f_vec = lts_f(i)*ones(N_UE, 1);
        tx_mult_f = [lts_f_vec lts_f_vec squeeze(ifft_in_mat(:,i,:))];
        tx_mult_mat(:,i,:) = squeeze(precoding_mat(:,i,:))*tx_mult_f; % N_BS_ANT * N_SC * N_OFDM_SYMS
    end
    ifft_out_mat = fft(tx_mult_mat, N_SC, 2);
    % Insert the cyclic prefix
    if(CP_LEN > 0)
        tx_cp = ifft_out_mat(:, (end-CP_LEN+1 : end), :);
        tx_dl_mat = cat(2, tx_cp, ifft_out_mat); %[tx_cp; tx_payload_mat];
    else
        tx_dl_mat = ifft_out_mat;
    end

    tx_dl_vec = reshape(tx_dl_mat, N_BS_ANT, numel(tx_dl_mat(1,:,:)));
    tx_dl_vec = TX_SCALE .* tx_dl_vec ./ repmat(max(abs(tx_dl_vec),[],2), 1, length(tx_dl_vec));

    Z_dl_mat = sqrt(N_0/2) * ( randn(N_UE,length(tx_dl_vec) ) + 1i*randn(N_UE,length(tx_dl_vec) ) );   % DL noise matrix
    rx_dl_vec = (1/(sqrt(N_BS_ANT))) .* H.'*tx_dl_vec + Z_dl_mat;

    rx_dl_mat = reshape(rx_dl_vec, N_UE, SYM_LEN, N_OFDM_SYMS+2);
    if(CP_LEN > 0)
        rx_dl_mat = rx_dl_mat(:, CP_LEN+1:end, :);
    end
    rx_dl_f_mat = fft(rx_dl_mat, N_SC, 2);
    rx_lts1_f = rx_dl_f_mat(:, :, 1);
    rx_lts2_f = rx_dl_f_mat(:, :, 2);

    dl_syms_f_mat = rx_dl_f_mat(:, :, 3:end);
    rx_H_est = repmat(lts_f, N_UE, 1).*(rx_lts1_f + rx_lts2_f)/2;

    dl_syms_eq_mat = zeros(N_UE, N_SC, N_OFDM_SYMS);
    for i=1:N_OFDM_SYMS
        dl_syms_eq_mat(:,:,i) = squeeze(dl_syms_f_mat(:,:,i))./rx_H_est;
    end

    pilots_eq_mat = dl_syms_eq_mat(:,SC_IND_PILOTS,:);
    pilots_eq_mat_comp = pilots_eq_mat.*pt_pilots_mat;
    pilot_dl_phase_err = squeeze(angle(mean(pilots_eq_mat_comp,2)));

    pilot_dl_phase_corr = zeros(N_UE, N_SC, N_OFDM_SYMS);
    for i=1:N_SC
        pilot_dl_phase_corr(:,i,:) = exp(-1i*pilot_dl_phase_err);
    end

    % Apply the pilot phase correction per symbol
    dl_syms_eq_pc_mat = dl_syms_eq_mat.* pilot_dl_phase_corr;
    payload_dl_syms_mat = dl_syms_eq_pc_mat(:, SC_IND_DATA, :);
    payload_dl_syms_mat = reshape(payload_dl_syms_mat, N_UE, numel(payload_dl_syms_mat(1,:,:)));

    %% Demodulate
    rx_syms = payload_syms_mat;
    rx_data = zeros(size(rx_syms));

    demod_fcn_bpsk = @(x) double(real(x)>0);
    demod_fcn_qpsk = @(x) double(2*(real(x)>0) + 1*(imag(x)>0));
    demod_fcn_16qam = @(x) (8*(real(x)>0)) + (4*(abs(real(x))<0.6325)) + (2*(imag(x)>0)) + (1*(abs(imag(x))<0.6325));
    demod_fcn_64qam = @(x) (32*(real(x)>0)) + (16*(abs(real(x))<0.6172)) + (8*((abs(real(x))<(0.9258))&&((abs(real(x))>(0.3086))))) + (4*(imag(x)>0)) + (2*(abs(imag(x))<0.6172)) + (1*((abs(imag(x))<(0.9258))&&((abs(imag(x))>(0.3086)))));

    for n_ue = 1:N_UE
        switch(MOD_ORDER)
            case 2         % BPSK
                rx_data_temp = arrayfun(demod_fcn_bpsk, rx_syms(n_ue,:) );
            case 4         % QPSK
                rx_data_temp = arrayfun(demod_fcn_qpsk, rx_syms(n_ue,:));
            case 16        % 16-QAM
                rx_data_temp = arrayfun(demod_fcn_16qam, rx_syms(n_ue,:));
            case 64        % 64-QAM
                rx_data_temp = arrayfun(demod_fcn_64qam, rx_syms(n_ue,:));
        end

        rx_data(n_ue, :) = complex( rx_data_temp );
    end

    %% Plots:
    if (0)
        cf = 0;

        % UL
        cf = cf + 1;
        figure(cf); clf;
        subplot(2,2,1)
        plot(payload_syms_mat(1, :),'ro','MarkerSize',1);
        axis square; axis(1.5*[-1 1 -1 1]);
        grid on;
        hold on;

        plot(tx_ul_syms(1, :),'bo');
        title('Uplink Tx and Rx Constellations')
        legend('Rx','Tx');


        subplot(2,2,2)
        plot(payload_syms_mat(2, :),'ro','MarkerSize',1);
        axis square; axis(1.5*[-1 1 -1 1]);
        grid on;
        hold on;

        plot(tx_ul_syms(2, :),'bo');
        legend('Rx','Tx');


        subplot(2,2,3)
        plot(payload_syms_mat(3, :),'ro','MarkerSize',1);
        axis square; axis(1.5*[-1 1 -1 1]);
        grid on;
        hold on;

        plot(tx_ul_syms(3, :),'bo');
        legend('Rx','Tx');


        subplot(2,2,4)
        plot(payload_syms_mat(4, :),'ro','MarkerSize',1);
        axis square; axis(1.5*[-1 1 -1 1]);
        grid on;
        hold on;

        plot(tx_ul_syms(4, :),'bo');
        legend('Rx','Tx');

        % DL
        cf = cf + 1;
        figure(cf); clf;
        subplot(2,2,1)
        plot(payload_dl_syms_mat(1, :),'ro','MarkerSize',1);
        axis square; axis(1.5*[-1 1 -1 1]);
        grid on;
        hold on;

        plot(tx_ul_syms(1, :),'bo');
        title('Downlink Tx and Rx Constellations')
        legend('Rx','Tx');


        subplot(2,2,2)
        plot(payload_dl_syms_mat(2, :),'ro','MarkerSize',1);
        axis square; axis(1.5*[-1 1 -1 1]);
        grid on;
        hold on;

        plot(tx_ul_syms(2, :),'bo');
        legend('Rx','Tx');


        subplot(2,2,3)
        plot(payload_dl_syms_mat(3, :),'ro','MarkerSize',1);
        axis square; axis(1.5*[-1 1 -1 1]);
        grid on;
        hold on;

        plot(tx_ul_syms(3, :),'bo');
        legend('Rx','Tx');


        subplot(2,2,4)
        plot(payload_dl_syms_mat(4, :),'ro','MarkerSize',1);
        axis square; axis(1.5*[-1 1 -1 1]);
        grid on;
        hold on;

        plot(tx_ul_syms(4, :),'bo');
        legend('Rx','Tx');

        % EVM & SNR
        cf = cf + 1;
        figure(cf); clf;
        tx_syms_vecs = reshape( tx_syms_mat,N_UE, numel(tx_syms_mat(1,:,:) ) );
        evm_mat = abs(payload_syms_mat - tx_syms_vecs).^2;
        aevms = mean(evm_mat,2);
        snrs = 10*log10(1./aevms);

        subplot(2,2,1)
        plot(100*evm_mat(1,:),'o','MarkerSize',1)
        axis tight
        hold on
        plot([1 length(evm_mat(1,:))], 100*[aevms(1,:), aevms(1,:)],'r','LineWidth',2)
        myAxis = axis;
        h = text(round(.05*length(evm_mat(1,:))), 100*aevms(1,:)+ .1*(myAxis(4)-myAxis(3)), sprintf('Effective SNR: %.1f dB', snrs(1,:)));
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


        subplot(2,2,2)
        plot(100*evm_mat(2,:),'o','MarkerSize',1)
        axis tight
        hold on
        plot([1 length(evm_mat(2,:))], 100*[aevms(2,:), aevms(2,:)],'r','LineWidth',2)
        myAxis = axis;
        h = text(round(.05*length(evm_mat(2,:))), 100*aevms(2,:)+ .1*(myAxis(4)-myAxis(3)), sprintf('Effective SNR: %.1f dB', snrs(2,:)));
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

        subplot(2,2,3)
        plot(100*evm_mat(3,:),'o','MarkerSize',1)
        axis tight
        hold on
        plot([1 length(evm_mat(3,:))], 100*[aevms(3,:), aevms(3,:)],'r','LineWidth',2)
        myAxis = axis;
        h = text(round(.05*length(evm_mat(3,:))), 100*aevms(3,:)+ .1*(myAxis(4)-myAxis(3)), sprintf('Effective SNR: %.1f dB', snrs(3,:)));
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

        subplot(2,2,4)
        plot(100*evm_mat(4,:),'o','MarkerSize',1)
        axis tight
        hold on
        plot([1 length(evm_mat(4,:))], 100*[aevms(4,:), aevms(4,:)],'r','LineWidth',2)
        myAxis = axis;
        h = text(round(.05*length(evm_mat(4,:))), 100*aevms(4,:)+ .1*(myAxis(4)-myAxis(3)), sprintf('Effective SNR: %.1f dB', snrs(4,:)));
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


        % ====================================================================
        % ====================================================================
        % PILOT SNR
        cf = cf + 1;
        figure(cf); clf;
        subplot(2,2,1);
        plot(snr_obch(:,1),'o','MarkerSize',1);
        text(1,1, sprintf('AVG SNR: %.2f dB', snr_obch_avg(1)));
        title('Pilot SNR');
        grid on;
        subplot(2,2,2);
        plot(snr_obch(:,2),'o','MarkerSize',1);
        text(1,1, sprintf('AVG SNR: %.2f dB', snr_obch_avg(2)));
        title('Pilot SNR');
        grid on;
        subplot(2,2,3);
        plot(snr_obch(:,3),'o','MarkerSize',1);
        text(1,1, sprintf('AVG SNR: %.2f dB', snr_obch_avg(3)));
        title('Pilot SNR');
        grid on;
        subplot(2,2,4);
        plot(snr_obch(:,4),'o','MarkerSize',1);
        text(1,1, sprintf('AVG SNR: %.2f dB', snr_obch_avg(4)));
        title('Pilot SNR');
        grid on;
        % ====================================================================
        % ====================================================================
    end

    %% Calculate Rx stats
    sym_errs = sum(sum( tx_ul_data ~= rx_data ) );                                                                                             % errors per user
    bit_errs = length(find(dec2bin(bitxor(tx_ul_data, rx_data),8) == '1'));
    %rx_evm   = sqrt(sum((real(rx_syms) - real(tx_syms)).^2 + (imag(rx_syms) - imag(tx_syms)).^2)/(length(SC_IND_DATA) * N_OFDM_SYMS));

    %fprintf('\nResults:\n');
    %fprintf('===== SNRs: =====\n');

    dl_snr_per_ue = zeros(1, N_UE);
    for n_ue = 1:N_UE
        snr_lin = mean( abs( payload_dl_syms_mat(n_ue,:) ).^2 ) / N_0;
        snr = 10*log10(snr_lin);
        dl_snr_per_ue(n_ue) = snr;
        %fprintf('DL SNR of user %d :   %f\n', n_ue ,snr );
    end

    %fprintf('\n===== Errors: =====\n');
    %fprintf('Num Bits:   %d\n', N_UE * N_DATA_SYMS * log2(MOD_ORDER) );
    %fprintf('Sym Errors:  %d (of %d total symbols)\n', sym_errs, N_UE * N_DATA_SYMS);
    %fprintf('Bit Errors:  %d (of %d total bits)\n', bit_errs, N_UE * N_DATA_SYMS * log2(MOD_ORDER));

    if SAVE_RX_DATA
        %%% save uplink rx signal
        rx_save_vec = [rx_vec_air(:, 1:N_UE*SYM_LEN) rx_vec_air(:, (N_UE+1)*SYM_LEN+1:end)];
        rx_save_mat = reshape(rx_save_vec, size(rx_vec_air, 1), SYM_LEN, N_OFDM_SYMS+N_UE);
        filename = 'rx_ul_'+string(N_BS_ANT)+'x'+string(N_UE)+'.bin';
        fileID = fopen(filename,'w');
        for i=1:N_OFDM_SYMS+N_UE
           for j =1:N_BS_ANT
               save_vec = [real(rx_save_mat(j,:,i));imag(rx_save_mat(j,:,i))];
               %save_vec = rx_save_mat(j,:,i);
               save_vec_i = uint16(real(save_vec)*32768);
               save_vec_q = uint16(imag(save_vec)*32768);
               save_vec_int = bitor(bitshift(uint32(save_vec_i), 16) , uint32(save_vec_q));
               %fwrite(fileID,save_vec_int,'uint32');
               fwrite(fileID,save_vec(:),'double');
           end
        end

        fclose(fileID);

        %%% save downlink rx signal
        dl_save_mat = reshape(rx_dl_vec, size(rx_dl_vec, 1), SYM_LEN, N_OFDM_SYMS+2);
        filename = 'rx_dl_'+string(N_BS_ANT)+'x'+string(N_UE)+'.bin';
        fileID = fopen(filename,'w');
        for i=1:N_OFDM_SYMS+2
           for j =1:N_UE
               save_vec = [real(dl_save_mat(j,:,i));imag(dl_save_mat(j,:,i))];
               %save_vec = dl_save_mat(j,:,i);
               save_vec_i = uint16(real(save_vec)*32768);
               save_vec_q = uint16(imag(save_vec)*32768);
               save_vec_int = bitor(bitshift(uint32(save_vec_i), 16) , uint32(save_vec_q));
               %fwrite(fileID,save_vec_int,'uint32');
               fwrite(fileID,save_vec(:),'double');
           end
        end

        fclose(fileID);

        %%% save precoding matrix
        filename = 'zf_'+string(N_BS_ANT)+'x'+string(N_UE)+'.bin';
        fileID = fopen(filename,'w');
        for i=1:N_SC
           for j =1:N_BS_ANT
               save_vec = [real(squeeze(precoding_mat(j,i,:))) imag(squeeze(precoding_mat(j,i,:)))].';
               fwrite(fileID,save_vec(:),'single');
           end
        end
        fclose(fileID);
    end
    
    % ==== OBCH ====
    % Compute bit errors per user
    bit_errs_ue = -999 * ones(1, N_UE);
    for ue_idx = 1:N_UE
        tmp = length(find(dec2bin(bitxor(tx_ul_data(ue_idx, :), rx_data(ue_idx, :)),8) == '1'));
        bit_errs_ue(ue_idx) = 100 * (tmp / (N_DATA_SYMS * log2(MOD_ORDER)));
    end
    bit_err_pct = 100 * (bit_errs / (N_UE * N_DATA_SYMS * log2(MOD_ORDER)));
    tx_syms_vecs = reshape( tx_syms_mat,N_UE, numel(tx_syms_mat(1,:,:) ) );
    evm_mat = abs(payload_syms_mat - tx_syms_vecs).^2;
    aevms = mean(evm_mat,2);
    evm_snr_corr(1, :, iter) = snr_obch_avg;
    evm_snr_corr(2, :, iter) = 100*aevms;
    evm_snr_corr(3, :, iter) = bit_errs_ue;
    evm_snr_corr(4, :, iter) = dl_snr_per_ue;
end

save('snr_results_oscar.mat');

% CORRELATIONS
cf = 0;
for ue_idx=1:N_UE
    cf = cf + 1;
    figure(cf); clf;
    subplot(2,2,1);
    plot(squeeze(evm_snr_corr(1, ue_idx, :)), squeeze(evm_snr_corr(2, ue_idx, :)), 'o', 'MarkerSize', 2);
    title(sprintf('Pilot SNR vs AEVM | User %d', ue_idx));
    xlabel('Pilot SNR (dB)');
    ylabel('AEVM (%)');
    grid on;
    
    subplot(2,2,2);
    plot(squeeze(evm_snr_corr(1, ue_idx, :)), squeeze(evm_snr_corr(3, ue_idx, :)), 'o', 'MarkerSize', 2);
    title(sprintf('Pilot SNR vs Bit Error | User %d', ue_idx));
    xlabel('Pilot SNR (dB)');
    ylabel('Bit Error (%)');
    grid on;
    
    subplot(2,2,3);
    plot(squeeze(evm_snr_corr(2, ue_idx, :)), squeeze(evm_snr_corr(3, ue_idx, :)), 'o', 'MarkerSize', 2);
    title(sprintf('AEVM vs Bit Error | User %d', ue_idx));
    xlabel('AEVM (%)');
    ylabel('Bit Error (%)');
    grid on;
    
    subplot(2,2,4);
    plot(squeeze(evm_snr_corr(1, ue_idx, :)), squeeze(evm_snr_corr(4, ue_idx, :)), 'o', 'MarkerSize', 2);
    title(sprintf('Pilot SNR vs DL SNR | User %d', ue_idx));
    xlabel('Pilot SNR (dB)');
    ylabel('DL SNR (dB)');
    grid on;
end
stop = 1

    cf = cf + 1;
    figure(cf); clf;
    subplot(2,2,1);
    plot(squeeze(evm_db(1, 1, :)), 'o', 'MarkerSize', 2);
    title(sprintf('Pilot SNR vs AEVM | User %d', ue_idx));
    xlabel('Pilot SNR (dB)');
    ylabel('AEVM (%)');
    grid on;