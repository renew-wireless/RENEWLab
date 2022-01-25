function [bit_errs_mrc, aevms_mrc, bit_errs_br, aevms_br]= process_simo(tx_vec_iris_mat_file, rx_vec_iris_mat_file)

% Waveform params
N_OFDM_SYM              = 46;         % Number of OFDM symbols for burst, it needs to be less than 47
MOD_ORDER               = 4;           % Modulation order (2/4/16/64 = BSPK/QPSK/16-QAM/64-QAM)
N_SC                    = 64;                                     % Number of subcarriers
CP_LEN                  = 16;     

% OFDM params
SC_IND_PILOTS           = [8 22 44 58];                           % Pilot subcarrier indices
SC_IND_DATA             = [2:7 9:21 23:27 39:43 45:57 59:64];     % Data subcarrier indices
SC_IND_DATA_PILOT       = [2:27 39:64]';
SC_IND_NULL = setdiff(1:N_SC, SC_IND_DATA_PILOT);
                                % Cyclic prefix length
N_DATA_SYMS             = N_OFDM_SYM * length(SC_IND_DATA);       % Number of data symbols (one per data-bearing subcarrier per OFDM symbol)
N_LTS_SYM               = 2;                                      % Number of 
N_SYM_SAMP              = N_SC + CP_LEN;                          % Number of samples that will go over the air
N_ZPAD_PRE              = 90;                                     % Zero-padding prefix for Iris
N_ZPAD_POST             = N_ZPAD_PRE - 14;                         % Zero-padding postfix for Iris

% Rx processing params
FFT_OFFSET                    = 16;          % Number of CP samples to use in FFT (on average)
DO_APPLY_PHASE_ERR_CORRECTION = 1;           % Enable Residual CFO estimation/correction

%% Define the preamble
% LTS for fine CFO and channel estimation
lts_f = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 ...
    1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1];
lts_t = ifft(lts_f, 64); %time domain
preamble = [lts_t(33:64) lts_t lts_t];


% Define the pilot tone values as BPSK symbols
pilots = [1 1 -1 1].';

% Repeat the pilots across all OFDM symbols
pilots_mat = repmat(pilots, 1, N_OFDM_SYM);

%load(tx_vec_iris_mat_file) % loads tx_data, tx_syms, tx_syms_mat
load(tx_vec_iris_mat_file, 'tx_data');
%load(tx_vec_iris_mat_file, 'tx_syms');
load(tx_vec_iris_mat_file, 'tx_syms_mat');
load(tx_vec_iris_mat_file, 'N_OFDM_SYM');
load(tx_vec_iris_mat_file, 'MOD_ORDER');

load(rx_vec_iris_mat_file, 'rx_vec_iris'); % loads rx_vec_iris

rx_data_mat = permute(rx_vec_iris, [2 3 1]);
n_f = size(rx_data_mat, 3);
N_BS_NODE = size(rx_data_mat, 1);
N_UE = 1;

bit_errs_mrc = zeros(n_f, 1);
aevms_mrc = zeros(n_f, 1);
snr_mrc = zeros(n_f, 1);

bit_errs_br = zeros(n_f, N_BS_NODE);
aevms_br = zeros(n_f, N_BS_NODE);
snr_br = zeros(n_f, N_BS_NODE);
elapsed = zeros(n_f, 1);

for n=1:n_f
    tic
    rx_vec_iris_n = squeeze(rx_data_mat(:, :, n));
    rx_vec_iris_n = rx_vec_iris_n.';
    %% Correlate for LTS

    a = 1;
    unos = ones(size(preamble.'))';
    data_len = (N_OFDM_SYM)*(N_SC +CP_LEN);
    rx_lts_mat = double.empty();
    payload_rx = zeros(data_len,N_BS_NODE);
%     payload_ind = int32.empty();
%     m_filt = zeros(length(rx_vec_iris),N_BS_NODE);
    lts_ind = 134;

    for ibs =1:N_BS_NODE
%             v0 = filter(flipud(preamble'),a,rx_vec_iris(:,ibs));
%             v1 = filter(unos,a,abs(rx_vec_iris(:,ibs)).^2);
%             m_filt(:,ibs) = (abs(v0).^2)./v1; % normalized correlation
%             [~, max_idx] = max(m_filt(:,ibs));
%             % In case of bad correlatons:
%             if (max_idx + data_len) > length(rx_vec_iris) || (max_idx < 0) || (max_idx - length(preamble) < 0)
%                 fprintf('Bad correlation at antenna %d max_idx = %d \n', ibs, max_idx);
%                 valid_it = false;
%                 % Real value doesn't matter since we have corrrupt data:
%                 max_idx = length(rx_vec_iris)-data_len -1;
%             end
%             payload_ind(ibs) = max_idx +1;
%             lts_ind = payload_ind(ibs) - length(preamble);

            
            %lts_ind = payload_ind(ibs) - length(preamble);
            rx_lts_mat(:,ibs) = rx_vec_iris_n(lts_ind: lts_ind + length(preamble) -1, ibs);
            payload_rx(1:data_len,ibs) = rx_vec_iris_n(lts_ind + length(preamble) : lts_ind + length(preamble) + data_len - 1, ibs);
    end

    % Extract LTS for channel estimate
    rx_lts_idx1 = -64+-FFT_OFFSET + (97:160);
    rx_lts_idx2 = -FFT_OFFSET + (97:160);
    % Just for two first brnaches: useful when 1x2 SIMO. Just to illustrate
    % improvement of MRC over two branches:

    % Channel Estimate of multiple branches:
    H_0_t = zeros(N_SC, N_LTS_SYM, N_BS_NODE);
    % Take N_SC samples from each rx_lts (we have sent two LTS)
    rx_lts_nsc = [rx_lts_mat(rx_lts_idx1,:); rx_lts_mat(rx_lts_idx2,:)];
    for ibs = 1:N_BS_NODE
        H_0_t(:,:,ibs) = reshape(rx_lts_nsc(:,ibs),[],N_LTS_SYM);
    end
    H_0_f = fft(H_0_t, N_SC, 1);
    H_0 =  H_0_f./ repmat(lts_f.',1,N_LTS_SYM,N_BS_NODE);

    rx_H_est_2d = squeeze(mean(H_0,2));
    rx_H_est_2d(SC_IND_NULL,:) = 0;


    %% Rx payload processing

    payload_mat = reshape(payload_rx, (N_SC+CP_LEN), N_OFDM_SYM, N_BS_NODE);

    % Remove the cyclic prefix, keeping FFT_OFFSET samples of CP (on average)
    payload_mat_noCP = payload_mat(CP_LEN-FFT_OFFSET+(1:N_SC), :,:);

    % Take the FFT
    syms_f_mat = fft(payload_mat_noCP, N_SC, 1);

    % Equalize MRC
    rx_H_est = reshape(rx_H_est_2d,N_SC,1,N_BS_NODE);       % Expand to a 3rd dimension to agree with the dimensions od syms_f_mat
    H_pow = sum(abs(conj(rx_H_est_2d).*rx_H_est_2d),2);
    H_pow = repmat(H_pow,1,N_OFDM_SYM);

    % Do yourselves: MRC equalization:
    syms_eq_mat_mrc =  sum( (repmat(conj(rx_H_est), 1, N_OFDM_SYM,1).* syms_f_mat), 3)./H_pow; % MRC equalization: combine The two branches and equalize. 

    %Equalize each branch separately

    syms_eq_mat_br = syms_f_mat ./ repmat(rx_H_est, 1, N_OFDM_SYM, 1);
    pilots_mat_br = repmat(pilots_mat, 1, 1, N_BS_NODE);


    if DO_APPLY_PHASE_ERR_CORRECTION
        % Extract the pilots and calculate per-symbol phase error
        pilots_f_mat_mrc = syms_eq_mat_mrc(SC_IND_PILOTS, :,:);
        pilots_f_mat_comp_mrc = pilots_f_mat_mrc.*pilots_mat;
        pilot_phase_err_mrc = angle(mean(pilots_f_mat_comp_mrc));
        
        pilots_f_mat_br = syms_eq_mat_br(SC_IND_PILOTS, :, :);
        pilots_f_mat_comp_br = pilots_f_mat_br.*pilots_mat_br;
        pilot_phase_err_br = angle(mean(pilots_f_mat_comp_br, 1));

    else
        % Define an empty phase correction vector (used by plotting code below)
        pilot_phase_err_mrc = zeros(1, N_OFDM_SYM);
        pilot_phase_err_br = zeros(1, N_OFDM_SYM, N_BS_NODE);

    end
    pilot_phase_err_corr_mrc = repmat(pilot_phase_err_mrc, N_SC, 1);
    pilot_phase_corr_mrc = exp(-1i*(pilot_phase_err_corr_mrc));
    pilot_phase_err_corr_br = repmat(pilot_phase_err_br, N_SC, 1, 1);
    pilot_phase_corr_br = exp(-1i*(pilot_phase_err_corr_br));


    % Apply the pilot phase correction per symbol
    syms_eq_pc_mat_mrc = syms_eq_mat_mrc .* pilot_phase_corr_mrc;
    payload_syms_mat_mrc = syms_eq_pc_mat_mrc(SC_IND_DATA, :);

    syms_eq_pc_mat_br = syms_eq_mat_br .* pilot_phase_corr_br;
    payload_syms_mat_br = syms_eq_pc_mat_br(SC_IND_DATA, :, :);
    
    %% Demodulate
    rx_syms_mrc = reshape(payload_syms_mat_mrc, 1, N_DATA_SYMS);
    rx_syms_br = reshape(payload_syms_mat_br, N_DATA_SYMS, N_BS_NODE);

    rx_data_mrc = demod_sym(rx_syms_mrc, MOD_ORDER);
    rx_data_br = demod_sym(rx_syms_br, MOD_ORDER);

    bit_errs_mrc(n) = length(find(dec2bin(bitxor(tx_data, rx_data_mrc),8) == '1'));
    bit_errs_br(n, :) = sum(length(find(dec2bin(bitxor(repmat(tx_data.', 1, N_BS_NODE), rx_data_br),8) == '1')));
    
    evm_mat_mrc = abs(payload_syms_mat_mrc - tx_syms_mat).^2;
    aevms_mrc(n) = mean(evm_mat_mrc(:));
    snr_mrc(n) = 10*log10(1./aevms_mrc(n));
    
    evm_mat_br = abs(payload_syms_mat_br - repmat(tx_syms_mat, 1, 1, N_BS_NODE)).^2;
    aevms_br(n, :) = mean(reshape(evm_mat_br, numel(tx_syms_mat), N_BS_NODE), 1);
    snr_br(n, :) = 10*log10(1./aevms_br(n, :));

    elapsed(n) = toc;
    fprintf('Frame %d processed in %2.2f\n', n, elapsed(n));
end
fprintf('Finished processing data in %5.2f\n', sum(elapsed));

end
