close all;
max_frame = 2500;
max_mrc_sample = 10;
mrc_subset = [1, 2, 4, 8, 16];
process_branch = 0;
cfo_ppm = [0 0.02 0.04 0.06 0.08 0.1];
sample_rate = 5e6;
f_c = 3.6e9;
cfo = (cfo_ppm * (f_c / 1e6)) / sample_rate;
cfo_mat = randn(32, length(cfo_ppm)) .* repmat(cfo, 32, 1);

load('good_16QAM_1x32_UE1.mat');
n_fr = length(good);

tx_vec_iris_mat_file = 'ds_16QAM_tx_signal.mat';

load(tx_vec_iris_mat_file, 'tx_data');
%load(tx_vec_iris_mat_file, 'tx_syms');
load(tx_vec_iris_mat_file, 'tx_syms_mat');
load(tx_vec_iris_mat_file, 'N_OFDM_SYM');
load(tx_vec_iris_mat_file, 'MOD_ORDER');

rx_vec_iris_mat_file = 'ds_16QAM_rx_data_1x32_UE1.mat';
load(rx_vec_iris_mat_file, 'rx_vec_iris'); % loads rx_vec_iris

%%%%%%%%%%%% 16QAM %%%%%%%%%%%%%

aevms_mrc = zeros(n_fr, length(mrc_subset) + 1, length(cfo_ppm));
SNR_MRC_MEAN_16QAM = zeros(length(cfo_ppm), length(mrc_subset) + 1);
for c = 1:length(cfo_ppm)
    tic
    [~, aevms_mrc_full, ~, ~]=process_simo(tx_data, tx_syms_mat, N_OFDM_SYM, MOD_ORDER, rx_vec_iris, [32], max_frame, process_branch, cfo_mat(:, c));

    aevms_mrc(:, length(mrc_subset) + 1, c) = aevms_mrc_full(good);
    %snr_mrc = 10*log10(1./aevms_mrc);
    %snr_mrc=-(3.7 + 20*log10(aevms_mrc));
    %good = find(snr_mrc > 10);

    snr_mrc = 10*log10(1./aevms_mrc);

    %snr_br_mean = 10*log10(1./mean(aevms_br(good, :), 2));
    %snr_br_max = 10*log10(1./min(aevms_br(good, :), [], 2));

    aevms_mrc_ss_mat = zeros(max_frame, length(mrc_subset), max_mrc_sample);
    for i = 1:max_mrc_sample
        %fprintf('Partial MRC Try %d', i);
        [~, aevms_mrc_ss_mat(:, :, i), ~, ~] = process_simo(tx_data, tx_syms_mat, N_OFDM_SYM, MOD_ORDER, rx_vec_iris, mrc_subset, max_frame, process_branch, cfo_mat(:, c));
    end

    aevms_mrc(:, 1:length(mrc_subset), c) = mean(aevms_mrc_ss_mat(good, :, :), 3);

    snr_mrc_ss = 10*log10(1./aevms_mrc(:, :, c));
    %snr_mrc=-(3.7 + 20*log10(aevms_mrc));

    figure;
    plot(snr_mrc_ss(:, 1))
    hold on
    plot(snr_mrc_ss(:, 2))
    hold on
    plot(snr_mrc_ss(:, 3))
    hold on
    plot(snr_mrc_ss(:, 4))
    hold on
    plot(snr_mrc_ss(:, 5))
    hold on
    plot(snr_mrc_ss(:, 6))
    legend('1x1', '1x2', '1x4', '1x8', '1x16', '1x32')
    title('16QAM')

    fprintf('Finished CFO %d/%d in %5.2f\n', c, length(cfo_ppm), toc);
    SNR_MRC_SS_16QAM = [10*log10(mean(10.^(snr_mrc_ss(:, 1)/10))) 10*log10(mean(10.^(snr_mrc_ss(:, 2)/10))) 10*log10(mean(10.^(snr_mrc_ss(:, 3)/10))) 10*log10(mean(10.^(snr_mrc_ss(:, 4)/10))) 10*log10(mean(10.^(snr_mrc_ss(:, 5)/10)))  10*log10(mean(10.^(snr_mrc_ss(:, 6)/10)))];

    SNR_MRC_MEAN_16QAM(c, :) = SNR_MRC_SS_16QAM;

end

GAIN_MRC_16QAM = SNR_MRC_MEAN_16QAM(:, 2:end) - repmat(SNR_MRC_MEAN_16QAM(:, 1), 1, length(mrc_subset));
figure;plot([0 20 40 60 80 100], GAIN_MRC_16QAM, 'LineWidth',2)
set(gca,'FontSize',14);
% xticks([0 20 40 60 80 100])
% xticklabels({'0', '20', '40', '60', '80', '100'})
xlabel('CFO Standard Deviation (ppb)')
ylabel('Beamforming Gain')
legend('1x2', '1x4', '1x8', '1x16', '1x32')
title('MRC - 16QAM')

save('aevms_mrc_16qam.mat', aevms_mrc);
save('snr_mrc_mean_16qam.mat', SNR_MRC_MEAN_16QAM);

