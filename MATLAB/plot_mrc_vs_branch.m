close all;
max_frame = 2500;
process_branch = 1;
max_mrc_sample = 10;
tx_vec_iris_mat_file = 'ds_16QAM_tx_signal.mat';

load(tx_vec_iris_mat_file, 'tx_data');
%load(tx_vec_iris_mat_file, 'tx_syms');
load(tx_vec_iris_mat_file, 'tx_syms_mat');
load(tx_vec_iris_mat_file, 'N_OFDM_SYM');
load(tx_vec_iris_mat_file, 'MOD_ORDER');

rx_vec_iris_mat_file = 'ds_64QAM_rx_data_1x32_UE1.mat';
load(rx_vec_iris_mat_file, 'rx_vec_iris'); % loads rx_vec_iris

rx_data_mat = permute(rx_vec_iris, [2 3 1]);
max_f = size(rx_data_mat, 1);
N_BS_NODE = size(rx_data_mat, 2);
n_samps = size(rx_data_mat, 3);
mrc_subset =  2.^(0:1:floor(log2(N_BS_NODE)));
if 2^floor(log2(N_BS_NODE)) == N_BS_NODE
    mrc_subset = mrc_subset(1:end-1);
end

aevms_mrc = zeros(n_fr, length(mrc_subset) + 1);
[~, aevms_mrc_full, ~, aevms_br]=process_simo(tx_data, tx_syms_mat, N_OFDM_SYM, MOD_ORDER, rx_vec_iris, [N_BS_NODE], max_frame, process_branch, zeros(1, N_BS_NODE));

aevms_mrc(:, length(mrc_subset) + 1) = aevms_mrc_full(good);
%snr_mrc = 10*log10(1./aevms_mrc);
%snr_mrc=-(3.7 + 20*log10(aevms_mrc));
%good = find(snr_mrc > 10);

snr_mrc_tmp = 10*log10(1./aevms_mrc(:, length(mrc_subset) + 1));
good = find(snr_mrc_tmp > 10);
save('good_64QAM_1x32_UE1', good);

aevms_mrc_ss_mat = zeros(max_frame, length(mrc_subset), max_mrc_sample);
for i = 1:max_mrc_sample
    %fprintf('Partial MRC Try %d', i);
    [~, aevms_mrc_ss_mat(:, :, i), ~, ~] = process_simo(tx_data, tx_syms_mat, N_OFDM_SYM, MOD_ORDER, rx_vec_iris, mrc_subset, max_frame, process_branch, cfo_mat(:, c));
end

aevms_mrc(:, 1:length(mrc_subset)) = mean(aevms_mrc_ss_mat(good, :, :), 3);

snr_mrc_ss = 10*log10(1./aevms_mrc);
%snr_mrc=-(3.7 + 20*log10(aevms_mrc));

figure;
for i=1:length(mrc_subset)
    SNR_MRC_SS_16QAM(i) = 10*log10(mean(10.^(snr_mrc_ss(:, i)/10)));
plot(snr_mrc_ss(:, i))
if i < length(mrc_subset)
    hold on
end
end
legend('1x1', '1x2', '1x4', '1x8', '1x16', '1x32')

SNR_MRC_SS_16QAM