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
N_END_ZERO_PAD          = 100;

% Massive-MIMO params
N_UE                    = 4;
N_BS_ANT                = 64;               % N_BS_ANT >> N_UE
N_UPLINK_SYMBOLS        = N_OFDM_SYMS;
N_0                     = 1e-2;
H_var                   = 1;

DO_SAVE_RX_DATA = 0;
DO_APPLY_HW_IMPERFECTION = 1;
DO_RECIPROCAL_CALIBRATION = 1;

% LTS for CFO and channel estimation
lts_f = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1];
lts_t = ifft(lts_f, 64);

%% Modulation & Demodulation functions
% Functions for data -> complex symbol mapping (like qammod, avoids comm toolbox requirement)
% These anonymous functions implement the modulation mapping from IEEE 802.11-2012 Section 18.3.5.8

modvec_bpsk   =  (1/sqrt(2))  .* [-1 1];
modvec_16qam  =  (1/sqrt(10)) .* [-3 -1 +3 +1];
modvec_64qam  =  (1/sqrt(43)) .* [-7 -5 -1 -3 +7 +5 +1 +3];

mod_fcn_bpsk  = @(x) complex(modvec_bpsk(1+x),0);
mod_fcn_qpsk  = @(x) complex(modvec_bpsk(1+bitshift(x, -1)), modvec_bpsk(1+mod(x, 2)));
mod_fcn_16qam = @(x) complex(modvec_16qam(1+bitshift(x, -2)), modvec_16qam(1+mod(x,4)));
mod_fcn_64qam = @(x) complex(modvec_64qam(1+bitshift(x, -3)), modvec_64qam(1+mod(x,8)));

demod_fcn_bpsk = @(x) double(real(x)>0);
demod_fcn_qpsk = @(x) double(2*(real(x)>0) + 1*(imag(x)>0));
demod_fcn_16qam = @(x) (8*(real(x)>0)) + (4*(abs(real(x))<0.6325)) + (2*(imag(x)>0)) + (1*(abs(imag(x))<0.6325));
demod_fcn_64qam = @(x) (32*(real(x)>0)) + (16*(abs(real(x))<0.6172)) + (8*((abs(real(x))<(0.9258))&&((abs(real(x))>(0.3086))))) + (4*(imag(x)>0)) + (2*(abs(imag(x))<0.6172)) + (1*((abs(imag(x))<(0.9258))&&((abs(imag(x))>(0.3086)))));

%% Generate mMIMO tranceivers random phase vector [-pi, pi]
if DO_APPLY_HW_IMPERFECTION
    rng('shuffle');
    dl_tx_hw_phase = exp((2 * pi * repmat(rand(N_BS_ANT, 1), 1, N_SC) - pi) * 1j);
    dl_rx_hw_phase = exp((2 * pi * repmat(rand(N_UE, 1), 1, N_SC) - pi) * 1j);
    ul_tx_hw_phase = exp((2 * pi * repmat(rand(N_UE, 1), 1, N_SC) - pi) * 1j);
    ul_rx_hw_phase = exp((2 * pi * repmat(rand(N_BS_ANT, 1), 1, N_SC) - pi) * 1j);

else
    dl_tx_hw_phase = ones(N_BS_ANT, N_SC);
    ul_rx_hw_phase = ones(N_BS_ANT, N_SC);
    dl_rx_hw_phase = ones(N_UE, N_SC);
    ul_tx_hw_phase = ones(N_UE, N_SC);
end

%% BS Calibration
calib_mat = ones(N_BS_ANT, N_SC);
if DO_RECIPROCAL_CALIBRATION
    tx_ref_hw_phase = exp((2 * pi * rand - pi) * 1j);
    rx_ref_hw_phase = exp((2 * pi * rand - pi) * 1j);

    ul_tx_calib = ifft(lts_f * tx_ref_hw_phase);
    H_ref = sqrt(H_var / 2) .* (randn(N_BS_ANT, 1) + 1i*randn(N_BS_ANT, 1));
    Z_mat = sqrt(1e-4 / 2) * (randn(N_BS_ANT, length(ul_tx_calib)) + 1i*randn(N_BS_ANT, length(ul_tx_calib)));
    ul_rx_calib = H_ref * ul_tx_calib + Z_mat;
    ul_rx_calib_fft = fft(ul_rx_calib, N_SC, 2) .* ul_rx_hw_phase;
    h_ul_calib = ul_rx_calib_fft .* repmat(lts_f, N_BS_ANT, 1);

    dl_tx_calib = ifft(repmat(lts_f, N_BS_ANT, 1) .* dl_tx_hw_phase, N_SC, 2);
    Z_mat = sqrt(1e-4 / 2) * (randn(N_BS_ANT, length(dl_tx_calib)) + 1i*randn(N_BS_ANT, length(dl_tx_calib)));
    dl_rx_calib = repmat(H_ref, 1, N_SC) .* dl_tx_calib + Z_mat;
    dl_rx_calib_fft = fft(dl_rx_calib, N_SC, 2) * rx_ref_hw_phase;
    h_dl_calib = dl_rx_calib_fft .* repmat(lts_f, N_BS_ANT, 1);

    calib_mat(:, SC_IND_DATA) = h_dl_calib(:, SC_IND_DATA) ./ h_ul_calib(:, SC_IND_DATA);
end

%% Uplink

% Generate a payload of random integers
tx_ul_data = randi(MOD_ORDER, N_UE, N_DATA_SYMS) - 1;

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
tx_ul_syms_mat = reshape(tx_ul_syms, N_UE, length(SC_IND_DATA), N_OFDM_SYMS);

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
ifft_in_mat(:, SC_IND_DATA, :) = tx_ul_syms_mat;
ifft_in_mat(:, SC_IND_PILOTS, :) = pt_pilots_mat;

% Apply hardware phase distortion
ifft_in_hw_mat = ifft_in_mat .* repmat(ul_tx_hw_phase, 1, 1, N_OFDM_SYMS);

%Perform the IFFT
tx_payload_mat = ifft(ifft_in_hw_mat, N_SC, 2);

% Insert the cyclic prefix
if(CP_LEN > 0)
    tx_cp = tx_payload_mat(:, (end-CP_LEN+1 : end), :);
    tx_payload_mat = cat(2, tx_cp, tx_payload_mat); %[tx_cp; tx_payload_mat];
end

% Reshape to a vector
tx_payload_vec = reshape(tx_payload_mat, N_UE, numel(tx_payload_mat(1,:,:)));
tx_pilot_vec = zeros(N_UE, SYM_LEN * (N_UE+1)); % additional pilot as noise
for i=1:N_UE
    lts_t = ifft(lts_f .* ul_tx_hw_phase(i, :), 64);
    tx_pilot_vec(i, (i-1)*SYM_LEN+1:i*SYM_LEN) = [lts_t(64-CP_LEN+1:64) lts_t];
end

% Construct the full time-domain OFDM waveform
tx_vec = [tx_pilot_vec tx_payload_vec];
tx_vec_air = TX_SCALE .* tx_vec ./ repmat(max(abs(tx_vec),[],2), 1, size(tx_vec, 2));

% Rayleight + AWGN:

rng('shuffle');

Z_mat = sqrt(N_0/2) * ( randn(N_BS_ANT,length(tx_vec_air) ) + 1i*randn(N_BS_ANT,length(tx_vec_air) ) );     % UL noise matrix
H = sqrt(H_var/2) .* ( randn(N_BS_ANT, N_UE) + 1i*randn(N_BS_ANT, N_UE) );                                  % Spatial Channel Matrix

rx_vec_air = H * tx_vec_air + Z_mat;
%rx_vec_air = rx_vec_air./repmat(max(abs(rx_vec_air'))', 1, size(rx_vec_air, 2));

rx_pilot_vec = zeros(N_BS_ANT, N_SC, N_UE);
for i=1:N_UE
    rx_pilot_vec(:, :, i) = rx_vec_air(:, (i-1)*SYM_LEN+CP_LEN+1:i*SYM_LEN);
end

lts_f_mat = zeros(N_BS_ANT, N_SC, N_UE);
for i = 1:N_UE
    lts_f_mat(:, :, i) = repmat(lts_f, N_BS_ANT, 1);
end
csi_mat = fft(rx_pilot_vec, N_SC, 2) .* repmat(ul_rx_hw_phase, 1, 1, N_UE) .* lts_f_mat;

rx_payload_vec=rx_vec_air(:, (N_UE+1)*SYM_LEN+1:end);
rx_payload_mat = reshape(rx_payload_vec, N_BS_ANT, SYM_LEN, N_OFDM_SYMS); % first two are preamble
rx_payload_mat_noCP = rx_payload_mat(:, CP_LEN+1:end, :);
fft_out_mat = fft(rx_payload_mat_noCP, N_SC, 2)  .* repmat(ul_rx_hw_phase, 1, 1, N_OFDM_SYMS);

precoding_mat = zeros(N_BS_ANT, N_SC, N_UE);
demult_mat = zeros(N_UE, N_SC, N_OFDM_SYMS);
sc_csi_mat = zeros(N_BS_ANT, N_UE);
for j=1:N_SC
    sc_csi_mat = squeeze(csi_mat(:, j, :));
    zf_mat = pinv(sc_csi_mat);
    demult_mat(:, j, :) = zf_mat * squeeze(fft_out_mat(:, j, :));
    dl_zf_mat = pinv(diag(calib_mat(:, i)) * sc_csi_mat);
    precoding_mat(:, j, :) = dl_zf_mat.'; %zf_mat.';
end

pilots_f_mat = demult_mat(:, SC_IND_PILOTS, :);
pilots_f_mat_comp = pilots_f_mat .* pt_pilots_mat;
pilot_phase_err = squeeze(angle(mean(pilots_f_mat_comp, 2)));

pilot_phase_corr = zeros(N_UE, N_SC, N_OFDM_SYMS);
for i=1:N_SC
    pilot_phase_corr(:,i,:) = exp(-1i*pilot_phase_err);
end
  
% Apply the pilot phase correction per symbol
demult_pc_mat = demult_mat.* pilot_phase_corr;
payload_syms_mat = demult_pc_mat(:, SC_IND_DATA, :);
payload_syms_mat = reshape(payload_syms_mat, N_UE, numel(payload_syms_mat(1,:,:)));

%% Demodulate uplink
rx_ul_syms = payload_syms_mat;
rx_ul_data = zeros(size(rx_ul_syms));

for n_ue = 1:N_UE
    switch(MOD_ORDER)
        case 2         % BPSK
            rx_data_temp = arrayfun(demod_fcn_bpsk, rx_ul_syms(n_ue,:) );
        case 4         % QPSK
            rx_data_temp = arrayfun(demod_fcn_qpsk, rx_ul_syms(n_ue,:));
        case 16        % 16-QAM
            rx_data_temp = arrayfun(demod_fcn_16qam, rx_ul_syms(n_ue,:));
        case 64        % 64-QAM
            rx_data_temp = arrayfun(demod_fcn_64qam, rx_ul_syms(n_ue,:));
    end

    rx_ul_data(n_ue, :) = complex( rx_data_temp );
end

%% Calculate UL Rx stats

ul_sym_errs = sum(sum(tx_ul_data ~= rx_ul_data)); % errors per user
ul_bit_errs = length(find(dec2bin(bitxor(tx_ul_data, rx_ul_data), 8) == '1'));
%rx_evm   = sqrt(sum((real(rx_syms) - real(tx_syms)).^2 + (imag(rx_syms) - imag(tx_syms)).^2)/(length(SC_IND_DATA) * N_OFDM_SYMS));

tx_ul_syms_vecs = reshape(tx_ul_syms_mat, N_UE, numel(tx_ul_syms_mat(1, :, :)));
ul_evm_mat = abs(payload_syms_mat - tx_ul_syms_vecs).^2;
ul_aevms = mean(ul_evm_mat, 2);
ul_snrs = 10*log10(1 ./ ul_aevms);


%% Downlink

tx_dl_data = tx_ul_data; % use same data for downlink as uplink
tx_dl_syms = tx_ul_syms; % use same data symbols for downlink as uplink
% Reshape the symbol vector to a matrix with one column per OFDM symbol
tx_dl_syms_mat = reshape(tx_dl_syms, N_UE, length(SC_IND_DATA), N_OFDM_SYMS);

tx_mult_mat = zeros(N_BS_ANT, N_SC, N_OFDM_SYMS + 2);
for i=1:N_SC
    lts_f_vec = lts_f(i)*ones(N_UE, 1);
    tx_mult_f = [lts_f_vec lts_f_vec squeeze(ifft_in_mat(:, i, :))];
    tx_mult_mat(:, i, :) = squeeze(precoding_mat(:, i, :)) * tx_mult_f; % N_BS_ANT * N_SC * N_OFDM_SYMS
end

tx_mult_hw_mat = tx_mult_mat .* repmat(dl_tx_hw_phase, 1, 1, N_OFDM_SYMS + 2);
ifft_out_mat = ifft(tx_mult_hw_mat, N_SC, 2);

% Insert the cyclic prefix
if(CP_LEN > 0)
    tx_cp = ifft_out_mat(:, (end - CP_LEN + 1:end), :);
    tx_dl_mat = cat(2, tx_cp, ifft_out_mat); %[tx_cp; tx_payload_mat];
else
    tx_dl_mat = ifft_out_mat;
end

tx_dl_vec = reshape(tx_dl_mat, N_BS_ANT, numel(tx_dl_mat(1, :, :)));
tx_dl_vec = TX_SCALE .* tx_dl_vec ./ repmat(max(abs(tx_dl_vec), [], 2), 1, length(tx_dl_vec));

Z_dl_mat = sqrt(N_0/2) * (randn(N_UE,length(tx_dl_vec)) + 1i*randn(N_UE,length(tx_dl_vec))); % DL noise matrix
rx_dl_vec = (1/(sqrt(N_BS_ANT))) .* H.'*tx_dl_vec + Z_dl_mat;

rx_dl_mat = reshape(rx_dl_vec, N_UE, SYM_LEN, N_OFDM_SYMS + 2);
if(CP_LEN > 0)
    rx_dl_mat = rx_dl_mat(:, CP_LEN+1:end, :);
end

rx_dl_f_mat = fft(rx_dl_mat, N_SC, 2) .* repmat(dl_rx_hw_phase, 1, 1, N_OFDM_SYMS + 2);
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

%% Demodulate downlink
rx_dl_syms = payload_dl_syms_mat;
rx_dl_data = zeros(size(rx_dl_syms));

for n_ue = 1:N_UE
    switch(MOD_ORDER)
        case 2         % BPSK
            rx_data_temp = arrayfun(demod_fcn_bpsk, rx_dl_syms(n_ue,:) );
        case 4         % QPSK
            rx_data_temp = arrayfun(demod_fcn_qpsk, rx_dl_syms(n_ue,:));
        case 16        % 16-QAM
            rx_data_temp = arrayfun(demod_fcn_16qam, rx_dl_syms(n_ue,:));
        case 64        % 64-QAM
            rx_data_temp = arrayfun(demod_fcn_64qam, rx_dl_syms(n_ue,:));
    end
    
    rx_dl_data(n_ue, :) = complex( rx_data_temp );
end

%% Calculate DL Rx stats

dl_sym_errs = sum(sum(tx_dl_data ~= rx_dl_data)); % errors per user
dl_bit_errs = length(find(dec2bin(bitxor(tx_dl_data, rx_dl_data), 8) == '1'));

tx_dl_syms_vecs = reshape(tx_dl_syms_mat, N_UE, numel(tx_dl_syms_mat(1, :, :)));
dl_evm_mat = abs(payload_dl_syms_mat - tx_dl_syms_vecs).^2;
dl_aevms = mean(dl_evm_mat, 2);
dl_snrs = 10*log10(1 ./ dl_aevms);

%% Plots:
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

plot(tx_dl_syms(1, :),'bo');
title('Downlink Tx and Rx Constellations')
legend('Rx','Tx');


subplot(2,2,2)
plot(payload_dl_syms_mat(2, :),'ro','MarkerSize',1);
axis square; axis(1.5*[-1 1 -1 1]);
grid on;
hold on;

plot(tx_dl_syms(2, :),'bo');
legend('Rx','Tx');


subplot(2,2,3)
plot(payload_dl_syms_mat(3, :),'ro','MarkerSize',1);
axis square; axis(1.5*[-1 1 -1 1]);
grid on;
hold on;

plot(tx_dl_syms(3, :),'bo');
legend('Rx','Tx');


subplot(2,2,4)
plot(payload_dl_syms_mat(4, :),'ro','MarkerSize',1);
axis square; axis(1.5*[-1 1 -1 1]);
grid on;
hold on;

plot(tx_dl_syms(4, :),'bo');
legend('Rx','Tx');

% EVM & SNR UL
cf = cf + 1;
figure(cf); clf;


subplot(2,2,1)
plot(100*ul_evm_mat(1,:),'o','MarkerSize',1)
axis tight
hold on
plot([1 length(ul_evm_mat(1,:))], 100*[ul_aevms(1,:), ul_aevms(1,:)],'r','LineWidth',2)
title('Downlink Rx Stats')
myAxis = axis;
h = text(round(.05*length(ul_evm_mat(1,:))), 100*ul_aevms(1,:)+ .1*(myAxis(4)-myAxis(3)), sprintf('Effective SNR: %.1f dB', ul_snrs(1,:)));
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
plot(100*ul_evm_mat(2,:),'o','MarkerSize',1)
axis tight
hold on
plot([1 length(ul_evm_mat(2,:))], 100*[ul_aevms(2,:), ul_aevms(2,:)],'r','LineWidth',2)
myAxis = axis;
h = text(round(.05*length(ul_evm_mat(2,:))), 100*ul_aevms(2,:)+ .1*(myAxis(4)-myAxis(3)), sprintf('Effective SNR: %.1f dB', ul_snrs(2,:)));
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
plot(100*ul_evm_mat(3,:),'o','MarkerSize',1)
axis tight
hold on
plot([1 length(ul_evm_mat(3,:))], 100*[ul_aevms(3,:), ul_aevms(3,:)],'r','LineWidth',2)
myAxis = axis;
h = text(round(.05*length(ul_evm_mat(3,:))), 100*ul_aevms(3,:)+ .1*(myAxis(4)-myAxis(3)), sprintf('Effective SNR: %.1f dB', ul_snrs(3,:)));
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
plot(100*ul_evm_mat(4,:),'o','MarkerSize',1)
axis tight
hold on
plot([1 length(ul_evm_mat(4,:))], 100*[ul_aevms(4,:), ul_aevms(4,:)],'r','LineWidth',2)
myAxis = axis;
h = text(round(.05*length(ul_evm_mat(4,:))), 100*ul_aevms(4,:)+ .1*(myAxis(4)-myAxis(3)), sprintf('Effective SNR: %.1f dB', ul_snrs(4,:)));
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

% EVM & SNR DL
cf = cf + 1;
figure(cf); clf;


subplot(2,2,1)
plot(100*dl_evm_mat(1,:),'o','MarkerSize',1)
axis tight
hold on
plot([1 length(dl_evm_mat(1,:))], 100*[dl_aevms(1,:), dl_aevms(1,:)],'r','LineWidth',2)
title('Downlink Rx Stats')
myAxis = axis;
h = text(round(.05*length(dl_evm_mat(1,:))), 100*dl_aevms(1,:)+ .1*(myAxis(4)-myAxis(3)), sprintf('Effective SNR: %.1f dB', dl_snrs(1,:)));
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
plot(100*dl_evm_mat(2,:),'o','MarkerSize',1)
axis tight
hold on
plot([1 length(dl_evm_mat(2,:))], 100*[dl_aevms(2,:), dl_aevms(2,:)],'r','LineWidth',2)
myAxis = axis;
h = text(round(.05*length(dl_evm_mat(2,:))), 100*dl_aevms(2,:)+ .1*(myAxis(4)-myAxis(3)), sprintf('Effective SNR: %.1f dB', dl_snrs(2,:)));
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
plot(100*dl_evm_mat(3,:),'o','MarkerSize',1)
axis tight
hold on
plot([1 length(dl_evm_mat(3,:))], 100*[dl_aevms(3,:), dl_aevms(3,:)],'r','LineWidth',2)
myAxis = axis;
h = text(round(.05*length(dl_evm_mat(3,:))), 100*dl_aevms(3,:)+ .1*(myAxis(4)-myAxis(3)), sprintf('Effective SNR: %.1f dB', dl_snrs(3,:)));
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
plot(100*dl_evm_mat(4,:),'o','MarkerSize',1)
axis tight
hold on
plot([1 length(dl_evm_mat(4,:))], 100*[dl_aevms(4,:), dl_aevms(4,:)],'r','LineWidth',2)
myAxis = axis;
h = text(round(.05*length(dl_evm_mat(4,:))), 100*dl_aevms(4,:)+ .1*(myAxis(4)-myAxis(3)), sprintf('Effective SNR: %.1f dB', dl_snrs(4,:)));
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


fprintf('\nUL Results:\n');
fprintf('===== SNRs: =====\n');

for n_ue = 1:N_UE
    fprintf('UL SNR of user %d :   %f\n', n_ue , ul_snrs(n_ue));
end


fprintf('\n===== Errors: =====\n');
fprintf('Num Bits:   %d\n', N_UE * N_DATA_SYMS * log2(MOD_ORDER) );
fprintf('UL Sym Errors:  %d (of %d total symbols)\n', ul_sym_errs, N_UE * N_DATA_SYMS);
fprintf('UL Bit Errors:  %d (of %d total bits)\n', ul_bit_errs, N_UE * N_DATA_SYMS * log2(MOD_ORDER));

fprintf('\n\nDL Results:\n');
fprintf('===== SNRs: =====\n');

for n_ue = 1:N_UE
    fprintf('DL SNR of user %d :   %f\n', n_ue , dl_snrs(n_ue));
end

fprintf('\n===== Errors: =====\n');
fprintf('Num Bits:   %d\n', N_UE * N_DATA_SYMS * log2(MOD_ORDER) );
fprintf('DL Sym Errors:  %d (of %d total symbols)\n', dl_sym_errs, N_UE * N_DATA_SYMS);
fprintf('DL Bit Errors:  %d (of %d total bits)\n', dl_bit_errs, N_UE * N_DATA_SYMS * log2(MOD_ORDER));

if DO_SAVE_RX_DATA
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

