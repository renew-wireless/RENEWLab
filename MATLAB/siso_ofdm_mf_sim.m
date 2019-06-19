%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% siso_ofdm_mf_sim.m
% A sane example of Simulated OFDM sysetm
% Copyright (c) 2015 Mango Communications - All Rights Reserved
% Distributed under the WARP License (http://warpproject.org/license)
% Author: C. Nicolas Barati. email: nicobarti@rice.edu
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
close all;
% Params:
WRITE_PNG_FILES         = 0;           % Enable writing plots to PNG
CHANNEL                 = 11;          % Channel to tune Tx and Rx radios
SIM_snr_db              = 30;          % simulation data SC SNR

% Waveform params
N_OFDM_SYMS             = 320;         % Number of OFDM symbols
MOD_ORDER               = 16;           % Modulation order (2/4/16/64 = BSPK/QPSK/16-QAM/64-QAM)
TX_SCALE                = 1.0;          % Scale for Tx waveform ([0:1])

% OFDM params
SC_IND_PILOTS           = [8 22 44 58];                           % Pilot subcarrier indices
SC_IND_DATA             = [2:7 9:21 23:27 39:43 45:57 59:64];     % Data subcarrier indices
N_SC                    = 64;                                     % Number of subcarriers
CP_LEN                  = 16;                                     % Cyclic prefix length
N_DATA_SYMS             = N_OFDM_SYMS * length(SC_IND_DATA);      % Number of data symbols (one per data-bearing subcarrier per OFDM symbol)
N_LTS                   = 2;

% Rx processing params
FFT_OFFSET                    = 16;          % Number of CP samples to use in FFT (on average)
LTS_CORR_THRESH               = 0.8;         % Normalized threshold for LTS correlation
DO_APPLY_CFO_CORRECTION       = 0;           % Enable CFO estimation/correction
DO_APPLY_SFO_CORRECTION       = 0;           % Enable SFO estimation/correction
DO_APPLY_PHASE_ERR_CORRECTION = 0;           % Enable Residual CFO estimation/correction

% WARPLab experiment params
USE_AGC                 = true;        % Use the AGC if running on WARP hardware
MAX_TX_LEN              = 2^20;        % Maximum number of samples to use for this experiment
TRIGGER_OFFSET_TOL_NS   = 3000;        % Trigger time offset toleration between Tx and Rx that can be accomodated


% Use sane defaults for hardware-dependent params in sim-only version
maximum_buffer_len = min(MAX_TX_LEN, 2^20);
SAMP_FREQ           = 40e6;
example_mode_string = 'sim';


%% Define a half-band 2x interpolation filter response
% Define the preamble
% LTS for CFO and channel estimation
lts_f = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1];


%% Generate a payload of random integers
tx_data = randi(MOD_ORDER, 1, N_DATA_SYMS) - 1;

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
        tx_syms = arrayfun(mod_fcn_bpsk, tx_data);
    case 4         % QPSK
        tx_syms = arrayfun(mod_fcn_qpsk, tx_data);
    case 16        % 16-QAM
        tx_syms = arrayfun(mod_fcn_16qam, tx_data);
    case 64        % 64-QAM
        tx_syms = arrayfun(mod_fcn_64qam, tx_data);
    otherwise
        fprintf('Invalid MOD_ORDER (%d)!  Must be in [2, 4, 16, 64]\n', MOD_ORDER);
        return;
end

% Reshape the symbol vector to a matrix with one column per OFDM symbol
tx_syms_mat = reshape(tx_syms, length(SC_IND_DATA), N_OFDM_SYMS);

% Define the pilot tone values as BPSK symbols
pilots = [1 1 -1 1].';

% Repeat the pilots across all OFDM symbols
pilots_mat = repmat(pilots, 1, N_OFDM_SYMS);

%% IFFT

% Construct the IFFT input matrix
ifft_in_mat_pl = zeros(N_SC, N_OFDM_SYMS);

% Insert the data and pilot values; other subcarriers will remain at 0
ifft_in_mat_pl(SC_IND_DATA, :)   = tx_syms_mat;
ifft_in_mat_pl(SC_IND_PILOTS, :) = pilots_mat;

ifft_in_mat = [repmat(lts_f.', 1, N_LTS) ifft_in_mat_pl];
%Perform the IFFT
tx_payload_mat = ifft(ifft_in_mat, N_SC, 1);

% Insert the cyclic prefix
if(CP_LEN > 0)
    tx_cp = tx_payload_mat((end-CP_LEN+1 : end), :);
    tx_payload_mat = [tx_cp; tx_payload_mat];
end
lts_t = tx_payload_mat(:,1:N_LTS);
lts_t = lts_t(:);
% Reshape to a vector
tx_payload_vec = reshape(tx_payload_mat, 1, numel(tx_payload_mat));

% Construct the full time-domain OFDM waveform
tx_vec = tx_payload_vec;

%Calculate the noise variance 
SIM_snr =10^(0.1* SIM_snr_db);
tx_vec_pow_avg = mean( abs(TX_SCALE .* tx_vec ./ max(abs(tx_vec))).^2);    % Average power on each SC
sim_N0 = tx_vec_pow_avg / SIM_snr;

% Pad with zeros for transmission
tx_vec_padded = [zeros(1,100) tx_vec];

%% Interpolate
% Zero pad then filter (same as interp or upfirdn without signal processing toolbox)

tx_vec_air = tx_vec_padded;

% Scale the Tx vector to +/- 1
tx_vec_air = TX_SCALE .* tx_vec_air ./ max(abs(tx_vec_air));

TX_NUM_SAMPS = length(tx_vec_air);

% AWGN:
rx_vec_air = tx_vec_air;
rx_vec_air = rx_vec_air + sqrt(sim_N0/2)*(randn(size(rx_vec_air)) + 1i*randn(size(rx_vec_air)));

%% Correlate for LTS through a normalized matched-filter:
a = 1;
unos = ones(size(lts_t))';
v0 = filter(fliplr(lts_t'),a, rx_vec_air);
v1 = filter(unos,a,abs(rx_vec_air).^2);
m_filt = (abs(v0).^2)./v1;
[rho_max, ipos] = max(m_filt);
pkt_strt = ipos - N_LTS*(N_SC + CP_LEN)+1;

lts_peaks = find(m_filt(1:800) > LTS_CORR_THRESH*rho_max);

% Rx signal
figure; clf;
subplot(2,1,1);
plot(real(rx_vec_air), 'b');
axis([0 length(rx_vec_air) -TX_SCALE TX_SCALE])
grid on;
title('Rx Waveform (I)');

subplot(2,1,2);
plot(imag(rx_vec_air), 'r');
axis([0 length(rx_vec_air) -TX_SCALE TX_SCALE])
grid on;
title('Rx Waveform (Q)');
% Stop if no valid correlation peak was found
if(isempty(lts_peaks))
    fprintf('No LTS Correlation Peaks Found!\n');
end

%% Rx payload processing

% Extract the payload samples (integral number of OFDM symbols following preamble)
payload_vec = rx_vec_air(pkt_strt : pkt_strt+(N_OFDM_SYMS + N_LTS) *(N_SC+CP_LEN)-1);
payload_mat = reshape(payload_vec, (N_SC+CP_LEN), N_LTS + N_OFDM_SYMS);

% Remove the cyclic prefix, keeping FFT_OFFSET samples of CP (on average)
payload_mat_noCP = payload_mat(CP_LEN-FFT_OFFSET+[1:N_SC], :);


% Take the FFT
syms_f_mat_sc = fft(payload_mat_noCP, N_SC, 1);
syms_f_mat = syms_f_mat_sc(SC_IND_DATA,:);

rx_lts_f = syms_f_mat(:,1:N_LTS);

rx_H_est0 = rx_lts_f ./repmat(lts_f(SC_IND_DATA).', 1, N_LTS);
rx_H_est = mean(rx_H_est0,2);

% Equalize (zero-forcing, just divide by complex chan estimates)
syms_eq_mat = syms_f_mat(:,N_LTS + 1:end) ./ repmat(rx_H_est, 1, N_OFDM_SYMS);
% Apply the pilot phase correction per symbol

payload_syms_mat =syms_eq_mat;

%% Demodulate
rx_syms = reshape(payload_syms_mat, 1, N_DATA_SYMS);

demod_fcn_bpsk = @(x) double(real(x)>0);
demod_fcn_qpsk = @(x) double(2*(real(x)>0) + 1*(imag(x)>0));
demod_fcn_16qam = @(x) (8*(real(x)>0)) + (4*(abs(real(x))<0.6325)) + (2*(imag(x)>0)) + (1*(abs(imag(x))<0.6325));
demod_fcn_64qam = @(x) (32*(real(x)>0)) + (16*(abs(real(x))<0.6172)) + (8*((abs(real(x))<(0.9258))&&((abs(real(x))>(0.3086))))) + (4*(imag(x)>0)) + (2*(abs(imag(x))<0.6172)) + (1*((abs(imag(x))<(0.9258))&&((abs(imag(x))>(0.3086)))));

switch(MOD_ORDER)
    case 2         % BPSK
        rx_data = arrayfun(demod_fcn_bpsk, rx_syms);
    case 4         % QPSK
        rx_data = arrayfun(demod_fcn_qpsk, rx_syms);
    case 16        % 16-QAM
        rx_data = arrayfun(demod_fcn_16qam, rx_syms);
    case 64        % 64-QAM
        rx_data = arrayfun(demod_fcn_64qam, rx_syms);
end

%% Plot Results
cf = 0;

% Tx signal
cf = cf + 1;
figure(cf); clf;

subplot(2,1,1);
plot(real(tx_vec_air), 'b');
axis([0 length(tx_vec_air) -TX_SCALE TX_SCALE])
grid on;
title('Tx Waveform (I)');

subplot(2,1,2);
plot(imag(tx_vec_air), 'r');
axis([0 length(tx_vec_air) -TX_SCALE TX_SCALE])
grid on;
title('Tx Waveform (Q)');

if(WRITE_PNG_FILES)
    print(gcf,sprintf('wl_ofdm_plots_%s_txIQ', example_mode_string), '-dpng', '-r96', '-painters')
end

% Rx signal
cf = cf + 1;
figure(cf); clf;
subplot(2,1,1);
plot(real(rx_vec_air), 'b');
axis([0 length(rx_vec_air) -TX_SCALE TX_SCALE])
grid on;
title('Rx Waveform (I)');

subplot(2,1,2);
plot(imag(rx_vec_air), 'r');
axis([0 length(rx_vec_air) -TX_SCALE TX_SCALE])
grid on;
title('Rx Waveform (Q)');

if(WRITE_PNG_FILES)
    print(gcf,sprintf('wl_ofdm_plots_%s_rxIQ', example_mode_string), '-dpng', '-r96', '-painters')
end

% Rx LTS correlation
cf = cf + 1;
figure(cf); clf;
lts_to_plot = m_filt(1:1000);
plot(lts_to_plot, '.-b', 'LineWidth', 1);
hold on;
grid on;
line([1 length(lts_to_plot)], LTS_CORR_THRESH*max(lts_to_plot)*[1 1], 'LineStyle', '--', 'Color', 'r', 'LineWidth', 2);
title('LTS Correlation and Threshold')
xlabel('Sample Index')
myAxis = axis();
axis([1, 1000, myAxis(3), myAxis(4)])

if(WRITE_PNG_FILES)
    print(gcf,sprintf('wl_ofdm_plots_%s_ltsCorr', example_mode_string), '-dpng', '-r96', '-painters')
end


%% Symbol constellation
cf = cf + 1;
figure(cf); clf;

plot(payload_syms_mat(:),'ro','MarkerSize',1);
axis square; axis(1.5*[-1 1 -1 1]);
grid on;
hold on;

plot(tx_syms_mat(:),'bo');
title('Tx and Rx Constellations')
legend('Rx','Tx','Location','EastOutside');

if(WRITE_PNG_FILES)
    print(gcf,sprintf('wl_ofdm_plots_%s_constellations', example_mode_string), '-dpng', '-r96', '-painters')
end


% EVM & SNR
cf = cf + 1;
figure(cf); clf;

evm_mat = abs(payload_syms_mat - tx_syms_mat).^2;
aevms = mean(evm_mat(:));
snr = 10*log10(1./aevms);

subplot(2,1,1)
plot(100*evm_mat(:),'o','MarkerSize',1)
axis tight
hold on
plot([1 length(evm_mat(:))], 100*[aevms, aevms],'r','LineWidth',4)
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
imagesc(1:N_OFDM_SYMS, (SC_IND_DATA - N_SC/2), 100*fftshift(evm_mat,1))

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

%% Calculate Rx stats

sym_errs = sum(tx_data ~= rx_data);
bit_errs = length(find(dec2bin(bitxor(tx_data, rx_data),8) == '1'));
rx_evm   = sqrt(sum((real(rx_syms) - real(tx_syms)).^2 + (imag(rx_syms) - imag(tx_syms)).^2)/(length(SC_IND_DATA) * N_OFDM_SYMS));

fprintf('\nResults:\n');
fprintf('Num Bytes:   %d\n', N_DATA_SYMS * log2(MOD_ORDER) / 8);
fprintf('Sym Errors:  %d (of %d total symbols)\n', sym_errs, N_DATA_SYMS);
fprintf('Bit Errors:  %d (of %d total bits)\n', bit_errs, N_DATA_SYMS * log2(MOD_ORDER));



