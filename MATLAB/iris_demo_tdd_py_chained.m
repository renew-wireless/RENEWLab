%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%	Author(s): C. Nicolas Barati nicobarati@rice.edu 
%		Rahman Doost-Mohamamdy: doost@rice.edu
%
%---------------------------------------------------------------------
% Copyright (c) 2018-2019, Rice University
% RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
% Original code copyright Mango Communications, Inc.
% Distributed under the WARP License http://warpproject.org/license
% ---------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
close all;

[version, executable, isloaded] = pyversion;
if ~isloaded
    pyversion /usr/bin/python
    py.print() %weird bug where py isn't loaded in an external script
end

% Params:
WRITE_PNG_FILES         = 0;           % Enable writing plots to PNG
CHANNEL                 = 11;          % Channel to tune Tx and Rx radios

% Waveform params
N_OFDM_SYM             = 32;           % Number of OFDM symbols for burst, it needs to be less than 47
MOD_ORDER               = 4;           % Modulation order (2/4/16/64 = BSPK/QPSK/16-QAM/64-QAM)
TX_SCALE                = 1.0;         % Scale for Tx waveform ([0:1])

% OFDM params
SC_IND_PILOTS           = [8 22 44 58];                           % Pilot subcarrier indices
SC_IND_DATA             = [2:7 9:21 23:27 39:43 45:57 59:64];     % Data subcarrier indices
N_SC                    = 64;                                     % Number of subcarriers
CP_LEN                  = 16;                                     % Cyclic prefix length
N_DATA_SYMS             = N_OFDM_SYM * length(SC_IND_DATA);       % Number of data symbols (one per data-bearing subcarrier per OFDM symbol)
N_LTS_SYM               = 2;                                      % Number of 
N_STS_SYM               = 6;                                      % Number of STS Symbols (taken as N_SC + CP_LEN units)
N_SYM_SAMP              = N_SC + CP_LEN;                          % Number of samples that will go over the air
N_ZPAD_PRE              = 88;                                     % Zero-padding prefix for Iris
N_ZPAD_POST             = N_ZPAD_PRE - 14;                        % Zero-padding postfix for Iris
N_PKT_SAMP              = N_SYM_SAMP * (N_OFDM_SYM + N_LTS_SYM + N_STS_SYM) +...
    N_ZPAD_PRE + N_ZPAD_POST;                                                       % Number of samples per packet

sc_data_idx             = [2:27 39:64]';     % indices of non-zero SCs                     

% Rx processing params
FFT_OFFSET                    = 16;          % Number of CP samples to use in FFT (on average)
LTS_CORR_THRESH               = 0.8;         % Normalized threshold for LTS correlation
DO_APPLY_CFO_CORRECTION       = 0;           % Enable CFO estimation/correction
DO_APPLY_SFO_CORRECTION       = 0;           % Enable SFO estimation/correction
DO_APPLY_PHASE_ERR_CORRECTION = 0;           % Enable Residual CFO estimation/correction

    
%% Define the preamble
% Note: The STS symbols in the preamble meet the requirements needed by the
% AGC core at the receiver. Details on the operation of the AGC are
% available on the wiki: http://warpproject.org/trac/wiki/WARPLab/AGC
sts_f = zeros(1,64);
sts_f(1:27) = [0 0 0 0 -1-1i 0 0 0 -1-1i 0 0 0 1+1i 0 0 0 1+1i 0 0 0 1+1i 0 0 0 1+1i 0 0];
sts_f(39:64) = [0 0 1+1i 0 0 0 -1-1i 0 0 0 1+1i 0 0 0 -1-1i 0 0 0 -1-1i 0 0 0 1+1i 0 0 0];
sts_t = ifft(sqrt(13/6).*sts_f, 64);
sts_t = sts_t(1:16);

% LTS for CFO and channel estimation
lts_f = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 ...
    1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1];
lts_t = ifft(lts_f, 64); %time domain

% Use 30 copies of the 16-sample STS for extra AGC settling margin
preamble = [repmat(sts_t, 1, 30)  lts_t(33:64) lts_t lts_t];

%% Generate a payload of random integers
tx_data = randi(MOD_ORDER, 1, N_DATA_SYMS) - 1;

% Functions for data -> complex symbol mapping (like qammod, avoids comm toolbox requirement)
% These anonymous functions implement the modulation mapping from IEEE 802.11-2012 Section 18.3.5.8
modvec_bpsk   =  (1/sqrt(2))  .* [-1 1];
modvec_16qam  =  (1/sqrt(10)) .* [-3 -1 +3 +1];
modvec_64qam  =  (1/sqrt(42)) .* [-7 -5 -1 -3 +7 +5 +1 +3];

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
tx_syms_mat = reshape(tx_syms, length(SC_IND_DATA), N_OFDM_SYM);

% Define the pilot tone values as BPSK symbols
pilots = [1 1 -1 1].';

% Repeat the pilots across all OFDM symbols
pilots_mat = repmat(pilots, 1, N_OFDM_SYM);

%% IFFT

% Construct the IFFT input matrix
ifft_in_mat = zeros(N_SC, N_OFDM_SYM);

% Insert the data and pilot values; other subcarriers will remain at 0
ifft_in_mat(SC_IND_DATA, :)   = tx_syms_mat;
ifft_in_mat(SC_IND_PILOTS, :) = pilots_mat;

%Perform the IFFT
tx_payload_mat = ifft(ifft_in_mat, N_SC, 1);

% Insert the cyclic prefix
if(CP_LEN > 0)
    tx_cp = tx_payload_mat((end-CP_LEN+1 : end), :);
    tx_payload_mat = [tx_cp; tx_payload_mat];
end

% Reshape to a vector
tx_payload_vec = reshape(tx_payload_mat, 1, numel(tx_payload_mat));


% Construct the full time-domain OFDM waveform
tx_vec = [zeros(1,N_ZPAD_PRE) preamble tx_payload_vec zeros(1,N_ZPAD_POST)];
%tx_vec = [preamble tx_payload_vec];


% Leftover from zero padding:
tx_vec_iris = tx_vec.';

%% Init Iris nodes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set up the Iris experiment
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 % Create a two Iris node objects:
    b_sched = 'GGPGGRGG';               % BS schedule
    u_sched = 'GGRGGPGG';               % UE schedule
    % Create a two node objects:
    sdr_params = struct(...
        'id', 'RF3C000007', ...
        'txfreq', 2.6e9, ...
        'rxfreq', 2.6e9, ...
        'txgain', 30, ...
        'rxgain', 30, ...
        'sample_rate', 5e6, ...
        'n_samp', length(tx_vec_iris), ...     % number of samples per frame time.
        'tdd_sched', b_sched, ...
        'n_zpad_samp', (N_ZPAD_PRE + N_ZPAD_POST) ... % number of zero-paddes samples
        );
    
    sdr_params(2) = sdr_params(1);
    sdr_params(2).id =  'RF3C000028'; % '0128'; %
    sdr_params(2).rxfreq = 2.6e9;
    sdr_params(2).txfreq = 2.6e9;
    sdr_params(2).tdd_sched = u_sched;
    node_bs = iris_py(sdr_params(1));
    node_ue = iris_py(sdr_params(2));

    SAMP_FREQ = sdr_params(1).sample_rate;

%% Iris Tx 
% Need to be done once for burst! Just burn the data onto the FPGAs RAM

% Scale the Tx vector to +/- 1
tx_vec_iris = TX_SCALE .* tx_vec_iris ./ max(abs(tx_vec_iris));

trig = 1;

node_bs.sdrsync(1);

node_ue.sdrrxsetup();
node_bs.sdrrxsetup();

chained_mode = 1;
node_ue.set_config(chained_mode,0,0);
node_bs.set_config(chained_mode,1,0);

node_bs.sdrtx(tx_vec_iris);
node_ue.sdrtx(tx_vec_iris);

node_ue.sdr_activate_rx();
node_bs.sdr_activate_rx();
 
node_bs.sdrtrigger(trig);

%% Iris Rx
% Need to be done as long as there is transmission or once? 
%Need to be done for both BS and UE.

[rx_vec_iris, data0_len] = node_ue.sdrrx();
[rx_vec_iris_bs, data0_len_bs] = node_bs.sdrrx();

node_bs.sdr_close();
node_ue.sdr_close();

fprintf('Matlab script: Length of the received vector:\tBS:%d \tUE:%d\n', data0_len_bs, data0_len);


rx_vec_iris = rx_vec_iris.';
raw_rx_dec = rx_vec_iris(:,1).';
rx_vec_air = raw_rx_dec;


%% Correlate for LTS

% Complex cross correlation of Rx waveform with time-domain LTS
lts_corr = abs(conv( conj(fliplr(lts_t)), sign(raw_rx_dec)));

% Skip early and late samples - avoids occasional false positives from pre-AGC samples
lts_corr = lts_corr(32:end-32);

% Find all correlation peaks
lts_peaks = find(lts_corr(1:800) > LTS_CORR_THRESH*max(lts_corr));

% Select best candidate correlation peak as LTS-payload boundary
[LTS1, LTS2] = meshgrid(lts_peaks,lts_peaks);
[lts_second_peak_index,y] = find(LTS2-LTS1 == length(lts_t));

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
if(isempty(lts_second_peak_index))
    fprintf('No LTS Correlation Peaks Found!\n');
    return;
end

% Set the sample indices of the payload symbols and preamble
% The "+32" corresponds to the 32-sample cyclic prefix on the preamble LTS
% The "-160" corresponds to the length of the preamble LTS (2.5 copies of 64-sample LTS)
payload_ind = lts_peaks(max(lts_second_peak_index)) + 32;
lts_ind = payload_ind-160;

if(DO_APPLY_CFO_CORRECTION)
    %Extract LTS (not yet CFO corrected)
    rx_lts = raw_rx_dec(lts_ind : lts_ind+159);
    rx_lts1 = rx_lts(-64+-FFT_OFFSET + [97:160]);
    rx_lts2 = rx_lts(-FFT_OFFSET + [97:160]);

    %Calculate coarse CFO est
    rx_cfo_est_lts = mean(unwrap(angle(rx_lts2 .* conj(rx_lts1))));
    rx_cfo_est_lts = rx_cfo_est_lts/(2*pi*64);
else
    rx_cfo_est_lts = 0;
end

% Apply CFO correction to raw Rx waveform
rx_cfo_corr_t = exp(-1i*2*pi*rx_cfo_est_lts*[0:length(raw_rx_dec)-1]); % =1 if there is no CF0_CORRECTION!
rx_dec_cfo_corr = raw_rx_dec .* rx_cfo_corr_t;

% Re-extract LTS for channel estimate
rx_lts = rx_dec_cfo_corr(lts_ind : lts_ind+159);
rx_lts1 = rx_lts(-64+-FFT_OFFSET + [97:160]);
rx_lts2 = rx_lts(-FFT_OFFSET + [97:160]);

rx_lts1_f = fft(rx_lts1);
rx_lts2_f = fft(rx_lts2);

% Calculate channel estimate from average of 2 training symbols
rx_H_est = lts_f .* (rx_lts1_f + rx_lts2_f)/2; 

%% Rx payload processing

% Extract the payload samples (integral number of OFDM symbols following preamble)
if( (length(rx_dec_cfo_corr) - payload_ind ) > (N_SYM_SAMP * N_OFDM_SYM) )
    payload_vec = rx_dec_cfo_corr(payload_ind : payload_ind + (N_SYM_SAMP * N_OFDM_SYM));
else
    payload_vec = rx_dec_cfo_corr(payload_ind : end);
end

missed_samps = (N_SC+CP_LEN) * N_OFDM_SYM - length(payload_vec); %sometimes it's bellow 0!

if (missed_samps > 0) 
    payload_vec = [payload_vec zeros(1, missed_samps)];
elseif (missed_samps < 0)
    payload_vec = payload_vec(:, 1:end+missed_samps);
end
    
payload_mat = reshape(payload_vec, (N_SC+CP_LEN), N_OFDM_SYM);

% Remove the cyclic prefix, keeping FFT_OFFSET samples of CP (on average)
payload_mat_noCP = payload_mat(CP_LEN-FFT_OFFSET+[1:N_SC], :);

% Take the FFT
syms_f_mat = fft(payload_mat_noCP, N_SC, 1);

% Equalize (zero-forcing, just divide by complex chan estimates)
syms_eq_mat = syms_f_mat ./ repmat(rx_H_est.', 1, N_OFDM_SYM);

if DO_APPLY_SFO_CORRECTION
    % SFO manifests as a frequency-dependent phase whose slope increases
    % over time as the Tx and Rx sample streams drift apart from one
    % another. To correct for this effect, we calculate this phase slope at
    % each OFDM symbol using the pilot tones and use this slope to
    % interpolate a phase correction for each data-bearing subcarrier.

	% Extract the pilot tones and "equalize" them by their nominal Tx values
    pilots_f_mat = syms_eq_mat(SC_IND_PILOTS, :);
    pilots_f_mat_comp = pilots_f_mat.*pilots_mat;

	% Calculate the phases of every Rx pilot tone
    pilot_phases = unwrap(angle(fftshift(pilots_f_mat_comp,1)), [], 1);

	% Calculate slope of pilot tone phases vs frequency in each OFDM symbol
    pilot_spacing_mat = repmat(mod(diff(fftshift(SC_IND_PILOTS)),64).', 1, N_OFDM_SYM);                        
	pilot_slope_mat = mean(diff(pilot_phases) ./ pilot_spacing_mat);

	% Calculate the SFO correction phases for each OFDM symbol
    pilot_phase_sfo_corr = fftshift((-32:31).' * pilot_slope_mat, 1);
    pilot_phase_corr = exp(-1i*(pilot_phase_sfo_corr));

    % Apply the pilot phase correction per symbol
    syms_eq_mat = syms_eq_mat .* pilot_phase_corr;
else
	% Define an empty SFO correction matrix (used by plotting code below)
    pilot_phase_sfo_corr = zeros(N_SC, N_OFDM_SYM);
end


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

% Apply the pilot phase correction per symbol
syms_eq_pc_mat = syms_eq_mat .* pilot_phase_corr;
payload_syms_mat = syms_eq_pc_mat(SC_IND_DATA, :);

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
plot(real(tx_vec_iris), 'b');
axis([0 length(tx_vec_iris) -TX_SCALE TX_SCALE])
grid on;
title('Tx Waveform (I)');

subplot(2,1,2);
plot(imag(tx_vec_iris), 'r');
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
lts_to_plot = lts_corr;
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

% Channel Estimates
cf = cf + 1;

rx_H_est_plot = repmat(complex(NaN,NaN),1,length(rx_H_est));
rx_H_est_plot(SC_IND_DATA) = rx_H_est(SC_IND_DATA);
rx_H_est_plot(SC_IND_PILOTS) = rx_H_est(SC_IND_PILOTS);

x = (20/N_SC) * (-(N_SC/2):(N_SC/2 - 1));

figure(cf); clf;
subplot(2,1,1);
stairs(x - (20/(2*N_SC)), fftshift(real(rx_H_est_plot)), 'b', 'LineWidth', 2);
hold on
stairs(x - (20/(2*N_SC)), fftshift(imag(rx_H_est_plot)), 'r', 'LineWidth', 2);
hold off
axis([min(x) max(x) -1.1*max(abs(rx_H_est_plot)) 1.1*max(abs(rx_H_est_plot))])
grid on;
title('Channel Estimates (I and Q)')

subplot(2,1,2);
bh = bar(x, fftshift(abs(rx_H_est_plot)),1,'LineWidth', 1);
shading flat
set(bh,'FaceColor',[0 0 1])
axis([min(x) max(x) 0 1.1*max(abs(rx_H_est_plot))])
grid on;
title('Channel Estimates (Magnitude)')
xlabel('Baseband Frequency (MHz)')

cf = cf + 1;
figure(cf); clf;
x = (20/N_SC) * (-(N_SC/2):(N_SC/2 - 1));
stairs(x - (20/(2*N_SC)), fftshift(angle(rx_H_est_plot)), 'g', 'LineWidth', 2);
hold off
axis([min(x) max(x) -pi pi])
grid on;
title('Channel Estimates Phase')

if(WRITE_PNG_FILES)
    print(gcf,sprintf('wl_ofdm_plots_%s_chanEst', example_mode_string), '-dpng', '-r96', '-painters')
end

%% Pilot phase error estimate
cf = cf + 1;
figure(cf); clf;
subplot(2,1,1)
plot(pilot_phase_err, 'b', 'LineWidth', 2);
title('Phase Error Estimates')
xlabel('OFDM Symbol Index')
ylabel('Radians')
axis([1 N_OFDM_SYM -3.2 3.2])
grid on

h = colorbar;
set(h,'Visible','off');

subplot(2,1,2)
imagesc(1:N_OFDM_SYM, (SC_IND_DATA - N_SC/2), fftshift(pilot_phase_sfo_corr,1))
xlabel('OFDM Symbol Index')
ylabel('Subcarrier Index')
title('Phase Correction for SFO')
colorbar
myAxis = caxis();
if(myAxis(2)-myAxis(1) < (pi))
   caxis([-pi/2 pi/2])
end


if(WRITE_PNG_FILES)
    print(gcf,sprintf('wl_ofdm_plots_%s_phaseError', example_mode_string), '-dpng', '-r96', '-painters')
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

%% Calculate Rx stats

sym_errs = sum(tx_data ~= rx_data);
bit_errs = length(find(dec2bin(bitxor(tx_data, rx_data),8) == '1'));
rx_evm   = sqrt(sum((real(rx_syms) - real(tx_syms)).^2 + (imag(rx_syms) - imag(tx_syms)).^2)/(length(SC_IND_DATA) * N_OFDM_SYM));

fprintf('\nResults:\n');
fprintf('Num Bytes:   %d\n', N_DATA_SYMS * log2(MOD_ORDER) / 8);
fprintf('Sym Errors:  %d (of %d total symbols)\n', sym_errs, N_DATA_SYMS);
fprintf('Bit Errors:  %d (of %d total bits)\n', bit_errs, N_DATA_SYMS * log2(MOD_ORDER));

cfo_est_lts = rx_cfo_est_lts*(SAMP_FREQ);
cfo_est_phaseErr = mean(diff(unwrap(pilot_phase_err)))/(4e-6*2*pi);
cfo_total_ppm = ((cfo_est_lts + cfo_est_phaseErr) /  ((3.6+(.005*(CHANNEL-1)))*1e9)) * 1e6;

fprintf('CFO Est:     %3.2f kHz (%3.2f ppm)\n', (cfo_est_lts + cfo_est_phaseErr)*1e-3, cfo_total_ppm);
fprintf('     LTS CFO Est:                  %3.2f kHz\n', cfo_est_lts*1e-3);
fprintf('     Phase Error Residual CFO Est: %3.2f kHz\n', cfo_est_phaseErr*1e-3);

if DO_APPLY_SFO_CORRECTION
    drift_sec = pilot_slope_mat / (2*pi*312500);
    sfo_est_ppm =  1e6*mean((diff(drift_sec) / 4e-6));
    sfo_est = sfo_est_ppm*20;
    fprintf('SFO Est:     %3.2f Hz (%3.2f ppm)\n', sfo_est, sfo_est_ppm);

end
