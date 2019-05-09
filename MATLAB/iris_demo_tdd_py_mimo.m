%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%	Author(s): C. Nicolas Barati nicobarati@rice.edu 
%		Rahman Doost-Mohamamdy: doost@rice.edu
%
%---------------------------------------------------------------------
% Original code copyright Mango Communications, Inc.
% Distributed under the WARP License http://warpproject.org/license
% Copyright (c) 2018-2019, Rice University
% RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
% ---------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
%close all;

[version, executable, isloaded] = pyversion;
if ~isloaded
    pyversion /usr/bin/python
    py.print() %weird bug where py isn't loaded in an external script
end

% Params:
WRITE_PNG_FILES         = 0;           % Enable writing plots to PNG
CHANNEL                 = 11;          % Channel to tune Tx and Rx radios
SIM_MOD                 = 1;
DEBUG                   = 1;
sim_N0                  = 0.01;     
sim_H_var               = 4;
%Iris params:
N_BS_NODE = 8;
N_UE = 2;
b_ids = string.empty();
b_scheds = string.empty();
ue_ids = string.empty();
ue_scheds = string.empty();


% Waveform params
N_OFDM_SYM              = 32;         % Number of OFDM symbols for burst, it needs to be less than 47
MOD_ORDER               = 16;           % Modulation order (2/4/16/64 = BSPK/QPSK/16-QAM/64-QAM)
TX_SCALE                = .5;         % Scale for Tx waveform ([0:1])

% OFDM params
SC_IND_PILOTS           = [8 22 44 58];                           % Pilot subcarrier indices
SC_IND_DATA             = [2:7 9:21 23:27 39:43 45:57 59:64];     % Data subcarrier indices
SC_IND_DATA_PILOT       = [2:27 39:64]';
N_SC                    = 64;                                     % Number of subcarriers
CP_LEN                  = 16;                                     % Cyclic prefix length
N_DATA_SYMS             = N_OFDM_SYM * length(SC_IND_DATA);       % Number of data symbols (one per data-bearing subcarrier per OFDM symbol)
N_LTS_SYM               = 2;                                      % Number of 
N_SYM_SAMP              = N_SC + CP_LEN;                          % Number of samples that will go over the air
N_ZPAD_PRE              = 70;                                     % Zero-padding prefix for Iris
N_ZPAD_POST             = N_ZPAD_PRE -14;                         % Zero-padding postfix for Iris

% Rx processing params
FFT_OFFSET                    = 16;          % Number of CP samples to use in FFT (on average)
LTS_CORR_THRESH               = 0.8;         % Normalized threshold for LTS correlation
DO_APPLY_CFO_CORRECTION       = 0;           % Enable CFO estimation/correction
DO_APPLY_SFO_CORRECTION       = 0;           % Enable SFO estimation/correction
DO_APPLY_PHASE_ERR_CORRECTION = 1;           % Enable Residual CFO estimation/correction

%% Define the preamble
% LTS for fine CFO and channel estimation
lts_f = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 ...
    1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1].';
lts_t = ifft(lts_f, N_SC); %time domain

preamble_common = [lts_t(33:64); lts_t; lts_t];
pre_z = zeros(size(preamble_common));
preamble = [ preamble_common pre_z;pre_z preamble_common];
cor_rng  = length(preamble) +  N_ZPAD_PRE + 100;
%% Generate a payload of random integers
tx_data = randi(MOD_ORDER, N_DATA_SYMS, N_UE) - 1;

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
tx_syms_mat = reshape(tx_syms, length(SC_IND_DATA), N_OFDM_SYM, N_UE);

% Define the pilot tone values as BPSK symbols
pilots = [1 1 -1 1].';

% Repeat the pilots across all OFDM symbols
pilots_mat = repmat(pilots, 1, N_OFDM_SYM, N_UE);

%% IFFT

% Construct the IFFT input matrix
ifft_in_mat = zeros(N_SC, N_OFDM_SYM, N_UE);

% Insert the data and pilot values; other subcarriers will remain at 0
ifft_in_mat(SC_IND_DATA, :, :)   = tx_syms_mat;
ifft_in_mat(SC_IND_PILOTS, :, :) = pilots_mat;

%Perform the IFFT
tx_payload_mat = ifft(ifft_in_mat, N_SC, 1);

% Insert the cyclic prefix
if(CP_LEN > 0)
    tx_cp = tx_payload_mat((end-CP_LEN+1 : end), :, :);
    tx_payload_mat = [tx_cp; tx_payload_mat];
end

% Reshape to a vector
tx_payload_vecs = reshape(tx_payload_mat, ceil(numel(tx_payload_mat)/N_UE), N_UE);

% Construct the full time-domain OFDM waveform
tx_vecs = [zeros(N_ZPAD_PRE, N_UE); preamble; tx_payload_vecs; zeros(N_ZPAD_POST, N_UE)];

% Leftover from zero padding:
tx_vecs_iris = tx_vecs;
% Scale the Tx vector to +/- 1
tx_vecs_iris = TX_SCALE .* tx_vecs_iris ./ max(abs(tx_vecs_iris));
if (SIM_MOD) 
    
    DO_APPLY_CFO_CORRECTION       = 0;       
    DO_APPLY_SFO_CORRECTION       = 0;  
    DO_APPLY_PHASE_ERR_CORRECTION = 1;

    H_ul = sqrt(sim_H_var/2) .* ( randn(N_BS_NODE, N_UE) + 1i*randn(N_BS_NODE, N_UE) );
    W_ul = sqrt(sim_N0/2) * (randn(N_BS_NODE, length(tx_vecs_iris)) + ...
        1i*randn(N_BS_NODE, length(tx_vecs_iris)) );
    rx_vec_iris = H_ul*tx_vecs_iris.' + W_ul;

        SAMP_FREQ = 5e6;
else

%% Init Iris nodes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set up the Iris experiment
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Create a two Iris node objects:
    b_ids = ["0328", "0339", "0268", "0282", "0344", "0233", "0334", "0402"];
    ue_ids= "RF3C000053";

    b_prim_sched = "PGGGGGRG";           % BS primary noede's schedule: Send Beacon only from one Iris board
    b_sec_sched = "GGGGGGRG"; 
    ue_sched = "GGGGGGPG";               % UE schedule

    b_scheds =  b_prim_sched;
    if (N_BS_NODE > 1)
        b_scheds = [b_scheds b_sec_sched];
    end

    ue_scheds = string.empty();
    for iu = 1:N_UE
        ue_scheds(iu,:) = ue_sched;
    end

    n_samp = length(tx_vecs_iris);
    % Iris nodes' parameters
    sdr_params = struct(...
        'id', b_ids, ...
        'n_chain',N_BS_NODE, ...
        'txfreq', 2.6e9, ...
        'rxfreq', 2.6e9, ...
        'txgain', 46, ...
        'rxgain', 30, ...
        'sample_rate', 5e6, ...
        'n_samp', n_samp, ...          % number of samples per frame time.
        'tdd_sched', b_scheds, ...     % number of zero-paddes samples
        'n_zpad_samp', (N_ZPAD_PRE + N_ZPAD_POST) ...
        );

    sdr_params(2) = sdr_params(1);
    sdr_params(2).id =  ue_ids(1);
    sdr_params(2).n_chain = 1;
    sdr_params(2).rxfreq = 2.6e9;
    sdr_params(2).txfreq = 2.6e9;
    sdr_params(2).tdd_sched = ue_scheds(1);
    % Iris nodes objects
    node_bs = iris_py(sdr_params(1));
    node_ue = iris_py(sdr_params(2));

    SAMP_FREQ = sdr_params(1).sample_rate;

    %% Iris Tx UL
    % Need to be done once for burst! Just burn the data onto the FPGAs RAM

    trig = 1;

    node_ue.sdr_txgainctrl();
    node_bs.sdrsync(1);
    node_ue.sdrsync(0);

    node_ue.sdrrxsetup();
    node_bs.sdrrxsetup();
    chained_mode = 0;
    node_bs.set_config(chained_mode,1,0);
    node_ue.set_config(chained_mode,0,0);


    node_bs.sdr_txbeacon(N_ZPAD_PRE);
    node_ue.sdrtx(tx_vecs_iris);

    node_bs.sdr_activate_rx();

    node_ue.sdr_setcorr()

    node_bs.sdrtrigger(trig);

    %% Iris Rx 
    % Only UL data:

    [rx_vec_iris, data0_len] = node_bs.sdrrx(n_samp);

    node_bs.sdr_close();
    node_ue.sdr_close();

    fprintf('Matlab script: Length of the received vector: \tUE:%d\n', data0_len);
end
raw_rx_dec = rx_vec_iris;
rx_vec_air = raw_rx_dec; %NB: Change this, unecessary assignment!

l_rx_dec=length(raw_rx_dec);

%% Correlate for LTS
%NB: start here!

% Complex cross correlation of Rx waveform with time-domain LTS
lts_corr = double.empty();
for ibs=1:N_BS_NODE
    lts_corr(:,ibs) = abs(conv( conj(flipud(lts_t)), sign(raw_rx_dec(ibs,:))));
end

% Skip early and late samples - avoids occasional false positives from pre-AGC samples
lts_corr = lts_corr(32:end-32,:);

if DEBUG
    figure,
    for sp = 1:N_BS_NODE
        subplot(8,1,sp);
        plot(lts_corr(:,sp)) 
        xlabel('Samples');
        y_label = sprintf('Anetnna %d',sp);
        ylabel(y_label);
    end
    sgtitle('LTS correlations accross antennas')
end

% Find all correlation peaks
pream_ind = double.empty();
for ibs=1:N_BS_NODE
    lts_peaks = find(lts_corr(1:cor_rng) > LTS_CORR_THRESH*max(lts_corr(:,ibs)));

    % Select best candidate correlation peak as LTS-payload boundary
    [LTS1, LTS2] = meshgrid(lts_peaks,lts_peaks);
    [lts_second_peak_index,y] = find(LTS2-LTS1 == length(lts_t));
    % Stop if no valid correlation peak was found
    if(isempty(lts_second_peak_index))
        fprintf('No LTS Correlation Peaks Found!\n');
        return;
    end    
    % Set the sample indices of the payload symbols and preamble
    payload_ind = lts_peaks(max(lts_second_peak_index)) + (2*CP_LEN);
    pream_ind(ibs) = payload_ind-length(preamble);
end

rx_cfo_est_lts = zeros(1,N_BS_NODE,1);
rx_dec_cfo_corr = raw_rx_dec;

% Extract LTS for channel estimate
rx_lts_mat = rx_dec_cfo_corr(:, pream_ind : pream_ind + length(preamble) -1);
ul_pilots_mat = reshape(rx_lts_mat, N_BS_NODE,ceil(length(rx_lts_mat)/N_UE),N_UE);

return;

rx_lts_idx1 = -64+-FFT_OFFSET + (97:160);
rx_lts_idx2 = -FFT_OFFSET + (97:160);
rx_lts_b1 = [rx_lts(rx_lts_idx1,1)  rx_lts(rx_lts_idx2,1)];
rx_lts_b2 = [rx_lts(rx_lts_idx1,2)  rx_lts(rx_lts_idx2,2)];

% Received LTSs for each branch.  
rx_lts_b1_f = fft(rx_lts_b1);
rx_lts_b2_f = fft(rx_lts_b2);

% Channel Estimates of each branch 
H0_b1 = rx_lts_b1_f ./ repmat(lts_f',1,N_LTS_SYM);
H0_b2 = rx_lts_b2_f ./ repmat(lts_f',1,N_LTS_SYM);
H_b1 = mean(H0_b1,2); 
H_b2 = mean(H0_b2,2);
idx_0 = find(lts_f == 0);
H_b1(idx_0,:) = 0;
H_b2(idx_0,:) = 0;
rx_H_est_2d = [H_b1 H_b2];

%% Rx payload processing
% Extract the payload samples (integer number of OFDM symbols following preamble)
if( (length(rx_dec_cfo_corr) - payload_ind ) > (N_SYM_SAMP * N_OFDM_SYM) )
    payload_vec = rx_dec_cfo_corr(payload_ind : payload_ind + (N_SYM_SAMP * N_OFDM_SYM), :);
else
    payload_vec = rx_dec_cfo_corr(payload_ind : end,:);
end

missed_samps = (N_SC+CP_LEN) * N_OFDM_SYM - length(payload_vec); %sometimes it's below .

if (missed_samps > 0) 
    payload_vec = [payload_vec; zeros( missed_samps, N_BS_NODE)];
elseif (missed_samps < 0)
    payload_vec = payload_vec(1:end+missed_samps, :);
end

% (N_CP + N_SC) x N_OFDM x N_BS_NODE
payload_mat = reshape(payload_vec, (N_SC+CP_LEN), N_OFDM_SYM, N_BS_NODE);

% Remove the cyclic prefix, keeping FFT_OFFSET samples of CP (on average)
 payload_mat_noCP = payload_mat(CP_LEN-FFT_OFFSET+(1:N_SC), :,:);

% Take the FFT
syms_f_mat_mrc = fft(payload_mat_noCP, N_SC, 1);
syms_f_mat_1 = syms_f_mat_mrc(:,:,1);
syms_f_mat_2 = syms_f_mat_mrc(:,:,2);

% Equalize MRC
rx_H_est = reshape(rx_H_est_2d,N_SC,1,N_BS_NODE);       % Expand to a 3rd dimension to agree with the dimensions od syms_f_mat
H_pow = sum(abs(conj(rx_H_est_2d).*rx_H_est_2d),2);
H_pow = repmat(H_pow,1,N_OFDM_SYM);
syms_eq_mat_mrc =  sum( (repmat(conj(rx_H_est), 1, N_OFDM_SYM,1).* syms_f_mat_mrc), 3)./H_pow;

%Equalize each branch separately
syms_eq_mat_1 = syms_f_mat_1 ./ repmat(H_b1, 1, N_OFDM_SYM);
syms_eq_mat_2 = syms_f_mat_2 ./ repmat(H_b2, 1, N_OFDM_SYM);

if DO_APPLY_SFO_CORRECTION
    % SFO manifests as a frequency-dependent phase whose slope increases
    % over time as the Tx and Rx sample streams drift apart from one
    % another. To correct for this effect, we calculate this phase slope at
    % each OFDM symbol using the pilot tones and use this slope to
    % interpolate a phase correction for each data-bearing subcarrier.

	% Extract the pilot tones and "equalize" them by their nominal Tx values
    pilots_f_mat_mrc = syms_eq_mat_mrc(SC_IND_PILOTS, :,:);
    pilots_f_mat_comp_mrc = pilots_f_mat_mrc.*pilots_mat;
    
    pilots_f_mat_1 = syms_eq_mat_1(SC_IND_PILOTS, :,:);
    pilots_f_mat_comp_1 = pilots_f_mat_1.*pilots_mat;
    
    pilots_f_mat_2 = syms_eq_mat_2(SC_IND_PILOTS, :,:);
    pilots_f_mat_comp_2 = pilots_f_mat_2.*pilots_mat;


	% Calculate the phases of every Rx pilot tone
    pilot_phases_mrc = unwrap(angle(fftshift(pilots_f_mat_comp_mrc,1)), [], 1);
    pilot_phases_1 = unwrap(angle(fftshift(pilots_f_mat_comp_1,1)), [], 1);
    pilot_phases_2 = unwrap(angle(fftshift(pilots_f_mat_comp_2,1)), [], 1);

	% Calculate slope of pilot tone phases vs frequency in each OFDM symbol
    pilot_spacing_mat_mrc = repmat(mod(diff(fftshift(SC_IND_PILOTS)),64).', 1, N_OFDM_SYM);                        
	pilot_slope_mat_mrc = mean(diff(pilot_phases_mrc) ./ pilot_spacing_mat_mrc);
    pilot_spacing_mat_1 = repmat(mod(diff(fftshift(SC_IND_PILOTS)),64).', 1, N_OFDM_SYM);                        
	pilot_slope_mat_1 = mean(diff(pilot_phases_1) ./ pilot_spacing_mat_1);
    pilot_spacing_mat_2 = repmat(mod(diff(fftshift(SC_IND_PILOTS)),64).', 1, N_OFDM_SYM);                        
	pilot_slope_mat_2 = mean(diff(pilot_phases_2) ./ pilot_spacing_mat_2);

	% Calculate the SFO correction phases for each OFDM symbol
    pilot_phase_sfo_corr_mrc = fftshift((-32:31).' * pilot_slope_mat_mrc, 1);
    pilot_phase_corr_mrc = exp(-1i*(pilot_phase_sfo_corr_mrc)); 
    pilot_phase_sfo_corr_1 = fftshift((-32:31).' * pilot_slope_mat_1, 1);
    pilot_phase_corr_1 = exp(-1i*(pilot_phase_sfo_corr_1));
    pilot_phase_sfo_corr_2 = fftshift((-32:31).' * pilot_slope_mat_2, 1);
    pilot_phase_corr_2 = exp(-1i*(pilot_phase_sfo_corr_2));

    % Apply the pilot phase correction per symbol
    syms_eq_mat_mrc = syms_eq_mat_mrc .* pilot_phase_corr_mrc;
    syms_eq_mat_1 = syms_eq_mat_1 .* pilot_phase_corr_1;
    syms_eq_mat_2 = syms_eq_mat_2 .* pilot_phase_corr_2;
else
	% Define an empty SFO correction matrix (used by plotting code below)
    pilot_phase_sfo_corr_mrc = zeros(N_SC, N_OFDM_SYM);
    pilot_phase_sfo_corr_1 = zeros(N_SC, N_OFDM_SYM);
    pilot_phase_sfo_corr_2 = zeros(N_SC, N_OFDM_SYM);
end


if DO_APPLY_PHASE_ERR_CORRECTION
    % Extract the pilots and calculate per-symbol phase error
    pilots_f_mat_mrc = syms_eq_mat_mrc(SC_IND_PILOTS, :,:);
    pilots_f_mat_comp_mrc = pilots_f_mat_mrc.*pilots_mat;
    pilot_phase_err_mrc = angle(mean(pilots_f_mat_comp_mrc));
    pilots_f_mat_1 = syms_eq_mat_1(SC_IND_PILOTS, :,:);
    pilots_f_mat_comp_1 = pilots_f_mat_1.*pilots_mat;
    pilot_phase_err_1 = angle(mean(pilots_f_mat_comp_1));  
    pilots_f_mat_2 = syms_eq_mat_2(SC_IND_PILOTS, :,:);
    pilots_f_mat_comp_2 = pilots_f_mat_2.*pilots_mat;
    pilot_phase_err_2 = angle(mean(pilots_f_mat_comp_2));
    
else
	% Define an empty phase correction vector (used by plotting code below)
    pilot_phase_err_mrc = zeros(1, N_OFDM_SYM);
    pilot_phase_err_1 = zeros(1, N_OFDM_SYM);
    pilot_phase_err_2 = zeros(1, N_OFDM_SYM);
    
end
pilot_phase_err_corr_mrc = repmat(pilot_phase_err_mrc, N_SC, 1);
pilot_phase_corr_mrc = exp(-1i*(pilot_phase_err_corr_mrc));
pilot_phase_err_corr_1 = repmat(pilot_phase_err_1, N_SC, 1);
pilot_phase_corr_1 = exp(-1i*(pilot_phase_err_corr_1));
pilot_phase_err_corr_2 = repmat(pilot_phase_err_2, N_SC, 1);
pilot_phase_corr_2 = exp(-1i*(pilot_phase_err_corr_2));


% Apply the pilot phase correction per symbol
syms_eq_pc_mat_mrc = syms_eq_mat_mrc .* pilot_phase_corr_mrc;
payload_syms_mat_mrc = syms_eq_pc_mat_mrc(SC_IND_DATA, :);

syms_eq_pc_mat_1 = syms_eq_mat_1 .* pilot_phase_corr_1;
payload_syms_mat_1 = syms_eq_pc_mat_1(SC_IND_DATA, :);

syms_eq_pc_mat_2 = syms_eq_mat_2 .* pilot_phase_corr_2;
payload_syms_mat_2 = syms_eq_pc_mat_2(SC_IND_DATA, :);
%% Demodulate
rx_syms_mrc = reshape(payload_syms_mat_mrc, 1, N_DATA_SYMS);
rx_syms_1 = reshape(payload_syms_mat_1, 1, N_DATA_SYMS);
rx_syms_2 = reshape(payload_syms_mat_2, 1, N_DATA_SYMS);

demod_fcn_bpsk = @(x) double(real(x)>0);
demod_fcn_qpsk = @(x) double(2*(real(x)>0) + 1*(imag(x)>0));
demod_fcn_16qam = @(x) (8*(real(x)>0)) + (4*(abs(real(x))<0.6325)) + (2*(imag(x)>0)) + (1*(abs(imag(x))<0.6325));
demod_fcn_64qam = @(x) (32*(real(x)>0)) + (16*(abs(real(x))<0.6172)) + (8*((abs(real(x))<(0.9258))&&((abs(real(x))>(0.3086))))) + (4*(imag(x)>0)) + (2*(abs(imag(x))<0.6172)) + (1*((abs(imag(x))<(0.9258))&&((abs(imag(x))>(0.3086)))));

switch(MOD_ORDER)
    case 2         % BPSK
        rx_data_mrc = arrayfun(demod_fcn_bpsk, rx_syms_mrc);
        rx_data_1 = arrayfun(demod_fcn_bpsk, rx_syms_1);
        rx_data_2 = arrayfun(demod_fcn_bpsk, rx_syms_2);
    case 4         % QPSK
        rx_data_mrc = arrayfun(demod_fcn_qpsk, rx_syms_mrc);
        rx_data_1 = arrayfun(demod_fcn_qpsk, rx_syms_1);
        rx_data_2 = arrayfun(demod_fcn_qpsk, rx_syms_2);
    case 16        % 16-QAM
        rx_data_mrc = arrayfun(demod_fcn_16qam, rx_syms_mrc);
        rx_data_1 = arrayfun(demod_fcn_16qam, rx_syms_1);
        rx_data_2 = arrayfun(demod_fcn_16qam, rx_syms_2);
    case 64        % 64-QAM
        rx_data_mrc = arrayfun(demod_fcn_64qam, rx_syms_mrc);
        rx_data_1 = arrayfun(demod_fcn_64qam, rx_syms_1);
        rx_data_2 = arrayfun(demod_fcn_64qam, rx_syms_2);
end

%% Plot Results
cf = 0;

% Tx signal
cf = cf + 1;
figure(cf);clf;

subplot(2,1,1);
plot(real(tx_vecs_iris), 'b');
axis([0 length(tx_vecs_iris) -TX_SCALE TX_SCALE])
grid on;
title('Tx Waveform (I)');

subplot(2,1,2);
plot(imag(tx_vecs_iris), 'r');
axis([0 length(tx_vecs_iris) -TX_SCALE TX_SCALE])
grid on;
title('Tx Waveform (Q)');

if(WRITE_PNG_FILES)
    print(gcf,sprintf('wl_ofdm_plots_%s_txIQ', example_mode_string), '-dpng', '-r96', '-painters')
end

% Rx signal
cf = cf + 1;
figure(cf);
subplot(2,2,1);
plot(real(rx_vec_air(:,1)), 'b');
axis([0 length(rx_vec_air) -TX_SCALE TX_SCALE])
grid on;
title('Rx Waveform (I) 1');

subplot(2,2,2);
plot(imag(rx_vec_air(:,1)), 'r');
axis([0 length(rx_vec_air) -TX_SCALE TX_SCALE])
grid on;
title('Rx Waveform (Q) 1');

subplot(2,2,3);
plot(real(rx_vec_air(:,2)), 'b');
axis([0 length(rx_vec_air) -TX_SCALE TX_SCALE])
grid on;
title('Rx Waveform (I) 2');

subplot(2,2,4);
plot(imag(rx_vec_air(:,2)), 'r');
axis([0 length(rx_vec_air) -TX_SCALE TX_SCALE])
grid on;
title('Rx Waveform (Q) 2');

if(WRITE_PNG_FILES)
    print(gcf,sprintf('wl_ofdm_plots_%s_rxIQ', example_mode_string), '-dpng', '-r96', '-painters')
end

% Rx LTS correlation (Both branches)
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

% Channel Estimates (MRC)
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

% Pilot phase error estimate
cf = cf + 1;
figure(cf); clf;
subplot(2,1,1)
plot(pilot_phase_err_mrc, 'b', 'LineWidth', 2);
title('Phase Error Estimates')
xlabel('OFDM Symbol Index')
ylabel('Radians')
axis([1 N_OFDM_SYM -3.2 3.2])
grid on

h = colorbar;
set(h,'Visible','off');

subplot(2,1,2)
imagesc(1:N_OFDM_SYM, (SC_IND_DATA - N_SC/2), fftshift(pilot_phase_sfo_corr_mrc,1))
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

plot(payload_syms_mat_mrc(:),'ro','MarkerSize',1);
axis square; axis(1.5*[-1 1 -1 1]);
xlabel('Inphase')
ylabel('Quadrature')
grid on;
hold on;

plot(tx_syms_mat(:),'bo');
title('Tx and Rx Constellations (MRC)')
legend('Rx','Tx','Location','EastOutside');

cf = cf + 1;
figure(cf); clf;

plot(payload_syms_mat_1(:),'ro','MarkerSize',1);
axis square; axis(1.5*[-1 1 -1 1]);
xlabel('Inphase')
ylabel('Quadrature')
grid on;
hold on;

plot(tx_syms_mat(:),'bo');
title('Tx and Rx Constellations (branch 1)')
legend('Rx','Tx','Location','EastOutside');

cf = cf + 1;
figure(cf); clf;

plot(payload_syms_mat_2(:),'ro','MarkerSize',1);
axis square; axis(1.5*[-1 1 -1 1]);
xlabel('Inphase')
ylabel('Quadrature')
grid on;
hold on;

plot(tx_syms_mat(:),'bo');
title('Tx and Rx Constellations (branch 2)')
legend('Rx','Tx','Location','EastOutside');

if(WRITE_PNG_FILES)
    print(gcf,sprintf('wl_ofdm_plots_%s_constellations', example_mode_string), '-dpng', '-r96', '-painters')
end


% EVM & SNR
cf = cf + 1;
figure(cf); clf;

evm_mat_mrc = abs(payload_syms_mat_mrc - tx_syms_mat).^2;
aevms_mrc = mean(evm_mat_mrc(:));
snr_mrc = 10*log10(1./aevms_mrc);

subplot(2,1,1)
plot(100*evm_mat_mrc(:),'o','MarkerSize',1)
axis tight
hold on
plot([1 length(evm_mat_mrc(:))], 100*[aevms_mrc, aevms_mrc],'r','LineWidth',4)
myAxis = axis;
h = text(round(.05*length(evm_mat_mrc(:))), 100*aevms_mrc+ .1*(myAxis(4)-myAxis(3)), sprintf('Effective SNR: %.1f dB', snr_mrc));
set(h,'Color',[1 0 0])
set(h,'FontWeight','bold')
set(h,'FontSize',10)
set(h,'EdgeColor',[1 0 0])
set(h,'BackgroundColor',[1 1 1])
hold off
xlabel('Data Symbol Index')
ylabel('EVM (%)');
legend('Per-Symbol EVM','Average EVM','Location','NorthWest');
title('EVM vs. Data Symbol Index (MRC)')
grid on

subplot(2,1,2)
imagesc(1:N_OFDM_SYM, (SC_IND_DATA - N_SC/2), 100*fftshift(evm_mat_mrc,1))

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


cf = cf + 1;
figure(cf); clf;

evm_mat_1 = abs(payload_syms_mat_1 - tx_syms_mat).^2;
aevms_1 = mean(evm_mat_1(:));
snr_1 = 10*log10(1./aevms_1);

subplot(2,1,1)
plot(100*evm_mat_1(:),'o','MarkerSize',1)
axis tight
hold on
plot([1 length(evm_mat_1(:))], 100*[aevms_1, aevms_1],'r','LineWidth',4)
myAxis = axis;
h = text(round(.05*length(evm_mat_1(:))), 100*aevms_1 + .1*(myAxis(4)-myAxis(3)), sprintf('Effective SNR: %.1f dB', snr_1));
set(h,'Color',[1 0 0])
set(h,'FontWeight','bold')
set(h,'FontSize',10)
set(h,'EdgeColor',[1 0 0])
set(h,'BackgroundColor',[1 1 1])
hold off
xlabel('Data Symbol Index')
ylabel('EVM (%)');
legend('Per-Symbol EVM','Average EVM','Location','NorthWest');
title('EVM vs. Data Symbol Index (branch 1)')
grid on

subplot(2,1,2)
imagesc(1:N_OFDM_SYM, (SC_IND_DATA - N_SC/2), 100*fftshift(evm_mat_mrc,1))

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



cf = cf + 1;
figure(cf); clf;

evm_mat_2 = abs(payload_syms_mat_2 - tx_syms_mat).^2;
aevms_2 = mean(evm_mat_2(:));
snr_2 = 10*log10(1./aevms_2);

subplot(2,1,1)
plot(100*evm_mat_2(:),'o','MarkerSize',1)
axis tight
hold on
plot([1 length(evm_mat_2(:))], 100*[aevms_2, aevms_2],'r','LineWidth',4)
myAxis = axis;
h = text(round(.05*length(evm_mat_2(:))), 100*aevms_2+ .1*(myAxis(4)-myAxis(3)), sprintf('Effective SNR: %.1f dB', snr_2));
set(h,'Color',[1 0 0])
set(h,'FontWeight','bold')
set(h,'FontSize',10)
set(h,'EdgeColor',[1 0 0])
set(h,'BackgroundColor',[1 1 1])
hold off
xlabel('Data Symbol Index')
ylabel('EVM (%)');
legend('Per-Symbol EVM','Average EVM','Location','NorthWest');
title('EVM vs. Data Symbol Index (branch 2)')
grid on

subplot(2,1,2)
imagesc(1:N_OFDM_SYM, (SC_IND_DATA - N_SC/2), 100*fftshift(evm_mat_mrc,1))

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

sym_errs = sum(tx_data ~= rx_data_mrc);
bit_errs = length(find(dec2bin(bitxor(tx_data, rx_data_mrc),8) == '1'));
rx_evm   = sqrt(sum((real(rx_syms_mrc) - real(tx_syms)).^2 + (imag(rx_syms_mrc) - imag(tx_syms)).^2)/(length(SC_IND_DATA) * N_OFDM_SYM));

fprintf('\n MRC Results:\n');
fprintf('Num Bytes:   %d\n', N_DATA_SYMS * log2(MOD_ORDER) / 8);
fprintf('Sym Errors:  %d (of %d total symbols)\n', sym_errs, N_DATA_SYMS);
fprintf('Bit Errors:  %d (of %d total bits)\n', bit_errs, N_DATA_SYMS * log2(MOD_ORDER));

sc_data_idx = [(2:27)'; (39:64)' ];
% SNR estimation based on the LTS signals
n_var_1 = sum( sum( abs(H0_b1(sc_data_idx,:) - repmat(H_b1(sc_data_idx), 1,2) ).^2,2 ))/(52*2);
n_var_2 = sum( sum( abs(H0_b2(sc_data_idx,:) - repmat(H_b2(sc_data_idx), 1,2) ).^2,2 ))/(52*2);
h_pow_1 =  TX_SCALE*H_b1(sc_data_idx)'*H_b1(sc_data_idx)/(52);
h_pow_2 =  TX_SCALE*H_b2(sc_data_idx)'*H_b2(sc_data_idx)/(52);
h_pow_mrc = TX_SCALE*sum(H_pow(sc_data_idx,1))/52;

n_var_mrc = (n_var_1 + n_var_2)/2;
fprintf('\n\tEVM-based SNRs:\n');
fprintf('Branch 1 SNR:%3.2f \tBranch 2 SNR:%3.2f\t MRC SNR:%3.2f\n',...
    snr_1, snr_2, snr_mrc);

snr_1_hat = 10*log10(h_pow_1/n_var_1);
snr_2_hat = 10*log10(h_pow_2/n_var_2);
snr_mrc_hat = 10*log10(h_pow_mrc/n_var_mrc);
snr_1_plus_snr = 10*log10(h_pow_1/n_var_1 +  h_pow_2/n_var_2);
fprintf('\tPilot SNR Estimates:\n');
fprintf('Branch 1 SNR:%3.2f \tBranch 2 SNR:%3.2f\t MRC SNR:%3.2f\n',...
    snr_1_hat, snr_2_hat, snr_mrc_hat);
