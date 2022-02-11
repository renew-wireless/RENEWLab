%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Dowlink Beamforming Demo with explicit pilot CSI
%---------------------------------------------------------------------
% Original code copyright Mango Communications, Inc.
% Distributed under the WARP License http://warpproject.org/license
% Copyright (c) 2018-2019, Rice University
% RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
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

%Iris params:
USE_HUB                 = 1;
WIRED_UE                 = 0;
TX_FRQ                  = 3.6e9;    
RX_FRQ                  = TX_FRQ;
TX_GN                   = 80;
RX_GN                   = 60;
SMPL_RT                 = 5e6;  
N_FRM                   = 1;
bs_ids = string.empty();
bs_sched = string.empty();

% Waveform params
TX_SCALE                = 0.5;         % Scale for Tx waveform ([0:1])

% OFDM params
SC_IND_PILOTS           = [8 22 44 58];                           % Pilot subcarrier indices
SC_IND_DATA             = [2:7 9:21 23:27 39:43 45:57 59:64];     % Data subcarrier indices
SC_IND_DATA_PILOT       = [2:27 39:64]';
N_SC                    = 64;                                     % Number of subcarriers
CP_LEN                  = 16;                                     % Cyclic prefix length
N_SYM_SAMP              = N_SC + CP_LEN;                          % Number of samples that will go over the air
N_SAMP                  = 4096;                                   % N_ZPAD_PRE + data_len + N_ZPAD_POST;
N_ZPAD_PRE              = 128;                                    % Zero-padding prefix for Iris
N_ZPAD_POST             = 128;                                     % Zero-padding postfix for Iris
N_OFDM_SYMS              = floor((N_SAMP - N_ZPAD_PRE - N_ZPAD_POST) / N_SYM_SAMP);  % Number of OFDM symbols for burst, it needs to be less than 47
N_PILOTS_SYMS           = 2;
N_DATA_SYMS             = (N_OFDM_SYMS - N_PILOTS_SYMS);       % Number of data symbols (one per data-bearing subcarrier per OFDM symbol)
N_DATA_SC               = N_DATA_SYMS * length(SC_IND_DATA);
MOD_ORDER               = 4;           % Modulation order (2/4/16/64 = BSPK/QPSK/16-QAM/64-QAM)

% Rx processing params
FFT_OFFSET                    = 16;          % Number of CP samples to use in FFT (on average)
AUTO_OFFSET_FIND = 1;
PILOT_PLOT = 1;
DOWNLINK_PLOT = 1;

%% Define the preamble
% LTS for fine CFO and channel estimation
lts_f = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 ...
    1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1];
lts_t = ifft(lts_f, 64); %time domain
lts = [lts_t(49:64) lts_t];


%% Init Iris nodes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set up the Iris experiment
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create BS Hub and UE objects. Note: BS object is a collection of Iris
% nodes.

if USE_HUB
    % RENEW LAB BS
    hub_id = "FH4B000011";
else
    hub_id = [];
end
% RENEW Lab BS
% chain2 = ["RF3E000087", "RF3E000084", "RF3E000107", "RF3E000110", "RF3E000086", "RF3E000162", "RF3E000127", "RF3E000597"];
% chain3 = ["RF3E000346", "RF3E000543", "RF3E000594", "RF3E000404", "RF3E000616", "RF3E000622", "RF3E000601", "RF3E000602"];
% chain4 = ["RF3E000146", "RF3E000122", "RF3E000150", "RF3E000128", "RF3E000168", "RF3E000136", "RF3E000213", "RF3E000142"];
% chain5 = ["RF3E000356", "RF3E000546", "RF3E000620", "RF3E000609", "RF3E000604", "RF3E000612", "RF3E000640", "RF3E000551"];
% chain6 = ["RF3E000208", "RF3E000636", "RF3E000632", "RF3E000568", "RF3E000558", "RF3E000633", "RF3E000566", "RF3E000635"];
% % RENEW Stadium BS, Note: bad boards at chain1:5
chain1 = ["RF3E000331", "RF3E000379", "RF3E000144", "RF3E000036", "RF3E000249", "RF3E000115", "RF3E000077", "RF3E000066"];
chain2 = ["RF3E000222", "RF3E000175", "RF3E000129", "RF3E000109", "RF3E000155", "RF3E000167", "RF3E000163", "RF3E000149"];
chain3 = ["RF3E000386", "RF3E000631", "RF3E000561", "RF3E000559", "RF3E000613", "RF3E000617", "RF3E000549", "RF3E000552"];
chain4 = ["RF3E000262", "RF3E000477", "RF3E000573", "RF3E000473", "RF3E000576", "RF3E000581", "RF3E000451", "RF3E000364"];
chain5 = ["RF3E000085", "RF3E000446", "RF3E000513", "RF3E000624", "RF3E000485", "RF3E000641", "RF3E000618", "RF3E000385"];

bs_ids = [chain2];
ue_ids= ["RF3E000027"]; %, "RF3D000016"]; %, "RF3E000180"];

% We send downlink pilots in the first phase
% And downlink data in the second phase

N_BS_NODE = length(bs_ids);
N_UE_NODE = length(ue_ids);

% Iris nodes' parameters
sdr_params = struct(...
    'bs_id', bs_ids, ...
    'ue_id', ue_ids,...
    'txfreq', TX_FRQ, ...
    'rxfreq', RX_FRQ, ...
    'txgain', TX_GN, ...
    'rxgain', RX_GN, ...
    'sample_rate', SMPL_RT);

%% Generate a payload of random integers
tx_data = randi(MOD_ORDER, N_UE_NODE, N_DATA_SC) - 1;

tx_syms = mod_sym(tx_data, MOD_ORDER);
% Reshape the symbol vector to a matrix with one column per OFDM symbol
tx_syms_mat = reshape(tx_syms, N_UE_NODE, length(SC_IND_DATA), N_DATA_SYMS);

% Define the pilot tone values as BPSK symbols
pilots = [1 1 -1 1].';

% Repeat the pilots across all OFDM symbols
pilots_mat = zeros(N_UE_NODE, length(SC_IND_PILOTS), N_DATA_SYMS);
for i=1:N_UE_NODE
    pilots_mat(i,:,:) = repmat(pilots, 1, N_DATA_SYMS);
end

% Construct the precoding input matrix
precoding_in_mat = zeros(N_UE_NODE, N_SC, N_OFDM_SYMS);

% Insert the data and pilot values; other subcarriers will remain at 0
for i = 1:N_PILOTS_SYMS
    precoding_in_mat(:, :, i) = repmat(lts_f, N_UE_NODE, 1);
end
precoding_in_mat(:, SC_IND_DATA, N_PILOTS_SYMS + 1:end)   = tx_syms_mat;
precoding_in_mat(:, SC_IND_PILOTS, N_PILOTS_SYMS + 1:end) = pilots_mat;

% Downlink TX pilots
DATA_REP = floor(N_OFDM_SYMS / N_BS_NODE);
data_len = N_BS_NODE * DATA_REP * N_SYM_SAMP;
postfix_len = N_SAMP - data_len - N_ZPAD_PRE;
exp_pilot_tx = zeros(N_BS_NODE, data_len);
for jp = 1:N_BS_NODE
    for rp = 1:DATA_REP
        start_index = ((rp - 1) + (jp - 1) * DATA_REP) * N_SYM_SAMP;
        exp_pilot_tx(jp, start_index + 1: start_index + N_SYM_SAMP) = lts;
    end
end
tx_pilot_signal = zeros(N_BS_NODE, N_SAMP);
for i=1:N_BS_NODE
    tx_pilot_signal(i, :) = [zeros(1, N_ZPAD_PRE) exp_pilot_tx(i, :) zeros(1, postfix_len)];
end

mimo_handle = mimo_driver(sdr_params);
rx_vec_iris = mimo_handle.mimo_txrx_downlink(tx_pilot_signal.', 1, N_ZPAD_PRE);
if isempty(rx_vec_iris)
    disp('Did not receive good signal during pilot phase!');
    return;
end
rx_vec_iris = rx_vec_iris.';

dl_pilot_rx = zeros(data_len, N_UE_NODE);
rx_fft = zeros(N_SC, N_BS_NODE, N_UE_NODE);
downlink_pilot_csi = zeros(N_SC, N_BS_NODE, N_UE_NODE);

rx_data_start = 189;
for iue = 1:N_UE_NODE
    dl_pilot_rx(:, iue) = rx_vec_iris(rx_data_start: rx_data_start + data_len - 1, iue);
    for ibs = 1:N_BS_NODE
        for rid = 1:DATA_REP
            start_index = CP_LEN + ((rid - 1) + (ibs - 1) * DATA_REP) * N_SYM_SAMP;
            rx_fft(:, ibs, iue) = rx_fft(:, ibs, iue) + fft(dl_pilot_rx(1 + start_index : start_index + N_SC, iue), N_SC);
        end
    end
end

for iue = 1:N_UE_NODE
    for ibs = 1:N_BS_NODE
        downlink_pilot_csi(:, ibs, iue) = (rx_fft(:, ibs, iue) / DATA_REP) .* lts_f.';
    end
end


%% Downlink CSI Calculation and Zeroforcing
ifft_in_mat = zeros(N_BS_NODE, N_SC, N_OFDM_SYMS);
for isc =1:N_SC
    downlink_beam_weights = pinv(squeeze(downlink_pilot_csi(isc, :, :)));
    for isym = 1:N_OFDM_SYMS
        ifft_in_mat(:, isc, isym) = downlink_beam_weights.' * precoding_in_mat(:, isc, isym);
    end
end

%% IFFT

%Perform the IFFT
tx_payload_mat = zeros(N_BS_NODE, N_SYM_SAMP, N_OFDM_SYMS);
for ibs = 1:N_BS_NODE
    for isym = 1:N_OFDM_SYMS
        tx_sym = squeeze(ifft(ifft_in_mat(ibs, :, isym)));
        tx_payload_mat(ibs, :, isym) = [tx_sym(end - CP_LEN + 1: end) tx_sym].';
    end
end


% Reshape to a vector
tx_payload_vec = reshape(tx_payload_mat, N_BS_NODE, numel(tx_payload_mat(1, :, :)));

rx_vec_downlink = mimo_handle.mimo_txrx_downlink(tx_payload_vec.', 1, N_ZPAD_PRE);
if isempty(rx_vec_downlink)
    disp('Did not receive good signal during pilot phase!');
    return;
end
rx_vec_downlink = rx_vec_downlink.';


mimo_handle.close();

%% Process downlink receive signal
dl_data_start = 188;

N_RX_OFDM_SYMS = min(N_OFDM_SYMS, floor((N_SAMP - dl_data_start)/N_SYM_SAMP));
rx_dl_vec = zeros(N_UE_NODE, N_RX_OFDM_SYMS * N_SYM_SAMP);
for iue=1:N_UE_NODE
    end_idx = min(4096, dl_data_start + N_RX_OFDM_SYMS * N_SYM_SAMP - 1);
    rx_dl_vec(iue, :) = rx_vec_downlink(iue, dl_data_start: end_idx);
end
rx_dl_mat = reshape(rx_dl_vec, N_UE_NODE, N_SYM_SAMP, N_RX_OFDM_SYMS);
if(CP_LEN > 0)
    rx_dl_mat = rx_dl_mat(:, CP_LEN+1:end, :);
end
rx_dl_f_mat = fft(rx_dl_mat, N_SC, 2);
rx_lts_f = zeros(N_UE_NODE, N_SC);
for p=1:N_PILOTS_SYMS
    rx_lts_f = rx_lts_f + repmat(lts_f, N_UE_NODE, 1).*rx_dl_f_mat(:, :, p);
end
dl_syms_f_mat = rx_dl_f_mat(:, :, N_PILOTS_SYMS+1:end);
rx_H_est = rx_lts_f/N_PILOTS_SYMS;

N_RX_DATA_OFDM_SYMS = N_RX_OFDM_SYMS - N_PILOTS_SYMS;
dl_syms_eq_mat = zeros(N_UE_NODE, N_SC, N_RX_DATA_OFDM_SYMS);
for i=1:N_RX_DATA_OFDM_SYMS
    dl_syms_eq_mat(:,:,i) = squeeze(dl_syms_f_mat(:,:,i))./rx_H_est;
end

pilots_eq_mat = dl_syms_eq_mat(:,SC_IND_PILOTS,:);
%pilots_eq_mat_comp = zeros(size(pilots_eq_mat));
%for iue=1:N_UE_NODE
%    pilots_eq_mat_comp(iue, :, :) = pilots_eq_mat(iue, :, :).*pilots_mat(iue, :, :);
%end
pilots_eq_mat_comp = pilots_eq_mat.*pilots_mat;

pilot_dl_phase_err = squeeze(angle(mean(pilots_eq_mat_comp,2)));

pilot_dl_phase_corr = zeros(N_UE_NODE, N_SC, N_RX_DATA_OFDM_SYMS);
for i=1:N_SC
    pilot_dl_phase_corr(:,i,:) = exp(-1i*pilot_dl_phase_err);
end
  
% Apply the pilot phase correction per symbol
dl_syms_eq_pc_mat = dl_syms_eq_mat.* pilot_dl_phase_corr;
payload_dl_syms_mat = dl_syms_eq_pc_mat(:, SC_IND_DATA, :);
payload_dl_syms_mat = reshape(payload_dl_syms_mat, N_UE_NODE, numel(payload_dl_syms_mat(1,:,:)));

%% Demodulate
rx_data = demod_sym(payload_dl_syms_mat ,MOD_ORDER);

sym_errs = sum(tx_data(:) ~= rx_data(:));
bit_errs = length(find(dec2bin(bitxor(tx_data(:), rx_data(:)),8) == '1'));
evm_mat = double.empty();
aevms = zeros(N_UE_NODE,1);
snr_mat = zeros(N_UE_NODE,1);

for sp = 1:N_UE_NODE
    tx_vec = tx_syms_mat(sp, :,:);
    evm_mat(:,sp)  = abs(tx_vec(:) - payload_dl_syms_mat(sp, :).' ).^2;
    aevms(sp) = mean(evm_mat(:,sp));
    snr_mat(sp) = -10*log10(aevms (sp));
end

%% Plotting
cf = 0;

% Downlink Pilot Rx Vectors Plots
if PILOT_PLOT
    cf = cf + 1;
    figure(cf);clf;

    for i = 1:N_UE_NODE
        subplot(N_UE_NODE, 1, i);
        plot(real(rx_vec_iris(:, i)));
        axis([0 N_SAMP -1 1])
        grid on;
        title('Received Downlink Pilots (Real)');
    end
end

% Downlink Rx Vector and Constellation Plots
if DOWNLINK_PLOT
    cf = cf + 1;
    figure(cf);clf;

    for i = 1:N_UE_NODE
        subplot(N_UE_NODE, 1, i);
        plot(real(rx_vec_downlink(i, :)));
        xline(dl_data_start,'--r')
        axis([0 N_SAMP -1 1])
        grid on;
        title('Received Downlink Data (Real)');
    end
    % DL
    cf = cf + 1;
    figure(cf); clf;
    
    for i = 1:N_UE_NODE
        subplot(N_UE_NODE,1,i)
        plot(payload_dl_syms_mat(i, :),'ro','MarkerSize',1);
        axis square; axis(2.5*[-1 1 -1 1]);
        grid on;
        hold on;

        plot(tx_syms(:),'bo');
        title('Downlink Tx and Rx Constellations')
        legend('Rx','Tx');
    end
end

sym_errs
snr_mat

