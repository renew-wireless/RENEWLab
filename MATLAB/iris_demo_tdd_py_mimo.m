
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
close all;

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

if SIM_MOD
    chan_type               = "rayleigh";
    sim_SNR_db              = 15;    
end
fprintf("Channel type: %s \n",chan_type);

% Iris params:
N_BS_NODE               = 8;
N_UE                    = 2;
TX_FRQ                  = 3.6e9;
RX_FRQ                  = TX_FRQ;
TX_GN                   = 40;
RX_GN                   = 20;
SMPL_RT                 = 5e6;
 

b_ids                   = string.empty();
b_scheds                = string.empty();
ue_ids                  = string.empty();
ue_scheds               = string.empty();

MIMO_ALG                = 'ZF';      % MIMO ALGORITHM: ZF or Conjugate 
fprintf("MIMO algorithm: %s \n",MIMO_ALG);

% Waveform params
N_OFDM_SYM              = 46;         % Number of OFDM symbols for burst, it needs to be less than 47
MOD_ORDER               = 16;          % Modulation order (2/4/16/64 = BSPK/QPSK/16-QAM/64-QAM)
TX_SCALE                = 1;         % Scale for Tx waveform ([0:1])

% OFDM params
SC_IND_PILOTS           = [8 22 44 58];                           % Pilot subcarrier indices
SC_IND_DATA             = [2:7 9:21 23:27 39:43 45:57 59:64];     % Data subcarrier indices
SC_IND_DATA_PILOT       = [2:27 39:64]';
N_SC                    = 64;                                     % Number of subcarriers
CP_LEN                  = 16;                                     % Cyclic prefix length
N_DATA_SYMS             = N_OFDM_SYM * length(SC_IND_DATA);       % Number of data symbols (one per data-bearing subcarrier per OFDM symbol) per UE
N_LTS_SYM               = 2;                                      % Number of 
N_SYM_SAMP              = N_SC + CP_LEN;                          % Number of samples that will go over the air
N_ZPAD_PRE              = 90;                                     % Zero-padding prefix for Iris
N_ZPAD_POST             = N_ZPAD_PRE -14;                         % Zero-padding postfix for Iris

% Rx processing params
FFT_OFFSET                    = 16;          % Number of CP samples to use in FFT (on average)
LTS_CORR_THRESH               = 0.50;         % Normalized threshold for LTS correlation
DO_APPLY_PHASE_ERR_CORRECTION = 1;           % Enable Residual CFO estimation/correction

%% Define the preamble
% LTS for fine CFO and channel estimation
lts_f = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 ...
    1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1].';
lts_t = ifft(lts_f, N_SC); %time domain

% Arrange time-orthogonal pilots 
preamble_common = [lts_t(33:64); repmat(lts_t,N_LTS_SYM,1)];
l_pre = length(preamble_common);
pre_z = zeros(size(preamble_common));
preamble = zeros(N_UE * l_pre, N_UE);
for jp = 1:N_UE
    preamble((jp-1)*l_pre + 1: (jp-1)*l_pre+l_pre,jp) = preamble_common;
end

%preamble = [ preamble_common pre_z;pre_z preamble_common];
cor_rng  = length(preamble) +  N_ZPAD_PRE + 100;
%% Generate a payload of random integers
tx_data = randi(MOD_ORDER, N_DATA_SYMS, N_UE) - 1;

tx_syms = mod_sym(tx_data, MOD_ORDER);

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

%% SIMULATION:
if (SIM_MOD) 
    
    rx_vec_iris = getRxVec(tx_vecs_iris, N_BS_NODE, N_UE, chan_type, sim_SNR_db);
    rx_vec_iris = rx_vec_iris.'; % just to agree with what the hardware spits out.
    
%% Init Iris nodes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set up the Iris experimenty
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
else
    % Create a two Iris node objects:
    b_ids = ["0328", "0339", "0268", "0282", "0344", "0233", "0334", "0402"];
    ue_ids= ["RF3C000025", "RF3C000045"];

    b_prim_sched = "PGGGGGGGGGGRG";           % BS primary noede's schedule: Send Beacon only from one Iris board
    b_sec_sched = "GGGGGGGGGGGRG"; 
    ue_sched = "GGGGGGGGGGGPG";               % UE schedule

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
        'txfreq', TX_FRQ, ...
        'rxfreq', TX_FRQ, ...
        'txgain', TX_GN, ...
        'rxgain', RX_GN, ...
        'sample_rate', SMPL_RT, ...
        'n_samp', n_samp, ...          % number of samples per frame time.
        'tdd_sched', b_scheds, ...     % number of zero-paddes samples
        'n_zpad_samp', (N_ZPAD_PRE + N_ZPAD_POST) ...
        );

    sdr_params(2) = sdr_params(1);
    sdr_params(2).id =  ue_ids(1);
    sdr_params(2).n_chain = 1;
    sdr_params(2).rxfreq = RX_FRQ;
    sdr_params(2).txfreq = RX_FRQ;
    
    
    % Can give all scheds together?
    sdr_params(2).tdd_sched = ue_scheds(1);
    
    sdr_params(3)= sdr_params(2);
    sdr_params(3).id =  ue_ids(2);
    
    SAMP_FREQ = sdr_params(1).sample_rate;
    
    rx_vec_iris = getRxVec(tx_vecs_iris, N_BS_NODE, N_UE, "iris", [], sdr_params(1), sdr_params(2:3));

end

%load 'rx_mimo_data_08_02.mat';
l_rx_dec=length(rx_vec_iris);

%% Correlate for LTS
% Complex cross correlation of Rx waveform with time-domain LTS

a = 1;
unos = ones(size(preamble_common'));
lts_corr = zeros(N_BS_NODE, length(rx_vec_iris));
rx_lts_mat = double.empty();
payload_ind = int32.empty();
payload_rx = double.empty();
lts_peaks = zeros(N_BS_NODE, N_UE);
for ibs =1:N_BS_NODE
        v0 = filter(fliplr(preamble_common'),a,rx_vec_iris(ibs,:));
        v1 = filter(unos,a,abs(rx_vec_iris(ibs,:)).^2);
        lts_corr(ibs,:) = (abs(v0).^2)./v1; % normalized correlation
        sort_corr = sort(lts_corr(ibs,:), 'descend');
        rho_max = sort_corr(1:N_UE);
        lts_peaks(ibs,:) = find(lts_corr(ibs,:) >= min(rho_max));
        
        % position of the last peak
        ipos = max(lts_peaks(ibs,:));
        
        payload_ind(ibs) = ipos +1;
        pream_ind_ibs = payload_ind(ibs) - length(preamble);
        % minimum sanity check of the correlator's output
        if pream_ind_ibs < 1
            fprintf("Bad correlator output!\n");
            figure,
            for sp = 1:N_BS_NODE
                subplot(N_BS_NODE,1,sp);
                plot(lts_corr(sp,:));
                grid on;
                xlabel('Samples');
                y_label = sprintf('Anetnna %d',sp);
                ylabel(y_label);
            end
            sgtitle('LTS correlations accross antennas')
            
            return; 
        else
            rx_lts_mat(ibs,:) = rx_vec_iris(ibs, pream_ind_ibs: pream_ind_ibs + length(preamble) -1 );
            payload_rx(ibs,:) = rx_vec_iris(ibs, payload_ind(ibs) : payload_ind(ibs)+(N_OFDM_SYM)*(N_SC +CP_LEN)-1);
        end
end

if DEBUG
    figure,
    for sp = 1:N_BS_NODE
        subplot(N_BS_NODE,1,sp);
        plot(lts_corr(sp,:));
        grid on;
        xlabel('Samples');
        y_label = sprintf('Anetnna %d',sp);
        ylabel(y_label);
    end
    sgtitle('LTS correlations accross antennas')
end



%% Rx processing
% Construct a matrix from the received pilots
n_plt_samp = floor(length(preamble)/ N_UE);     % number of samples in a per-UE pilot 
Y_lts = zeros(N_BS_NODE,N_UE, n_plt_samp);
for iue = 1:N_UE
   plt_j_ix = (iue-1) * n_plt_samp +1:iue * n_plt_samp;
   Y_lts(:,iue,:) = rx_lts_mat(:,plt_j_ix);
end
% Take N_SC spamples from each LTS
rx_lts_idx = CP_LEN +1 : N_LTS_SYM * N_SC +CP_LEN;
Y_lts = Y_lts(:,:,rx_lts_idx);
% Reshape the matix to have each lts pilot in a different dimension:
% N_BS_NODE x N_UE x 64 x 2
Y_lts = reshape(Y_lts, N_BS_NODE, N_UE, [], N_LTS_SYM); 
% Take FFT:
Y_lts_f = fft(Y_lts, N_SC,3);
% Construct known pilot matrix to use i next step:
lts_f_mat = repmat(lts_f, N_BS_NODE *N_UE * N_LTS_SYM,1);
lts_f_mat = reshape(lts_f_mat, [], N_LTS_SYM, N_UE, N_BS_NODE);
lts_f_mat = permute(lts_f_mat, [4 3 1 2]);

% Get the channel by dividing by the pilots
G_lts = Y_lts_f ./ lts_f_mat;
% Estimate the channel by averaging over the two LTS symbols:
H_hat = mean(G_lts, 4);

% Reshape the payload and take subcarriers without the CP
payload_rx = reshape(payload_rx,N_BS_NODE, (N_SC + CP_LEN),[]);
payload_noCP = payload_rx(:,CP_LEN-FFT_OFFSET+(1:N_SC),:);
% Take FFT
Y_data = fft(payload_noCP, N_SC, 2);

% Apply ZF by multiplying the pseudo inverse of H_hat[N_BS_NODE x NUE] for each suubcarrier:
nz_sc = find(lts_f ~= 0); % non-zero subcarriers
syms_eq = zeros(N_UE,N_SC,N_OFDM_SYM);
channel_condition = double.empty();
channel_condition_db = double.empty();
for j=1:length(nz_sc)
    
    if(strcmp(MIMO_ALG,'ZF'))
       % Pseudo-inverse:(H'*H)^(-1)*H':
        HH_inv = inv((squeeze(H_hat(:,:, nz_sc(j) ) )' * squeeze(H_hat(:,:, nz_sc(j) ) ) ) ) * squeeze(H_hat(:,:, nz_sc(j) ) )';
        x = HH_inv*squeeze(Y_data(:,nz_sc(j),:));
    else
        % Conjugate:
        H_pow = diag(abs (H_hat(:,:, nz_sc(j) )' * H_hat(:,:, nz_sc(j) ) ));
        % normalization: 
        x = (H_hat(:,:, nz_sc(j) )') * squeeze(Y_data(:,nz_sc(j),:))./ repmat(H_pow, 1, N_OFDM_SYM);
    end
    syms_eq(:,nz_sc(j),:) = x;
    channel_condition(nz_sc(j)) = cond(H_hat(:,:,nz_sc(j) ) );
    channel_condition_db(nz_sc(j)) = 10*log10(channel_condition(nz_sc(j)) );
end

%DO_APPLY_PHASE_ERR_CORRECTION = 1

if DO_APPLY_PHASE_ERR_CORRECTION
    % Extract the pilots and calculate per-symbol phase error
    pilots_f_mat = syms_eq(:,SC_IND_PILOTS,:);
    pilots_f_mat_comp = pilots_f_mat.* permute(pilots_mat, [3 1 2]);
    pilot_phase_err = angle(mean(pilots_f_mat_comp,2));
    
else
	% Define an empty phase correction vector (used by plotting code below)
    pilot_phase_err = zeros(N_UE, 1, N_OFDM_SYM);
end

pilot_phase_err_corr = repmat(pilot_phase_err, 1, N_SC, 1);
pilot_phase_corr = exp(-1i*(pilot_phase_err_corr));

% Apply the pilot phase correction per symbol
syms_eq_pc = syms_eq.* pilot_phase_corr;

% Take only data SCs
syms_eq_pc = syms_eq_pc(:,SC_IND_DATA,:);

% Reshape the 3-D matrix to 2-D:
syms_eq_pc = reshape(syms_eq_pc, N_UE, [] );
syms_eq_pc = syms_eq_pc.';


%% Demodulate
rx_data = demod_sym(syms_eq_pc ,MOD_ORDER);

%% Plot results

cf = 0;
fst_clr = [0, 0.4470, 0.7410];
sec_clr = [0.8500, 0.3250, 0.0980];

% Tx signal
cf = cf + 1;
figure(cf); clf;
for sp=1:N_UE
    subplot(N_UE,2,2*(sp -1) + 1);
    plot(real(tx_vecs_iris(:,sp)));
    axis([0 length(tx_vecs_iris(:,sp)) -TX_SCALE TX_SCALE])
    grid on;
    title(sprintf('UE %d Tx Waveform (I)', sp));

    subplot(N_UE,2,2*sp);
    plot(imag(tx_vecs_iris(:,sp)), 'color' , sec_clr);
    axis([0 length(tx_vecs_iris(:,sp)) -TX_SCALE TX_SCALE])
    grid on;
    title(sprintf('UE %d Tx Waveform (Q)',sp));
end

% Rx signal
cf = cf + 1;
figure(cf); clf;
for sp = 1:N_BS_NODE
    subplot(N_BS_NODE,2,2*(sp -1) + 1 );
    plot(real(rx_vec_iris(sp,:)));
    axis([0 length(rx_vec_iris(sp,:)) -TX_SCALE TX_SCALE])
    grid on;
    title(sprintf('BS antenna %d Rx Waveform (I)', sp));

    subplot(N_BS_NODE,2,2*sp);
    plot(imag(rx_vec_iris(sp,:)), 'color' , sec_clr);
    axis([0 length(rx_vec_iris(sp,:)) -TX_SCALE TX_SCALE]);
    grid on;
    title(sprintf('BS antenna %d Rx Waveform (Q)', sp));
end 

%% Rx LTS correlation
cf = cf+ 1;
figure(cf); clf;
for sp = 1:N_BS_NODE
    subplot(N_BS_NODE,1,sp);
    plot(lts_corr(sp,:)) 
    grid on;
    xlabel('Samples');
    y_label = sprintf('Anetnna %d',sp);
    ylabel(y_label);
    myAxis = axis();
    axis([1, 1000, myAxis(3), myAxis(4)])
end
tb = annotation('textbox', [0 0.87 1 0.1], ...
    'String', 'LTS Correlations', ...
    'EdgeColor', 'none', ...
    'HorizontalAlignment', 'center');
tb.FontWeight = 'bold';

%% Constellations
cf = cf+ 1;
figure(cf); clf;
if N_BS_NODE >=4
    sp_rows = ceil(N_BS_NODE/4)+1;
else
    sp_rows = ceil(N_BS_NODE/2)+1;
end
sp_cols = ceil(N_BS_NODE/(sp_rows -1));

for sp=1:N_UE
    subplot(sp_rows,sp_cols, sp);
    plot(syms_eq_pc(:,sp),'o','MarkerSize',1, 'color', sec_clr);
    axis square; axis(1.5*[-1 1 -1 1]);
    grid on;
    hold on;
    plot(tx_syms(:, sp),'*', 'MarkerSize',10, 'LineWidth',2, 'color', fst_clr);
    title(sprintf('Equalized Uplink Tx and Rx symbols for stream %d', sp));
    legend('Rx','Tx','Location','EastOutside', 'fontsize', 12);
end

for sp=1:N_BS_NODE
    subplot(sp_rows,sp_cols, sp_cols+sp);
    plot(squeeze(Y_data(sp,:,:)),'co','MarkerSize',1);
    axis square; axis(max(max(max( abs( Y_data)) ))*[-1 1 -1 1]);
    title(sprintf('Unequalized received symbols at BS ant. %d', sp));
    grid on;
    hold on;
end

%% Channel Estimates
cf = cf + 1;
cond_clr = [0.8500, 0.250, 0.1980];

bw_span = (20/N_SC) * (-(N_SC/2):(N_SC/2 - 1)).';

figure(cf); clf;
sp = 0;
for ibs = 1:N_BS_NODE
    for iue = 1:N_UE
        sp = sp+1;
        subplot(N_BS_NODE,N_UE,sp); 
        bar(bw_span, fftshift(abs( squeeze(H_hat(ibs, iue, : ) ) ) ),1,'LineWidth', 1);
        axis([min(bw_span) max(bw_span) 0 1.1*max(abs( squeeze(H_hat(ibs, iue, :) ) ) )])
        grid on;
        title(sprintf('UE %d -> BS ant. %d Channel Estimates (Magnitude)', iue, ibs))
        xlabel('Baseband Frequency (MHz)')
    end
end

subplot(N_BS_NODE+1,1,N_BS_NODE+1);
bh = bar(bw_span, fftshift(channel_condition_db) ,1, 'LineWidth', 1);
set(bh,'FaceColor',cond_clr);
axis([min(bw_span) max(bw_span) 0 max(channel_condition_db)+1])
grid on;
title('Channel Condition (dB)')
xlabel('Baseband Frequency (MHz)')

%% EVM & SNR
sym_errs = sum(tx_data(:) ~= rx_data(:));
bit_errs = length(find(dec2bin(bitxor(tx_data(:), rx_data(:)),8) == '1'));

cf = cf + 1;
figure(cf); clf;
evm_mat = double.empty();
aevms = zeros(N_UE,1);
snr_mat = zeros(N_UE,1);

for sp = 1:N_UE
    tx_vec = tx_syms_mat(:,:,sp);
    evm_mat(:,sp)  = abs(tx_vec(:) - syms_eq_pc(:,sp) ).^2;
    aevms(sp) = mean(evm_mat(:,sp));
    snr_mat(sp) = -10*log10(aevms (sp));
    
    subplot(2,N_UE,sp)
    plot(100*evm_mat(:,sp),'o','MarkerSize',1)
    axis tight
    hold on
    plot([1 length(evm_mat(:,sp) )], 100*[aevms(sp), aevms(sp)],'color', sec_clr,'LineWidth',4)
    hold off
    xlabel('Data Symbol Index')
    ylabel('EVM (%)');
    legend('Per-Symbol EVM','Average EVM','Location','NorthWest');
    
    h = text(round(.05*length(evm_mat(:,sp))), 100*aevms(sp), sprintf('Effective SINR: %.1f dB', snr_mat(sp)));
    set(h,'Color',[1 0 0])
    set(h,'FontWeight','bold')
    set(h,'FontSize',10)
    set(h,'EdgeColor',[1 0 0])
    set(h,'BackgroundColor',[1 1 1])
    
    title(sprintf('Stream from UE %d', sp));
    grid on
    
end


for sp=1:N_UE
   subplot(2,N_UE,N_UE+sp);
   imagesc(1:N_OFDM_SYM, (SC_IND_DATA - N_SC/2), 100*fftshift( reshape(evm_mat(:,sp), [], N_OFDM_SYM), 1));
    
    hold on;
    h = line([1,N_OFDM_SYM],[0,0]);
    set(h,'Color',[1 0 0]);
    set(h,'LineStyle',':');
    hold off;
    grid on
    xlabel('OFDM Symbol Index');
    ylabel('Subcarrier Index');
    title(sprintf('Stream from UE %d', sp));
    h = colorbar;
    set(get(h,'title'),'string','EVM (%)'); 
end

fprintf('\n MIMO Results:\n');
fprintf('Num Bytes:   %d\n', N_UE*N_DATA_SYMS * log2(MOD_ORDER) / 8);
fprintf('Sym Errors:  %d (of %d total symbols)\n', sym_errs, N_UE * N_DATA_SYMS);
fprintf('Bit Errors:  %d (of %d total bits)\n', bit_errs, N_UE*N_DATA_SYMS * log2(MOD_ORDER));

fprintf("SNRs (dB): \n");
fprintf("%.2f\t", snr_mat);
fprintf("\n");