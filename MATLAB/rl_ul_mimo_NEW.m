%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%	Author(s): C. Nicolas Barati nicobarati@rice.edu 
%		          Rahman Doost-Mohamamdy: doost@rice.edu
%   
%    
% Single-shot transmission from N_UE single-antenna clients to 
% N_BS_NODE base station radios (UE stands for User Equipment).
% We define two modes:
% OTA (Over-the-air) and SIM_MODE (simulation).
% In simulation mode we simply use a Rayleigh channel whereas the OTA mode
% relies on the Iris hardware for transmission and reception.
% In both cases the clients transmit an OFDM signal that resembles a
% typical 802.11 WLAN waveform. 
%
% We implement a frame structure that allows the base
% station to capture clean (non-overlaping) training sequences for
% equalization and demultiplexing of the concurrent data streams.
%
% Users can trigger multiple sequential transmissions by setting the 
% number of frames variable (N_FRM) greater than one.
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
    py.print() % weird bug where py isn't loaded in an external script
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
WRITE_PNG_FILES         = 0;                % Enable writing plots to PNG
SIM_MODE                = 0;
DEBUG                   = 0;
PLOT                    = 0;

if SIM_MODE
    TX_SCALE                = 1;            % Scale for Tx waveform ([0:1])
    N_BS_NODE               = 8;            % Number of SDRs (Matlab scripts only using antenna A)
    N_UE                    = 2;
    SNR_db                  = 10;
    N_FRM                   = 1;
    bs_ids                  = ones(1, N_BS_NODE);
    ue_ids                  = ones(1, N_UE);

else 
    %Iris params:
    TX_SCALE                = 0.8;          % Scale for Tx waveform ([0:1])
    USE_HUB                 = 1;
    TX_FRQ                  = 3.548e9;
    RX_FRQ                  = TX_FRQ;
    ANT_BS                  = 'AB';         % Options: {A, AB}. To use both antennas per board, set to 'AB'
    ANT_UE                  = 'A';         % Only tested with single-antenna UE (i.e., 'A')
    TX_GN                   = 80;
    RX_GN                   = 65;
    SMPL_RT                 = 5e6;
    N_FRM                   = 3;
    bs_ids                  = string.empty();
    ue_ids                  = string.empty();
    ue_scheds               = string.empty();

    if USE_HUB
        % Using chains of different size requires some internal
        % calibration on the BS. This functionality will be added later.
        % For now, we use only the 4-node chains:
        bs_ids = ["RF3E000731","RF3E000747","RF3E000734","RF3E000654","RF3E000458","RF3E000463","RF3E000424"];
        hub_id = ["FH4B000003"];
    else
        bs_ids = ["RF3E000731","RF3E000747","RF3E000734","RF3E000654","RF3E000458","RF3E000463","RF3E000424"];
        hub_id = [];
    end
    ue_ids= ["RF3E000353", "RF3E000706"];

    N_BS_NODE               = length(bs_ids);           % Number of nodes/antennas at the BS
    N_BS_ANT                = length(bs_ids) * length(ANT_BS);  % Number of antennas at the BS
    N_UE                    = length(ue_ids);           % Number of UE nodes (single antenna UE)
    N_UE_ANT                = length(ue_ids) * length(ANT_UE);

end

% MIMO params
MIMO_ALG                = 'ZF';      % MIMO ALGORITHM: ZF or Conjugate 

% Waveform params
N_OFDM_SYM              = 44;         % Number of OFDM symbols for burst, it needs to be less than 47
MOD_ORDER               = 16;          % Modulation order (2/4/16/64 = BSPK/QPSK/16-QAM/64-QAM)

% OFDM params
SC_IND_PILOTS           = [8 22 44 58];                           % Pilot subcarrier indices
SC_IND_DATA             = [2:7 9:21 23:27 39:43 45:57 59:64];     % Data subcarrier indices
SC_IND_DATA_PILOT       = [2:27 39:64]';
N_SC                    = 64;                                     % Number of subcarriers
CP_LEN                  = 16;                                     % Cyclic prefix length
CP_LEN_LTS              = 32;
N_DATA_SYMS             = N_OFDM_SYM * length(SC_IND_DATA);       % Number of data symbols (one per data-bearing subcarrier per OFDM symbol) per UE
N_LTS_SYM               = 2;                                      % Number of 
N_SYM_SAMP              = N_SC + CP_LEN;                          % Number of samples that will go over the air
N_ZPAD_PRE              = 100;                                     % Zero-padding prefix for Iris
N_ZPAD_POST             = N_ZPAD_PRE - 14;                         % Zero-padding postfix for Iris

% Rx processing params
FFT_OFFSET                    = 16;          % Number of CP samples to use in FFT (on average)
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
preamble_vec = [];
for jp = 1:N_UE
    preamble((jp-1)*l_pre + 1: (jp-1)*l_pre+l_pre,jp) = preamble_common;
    preamble_vec = [preamble_vec; preamble_common];
end

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
tx_payload_vec = reshape(tx_payload_mat, ceil(numel(tx_payload_mat)/N_UE), N_UE);

% Construct the full time-domain OFDM waveform
tx_vec = [zeros(N_ZPAD_PRE, N_UE); preamble; tx_payload_vec; zeros(N_ZPAD_POST, N_UE)];

% Leftover from zero padding:
tx_vec_iris = tx_vec;
% Scale the Tx vector to +/- 1
tx_vec_iris = TX_SCALE .* tx_vec_iris ./ max(abs(tx_vec_iris));
%number of samples in a subframe or symbol
n_samp = length(tx_vec_iris);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TX/RX
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if SIM_MODE
    disp("Running: RAYLEIGH FADING CHANNEL SIMULATION MODE");
    % Replicate what's done in hardware just to ensure we are doing the same.
    tx_vec_iris_reshape = zeros(length(tx_vec)*N_UE, N_UE);
    for symIdx = 1:N_UE
        tx_vec_iris_reshape((symIdx-1)*n_samp+1:symIdx*n_samp, symIdx) = tx_vec_iris(:,symIdx);
    end
    tx_vec_iris_reshape = [tx_vec_iris_reshape; tx_vec_iris];
    tx_data_tmp = tx_vec_iris_reshape;

    % Rayleigh Fading Channel
    tx_var = mean(mean(abs(tx_data_tmp).^2 )) * (64/48);
    nvar =  tx_var / 10^(0.1*SNR_db); % noise variance per data sample
    % noise vector
    W_ul = sqrt(nvar/2) * (randn(N_BS_NODE, length(tx_data_tmp)) + ...
        1i*randn(N_BS_NODE, length(tx_data_tmp)) );
    hvar = 1;
    if N_UE == 1
        H_ul = sqrt(hvar/2).*randn(N_BS_NODE, length(tx_data_tmp)) + 1i*randn(N_BS_NODE, length(tx_data_tmp));
        H_ul = sqrt(abs(H_ul).^2);
        H_ul = smoothdata(H_ul, 2, 'movmean',15);
        y0 = H_ul.*tx_data_tmp.';
    else
        H_ul = sqrt(hvar/2).*randn(N_BS_NODE, N_UE) + 1i*randn(N_BS_NODE, N_UE);
        y0 = H_ul*tx_data_tmp.';
    end
    
    y = y0 + W_ul;
    numRxSyms = N_UE+1;    % Pilots plus data slots
    rx_vec_iris = reshape(y, [N_FRM, N_BS_NODE, n_samp, numRxSyms]);
    numGoodFrames = N_FRM;
else

    disp("Running: HARDWARE MODE");

    %############################# HERE ##########################
    % Iris nodes' parameters
    sdr_params = struct(...
        'bs_id', bs_ids, ...
        'ue_id', ue_ids,...
        'bs_ant', ANT_BS, ...
        'ue_ant', ANT_UE, ...
        'txfreq', TX_FRQ, ...
        'rxfreq', RX_FRQ, ...
        'txgain', TX_GN, ...
        'rxgain', RX_GN, ...
        'sample_rate', SMPL_RT);

    mimo_handle = mimo_driver(sdr_params);
    [rx_vec_iris, numGoodFrames, numRxSyms] = mimo_handle.mimo_txrx_uplink(tx_vec_iris, N_FRM, N_ZPAD_PRE);
    mimo_handle.mimo_close();
    if isempty(rx_vec_iris)
        error("Driver returned empty array. No good data received by base station");
    end

    rx_vec_iris = permute(rx_vec_iris, [1,2,4,3]);
    
end
%###############################################################################



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Process Data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% If multiple frames were sent, select one that is likely to work...
N_BS_NODE = N_BS_ANT;    % We actually need # of BS ANT not NODES
for iframe = 1:numGoodFrames
    fprintf(' =============================== \n');
    fprintf('Frame #%d Out of %d Triggered Frames \n', iframe, numGoodFrames);
    % Data shape: (# good frames, # BS antenna, # number samps, # numRxSyms)  SWAPPED last two
    rx_vec_iris_tmp = squeeze(rx_vec_iris(iframe, :, :, :));

    %% Correlate for LTS
    % Complex cross correlation of Rx waveform with time-domain LTS
    data_len = (N_OFDM_SYM)*(N_SC +CP_LEN);
    rx_lts_mat = double.empty();
    payload_ind = int32.empty();
    preamble_pk = nan(N_BS_NODE, N_UE);
    payload_rx = zeros(N_BS_NODE, data_len);
    badrx = zeros(1, N_BS_NODE);

    % Break into the number of R symbols in the BS schedule
    for ibs = 1:N_BS_NODE
        for isym = 1: N_UE
            currSym = squeeze(rx_vec_iris_tmp(ibs, :, isym));
            lts_corr = abs(conv(conj(fliplr(lts_t.')), sign(currSym.')));

            if DEBUG
                figure; plot(lts_corr);
            end
            lts_peaks = find(lts_corr > 0.8*max(lts_corr));
            [LTS1, LTS2] = meshgrid(lts_peaks,lts_peaks);
            [lts_second_peak_index2,y] = find(LTS2-LTS1 == length(lts_t));
            if(isempty(lts_second_peak_index2))
                fprintf('NO correlation peak at antenna %d sched sym idx: %d \n', ibs, isym);
                badrx(ibs) = badrx(ibs) + 1;
            else
                if length(lts_second_peak_index2) > 1
                    preamble_pk(ibs, isym) = lts_peaks(lts_second_peak_index2(2));
                else
                    preamble_pk(ibs, isym) = lts_peaks(lts_second_peak_index2(1));
                end

                % Check if valid...
                pk_tmp = preamble_pk(ibs,isym);
                if ((pk_tmp + data_len) > n_samp || (pk_tmp - (isym * l_pre)) < 0)
                    fprintf('INVALID correlation peak at antenna %d sched sym idx: %d \n', ibs, isym);
                    preamble_pk(ibs,isym) = nan;
                    badrx(ibs) = badrx(ibs) + 1;
                end
            end
        end
    end

    % Get symbol of interest (last R)
    %startIdx = (numRxSyms-1)*n_samp + 1;
    %endIdx = numRxSyms*n_samp;
    %rx_vec_iris_tmp = rx_vec_iris(:, startIdx:endIdx);
    rx_vec_iris_tmp = squeeze(rx_vec_iris_tmp(:,:,end));
    [maxVal, maxIdx] = max(preamble_pk, [], 2);

    % Find payload start and deal with bad correlation
    for ibs = 1:N_BS_NODE
        if isnan(maxVal(ibs))
            last_pk_loc = l_pre;
        else
            last_pk_loc = maxVal(ibs);
        end
        payload_ind = (N_UE - maxIdx(ibs)) * l_pre + last_pk_loc + 1;
        pream_ind = payload_ind - length(preamble);
        preamble_range = pream_ind: pream_ind + length(preamble) - 1;
        payload_range = payload_ind : payload_ind + data_len - 1;
        rx_lts_mat(ibs,:) = rx_vec_iris_tmp(ibs, preamble_range);
        payload_rx(ibs,:) = rx_vec_iris_tmp(ibs, payload_range);
    end

    % Remove problematic entries (BS boards with bad correlation)
    % Only remove those that didn't capture pilots from any UE
    rx_lts_mat = rx_lts_mat(~(badrx == N_UE), :);
    payload_rx = payload_rx(~(badrx == N_UE), :);
    badrx(badrx ~= N_UE) = 0;  % line order matters (1)
    badrx(badrx == N_UE) = 1;  % line order matters (2)

    % Verify
    if sum(badrx) >= N_BS_NODE - 1 
        fprintf('[WARNING] Bad Frame (Frame #%d). Not enough BS antennas captured signal. Continue... \n', iframe);
        continue;
    end

    %% Rx processing
    % Construct a matrix from the received pilots
    n_plt_samp = l_pre;     % number of samples in a per-UE pilot
    Y_lts = zeros(N_BS_NODE-sum(badrx), N_UE, n_plt_samp);
    for iue = 1:N_UE
    plt_j_ix = (iue-1) * n_plt_samp +1:iue * n_plt_samp;
    Y_lts(:,iue,:) = rx_lts_mat(:,plt_j_ix);
    end

    % Take N_SC spamples from each LTS
    rx_lts_idx = CP_LEN_LTS-FFT_OFFSET + 1 : N_LTS_SYM * N_SC + CP_LEN_LTS-FFT_OFFSET;
    Y_lts = Y_lts(:,:,rx_lts_idx);

    % Reshape the matix to have each lts pilot in a different dimension:
    % N_BS_NODE x N_UE x 64 x 2
    Y_lts = reshape(Y_lts, N_BS_NODE-sum(badrx), N_UE, [], N_LTS_SYM);

    % Take FFT:
    Y_lts_f = fft(Y_lts, N_SC,3);
    % Construct known pilot matrix to use in next step:
    lts_f_mat = repmat(lts_f, (N_BS_NODE-sum(badrx)) * N_UE * N_LTS_SYM,1);
    lts_f_mat = reshape(lts_f_mat, [], N_LTS_SYM, N_UE, N_BS_NODE-sum(badrx));
    lts_f_mat = permute(lts_f_mat, [4 3 1 2]);

    % Get the channel by dividing by the pilots
    G_lts = Y_lts_f ./ lts_f_mat;
    % Estimate the channel by averaging over the two LTS symbols:
    H_hat = mean(G_lts, 4);

    % Calculate Condition Number
    validIdx = find(lts_f ~=0);
    condNum = zeros(1, length(validIdx));
    for sc = 1:length(validIdx)
        condNum(sc) = cond(H_hat(:, :, validIdx(sc))'*H_hat(:, :, validIdx(sc)));
    end
    avgCond = mean(condNum);

    % Reshape the payload and take subcarriers without the CP
    payload_rx = reshape(payload_rx, N_BS_NODE-sum(badrx), (N_SC + CP_LEN), []);
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
            % Do yourselves: Conj BF: 
            % Normalization coeff:
            H_pow = diag(abs (H_hat(:,:, nz_sc(j) )' * H_hat(:,:, nz_sc(j) ) ));
            % Apply BF: 
            x = (H_hat(:,:, nz_sc(j) )') * squeeze(Y_data(:,nz_sc(j),:))./ repmat(H_pow, 1, N_OFDM_SYM);
        end
        syms_eq(:,nz_sc(j),:) = x;
        channel_condition(nz_sc(j)) = cond(H_hat(:,:,nz_sc(j) ) );
        channel_condition_db(nz_sc(j)) = 10*log10(channel_condition(nz_sc(j)) );
    end

    if (DO_APPLY_PHASE_ERR_CORRECTION)
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


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plotter
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    cf = 0;
    fst_clr = [0, 0.4470, 0.7410];
    sec_clr = [0.8500, 0.3250, 0.0980];
    if PLOT
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


        %Tx signal
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
            title(sprintf('Equalized Uplink Tx (blue) and \n Rx (red) symbols for stream %d', sp));
            % legend({'Rx','Tx'},'Location','EastOutside', 'fontsize', 12);
        end

        for sp=1:N_BS_NODE-sum(badrx)
            subplot(sp_rows,sp_cols, sp_cols+sp);
            plot(squeeze(Y_data(sp,:,:)),'co','MarkerSize',1);
            axis square; axis(max(max(max( abs( Y_data)) ))*[-1 1 -1 1]);
            title(sprintf('Unequalized received symbols \n at BS ant. %d', sp));
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
                title(sprintf('UE %d -> BS ant. %d Channel Est. (Mag.)', iue, ibs))
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
    end

    %% EVM & SNR
    sym_errs = sum(tx_data(:) ~= rx_data(:));
    bit_errs = length(find(dec2bin(bitxor(tx_data(:), rx_data(:)),8) == '1'));
    evm_mat = double.empty();
    aevms = zeros(N_UE,1);
    snr_mat = zeros(N_UE,1);

    for sp = 1:N_UE
        tx_vec = tx_syms_mat(:,:,sp);
        evm_mat(:,sp)  = abs(tx_vec(:) - syms_eq_pc(:,sp) ).^2;
        aevms(sp) = mean(evm_mat(:,sp));
        snr_mat(sp) = -10*log10(aevms (sp));
    end

    if PLOT
        cf = cf + 1;
        figure(cf); clf;
        for sp = 1:N_UE
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
    end

    fprintf('\n MIMO Results:\n');
    fprintf('Num Bytes:   %d\n', N_UE*N_DATA_SYMS * log2(MOD_ORDER) / 8);
    fprintf('Sym Errors:  %d (of %d total symbols)\n', sym_errs, N_UE * N_DATA_SYMS);
    fprintf('Bit Errors:  %d (of %d total bits)\n', bit_errs, N_UE*N_DATA_SYMS * log2(MOD_ORDER));

    fprintf("SNRs (dB): \n");
    fprintf("%.2f\t", snr_mat);
    fprintf("\n");
    fprintf("Condition Number: %f  \n", avgCond);
    fprintf(' =============================== \n');
end
