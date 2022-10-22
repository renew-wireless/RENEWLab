%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%	Author(s): C. Nicolas Barati nicobarati@rice.edu
%		       Rahman Doost-Mohamamdy: doost@rice.edu
%
% - Supports Downlink MISO transmissions from multiple antennas in the
%   mMIMO base station to one single-antenna user.
% - Relies on explicit feedback (i.e., downlink pilots for beamforming
%   weight computation)
% - Supports transmission from one or two antennas per Iris board in the
%   base station (each Iris board has two antennas)
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


% [version, executable, isloaded] = pyversion;
% if ~isloaded
%     pyversion /usr/bin/python
%     py.print() % weird bug where py isn't loaded in an external script
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
WRITE_PNG_FILES         = 0;                % Enable writing plots to PNG
SIM_MODE                = 0;
DEBUG                   = 0;

PILOT_PLOT              = 0;
CONST_PLOT              = 0;
CHANNEL_PLOT            = 0;
DOWNLINK_PLOT           = 0;
EVM_SNR_PLOT            = 0;


%Iris params:
TX_SCALE                = 1;            % Scale for Tx waveform ([0:1])
ANT_BS                  = 'A';         % Options: {A, AB}
ANT_UE                  = 'A';          % Currently, only support single antenna UE, i.e., A
USE_HUB                 = 1;
TX_FRQ                  = 3.5475e9;
RX_FRQ                  = TX_FRQ;
TX_GN                   = 100;
TX_GN_BF                = 95;           % BS gain during DL BF transmission
TX_GN_UE                = [100, 100];
RX_GN                   = 70;
SMPL_RT                 = 5e6;
N_FRM                   = 1;
bs_ids                  = string.empty();
ue_ids                  = string.empty();
ue_scheds               = string.empty();
TX_ADVANCE              = 400;          % !!!! IMPORTANT: DO NOT MODIFY - Default is 235!!!!
if USE_HUB

    % Using chains of different size requires some internal
    % calibration on the BS. This functionality will be added later.
    % For now, we use only the 4-node chains:
    %bs_ids = ["RF3E000654","RF3E000458","RF3E000463","RF3E000424", ... % Chain1
    %          "RF3E000731","RF3E000747","RF3E000734", ...               % Chain1
    %          "RF3E000748","RF3E000492", ...                            % Chain5
    %          "RF3E000708","RF3E000437","RF3E000090"];                  % Chain5

    % 2 x 4 top 2 rows config [NOT WORKING]
    %bs_ids = ["RF3E000654","RF3E000458","RF3E000463","RF3E000424", ... % Chain1
    %          "RF3E000053","RF3E000177","RF3E000192", "RF3E000117"];  % Chain2
    % 2 x 4 with gap in the middle (chain 2, row 2 and 4) [NOT WORKING]
    %bs_ids = ["RF3E000053","RF3E000177","RF3E000192", "RF3E000117", ...
    %         "RF3E000257", "RF3E000430", "RF3E000311", "RF3E000565"];

    % 1 x 4 1st row only (chain 1) [WORKING]
    %bs_ids = ["RF3E000654","RF3E000458","RF3E000463","RF3E000424"]

    % 1 x 4 2nd row only (chain 2) [NOT WORKING]
    % bs_ids = ["RF3E000053","RF3E000177","RF3E000192", "RF3E000117"];

    % 1 x 4 5th row only (chain 3) [WORKING]
    % bs_ids = ["RF3E000686","RF3E000574","RF3E000595","RF3E000585"];

    % 1 x 4 6th row only (chain 4) [WORKING]
    % bs_ids = ["RF3E000722","RF3E000494","RF3E000592","RF3E000333"];

    % 2 x 4 5th and 6th row (chain3 and 4) [WORKING!]
    %bs_ids = ["RF3E000686","RF3E000574","RF3E000595","RF3E000585", "RF3E000722","RF3E000494","RF3E000592","RF3E000333"];
    % same, but with the order specified when recreating the array in matlab with 'lattice' option
    bs_ids = ["RF3E000686", "RF3E000722", "RF3E000574", "RF3E000494", "RF3E000595", "RF3E000592", "RF3E000585",  "RF3E000333"];
    hub_id = ["FH4B000003"];
    mimo_handle = false;

    cb_size = 25
    doSweep = false
    [BS_array, BS_code, BS_angles] = replicate_RENEW_array(2,4,TX_FRQ, cb_size, false);
    N_reps = 30 % number of beamsweeps to perform
    max_try = 25 % max try to receive DL data
    done_reps = 0;
    file_prefix = datestr(now, 'yy_mm_dd__HH_MM_SS_FFF'); % use datetime to keep track of the channel conditions
    mkdir('dataset')
    if ~doSweep
        n_codes = 0;    % only ZF/MRT
    else
        n_codes = length(BS_angles); % test also the predefined codebook
    end
    BER_vs_code = zeros(N_reps,n_codes+1, 1);


else
    bs_ids = ["RF3E000654","RF3E000458","RF3E000463","RF3E000424"];
    hub_id = [];
end
ue_ids= ["RF3E000706"];

N_BS_NODE               = length(bs_ids);                   % Number of nodes at the BS
N_BS_ANT                = length(bs_ids) * length(ANT_BS);  % Number of antennas at the BS
N_UE                    = length(ue_ids) * length(ANT_UE);  % Number of UE nodes (assume it is the same as number of antennas)

for nr=1:N_reps


    for ix_ang = 0:n_codes


        fprintf('Running: HARDWARE MODE \n');
        fprintf('=============================== \n');
        fprintf('Initiate Sounding Process \n');
        % Iris nodes' parameters
        sdr_params = struct(...
            'bs_id', bs_ids, ...
            'ue_id', ue_ids, ...
            'hub_id', hub_id,...
            'bs_ant', ANT_BS, ...
            'ue_ant', ANT_UE, ...
            'txfreq', TX_FRQ, ...
            'rxfreq', RX_FRQ, ...
            'txgain', TX_GN, ...
            'tx_gain_ue', TX_GN_UE, ...
            'rxgain', RX_GN, ...
            'sample_rate', SMPL_RT, ...
            'trig_offset', TX_ADVANCE);

        if (islogical(mimo_handle))
            if mimo_handle == false
                mimo_handle = mimo_driver(sdr_params);
            end
        end


        % codebook
        disp("***********************")
        disp("Using angle:")
        disp(BS_angles(ix_ang))
        disp("***********************")
        W = BS_code(ix_ang,:,1);    % retrieve code weights


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %                       DOWNLINK BEAMFORMING
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % No need to send explicit feedback to base station, simply use the computed
        % weights
        fprintf('=============================== \n');
        fprintf('Downlink Beamforming \n');

        % Update TX gain
        is_bs = 1;   % Is base station node
        param = 1;   % txgain == 1 [TODO, change to string, txgain=1, rxgain=2]
        mimo_handle.mimo_update_sdr_params(param, TX_GN_BF, is_bs);

        % Apply precoding weights to data (in freq. domain)
        % TODO create tx_payload with the 5GNR waveform
        %   tx_payload size is [N_BS_ANT x waveform_len]
        load()

        SSBbf =  W*txWaveform;

        [rx_vec_iris_tmp, numGoodFrames, ~] = mimo_handle.mimo_txrx_downlink(SSBbf, N_FRM, N_ZPAD_PRE);
        %mimo_handle.mimo_close(); % instead of closing here, we close the MIMO array at the end of the beamsweep

        if isempty(rx_vec_iris_tmp)
            %error("Driver returned empty array. No good data received by UE");
            %exit(0);
            fprintf("Driver returned empty array. No good data received by UE\n");
            BER_vs_code(nr, ix_ang+1, 1) = 1;     % max error if nothing is received
            continue;   % to allow check of other codes

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %                        PROCESS DOWNLINK DATA
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        igf = numGoodFrames;
        preamble_pk = zeros(1, N_UE);
        golden_number = 20;
        for iue = 1:N_UE

            %%%%% Find Preamble
            curr_vec = squeeze(rx_vec_iris_tmp(igf, iue, 1, :)); % Dimensions: (nGoodFrames,nUE,numRxSyms,numSamps)
            lts_corr = abs(conv(conj(fliplr(lts_t.')), sign(curr_vec)));
            lts_peaks = find(lts_corr > 0.8*max(lts_corr));
            [LTS1, LTS2] = meshgrid(lts_peaks,lts_peaks);
            [lts_second_peak_index,y] = find(LTS2-LTS1 == length(lts_t));

            if DEBUG
                figure; subplot(2,1,1); plot(abs(curr_vec)); subplot(2,1,2); plot(lts_corr); title(sprintf('DL DATA: UE %d,',iue));
            end

            % Stop if no valid correlation peak was found
            if(isempty(lts_second_peak_index))
                fprintf('SOUNDING: NO correlation peak at UE %d. Exit now! \n', iue);
                %return;
                BER_vs_code(nr, ix_ang+1, 1) = 1;     % max error if nothing is received
                continue;   % to allow check of other codes
            end

            if length(lts_second_peak_index) > 1
                preamble_pk(iue) = lts_peaks(lts_second_peak_index(2));
            else
                preamble_pk(iue) = lts_peaks(lts_second_peak_index(1));
            end

            % Check if valid...
            pk_tmp = preamble_pk(iue) + golden_number;
            lts_ind = pk_tmp - length(preamble_common) + 1;
            dl_data_start = pk_tmp + 1;

            if lts_ind <= 0
                fprintf('INVALID correlation peak at UE %d. Exit now! \n', iue);
                %return;
                BER_vs_code(nr, ix_ang+1, 1) = 1;     % max error if nothing is received
                continue;   % to allow check of other codes
            else
                fprintf('LTS Index: %d \n', lts_ind);
            end

            % Re-extract LTS for channel estimate
            rx_lts = curr_vec(lts_ind : lts_ind+159);
            rx_lts1 = rx_lts(-64+-FFT_OFFSET + [97:160]);
            rx_lts2 = rx_lts(-FFT_OFFSET + [97:160]);
            rx_lts1_f = fft(rx_lts1);
            rx_lts2_f = fft(rx_lts2);

            % Calculate channel estimate from average of 2 training symbols:
            %rx_H_est = mean([rx_lts1_f./lts_f   rx_lts2_f./ lts_f], 2);
            %rx_H_est = (lts_f.') .* (rx_lts1_f + rx_lts2_f) / 2;
            rx_H_est = lts_f .* (rx_lts1_f + rx_lts2_f) / 2;

            %%%%% Retrieve data and apply corrections
            %data_samples = dl_data_start + N_DATA_SAMP;
            rx_vec_downlink = curr_vec;
            N_RX_DATA_SYMS = min(N_DATA_SYM, floor((length(curr_vec) - dl_data_start) / N_SYM_SAMP));
            end_idx = min(N_SAMPS, dl_data_start + N_RX_DATA_SYMS * N_SYM_SAMP - 1);
            rx_dl_data_vec = rx_vec_downlink(dl_data_start: end_idx);
            rx_dl_data_mat = reshape(rx_dl_data_vec, N_SYM_SAMP, N_RX_DATA_SYMS);

            if(CP_LEN > 0)
                rx_dl_data_mat = rx_dl_data_mat(CP_LEN+1-FFT_OFFSET:end-FFT_OFFSET, :);
            end
            rx_dl_f_mat = fft(rx_dl_data_mat, N_SC, 1);
            N_RX_DATA_OFDM_SYMS = N_RX_DATA_SYMS;

            dl_syms_eq_mat = zeros(N_SC, N_RX_DATA_OFDM_SYMS);
            for i=1:N_RX_DATA_OFDM_SYMS
                dl_syms_eq_mat(:,i) = squeeze(rx_dl_f_mat(:,i))./rx_H_est;
            end

            pilots_eq_mat = dl_syms_eq_mat(SC_IND_PILOTS,:);
            pilots_eq_mat_comp = pilots_eq_mat.*repmat(pilots, 1, N_RX_DATA_OFDM_SYMS);

            pilot_dl_phase_err = squeeze(angle(mean(pilots_eq_mat_comp,1)));

            pilot_dl_phase_corr = zeros(N_SC, N_RX_DATA_OFDM_SYMS);
            for i=1:N_SC
                pilot_dl_phase_corr(i,:) = exp(-1i*pilot_dl_phase_err);
            end

            % Apply the pilot phase correction per symbol
            dl_syms_eq_pc_mat = dl_syms_eq_mat.* pilot_dl_phase_corr;
            payload_dl_syms_mat = dl_syms_eq_pc_mat(SC_IND_DATA, :);


            %%%%% Demodulate
            N_DATA_SC_RX = N_RX_DATA_OFDM_SYMS * length(SC_IND_DATA);

            if N_DATA_SC_RX ~= N_DATA_SC
                disp('Missing Data. Exit now!');
                %return;
                BER_vs_code(nr, ix_ang+1, 1) = 1;     % max error if nothing is received
                continue;   % to allow check of other codes
            end
            rx_syms = reshape(payload_dl_syms_mat, 1, N_DATA_SC);
            rx_data = demod_sym(rx_syms ,MOD_ORDER);


            %%%%% Calculate EVM & SNR
            % Do yourselves. Calculate EVM and effective SNR:
            evm_mat = abs(payload_dl_syms_mat - squeeze(tx_syms_mat(iue, :, :))).^2;
            aevms = mean(evm_mat(:)); % needs to be a scalar
            snr = 10*log10(1./aevms); % calculate in dB scale.

            sym_errs = sum(tx_data ~= rx_data);
            bit_errs = length(find(dec2bin(bitxor(tx_data, rx_data),8) == '1'));
            rx_evm   = sqrt(sum((real(rx_syms) - real(tx_syms)).^2 + (imag(rx_syms) - imag(tx_syms)).^2)/(length(SC_IND_DATA) * N_DATA_SYM));

            fprintf('\nResults:\n');
            fprintf('Num Bytes:  \t  %d\n', N_DATA_SC * log2(MOD_ORDER) / 8);
            fprintf('Sym Errors:  \t %d (of %d total symbols)\n', sym_errs, N_DATA_SC);
            fprintf('Bit Errors: \t %d (of %d total bits)\n', bit_errs, N_DATA_SC * log2(MOD_ORDER));
            fprintf('EVM: \t %f%%, SNR: %f \n', 100*aevms, snr);

            BER_vs_code(nr, ix_ang+1, 1) = bit_errs / (N_DATA_SC * log2(MOD_ORDER))
        end


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %                               PLOTTER
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for iue = 1:N_UE
            cf = 0 + 10^iue;
            fst_clr = [0, 0.4470, 0.7410];
            sec_clr = [0.8500, 0.3250, 0.0980];

            if PILOT_PLOT
                cf = cf + 1;
                figure(cf); clf;
                numRows = 5;
                numCols = 5;
                for ibs = 1:N_BS_ANT
                    subplot(numRows,numCols,ibs);
                    plot(abs(squeeze(rx_vec_iris_sound(1, iue, ibs, :))));
                end
            end


            if CHANNEL_PLOT
                cf = cf + 1;
                figure(cf); clf;
                x = (20/N_SC) * (-(N_SC/2):(N_SC/2 - 1));

                subplot(1,2,1);
                rx_H_est_plot = repmat(complex(NaN,NaN),1,length(rx_H_est));
                rx_H_est_plot(SC_IND_DATA) = rx_H_est_sound(SC_IND_DATA);
                rx_H_est_plot(SC_IND_PILOTS) = rx_H_est_sound(SC_IND_PILOTS);
                bar(x, fftshift(abs(rx_H_est_plot)),1,'LineWidth', 1);
                axis([min(x) max(x) 0 1.1*max(abs(rx_H_est_plot))])
                grid on;
                title('Channel Estimates (Magnitude)')
                xlabel('Baseband Frequency (MHz)')

                subplot(1,2,2);
                rx_H_est_plot = repmat(complex(NaN,NaN),1,length(rx_H_est));
                rx_H_est_plot(SC_IND_DATA) = rx_H_est(SC_IND_DATA);
                rx_H_est_plot(SC_IND_PILOTS) = rx_H_est(SC_IND_PILOTS);
                bar(x, fftshift(abs(rx_H_est_plot)),1,'LineWidth', 1);
                axis([min(x) max(x) 0 1.1*max(abs(rx_H_est_plot))])
                grid on;
                title('Channel Estimates (Magnitude)')
                xlabel('Baseband Frequency (MHz)')
            end

            if CONST_PLOT
                cf = cf + 1;
                figure(cf); clf;

                plot(payload_dl_syms_mat(:),'o','MarkerSize',2, 'color', sec_clr);
                axis square; axis(1.5*[-1 1 -1 1]);
                xlabel('Inphase')
                ylabel('Quadrature')
                grid on;
                hold on;

                plot(squeeze(tx_syms_mat(1,:,:)),'*', 'MarkerSize',16, 'LineWidth',2, 'color', fst_clr);
                title('Tx and Rx Constellations')
                legend('Rx','Tx','Location','EastOutside');
            end

            % Downlink Rx Vector and Constellation Plots
            if DOWNLINK_PLOT
                cf = cf + 1;
                figure(cf);clf;

                subplot(2,1,1);
                plot(real(squeeze(rx_vec_iris_tmp(1,iue,1,:))));
                xline(dl_data_start,'--r')
                axis([0 N_SAMPS -1 1])
                grid on;
                title('Downlink Data - Received Data (Real)');

                subplot(2,1,2);
                plot(lts_corr);
                axis([0 N_SAMPS -1 6])
                grid on;
                title('Downlink Data - LTS Correlation');
            end

            if EVM_SNR_PLOT
                % EVM & SNR
                cf = cf + 1;
                figure(cf); clf;
                subplot(2,1,1)
                plot(100*evm_mat(:),'o','MarkerSize',1)
                axis tight
                hold on
                plot([1 length(evm_mat(:))], 100*[aevms, aevms],'color', sec_clr,'LineWidth',4)
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
                imagesc(1:N_DATA_SYM, (SC_IND_DATA - N_SC/2), 100*fftshift(evm_mat,1))

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
            end
        end
    end

    done_reps = done_reps + 1;
end

% close the array only after the loop
mimo_handle.mimo_close();
save(strcat('dataset/',file_prefix,'__BER_vs_code.mat'), 'BER_vs_code');

%mean_bers = []
%disp("Summary of BERs for each code")
%for ix_ang = 0:length(BS_angles)
%    disp("--------------------")
%
%    ber_ixang = mean(BER_vs_code(:, ix_ang+1));
%    mean_bers(ix_ang+1) = ber_ixang;
%    if (ix_ang == 0)
%
%        disp("Optimal precoder")
%        disp("BER")
%        disp(ber_ixang)
%    else
%        disp("Code/Angle")
%        disp(ix_ang)
%        disp(BS_angles(ix_ang))
%        disp("Avg BER")
%        disp(mean_bers(1,ix_ang+1))
%    end
%    disp("--------------------")
%end

%semilogy(BS_angles,mean_bers(2:end), 'o-')
%ylabel('BER')
%xlabel('Angle / Code')
