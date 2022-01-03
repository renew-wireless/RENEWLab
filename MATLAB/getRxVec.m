function y = getRxVec(tx_data, n_bs, n_ue, chan_type, snr, bs_param, ue_param, hub_id)
%%% Returns Rx vector passed through the channel type given in the input
%%% list.

assert(length(bs_param.id) == n_bs);
assert(length(ue_param.id) == n_ue);

if chan_type == "awgn"
% AWGN
    tx_var = mean(mean(abs(tx_data).^2 )) * (64/48);
    nvar =  tx_var / 10^(0.1*snr); % noise variance per data sample
        
    H_ul = ones(size(tx_data.'));
    % noise vector
    W_ul = sqrt(nvar/2).* (randn(n_bs, length(tx_data)) + ...
        1i*randn(n_bs, length(tx_data)) );
    
    % output vector
    y = H_ul.*tx_data.' + W_ul;
    y = y.';
elseif chan_type == "rayleigh"
    % Rayleigh fading (slower than iid!)
    tx_var = mean(mean(abs(tx_data).^2 )) * (64/48);
    nvar =  tx_var / 10^(0.1*snr); % noise variance per data sample
    % noise vector
    W_ul = sqrt(nvar/2) * (randn(n_bs, length(tx_data)) + ...
        1i*randn(n_bs, length(tx_data)) );
    hvar = 1;
    if n_ue == 1
        H_ul = sqrt(hvar/2).*randn(n_bs, length(tx_data)) + 1i*randn(n_bs, length(tx_data));
        H_ul = sqrt(abs(H_ul).^2);
        H_ul = smoothdata(H_ul, 2, 'movmean',15);
        % output vector
        y0 = H_ul.*tx_data.';
    else
        H_ul = sqrt(hvar/2).*randn(n_bs, n_ue) + 1i*randn(n_bs, n_ue);
        y0 = H_ul*tx_data.';
        
    end
    
    y = y0 + W_ul;
    y = y.';
    
elseif chan_type == "mpc"
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%-------------------------------------------------------------------------
% Based on a lab assignment from EL6023--Wireless Comm. class taught by
% Prof. Sundeep Rangan in Fall 2015 at NYU Tandon Sccool of Engineering.
% ------------------------------------------------------------------------
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Simplified MIMO MPC: SISO and SIMO only!

        nsub = 20;      % number of subpaths per cluster
        fmaxHz = 0;     % max Doppler spread in Hz. NOT USED!
        freqErrHz = 0;  % constant frequency error in Hz
        dlyns=0;        % constant delay in ns
        angMotion=0;    % angle of motion. KEEP at 0.
        
        % Cluster parameters represented as a vector with one
        % component per cluster
        angcRx = [0 90]'*pi/180;      % RX center angle in radians
        angspdRx = 10*pi/180;           % RX angular spread in radians
        angcTx = 0;                     % TX center angle in radians
        angspdTx = 0;                   % TX angular spread in radians
        dlycns = [0 100]';              % excess delay of first path in cluster nsec
        dlyspdns = 30;                  % delay spread within cluster in nsec
        powcdB = [-3 -3]';              % gain power in dB 1/2 the power on each cluster
        fadec = [1 1]';                 % fadec(i)= 0 => cluster i is non-fading
        
        % Antenna array
        nantTx = n_ue;      % number of TX antennas
        nantRx = n_bs;      % number of RX antennas
        dsepTx = 0.5;       % TX antenna separation
        dsepRx = 0.5;       % RX antenna separation

        nov = 16;     % oversampling ratio
        tx_var = mean(abs(tx_data).^2 ) * (64/48);
        nvar =  tx_var / 10^(0.1*snr); % noise variance per data sample
        
        % Carrier and sample rate
        fsampMHz = 5;     % sample frequency in MHz
        
        x = tx_data;
        
            
      % Convert scalar parameters to vectors
      nc = length(angcRx);      % num clusters
      if length(angcTx) == 1
        angcTx = angcTx*ones(nc,1);
      end
      if length(angspdRx) == 1
        angspdRx = angspdRx*ones(nc,1);
      end
      if length(angspdTx) == 1
        angspdTx = angspdTx*ones(nc,1);
      end
      if length(dlycns) == 1
        dlycns = dlycns*ones(nc,1);
      end
      if length(dlyspdns) == 1
        dlyspdns = dlyspdns*ones(nc,1);
      end
      if length(powcdB) == 1
        powcdB = powcdB*ones(nc,1);
      end
            
      % Compute total number of paths
      ncfade = sum(fadec);    % num fading clusters
      np = nsub*ncfade + nc-ncfade; % total num of subpaths
            
      % Select random parameters for the sub-paths within each
      % cluster
      angpRx = zeros(np,1);     % AoA in radians
      angpTx = zeros(np,1);     % AoD in radians
      dlypns = zeros(np,1);     % delay in us of each path
      powp = zeros(np,1);       % power of each path in linear scale
      ip = 0;
      % Spatial signature of each path
      urx = double.empty();    % urx(:,i) = RX spatial signature of i-th path
      utx = double.empty();    % utx(:,i) = TX spatial signature of i-th path
      
    for ic = 1:nc
        % Compute number of subpaths in the cluster
        % 1 for non-fading, nsub for fading
        if fadec(ic)
            nsubi = nsub;
        else
            nsubi = 1;
        end
        I = (ip+1:ip+nsubi)';
                
        % Set the sub-path angles, delay spread
        angpRx(I) = angcRx(ic) ...
                    + angspdRx(ic)*(rand(nsubi,1)-0.5);
        angpTx(I) = angcTx(ic) ...
                    + angspdTx(ic)*(rand(nsubi,1)-0.5);
        dlypns(I) = dlycns(ic)  ...
                    - dlyspdns(ic)*log(rand(nsubi,1));
        powp(I) = 10^(0.1*powcdB(ic))/nsubi;
        ip = ip + nsubi;
    end
            
   % Compute the spatial signatures urx and utx
   % urx(:,ip) = RX spatial signature of path ip
   % utx(:,ip) = RX spatial signature of path ip 
   omgiRx = 2*pi*dsepRx*cos(angpRx);
   omgiTx = 2*pi*dsepTx*cos(angpTx);
   for ip=1:np
       urx(:,ip) = exp(1i*(0:(nantRx-1))'*omgiRx(ip));
       utx(:,ip) = exp(1i*(0:(nantTx-1))'*omgiTx(ip));
   end

                     
  % Generate complex gains and Doppler shifts
   gain = sqrt(powp).*exp(2*pi*1i*rand(np,1));      % complex gain of each path
   fdHz = fmaxHz*cos(angpRx-angMotion) + freqErrHz; % Doppler shift in Hz = fmaxHz*cos(angp)
   
   % Get dimensions
   tsms = 1e-3/fsampMHz;    % sample period in ms
   nt = size(x,1);              % number of input samples
   ntx = nt*nov;            % num input time samples after upconverting
            
   % Compute size of y based on maximum path delay
   ts1 = tsms/nov; % sample period of upcoverted signal in ms
   idlyMax = max( round((dlypns+dlyns)*1e-6/ts1)); %idlyMax expressed in units of ts1
   nty = ntx + idlyMax; 
   % Upsample
   x1 = resample(x,nov,1);
   t = (0:nty-1)'*ts1*1e-3; % must be in sec since dfHz is in sec!
   y = zeros(nty,nantRx);
   % Filter the signal by summing each path
   % Sum shifted and scaled versions of x1 to create the output y          
   for irx=1:size(urx,1)
    for ip=1:np
        idly = round((dlypns(ip) + dlyns)...
            *1e-6/ts1);                       %delay of path ip
        xs = [zeros(idly,1); x1; ...
            zeros((idlyMax-idly),1)]; % x delayed vector per idly
        y(:,irx) = y(:,irx) ...
            + exp(2*pi*1i*t*fdHz(ip))...
            *gain(ip).*(urx(irx,ip)*xs);
    end
   end
   
   y0 = resample(y,1,nov);
   
   % Add noise
   ny = size(y0,1);
   w = sqrt(nvar/2)*(randn(ny,nantRx) + 1i*randn(ny,nantRx));
   y = y0 + w;

elseif chan_type == "iris"
% Real HW:
    n_samp = bs_param.n_samp;
    if ~isempty(hub_id)
        node_bs = iris_py(bs_param,hub_id);
    else
        node_bs = iris_py(bs_param,[]);        % initialize BS
    end
    node_ue = iris_py(ue_param,[]);    % initialize UE

    node_ue.sdr_configgainctrl();
    
    node_bs.sdrsync();                 % synchronize delays only for BS
    
    node_ue.sdrrxsetup();             % set up reading stream
    node_bs.sdrrxsetup();
    
    tdd_sched_index = 1; % for uplink only one frame schedule is sufficient
    node_bs.set_tddconfig(1, bs_param.tdd_sched(tdd_sched_index)); % configure the BS: schedule etc.
    node_ue.set_tddconfig(0, ue_param.tdd_sched(tdd_sched_index));

    if exist('bs_param.beacon_sweep', 'var')
        if bs_param.beacon_sweep
            disp('Beacon sweep');
            node_bs.sdr_setupbeacon();       % Burn beacon to the BS RAM
        else
            disp('Beacon from single TX');
            node_bs.sdr_setupbeacon_single();   % Burn beacon to the BS(1) RAM
        end
    else
        disp('Beacon from single TX');
        node_bs.sdr_setupbeacon_single();   % Burn beacon to the BS(1) RAM
    end

    for i=1:n_ue
        node_ue.sdrtx_single(tx_data(:,i), i);       % Burn data to the UE RAM
    end
    node_bs.sdr_activate_rx();          % activate reading stream

    node_ue.sdr_setcorr();              % activate correlator
    %node_bs.sdrtrigger(trig);           % set trigger to start the frame  
    
    % Iris Rx 
    % Only UL data:
    [y, data0_len] = node_bs.sdrrx(n_samp); % read data

    node_ue.sdr_gettriggers();

    node_bs.sdr_close();                % close streams and exit gracefully.
    node_ue.sdr_close();
    fprintf('Length of the received vector from HW: \tUE:%d\n', data0_len);


elseif chan_type == "past_run"
    % Past data
    old_data = load('old_data/mimo_60_145_try_5.mat');
    y = old_data.rx_vec_iris;
else 
    y = Nan;
    fprintf("No valid channel type was given!");
end
end