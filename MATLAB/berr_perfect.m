function berr_th = berr_perfect(snr, N_ANT,MOD_ORDER, awgn)
% Theoretical BER with no chan. est error.

N_DATA_SYMS = 1e5;
tx_data = randi(MOD_ORDER, N_DATA_SYMS, 1) - 1;

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

% Create channel
if awgn == 0
    hvar = 1;
    h = sqrt(hvar/2).*(randn(N_DATA_SYMS, N_ANT) + 1i*randn(N_DATA_SYMS, N_ANT));
    h = sqrt( sum(abs(h).^2, 2) );
    h = smoothdata(h, 'movmean', 15);
else
    h = ones(N_DATA_SYMS,1).*sqrt(N_ANT);
end

% Add noise
nvar =  1/ 10^(0.1*snr);
w = sqrt(nvar/2).*(randn(N_DATA_SYMS,1) + 1i*randn(N_DATA_SYMS,1)); % CN(0,nvar);
rx_syms = h.*tx_syms + w;

% Equalize with *known* channel:
syms_eq = rx_syms ./ h;

% Demodulate
demod_fcn_bpsk = @(x) double(real(x)>0);
demod_fcn_qpsk = @(x) double(2*(real(x)>0) + 1*(imag(x)>0));
demod_fcn_16qam = @(x) (8*(real(x)>0)) + (4*(abs(real(x))<0.6325)) + (2*(imag(x)>0)) + (1*(abs(imag(x))<0.6325));
demod_fcn_64qam = @(x) (32*(real(x)>0)) + (16*(abs(real(x))<0.6172)) + ... 
    (8*((abs(real(x))<(0.9258))&&((abs(real(x))>(0.3086))))) + ...
    (4*(imag(x)>0)) + (2*(abs(imag(x))<0.6172)) + ...
    (1*((abs(imag(x))<(0.9258))&&((abs(imag(x))>(0.3086)))));

switch(MOD_ORDER)
    case 2         % BPSK
        rx_data = arrayfun(demod_fcn_bpsk, syms_eq);
    case 4         % QPSK
        rx_data = arrayfun(demod_fcn_qpsk, syms_eq);
    case 16        % 16-QAM
        rx_data = arrayfun(demod_fcn_16qam, syms_eq);
    case 64        % 64-QAM
        rx_data = arrayfun(demod_fcn_64qam, syms_eq);
end

% Error rates
nbits = N_DATA_SYMS * log2(MOD_ORDER);
bit_errs = length(find(dec2bin(bitxor(tx_data, rx_data),8) == '1'));

berr_th = bit_errs/ nbits;

end