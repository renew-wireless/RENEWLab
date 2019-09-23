%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%	Author(s): C. Nicolas Barati nicobarati@rice.edu 
%
%---------------------------------------------------------------------
% Original code copyright Mango Communications, Inc.
% Distributed under the WARP License http://warpproject.org/license
% Copyright (c) 2018-2019, Rice University
% RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
% ---------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [syms_eq, rx_data, bits] = syms2bits(syms,snr, n_ant,MOD_ORDER, awgn)

% Returns demodulated complex symbols, symbols in form of integer numbers
% and bits.

n_syms = length(syms);

% Create channel
if awgn == 0
    hvar = 1;
    h = sqrt(hvar/2).*(randn(n_syms, n_ant) + 1i*randn(n_syms, n_ant));
    h = sqrt( sum(abs(h).^2, 2) );
else
    h = ones(n_syms,1).*sqrt(n_ant);
end

% Add noise
nvar =  mean(mean( abs(syms).^2))/ 10^(0.1*snr);
w = sqrt(nvar/2).*(randn(n_syms,1) + 1i*randn(n_syms,1)); % CN(0,nvar);
rx_syms = h.*syms + w;

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

%  Do your selves: Get bits from rx_data:
bits_matrix = de2bi([rx_data; (MOD_ORDER-1)], 'left-msb')'; % This is a matrix now
bits_matrix = bits_matrix(:,1:end-1);
% Maybe you need to transpose bits_matrix before reshaping it to a vector:
% bits_matrix = bits_matrix';
bits = bits_matrix(:);

end