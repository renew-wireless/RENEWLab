%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%	Author(s): C. Nicolas Barati nicobarati@rice.edu 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



close all;
clear;


MOD_ORDER = 16;                         % Modulation order 
N = 1000;       
nbits = N * log2(MOD_ORDER);            % number of bits
snr = 15;
n_ant =  1;                             % number of antennas
awgn = 1;                               % AWGN channel(1) or fading(0)

% random bits
mybits = randi(2, nbits,1) - 1;

[syms, tx_data]  = bits2syms(mybits, MOD_ORDER);
[rx_syms, rx_data ,bits] = syms2bits(syms,snr, n_ant,MOD_ORDER, awgn);

% Plot Tx constellation
cf = 0;
cf = cf + 1;
figure(cf); clf;
ln_clr = [0.25, 0.25, 0.25];
line([-1.5, 1.5], [0 0], 'LineStyle', '-', 'Color', ln_clr, 'LineWidth', 1);
grid on;
hold on;
line([0 0], [-1.5, 1.5], 'LineStyle', '-', 'Color', ln_clr, 'LineWidth', 1);
if MOD_ORDER ~= 2
    plot(syms(:),'*','MarkerSize',16, 'LineWidth',2);
else
    plot(syms(:), 0,'*','MarkerSize',16, 'LineWidth',2);
end
axis square; axis(1.5*[-1 1 -1 1]);
xlabel('Inphase');
ylabel('Quadrature');
title('Constellation');
hold off;

%Plot Rx constellation
cf = cf + 1;
figure(cf); clf;
ln_clr = [0.25, 0.25, 0.25];
tx_clr = [0, 0.4470, 0.7410];
rx_clr = [0.8500, 0.3250, 0.0980];
line([-1.5, 1.5], [0 0], 'LineStyle', '-', 'Color', ln_clr, 'LineWidth', 1);
grid on;
hold on;
line([0 0], [-1.5, 1.5], 'LineStyle', '-', 'Color', ln_clr, 'LineWidth', 1);
if MOD_ORDER ~= 2
    hleg1 = plot(rx_syms(:),'o','MarkerSize',3,  'Color', rx_clr);
    hleg2 = plot(syms(:),'*','MarkerSize',16, 'LineWidth',2, 'Color', tx_clr);
    
else
    hleg1 = plot(rx_syms(:), 0,'o','MarkerSize',3, 'Color', rx_clr);
    hleg2 = plot(syms(:), 0,'*','MarkerSize',16, 'LineWidth',2,  'Color', tx_clr);
    
end
hlegs = [hleg1 hleg2];
axis square; axis(1.5*[-1 1 -1 1]);
xlabel('Inphase');
ylabel('Quadrature');
legend(hlegs, 'Tx','Rx', 'Fontsize', 16);
title('Constellation');
hold off;

% Do your selves: Calculate error rates:
sym_err = mean(tx_data ~= rx_data);
bit_err = mean(mybits ~= bits);
fprintf("Symbol Error Rate = %12.4e\t Bit Error Rate = %12.4e\n", sym_err, bit_err );
