close all;

[bit_errs_mrc2, aevms_mrc2, bit_errs_br2, aevms_br2] = process_simo('tx_syms.mat', 'rx_data1x2.mat');
snr_mrc2 = 10*log10(1./aevms_mrc2);
good = find(snr_mrc2 > 0);

snr_mrc2 = 10*log10(1./aevms_mrc2(good));
snr_br_mean2 = 10*log10(1./mean(aevms_br2(good, :), 2));


[bit_errs_mrc4, aevms_mrc4, bit_errs_br4, aevms_br4] = process_simo('tx_syms.mat', 'rx_data1x4.mat');
snr_mrc4 = 10*log10(1./aevms_mrc4);
good = find(snr_mrc4 > 0);

snr_mrc4 = 10*log10(1./aevms_mrc4(good));
snr_br_mean4 = 10*log10(1./mean(aevms_br4(good, :), 2));




[bit_errs_mrc8, aevms_mrc8, bit_errs_br8, aevms_br8] = process_simo('tx_syms.mat', 'rx_data1x8.mat');
snr_mrc8 = 10*log10(1./aevms_mrc8);
good = find(snr_mrc8 > 0);

snr_mrc8 = 10*log10(1./aevms_mrc8(good));
min_id8 = min(aevms_br8(good, :)');
snr_br_mean8 = 10*log10(1./mean(aevms_br8(good, :), 2));
snr_br_max8 = 10*log10(1./aevms_br16(good(min_id8), :));




[bit_errs_mrc16, aevms_mrc16, bit_errs_br16, aevms_br16] = process_simo('tx_syms.mat', 'rx_data1x16.mat');
snr_mrc16 = 10*log10(1./aevms_mrc16);
good = find(snr_mrc16 > 0);

snr_mrc16 = 10*log10(1./aevms_mrc16(good));
min_id16 = min(aevms_br16(good, :)');
snr_br_mean16 = 10*log10(1./mean(aevms_br16(good, :), 2));
snr_br_max16 = 10*log10(1./aevms_br16(good(min_id16), :));



figure; plot(snr_mrc2); hold on; plot(snr_br_mean2)
title('1x2')
figure; plot(snr_mrc4); hold on; plot(snr_br_mean4)
title('1x4')
figure; plot(snr_mrc8); hold on; plot(snr_br_mean8)
title('1x8')
figure; plot(snr_mrc16); hold on; plot(snr_br_mean16); plot(snr_br_max16)
title('1x16')
