TX_FRQ                  = 3.5475e9;
[BS_array, BS_code, BS_angles] = replicate_RENEW_array(2,4,TX_FRQ, 0, false);
load('/home/mauro/Research/RL-beamsteering/RENEW_dataset/MEBmimo/bersvscode.mat')

mean_bers = [];
std_devs = [];

disp("Summary of BERs for each code")
for ix_ang = 0:length(BS_angles)
%    disp("--------------------")

   bers_this_code = bersvscode(:, ix_ang+1);
   bers_nooutliers = bers_this_code(bers_this_code ~= 1.0); % remove the outliers from the data
   
   ber_ixang = mean(bers_nooutliers);
   std_ixang = std(bers_nooutliers);
   mean_bers(ix_ang+1) = ber_ixang;
   std_devs(ix_ang+1) = std_ixang;
   if (ix_ang == 0)

       disp("Optimal precoder")
       disp(strcat(["BER ",ber_ixang]))
       disp(strcat(["STD ",std_ixang]))
%    else
%        disp("Code/Angle")
%        disp(ix_ang)
%        disp(BS_angles(ix_ang))
%        disp("Avg BER")
%        disp(mean_bers(1,ix_ang+1))
   end
%    disp("--------------------")
end

semilogy(BS_angles,mean_bers(2:end), 'o-')
ylabel('BER')
xlabel('Angle / Code')


curve1 = mean_bers(2:end) + std_devs(2:end);
curve2 = mean_bers(2:end) - std_devs(2:end);
x2 = [BS_angles, fliplr(BS_angles)];
inBetween = [curve1, fliplr(curve2)];
fill(x2, inBetween,[0.6 0.7 0.2]);
hold on;
semilogy(BS_angles, mean_bers(2:end), 'r', 'LineWidth', 2);


