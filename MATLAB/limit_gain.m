function [txgain] = limit_gain(TX_GN)

if TX_GN > 81
    display('WARNING: MAXIMUM TX GAIN IS 81!');
    txgain = 81;
else
    txgain = TX_GN;
end

end
