function obj = initialize(obj, txPower, bandwidth, centerFreq)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")}
        txPower (1, 1) double;
        bandwidth (1, 1) double;
        centerFreq (1, 1) double;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "rfSensor")}
    end

    obj.P_TX = txPower;
    obj.BW = bandwidth;
    obj.f_c = centerFreq;
    
end