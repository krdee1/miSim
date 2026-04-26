function obj = initialize(obj, txPower, bandwidth, centerFreq, rxGain_dBi)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")}
        txPower (1, 1) double;
        bandwidth (1, 1) double;
        centerFreq (1, 1) double;
        rxGain_dBi (1, 1) double;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "rfSensor")}
    end

    % Provided values
    obj.P_TX = txPower; % Transmit power (W)
    obj.BW = bandwidth; % Bandwidth (Hz)
    obj.f_c = centerFreq; % Center frequency (Hz)
    obj.G_RX_dBi = rxGain_dBi; % Receiving Antenna Gain (dBi)

    % Computed values
    obj.P_TX_dBm = 10*log10(obj.P_TX/1e-3); % Transmit power in dBm
    obj.N = obj.k_B * obj.T_0 * obj.BW; % Thermal noise
end