classdef rfSensor
    properties (SetAccess = private, GetAccess = public)
        % Physical parameters
        c = 3e8; % Speed of light (m/s)
        k_B = 1.38e-23 % Boltzmann constant (W/Hz/K) for thermal noise model
        T_0 = 300; % Ambient temperature (Kelvin) for thermal noise model
        % Sensor parameters
        P_TX = NaN; % Transmit power (Watts)
        BW = NaN; % Bandwidth (Hz)
        f_c = NaN; % Center frequency (Hz)
        G_RX_dBi = NaN; % Receiver antenna gain
        % Values computed at initialization
        P_TX_dBm = NaN; % Transmit power (dBm)
        N = NaN; % Thermal noise
    end

    methods (Access = public)
        [obj] = initialize(obj, txPower, bandwidth, centerFreq, rxGain); % initialize sensor, define parameters
        [SINR, SNR] = sensorPerformance(obj, agentPos, targetPos, otherSensorsPos, otherSensors); % determine sensor performance for a given single sensor and target geometry
        [d, t, a] = computePointToPoints(obj, agentPos, targetPos);
        [f] = plotParameters(obj); % debug, plot sensor response as a function of distance and tilt angle
        [f] = plotPerformance(obj, altitude, otherSensorsPos, otherSensors); % debug, plot SNR or SINR ground heatmap for a given geometry
    end
    methods (Access = private)
        x = RSS(obj, d, t, a); % Received signal strength (function of distance and tilt angle)
        G_TX_dB = transmitterGain(obj, t, a); % TODO Antenna gain for a given TX/RX pair 
        L_FSPL_dB = pathLoss(obj, d); % Free space path loss for a given TX/RX pair
    end
end