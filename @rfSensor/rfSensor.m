classdef rfSensor
    properties (SetAccess = private, GetAccess = public)
        % Physical parameters
        c = 3e8; % Speed of light (m/s)
        k_B = 1.38e-23 % Boltzmann constant (W/Hz/K) for thermal noise model
        T_0 = 300; % Ambient temperature (Kelvin) for thermal noise model
        % Sensor parameters
        P_TX = 10e-3; % Transmit power (Watts)
        BW = 1e6; % Bandwidth (Hz)
        f_c = 2e9; % Center frequency (Hz)
    end

    methods (Access = public)
        [obj] = initialize(obj, alphaDist, betaDist, alphaTilt, betaTilt); % TODO initialize sensor, define parameters
        [SINR] = sensorPerformance(obj, agentPos, agentPan, agentTilt, targetPos); % determine sensor performance for a given single sensor and target geometry
        [f] = plotParameters(obj); % debug, plot sensor response as a function of distance and tilt angle
    end
    methods (Access = private)
        x = RSS(obj, d, t); % Received signal strength (function of distance and tilt angle)
        G_TX_dB = antennaGain(obj, agentPos, targetPos); % TODO Antenna gain for a given TX/RX pair 
        L_FSPL_dB = pathLoss(obj, agentPos, targetPos); % Free space path loss for a given TX/RX pair
    end
end