classdef rfSensor
    properties (SetAccess = private, GetAccess = public)
        % Physical parameters
        c = 3e8; % Speed of light (m/s)
        k_B = 1.38e-23 % Boltzmann constant (W/Hz/K) for thermal noise model
        T_0 = 300; % Ambient temperature (Kelvin) for thermal noise model
        lossExponent = NaN; % Path loss exponent (2 for free space, up to 6 for the lossiest environments)
        % Sensor parameters
        P_TX = NaN; % Transmit power (Watts)
        BW = NaN; % Bandwidth (Hz)
        f_c = NaN; % Center frequency (Hz)
        G_RX_dBi = NaN; % Receiver antenna gain
        beamwidthExponent = NaN; % Antenna beamwidth exponent for cosine radiation pattern, larger exponent -> narrower beam
        % Values computed at initialization
        P_TX_dBm = NaN; % Transmit power (dBm)
        N = NaN; % Thermal noise
        % Cached state (per timestep)
        rssCache (:,1) double = double.empty(0,1); % linear-scale RSS to last ground targets grid
    end
    properties (Access = public)
        tilt = NaN;     % Antenna boresight tilt (deg): 0=nadir, 90=horizon
        azimuth = NaN;  % Antenna boresight azimuth (deg): 0=+y, 90=+x, 180=-y, 270=-x
    end

    methods (Access = public)
        [obj] = initialize(obj, txPower, bandwidth, centerFreq, rxGain, beamwidthExponent, tilt, azimuth); % initialize sensor, define parameters
        [SINR, SNR, obj, otherSensors] = sensorPerformance(obj, agentPos, targetPos, otherSensorsPos, otherSensors); % determine sensor performance for a given single sensor and target geometry
        [d, t, a] = computePointToPoints(obj, agentPos, targetPos);
        [value] = halfAngle(obj); % tilt angle (deg) at which sensor performance is halved
        [f] = plotParameters(obj); % debug, plot sensor response as a function of distance and tilt angle
        [f] = plotPerformance(obj, altitude, otherSensorsPos, otherSensors); % debug, plot SNR or SINR ground heatmap for a given geometry
        obj = clearRssCache(obj);
    end
    methods (Access = private)
        x = RSS(obj, d, t, a); % Received signal strength (function of distance and tilt angle)
        G_TX_dB = transmitterGain(obj, t, a); % Antenna gain for a given TX/RX pair 
        L_FSPL_dB = pathLoss(obj, d); % Free space path loss for a given TX/RX pair
    end
end