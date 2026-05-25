function obj = initialize(obj, alphaDist, betaDist, alphaTilt, betaTilt, tilt, azimuth)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "sigmoidSensor")}
        alphaDist (1, 1) double;
        betaDist (1, 1) double;
        alphaTilt (1, 1) double;
        betaTilt (1, 1) double;
        tilt (1, 1) double = 0;
        azimuth (1, 1) double = 0;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "sigmoidSensor")}
    end
    
    % Sensor performance parameters
    obj.alphaDist = alphaDist;
    obj.betaDist = betaDist;
    obj.alphaTilt = alphaTilt;
    obj.betaTilt = betaTilt;

    % Sensor pointing parameters
    obj.tilt = tilt;
    obj.azimuth = azimuth;
end