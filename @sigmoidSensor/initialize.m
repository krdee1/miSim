function obj = initialize(obj, alphaDist, betaDist, alphaTilt, betaTilt)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "sigmoidSensor")}
        alphaDist (1, 1) double;
        betaDist (1, 1) double;
        alphaTilt (1, 1) double;
        betaTilt (1, 1) double;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "sigmoidSensor")}
    end
    
    obj.alphaDist = alphaDist;
    obj.betaDist = betaDist;
    obj.alphaTilt = alphaTilt;
    obj.betaTilt = betaTilt;
end