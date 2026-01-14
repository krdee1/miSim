function obj = initialize(obj, pos, collisionGeometry, sensorModel, comRange, maxIter, label, plotCommsGeometry)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'agent')};
        pos (1, 3) double;
        collisionGeometry (1, 1) {mustBeGeometry};
        sensorModel (1, 1) {mustBeSensor};
        comRange (1, 1) double;
        maxIter (1, 1) double;
        label (1, 1) string = "";
        plotCommsGeometry (1, 1) logical = false;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'agent')};
    end

    obj.pos = pos;
    obj.collisionGeometry = collisionGeometry;
    obj.sensorModel = sensorModel;
    obj.label = label;
    obj.plotCommsGeometry = plotCommsGeometry;
    obj.stepDecayRate = obj.initialStepSize / maxIter;

    % Initialize performance vector
    obj.performance = [0, NaN(1, maxIter), 0];

    % Add spherical geometry based on com range
    obj.commsGeometry = obj.commsGeometry.initialize(obj.pos, comRange, REGION_TYPE.COMMS, sprintf("%s Comms Geometry", obj.label));

    % Initialize FOV cone
    obj.fovGeometry = cone;
    obj.fovGeometry = obj.fovGeometry.initialize([obj.pos(1:3)], tand(obj.sensorModel.alphaTilt) * obj.pos(3), obj.pos(3), REGION_TYPE.FOV, sprintf("%s FOV", obj.label));
end