function obj = initialize(obj, pos, vel, pan, tilt, collisionGeometry, sensorModel, guidanceModel, comRange, index, label)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'agent')};
        pos (1, 3) double;
        vel (1, 3) double;
        pan (1, 1) double;
        tilt (1, 1) double;
        collisionGeometry (1, 1) {mustBeGeometry};
        sensorModel (1, 1) {mustBeSensor}
        guidanceModel (1, 1) {mustBeA(guidanceModel, 'function_handle')};
        comRange (1, 1) double = NaN;
        index (1, 1) double = NaN;
        label (1, 1) string = "";
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'agent')};
    end

    obj.pos = pos;
    obj.vel = vel;
    obj.pan = pan;
    obj.tilt = tilt;
    obj.collisionGeometry = collisionGeometry;
    obj.sensorModel = sensorModel;
    obj.guidanceModel = guidanceModel;
    obj.comRange = comRange;
    obj.index = index;
    obj.label = label;

    % Initialize FOV cone
    obj.fovGeometry = cone;
    obj.fovGeometry = obj.fovGeometry.initialize([obj.pos(1:2), 0], tan(obj.sensorModel.alphaTilt) * obj.pos(3), obj.pos(3), REGION_TYPE.FOV, sprintf("%s FOV", obj.label));
end