function obj = run(obj, sensingObjective, domain, partitioning)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'agent')};
        sensingObjective (1, 1) {mustBeA(sensingObjective, 'sensingObjective')};
        domain (1, 1) {mustBeGeometry};
        partitioning (:, :) double;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'agent')};
    end

    % Do sensing
    [sensedValues, sensedPositions] = obj.sensorModel.sense(obj, sensingObjective, domain, partitioning);

    % Determine next planned position
    nextPos = obj.guidanceModel(sensedValues, sensedPositions, obj.pos);

    % Move to next position
    % (dynamics not modeled at this time)
    obj.lastPos = obj.pos;
    obj.pos = nextPos;

    % Calculate movement
    d = obj.pos - obj.collisionGeometry.center;

    % Reinitialize collision geometry in the new position
    obj.collisionGeometry = obj.collisionGeometry.initialize([obj.collisionGeometry.minCorner; obj.collisionGeometry.maxCorner] + d, obj.collisionGeometry.tag, obj.collisionGeometry.label);
end