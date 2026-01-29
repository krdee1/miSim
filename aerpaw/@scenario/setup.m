function obj = setup(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'scenario')};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'scenario')};
    end
    
    % Command drones to their starting positions
    for ii = 1:size(obj.uavs, 1)
        obj.uavs{ii} = obj.sendTarget(obj.uavs{ii}, obj.inits.pos(ii, 1:3));
    end

    % Update opMode
    obj.opMode = OPMODE.SET;
    
end