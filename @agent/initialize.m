function obj = initialize(obj, pos, vel, pan, tilt, collisionGeometry, sensorModel, comRange, label, debug, plotCommsGeometry)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'agent')};
        pos (1, 3) double;
        vel (1, 3) double;
        pan (1, 1) double;
        tilt (1, 1) double;
        collisionGeometry (1, 1) {mustBeGeometry};
        sensorModel (1, 1) {mustBeSensor};
        comRange (1, 1) double;
        label (1, 1) string = "";
        debug (1, 1) logical = false;
        plotCommsGeometry (1, 1) logical = false;
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
    obj.label = label;
    obj.debug = debug;
    obj.plotCommsGeometry = plotCommsGeometry;

    % Add spherical geometry based on com range
    obj.commsGeometry = obj.commsGeometry.initialize(obj.pos, comRange, REGION_TYPE.COMMS, sprintf("%s Comms Geometry", obj.label));

    if obj.debug
        obj.debugFig = figure;
        tiledlayout(obj.debugFig, "TileSpacing", "tight", "Padding", "compact");
        nexttile;
        axes(obj.debugFig.Children(1).Children(1));
        axis(obj.debugFig.Children(1).Children(1), "image");
        xlabel(obj.debugFig.Children(1).Children(1), "X"); ylabel(obj.debugFig.Children(1).Children(1), "Y");
        title(obj.debugFig.Children(1).Children(1), "Objective");
        nexttile;
        axes(obj.debugFig.Children(1).Children(1));
        axis(obj.debugFig.Children(1).Children(1), "image");
        xlabel(obj.debugFig.Children(1).Children(1), "X"); ylabel(obj.debugFig.Children(1).Children(1), "Y");
        title(obj.debugFig.Children(1).Children(1), "Sensor Performance");
        nexttile;
        axes(obj.debugFig.Children(1).Children(1));
        axis(obj.debugFig.Children(1).Children(1), "image");
        xlabel(obj.debugFig.Children(1).Children(1), "X"); ylabel(obj.debugFig.Children(1).Children(1), "Y");
        title(obj.debugFig.Children(1).Children(1), "Gradient Objective");
        nexttile;
        axes(obj.debugFig.Children(1).Children(1));
        axis(obj.debugFig.Children(1).Children(1), "image");
        xlabel(obj.debugFig.Children(1).Children(1), "X"); ylabel(obj.debugFig.Children(1).Children(1), "Y");
        title(obj.debugFig.Children(1).Children(1), "Gradient Sensor Performance");
        nexttile;
        axes(obj.debugFig.Children(1).Children(1));
        axis(obj.debugFig.Children(1).Children(1), "image");
        xlabel(obj.debugFig.Children(1).Children(1), "X"); ylabel(obj.debugFig.Children(1).Children(1), "Y");
        title(obj.debugFig.Children(1).Children(1), "Sensor Performance x Gradient Objective");
        nexttile;
        axes(obj.debugFig.Children(1).Children(1));
        axis(obj.debugFig.Children(1).Children(1), "image");
        xlabel(obj.debugFig.Children(1).Children(1), "X"); ylabel(obj.debugFig.Children(1).Children(1), "Y");
        title(obj.debugFig.Children(1).Children(1), "Gradient Sensor Performance x Objective");
        nexttile;
        axes(obj.debugFig.Children(1).Children(1));
        axis(obj.debugFig.Children(1).Children(1), "image");
        xlabel(obj.debugFig.Children(1).Children(1), "X"); ylabel(obj.debugFig.Children(1).Children(1), "Y");
        title(obj.debugFig.Children(1).Children(1), "Agent Performance (C)");
        nexttile;
        axes(obj.debugFig.Children(1).Children(1));
        axis(obj.debugFig.Children(1).Children(1), "image");
        xlabel(obj.debugFig.Children(1).Children(1), "X"); ylabel(obj.debugFig.Children(1).Children(1), "Y");
        title(obj.debugFig.Children(1).Children(1), "Gradient Agent Performance (del C)");
    end

    % Initialize FOV cone
    obj.fovGeometry = cone;
    obj.fovGeometry = obj.fovGeometry.initialize([obj.pos(1:2), 0], tand(obj.sensorModel.alphaTilt) * obj.pos(3), obj.pos(3), REGION_TYPE.FOV, sprintf("%s FOV", obj.label));
end