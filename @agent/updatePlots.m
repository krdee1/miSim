function updatePlots(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "agent")};
    end
    arguments (Output)
    end

    % Find change in agent position since last timestep
    deltaPos = obj.pos - obj.lastPos;
    posChanged    = ~(all(isnan(deltaPos)) || all(deltaPos == zeros(1, 3)));
    orientChanged = obj.sensorModel.tilt    ~= obj.fovGeometry.tilt || ...
                    obj.sensorModel.azimuth ~= obj.fovGeometry.azimuth;

    if ~posChanged && ~orientChanged
        return;
    end

    if posChanged
        % Scatterplot point positions
        for ii = 1:size(obj.scatterPoints, 1)
            obj.scatterPoints(ii).XData = obj.pos(1);
            obj.scatterPoints(ii).YData = obj.pos(2);
            obj.scatterPoints(ii).ZData = obj.pos(3);
        end

        % Collision geometry edges
        for jj = 1:size(obj.collisionGeometry.lines, 2)
            for ii = 1:size(obj.collisionGeometry.lines(:, jj), 1)
                obj.collisionGeometry.lines(ii, jj).XData = obj.collisionGeometry.lines(ii, jj).XData + deltaPos(1);
                obj.collisionGeometry.lines(ii, jj).YData = obj.collisionGeometry.lines(ii, jj).YData + deltaPos(2);
                obj.collisionGeometry.lines(ii, jj).ZData = obj.collisionGeometry.lines(ii, jj).ZData + deltaPos(3);
            end
        end

        % Communications geometry edges
        if obj.plotCommsGeometry
            for jj = 1:size(obj.commsGeometry.lines, 2)
                for ii = 1:size(obj.collisionGeometry.lines(:, jj), 1)
                    obj.collisionGeometry.lines(ii, jj).XData = obj.collisionGeometry.lines(ii, jj).XData + deltaPos(1);
                    obj.collisionGeometry.lines(ii, jj).YData = obj.collisionGeometry.lines(ii, jj).YData + deltaPos(2);
                    obj.collisionGeometry.lines(ii, jj).ZData = obj.collisionGeometry.lines(ii, jj).ZData + deltaPos(3);
                end
            end
        end
    end

    % FOV cone: recompute full mesh whenever position or orientation changes
    if ~isempty(obj.fovGeometry.surface)
        % Sync fovGeometry state to current agent position and sensor orientation
        obj.fovGeometry = obj.fovGeometry.initialize( ...
            obj.pos, obj.fovGeometry.radius, obj.fovGeometry.height, ...
            obj.fovGeometry.tag, obj.fovGeometry.label, ...
            obj.sensorModel.tilt, obj.sensorModel.azimuth);

        % Recompute cone mesh (mirrors cone.plot logic)
        maxAlt = obj.fovGeometry.surface(1).Parent.ZLim(2);
        scalingFactor = maxAlt / obj.fovGeometry.height;
        [X, Y, Z] = cylinder([scalingFactor * obj.fovGeometry.radius, 0], obj.fovGeometry.n);
        Z = Z * maxAlt;
        Ry = [cosd(obj.fovGeometry.tilt), 0, -sind(obj.fovGeometry.tilt); 0, 1, 0; sind(obj.fovGeometry.tilt), 0, cosd(obj.fovGeometry.tilt)];
        Rz = [sind(obj.fovGeometry.azimuth), -cosd(obj.fovGeometry.azimuth), 0; cosd(obj.fovGeometry.azimuth), sind(obj.fovGeometry.azimuth), 0; 0, 0, 1];
        R  = Rz * Ry;
        pts = R * [X(:)'; Y(:)'; Z(:)' - maxAlt];
        X = reshape(pts(1, :), size(X)) + obj.pos(1);
        Y = reshape(pts(2, :), size(Y)) + obj.pos(2);
        Z = reshape(pts(3, :) + maxAlt, size(Z)) + obj.pos(3) - maxAlt;

        for jj = 1:size(obj.fovGeometry.surface, 2)
            obj.fovGeometry.surface(jj).XData = X;
            obj.fovGeometry.surface(jj).YData = Y;
            obj.fovGeometry.surface(jj).ZData = Z;
        end
    end
end
