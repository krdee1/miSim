function updatePlots(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'agent')};
    end
    arguments (Output)
    end

    % Find change in agent position since last timestep
    deltaPos = obj.pos - obj.lastPos;
    if all(isnan(deltaPos)) || all(deltaPos == zeros(1, 3))
        % Agent did not move, so nothing has to move on the plots
        return;
    end

    % Scatterplot point positions
    for ii = 1:size(obj.scatterPoints, 1)
        obj.scatterPoints(ii).XData = obj.pos(1);
        obj.scatterPoints(ii).YData = obj.pos(2);
        obj.scatterPoints(ii).ZData = obj.pos(3);
    end

    % Collision geometry edges
    for jj = 1:size(obj.collisionGeometry.lines, 2)
        % Update plotting
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

    % Update FOV geometry surfaces
    for jj = 1:size(obj.fovGeometry.surface, 2)
        % Update each plot
        % obj.fovGeometry = obj.fovGeometry.plot(obj.spatialPlotIndices)
        obj.fovGeometry.surface(jj).XData = obj.fovGeometry.surface(jj).XData + deltaPos(1);
        obj.fovGeometry.surface(jj).YData = obj.fovGeometry.surface(jj).YData + deltaPos(2);
        obj.fovGeometry.surface(jj).ZData = obj.fovGeometry.surface(jj).ZData + deltaPos(3);
    end
end