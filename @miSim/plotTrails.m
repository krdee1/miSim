function obj = plotTrails(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')}
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')}
    end
    
    % fast exit when plotting is disabled
    if ~obj.makePlots
        return;
    end

    % Plot full range of position history on each spatial plot axes
    o = [];
    for ii = 1:(size(obj.posHist, 1))
        hold(obj.f.Children(1).Children(obj.spatialPlotIndices(1)), 'on');
        o = [o; plot3(obj.f.Children(1).Children(obj.spatialPlotIndices(1)), obj.posHist(ii, 1:obj.maxIter, 1), obj.posHist(ii, 1:obj.maxIter, 2), obj.posHist(ii, 1:obj.maxIter, 3), 'Color', 'k', 'LineWidth', 1)];
        hold(obj.f.Children(1).Children(obj.spatialPlotIndices(1)), 'off');
    end

    % Copy trails to other figures?
    obj.trailPlot = o;

    % Add legend?
    % TODO legend?
    % TODO unique colors for agent trails (same as performance plot?)
    % Could be agent-specific colors (stored as agent property)
    keyboard

end