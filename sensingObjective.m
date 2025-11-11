classdef sensingObjective
    % Sensing objective definition parent class
    properties (SetAccess = private, GetAccess = public)
        label = "";
        groundAlt = 0;
        groundPos = [0, 0];
        discretizationStep = 1;
        objectiveFunction = @(x, y) 0; % define objective functions over a grid in this manner
        X = [];
        Y = [];
        values = [];
    end

    methods (Access = public)
        function obj = initialize(obj, objectiveFunction, footprint, groundAlt, discretizationStep)
            arguments (Input)
                obj (1,1) {mustBeA(obj, 'sensingObjective')};
                objectiveFunction (1, 1) {mustBeA(objectiveFunction, 'function_handle')};
                footprint (:, 2) double;
                groundAlt (1, 1) double = 0;
                discretizationStep (1, 1) double = 1;
            end
            arguments (Output)
                obj (1,1) {mustBeA(obj, 'sensingObjective')};
            end

            obj.groundAlt = groundAlt;

            % Extract footprint limits
            xMin = min(footprint(:, 1));
            xMax = max(footprint(:, 1));
            yMin = min(footprint(:, 2));
            yMax = max(footprint(:, 2));

            xGrid = unique([xMin:discretizationStep:xMax, xMax]);
            yGrid = unique([yMin:discretizationStep:yMax, yMax]);
            
            % Store grid points for plotting later
            [obj.X, obj.Y] = meshgrid(xGrid, yGrid);

            % Evaluate function over grid points
            obj.objectiveFunction = objectiveFunction;
            obj.values = reshape(obj.objectiveFunction(obj.X, obj.Y), size(obj.X));

            % store ground position
            idx = obj.values == max(obj.values, [], "all");
            obj.groundPos = [obj.X(idx), obj.Y(idx)];
        end
        function f = plot(obj, ind, f)
            arguments (Input)
                obj (1,1) {mustBeA(obj, 'sensingObjective')};
                ind (1, :) double = NaN;
                f (1,1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
            end
            arguments (Output)
                f (1,1) {mustBeA(f, 'matlab.ui.Figure')};
            end

            % Create axes if they don't already exist
            f = firstPlotSetup(f);

            % Plot gradient on the "floor" of the domain
            if isnan(ind)
                hold(f.CurrentAxes, "on");
                o = surf(f.CurrentAxes, obj.X, obj.Y, repmat(obj.groundAlt, size(obj.X)), obj.values ./ max(obj.values, [], "all"), 'EdgeColor', 'none');
                o.HitTest = 'off';
                o.PickableParts = 'none';
                hold(f.CurrentAxes, "off");

            else
                hold(f.Children(1).Children(ind(1)), "on");
                o = surf(f.Children(1).Children(ind(1)), obj.X, obj.Y, repmat(obj.groundAlt, size(obj.X)), obj.values ./ max(obj.values, [], "all"), 'EdgeColor', 'none');
                o.HitTest = 'off';
                o.PickableParts = 'none';
                hold(f.Children(1).Children(ind(1)), "off");
            end

            % Add to other perspectives
            if size(ind, 2) > 1
                for ii = 2:size(ind, 2)
                    copyobj(o, f.Children(1).Children(ind(ii)));
                end
            end
        end
    end
end