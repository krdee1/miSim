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
        function f = plot(obj, f)
            arguments (Input)
                obj (1,1) {mustBeA(obj, 'sensingObjective')};
                f (1,1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
            end
            arguments (Output)
                f (1,1) {mustBeA(f, 'matlab.ui.Figure')};
            end

            % Create axes if they don't already exist
            f = firstPlotSetup(f);

            % Plot gradient on the "floor" of the domain
            hold(f.CurrentAxes, "on");
            s = surf(obj.X, obj.Y, repmat(obj.groundAlt, size(obj.X)), obj.values ./ max(obj.values, [], "all"), 'EdgeColor', 'none');
            s.HitTest = 'off';
            s.PickableParts = 'none';
            hold(f.CurrentAxes, "off");
        end
    end
end