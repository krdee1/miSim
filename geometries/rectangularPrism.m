classdef rectangularPrism
    % Rectangular prism geometry
    properties (SetAccess = private, GetAccess = public)
        tag = REGION_TYPE.INVALID;
        label = "";

        minCorner = NaN(1, 3);
        maxCorner = NaN(1, 3);

        dimensions = NaN(1, 3);

        center = NaN;

        vertices = NaN(8, 3);

        footprint = NaN(4, 2);
    end

    methods (Access = public)
        function obj = initialize(obj, bounds, tag, label)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'rectangularPrism')};
                bounds (2, 3) double;
                tag (1, 1) REGION_TYPE = REGION_TYPE.INVALID;
                label (1, 1) string = "";
            end
            arguments (Output)
                obj (1, 1) {mustBeA(obj, 'rectangularPrism')};
            end

            obj.tag = tag;
            obj.label = label;

            %% Define geometry bounds by LL corner and UR corner
            obj.minCorner = bounds(1, 1:3);
            obj.maxCorner = bounds(2, 1:3);

            % Compute L, W, H
            obj.dimensions = [obj.maxCorner(1) - obj.minCorner(1), obj.maxCorner(2) - obj.minCorner(2), obj.maxCorner(3) - obj.minCorner(3)];

            % Compute center
            obj.center = obj.minCorner + obj.dimensions ./ 2;

            % Compute vertices
            obj.vertices = [obj.minCorner;
                            obj.maxCorner(1), obj.minCorner(2:3);
                            obj.maxCorner(1:2), obj.minCorner(3);
                            obj.minCorner(1), obj.maxCorner(2), obj.minCorner(3);
                            obj.minCorner(1:2), obj.maxCorner(3);
                            obj.maxCorner(1), obj.minCorner(2), obj.maxCorner(3);
                            obj.minCorner(1), obj.maxCorner(2:3)
                            obj.maxCorner;];

            % Compute footprint
            obj.footprint = [obj.minCorner(1:2); ...
                             [obj.minCorner(1), obj.maxCorner(2)]; ...
                             [obj.maxCorner(1), obj.minCorner(2)]; ...
                             obj.maxCorner(1:2)];
        end
        function r = random(obj)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'rectangularPrism')};
            end
            arguments (Output)
                r (1, 3) double
            end
            r = (obj.vertices(1, 1:3) + rand(1, 3) .* obj.vertices(8, 1:3) - obj.vertices(1, 1:3))';
        end
        function d = distance(obj, pos)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'rectangularPrism')};
                pos (:, 3) double;
            end
            arguments (Output)
                d (:, 1) double
            end
            cPos = NaN(1, 3);
            for ii = 1:3
                if pos(ii) < obj.minCorner(ii)
                    cPos(ii) = obj.minCorner(ii);
                elseif pos(ii) > obj.maxCorner(ii)
                    cPos(ii) = obj.maxCorner(ii);
                else
                    cPos(ii) = pos(ii);
                end
            end
            d = norm(cPos - pos);
        end
        function d = interiorDistance(obj, pos)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'rectangularPrism')};
                pos (:, 3) double;
            end
            arguments (Output)
                d (:, 1) double
            end
            % find minimum distance to any face
            d = min([pos(1) - obj.minCorner(1), ...
                     pos(2) - obj.minCorner(2), ...
                     pos(3) - obj.minCorner(3), ...
                     obj.maxCorner(1) - pos(1), ...
                     obj.maxCorner(2) - pos(2), ...
                     obj.maxCorner(3) - pos(3)]);
        end
        function c = contains(obj, pos)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'rectangularPrism')};
                pos (:, 3) double;
            end
            arguments (Output)
                c (:, 1) logical
            end
            c = all(pos >= repmat(obj.minCorner, size(pos, 1), 1), 2) & all(pos <= repmat(obj.maxCorner, size(pos, 1), 1), 2);
        end
        function f = plotWireframe(obj, f)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'rectangularPrism')};
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')} = figure;
            end
            arguments (Output)
                f (1, 1) {mustBeA(f, 'matlab.ui.Figure')};
            end

            % Create axes if they don't already exist
            f = firstPlotSetup(f);

            edges = [1 2; 2 3; 3 4; 4 1;  % bottom square
                5 6; 6 8; 8 7; 7 5;  % top square
                1 5; 2 6; 3 8; 4 7]; % vertical edges

            % Create plotting inputs from vertices and edges
            X = [obj.vertices(edges(:,1),1), obj.vertices(edges(:,2),1)]';
            Y = [obj.vertices(edges(:,1),2), obj.vertices(edges(:,2),2)]';
            Z = [obj.vertices(edges(:,1),3), obj.vertices(edges(:,2),3)]';

            % Plot the boundaries of the geometry
            hold(f.CurrentAxes, "on");
            plot3(X, Y, Z, '-', 'Color', obj.tag.color, 'LineWidth', 2);
            hold(f.CurrentAxes, "off");
        end
    end
end