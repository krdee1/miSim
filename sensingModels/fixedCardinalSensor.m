classdef fixedCardinalSensor
    % Senses in the +/-x, +/- y directions at some specified fixed length
    properties
        r = 0.1; % fixed sensing length
    end

    methods (Access = public)
        function obj = initialize(obj, r)
            arguments(Input)
                obj (1, 1) {mustBeA(obj, 'fixedCardinalSensor')};
                r (1, 1) double;
            end
            arguments(Output)
                obj (1, 1) {mustBeA(obj, 'fixedCardinalSensor')};
            end
            obj.r = r;
        end
        function [neighborValues, neighborPos] = sense(obj, objectiveFunction, domain, pos)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'fixedCardinalSensor')};
                objectiveFunction (1, 1) {mustBeA(objectiveFunction, 'function_handle')};
                domain (1, 1) {mustBeGeometry};
                pos (1, 3) double;
            end
            arguments (Output)
                neighborValues (4, 1) double;
                neighborPos (4, 3) double;
            end
        
            % Evaluate objective at position offsets +/-[r, 0, 0] and +/-[0, r, 0]
            currentPos = pos(1:2);
            neighborPos = [currentPos(1) + obj.r, currentPos(2); ... % (+x)
                           currentPos(1), currentPos(2) + obj.r; ... % (+y)
                           currentPos(1) - obj.r, currentPos(2); ... % (-x)
                           currentPos(1), currentPos(2) - obj.r; ... % (-y)
                          ];
        
            % Check for neighbor positions that fall outside of the domain
            outOfBounds = false(size(neighborPos, 1), 1);
            for ii = 1:size(neighborPos, 1)
                if ~domain.contains([neighborPos(ii, :), 0])
                    outOfBounds(ii) = true;
                end
            end
        
            % Replace out of bounds positions with inoffensive in-bounds positions
            neighborPos(outOfBounds, 1:3) = repmat(pos, sum(outOfBounds), 1);
        
            % Sense values at selected positions
            neighborValues = [objectiveFunction(neighborPos(1, 1), neighborPos(1, 2)), ... % (+x)
                              objectiveFunction(neighborPos(2, 1), neighborPos(2, 2)), ... % (+y)
                              objectiveFunction(neighborPos(3, 1), neighborPos(3, 2)), ... % (-x)
                              objectiveFunction(neighborPos(4, 1), neighborPos(4, 2)), ... % (-y)
                             ];
        
            % Prevent out of bounds locations from ever possibly being selected
            neighborValues(outOfBounds) = 0;
        end
    end
end