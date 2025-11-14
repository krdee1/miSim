classdef fixedCardinalSensor
    % Senses in the +/-x, +/- y directions at some specified fixed length
    properties
        alphaTilt = NaN;
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
        function [neighborValues, neighborPos] = sense(obj, agent, sensingObjective, domain, partitioning)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'fixedCardinalSensor')};
                agent (1, 1) {mustBeA(agent, 'agent')};
                sensingObjective (1, 1) {mustBeA(sensingObjective, 'sensingObjective')};
                domain (1, 1) {mustBeGeometry};
                partitioning (:, :) double = NaN;
            end
            arguments (Output)
                neighborValues (4, 1) double;
                neighborPos (4, 3) double;
            end

            % Set alphaTilt to produce an FOV cone with radius 'r' on the ground
            obj.alphaTilt = atan2(obj.r, agent.pos(3));
        
            % Evaluate objective at position offsets +/-[r, 0, 0] and +/-[0, r, 0]
            currentPos = agent.pos(1:2);
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
            neighborPos(outOfBounds, 1:3) = repmat(agent.pos, sum(outOfBounds), 1);
        
            % Sense values at selected positions
            neighborValues = [sensingObjective.objectiveFunction(neighborPos(1, 1), neighborPos(1, 2)), ... % (+x)
                              sensingObjective.objectiveFunction(neighborPos(2, 1), neighborPos(2, 2)), ... % (+y)
                              sensingObjective.objectiveFunction(neighborPos(3, 1), neighborPos(3, 2)), ... % (-x)
                              sensingObjective.objectiveFunction(neighborPos(4, 1), neighborPos(4, 2)), ... % (-y)
                             ];
        
            % Prevent out of bounds locations from ever possibly being selected
            neighborValues(outOfBounds) = 0;
        end
        function value = sensorPerformance(obj, agentPos, agentPan, agentTilt, targetPos)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'fixedCardinalSensor')};
                agentPos (1, 3) double;
                agentPan (1, 1) double;
                agentTilt (1, 1) double;
                targetPos (:, 3) double;
            end
            arguments (Output)
                value (:, 1) double;
            end

            value = 0.5 * ones(size(targetPos, 1), 1);
        end
    end
end