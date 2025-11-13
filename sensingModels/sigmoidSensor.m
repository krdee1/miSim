classdef sigmoidSensor
    properties (SetAccess = private, GetAccess = public)
        % Sensor parameters
        alphaDist = NaN;
        betaDist = NaN;
        alphaPan = NaN;
        betaPan = NaN;
        alphaTilt = NaN;
        betaTilt = NaN;

        r = NaN;
    end

    methods (Access = public)
        function obj = initialize(obj, alphaDist, betaDist, alphaPan, betaPan, alphaTilt, betaTilt)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'sigmoidSensor')}
                alphaDist (1, 1) double;
                betaDist (1, 1) double;
                alphaPan (1, 1) double;
                betaPan (1, 1) double;
                alphaTilt (1, 1) double;
                betaTilt (1, 1) double;
            end
            arguments (Output)
                obj (1, 1) {mustBeA(obj, 'sigmoidSensor')}
            end

            obj.alphaDist = alphaDist;
            obj.betaDist = betaDist;
            obj.alphaPan = alphaPan;
            obj.betaPan = betaPan;
            obj.alphaTilt = alphaTilt;
            obj.betaTilt = betaTilt;

            obj.r = obj.alphaDist;
        end
        function [values, positions] = sense(obj, agent, sensingObjective, domain, partitioning)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'sigmoidSensor')};
                agent (1, 1) {mustBeA(agent, 'agent')};
                sensingObjective (1, 1) {mustBeA(sensingObjective, 'sensingObjective')};
                domain (1, 1) {mustBeGeometry};
                partitioning (:, :) double;
            end
            arguments (Output)
                values (:, 1) double;
                positions (:, 3) double;
            end

            % Find positions for this agent's assigned partition in the domain
            idx = partitioning == agent.index;
            positions = [sensingObjective.X(idx), sensingObjective.Y(idx), zeros(size(sensingObjective.X(idx)))];

            % Evaluate objective function at every point in this agent's
            % assigned partiton
            values = sensingObjective.values(idx);
        end
        function value = sensorPerformance(obj, agentPos, agentPan, agentTilt, targetPos)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'sigmoidSensor')};
                agentPos (1, 3) double;
                agentPan (1, 1) double;
                agentTilt (1, 1) double;
                targetPos (:, 3) double;
            end
            arguments (Output)
                value (:, 1) double;
            end

            d = vecnorm(agentPos - targetPos, 2, 2);
            panAngle = atan2(targetPos(:, 2) - agentPos(2), targetPos(:, 1) - agentPos(1)) - agentPan;
            tiltAngle = atan2(targetPos(:, 3) - agentPos(3), d) - agentTilt;

            % Membership functions
            mu_d = 1 - (1 ./ (1 + exp(-obj.betaDist .* (d - obj.alphaDist)))); % distance
            mu_p = (1 ./ (1 + exp(-obj.betaPan .* (panAngle + obj.alphaPan)))) - (1 ./ (1 + exp(-obj.betaPan .* (panAngle - obj.alphaPan)))); % pan
            mu_t = (1 ./ (1 + exp(-obj.betaPan .* (tiltAngle + obj.alphaPan)))) - (1 ./ (1 + exp(-obj.betaPan .* (tiltAngle - obj.alphaPan)))); % tilt

            value = mu_d .* mu_p .* mu_t;
        end
    end
end