classdef sigmoidSensor
    properties (SetAccess = private, GetAccess = public)
        % Sensor parameters
        alphaDist;
        betaDist;
        alphaPan;
        betaPan;
        alphaTilt;
        betaTilt;
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
        end
        function [neighborValues, neighborPos] = sense(obj, objectiveFunction, domain, pos)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'sigmoidSensor')};
                objectiveFunction (1, 1) {mustBeA(objectiveFunction, 'function_handle')};
                domain (1, 1) {mustBeGeometry};
                pos (1, 3) double;
            end
            arguments (Output)
                neighborValues (4, 1) double;
                neighborPos (4, 3) double;
            end
            
            
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