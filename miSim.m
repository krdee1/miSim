classdef miSim
    % multiagent interconnection simulation

    % Simulation parameters
    properties (SetAccess = private, GetAccess = public)
        domain = rectangularPrismConstraint;
        objective = sensingObjective;
        constraintGeometries = cell(0, 1); % geometries that define constraints within the domain
        agents = cell(0, 1); % agents that move within the domain
    end

    methods (Access = public)
        function obj = initialize(obj, domain, objective, agents, constraintGeometries)
            arguments (Input)
                obj (1, 1) {mustBeA(obj, 'miSim')};
                domain (1, 1) {mustBeConstraintGeometries};
                objective (1, 1) {mustBeA(objective, 'sensingObjective')};
                agents (:, 1) cell {mustBeAgents};
                constraintGeometries (:, 1) cell {mustBeConstraintGeometries} = cell(0, 1);
            end
            arguments (Output)
                obj (1, 1) {mustBeA(obj, 'miSim')};
            end

            %% Define domain
            obj.domain = domain;

            %% Add constraint geometries against the domain
            obj.constraintGeometries = constraintGeometries;

            %% Define objective
            obj.objective = objective;

            %% Define agents
            obj.agents = agents;

        end
    end

    methods (Access = private)
        function validateInitialization(obj)
            % Assert obstacles do not intersect with the domain

            % Assert obstacles do not intersect with each other

            % Assert the objective has only one maxima within the domain

            % Assert the objective's sole maximum is not inaccessible due
            % to the placement of an obstacle

        end
        function validateLoop(obj)
            % Assert that agents are safely inside the domain

            % Assert that agents are not in proximity to obstacles

            % Assert that agents are not in proximity to each other

            % Assert that agents form a connected graph


        end
    end
end