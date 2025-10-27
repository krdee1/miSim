classdef test_miSim < matlab.unittest.TestCase
    properties (Access = private)
        testClass = miSim;

        % Domain
        domain = rectangularPrism; % domain geometry

        % Obstacles
        minNumObstacles = 1; % Minimum number of obstacles to be randomly generated
        maxNumObstacles = 3; % Maximum number of obstacles to be randomly generated
        minObstacleSize = 1; % Minimum size of a randomly generated obstacle
        maxObstacleSize = 6; % Maximum size of a randomly generated obstacle
        obstacles = cell(1, 0);
        
        % Objective
        objectiveDiscretizationStep = 0.01; % Step at which the objective function is solved in X and Y space
        protectedRange = 1; % Minimum distance between the sensing objective and the edge of the domain
        objective = sensingObjective;

        % Agents
        minAgents = 3; % Minimum number of agents to be randomly generated
        maxAgents = 9; % Maximum number of agents to be randomly generated
        agents = cell(0, 1);
        
        % Collision
        minCollisionRange = 0.1; % Minimum randomly generated collision geometry size
        maxCollisionRange = 0.5; % Maximum randomly generated collision geometry size
        collisionRanges = NaN;

        % Communications
        comRange = 5; % Maximum range between agents that forms a communications link
    end

    % Setup for each test
    methods (TestMethodSetup)
        % Generate a random domain
        function tc = setDomain(tc)
            % random integer-sized cube domain ranging from [0, 5 -> 25]
            % in all dimensions
            L = ceil(5 + rand * 10 + rand * 10);
            tc.domain = tc.domain.initialize([zeros(1, 3); L * ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");
        end
        % Generate a random sensing objective within that domain
        function tc = setSensingObjective(tc)
            % Using a bivariate normal distribution
            % Set peak position (mean)
            mu = tc.domain.minCorner;
            while tc.domain.interiorDistance(mu) < tc.protectedRange
                mu = tc.domain.random();
            end
            mu(3) = 0;

            % Set standard deviations of bivariate distribution
            sig = [2 + rand * 2, 1; 1, 2 + rand * 2];

            % Define objective
            tc.objective = tc.objective.initialize(@(x, y) mvnpdf([x(:), y(:)], mu(1:2), sig), tc.domain.footprint, tc.domain.minCorner(3), tc.objectiveDiscretizationStep);
        end
        % Instantiate agents
        function tc = setAgents(tc)
            % Agents will be initialized under different parameters in
            % individual test cases

            % Instantiate a random number of agents according to parameters
            for ii = 1:randi([tc.minAgents, tc.maxAgents])
                tc.agents{ii, 1} = agent;
            end

            % Define random collision ranges for each agent
            tc.collisionRanges = tc.minCollisionRange + rand(size(tc.agents, 1), 1) * (tc.maxCollisionRange - tc.minCollisionRange);
        end
    end

    methods (Test)
        % Test methods
        function misim_initialization(tc)
            % randomly create 2-3 obstacles
            nGeom = tc.minNumObstacles + randi(tc.maxNumObstacles - tc.minNumObstacles);
            tc.obstacles = cell(nGeom, 1);

            % Iterate over obstacles to initialize
            for ii = 1:size(tc.obstacles, 1)
                badCandidate = true;
                while badCandidate
                    % Instantiate a rectangular prism obstacle
                    tc.obstacles{ii} = rectangularPrism;
    
                    % Randomly generate min corner for the obstacle
                    candidateMinCorner = tc.domain.random();
                    candidateMinCorner = [candidateMinCorner(1:2), 0]; % bind obstacles to floor of domain
    
                    % Randomly select a corresponding maximum corner that
                    % satisfies min/max obstacle size specifications
                    candidateMaxCorner = candidateMinCorner + tc.minObstacleSize + rand(1, 3) * (tc.maxObstacleSize - tc.minObstacleSize);
                
                    % Initialize obstacle
                    tc.obstacles{ii} = tc.obstacles{ii}.initialize([candidateMinCorner; candidateMaxCorner], REGION_TYPE.OBSTACLE, sprintf("Column obstacle %d", ii));

                    % Check if the obstacle intersects with any existing
                    % obstacles
                    violation = false;
                    for kk = 1:(ii - 1)
                        if geometryIntersects(tc.obstacles{kk}, tc.obstacles{ii})
                            violation = true;
                            break;
                        end
                    end
                    if violation
                        continue;
                    end

                    % Make sure that the obstacles are fully contained by 
                    % the domain
                    if ~domainContainsObstacle(tc.domain, tc.obstacles{ii})
                        continue;
                    end

                    % Make sure that the obstacles don't cover the sensing
                    % objective
                    if obstacleCoversObjective(tc.objective, tc.obstacles{ii})
                        continue;
                    end

                    % Make sure that the obstacles aren't too close to the
                    % sensing objective
                    if obstacleCrowdsObjective(tc.objective, tc.obstacles{ii}, tc.protectedRange)
                        continue;
                    end
                    
                    badCandidate = false;
                end
            end

            % Add agents individually, ensuring that each addition does not
            % invalidate the initialization setup
            for ii = 1:size(tc.agents, 1)
                initInvalid = true;
                while initInvalid
                    candidatePos = [tc.objective.groundPos, 0];
                    % Generate a random position for the agent based on
                    % existing agent positions
                    if ii == 1
                        while agentsCrowdObjective(tc.objective, candidatePos, mean(tc.domain.dimensions) / 2)
                            candidatePos = tc.domain.random();
                        end
                    else
                        candidatePos = tc.agents{randi(ii - 1)}.pos + sign(randn([1, 3])) .* (rand(1, 3) .* tc.comRange/sqrt(2));
                    end

                    % Make sure that the candidate position is within the
                    % domain
                    if ~tc.domain.contains(candidatePos)
                        continue;
                    end

                    % Make sure that the candidate position does not crowd
                    % the sensing objective and create boring scenarios
                    if agentsCrowdObjective(tc.objective, candidatePos, mean(tc.domain.dimensions) / 2)
                        continue;
                    end

                    % Make sure that there exist unobstructed lines of sight at
                    % appropriate ranges to form a connected communications 
                    % graph between the agents
                    connections = false(1, ii - 1);
                    for jj = 1:(ii - 1)
                        if norm(tc.agents{jj}.pos - candidatePos) <= tc.comRange
                            % Check new agent position against all existing
                            % agent positions for communications range
                            connections(jj) = true;
                            for kk = 1:size(tc.obstacles, 1)
                                if tc.obstacles{kk}.containsLine(tc.agents{jj}.pos, candidatePos)
                                    connections(jj) = false;
                                end
                            end
                        end
                    end

                    % New agent must be connected to an existing agent to
                    % be valid
                    if ii ~= 1 && ~any(connections)
                        continue;
                    end

                    % Initialize candidate agent
                    candidateGeometry = rectangularPrism;
                    newAgent = tc.agents{ii}.initialize(candidatePos, zeros(1,3), eye(3),candidateGeometry.initialize([candidatePos - tc.collisionRanges(ii) * ones(1, 3); candidatePos + tc.collisionRanges(ii) * ones(1, 3)], REGION_TYPE.COLLISION, sprintf("Agent %d collision volume", ii)), ii, sprintf("Agent %d", ii));
                    
                    % Make sure candidate agent doesn't collide with
                    % domain
                    violation = false;
                    for jj = 1:size(newAgent.collisionGeometry.vertices, 1)
                        % Check if collision geometry exits domain
                        if ~tc.domain.contains(newAgent.collisionGeometry.vertices(jj, 1:3))
                            violation = true;
                            break;
                        end
                    end
                    if violation
                        continue;
                    end

                    % Make sure candidate doesn't collide with obstacles
                    violation = false;
                    for kk = 1:size(tc.obstacles, 1)
                        if geometryIntersects(tc.obstacles{kk}, newAgent.collisionGeometry)
                            violation = true;
                            break;
                        end
                    end
                    if violation
                        continue;
                    end

                    % Make sure candidate doesn't collide with existing
                    % agents
                    violation = false;
                    for kk = 1:(ii - 1)
                        if geometryIntersects(tc.agents{kk}.collisionGeometry, newAgent.collisionGeometry)
                            violation = true;
                            break;
                        end
                    end
                    if violation
                        continue;
                    end

                    % Candidate agent is valid, store to pass in to sim
                    initInvalid = false;
                    tc.agents{ii} = newAgent;
                end
            end

            % Initialize the simulation
            tc.testClass = tc.testClass.initialize(tc.domain, tc.objective, tc.agents, tc.obstacles);

            % Plot domain
            f = tc.testClass.domain.plotWireframe;

            % Set plotting limits to focus on the domain
            xlim([tc.testClass.domain.minCorner(1) - 0.5, tc.testClass.domain.maxCorner(1) + 0.5]);
            ylim([tc.testClass.domain.minCorner(2) - 0.5, tc.testClass.domain.maxCorner(2) + 0.5]);
            zlim([tc.testClass.domain.minCorner(3) - 0.5, tc.testClass.domain.maxCorner(3) + 0.5]);

            % Plot obstacles
            for ii = 1:size(tc.testClass.obstacles, 1)
                tc.testClass.obstacles{ii}.plotWireframe(f);
            end

            % Plot objective gradient
            f = tc.testClass.objective.plot(f);

            % Plot agents and their collision geometries
            for ii = 1:size(tc.testClass.agents, 1)
                f = tc.testClass.agents{ii}.plot(f);
                f = tc.testClass.agents{ii}.collisionGeometry.plotWireframe(f);
            end
        end
    end
end