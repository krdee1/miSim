classdef test_miSim < matlab.unittest.TestCase
    properties (Access = private)
        testClass = miSim;

        % Sim
        maxIter = 250;
        timestep = 0.05
        partitoningFreq = 5;

        % Domain
        domain = rectangularPrism; % domain geometry
        minDimension = 10;

        % Obstacles
        minNumObstacles = 1; % Minimum number of obstacles to be randomly generated
        maxNumObstacles = 3; % Maximum number of obstacles to be randomly generated
        minObstacleSize = 1; % Minimum size of a randomly generated obstacle
        maxObstacleSize = 6; % Maximum size of a randomly generated obstacle
        obstacles = cell(1, 0);
        
        % Objective
        discretizationStep = 0.01; % Step at which the objective function is solved in X and Y space
        protectedRange = 1; % Minimum distance between the sensing objective and the edge of the domain
        objective = sensingObjective;

        % Agents
        minAgents = 3; % Minimum number of agents to be randomly generated
        maxAgents = 9; % Maximum number of agents to be randomly generated
        sensingLength = 0.05; % length parameter used by sensing function
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
            % random integer-dimensioned cubic domain
            tc.domain = tc.domain.initializeRandom(tc.minDimension, REGION_TYPE.DOMAIN, "Domain");
            % Random bivariate normal PDF objective
            tc.domain.objective = tc.domain.objective.initializeRandomMvnpdf(tc.domain, tc.protectedRange, tc.discretizationStep);
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
            % randomly create obstacles
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
                    if obstacleCoversObjective(tc.domain.objective, tc.obstacles{ii})
                        continue;
                    end

                    % Make sure that the obstacles aren't too close to the
                    % sensing objective
                    if obstacleCrowdsObjective(tc.domain.objective, tc.obstacles{ii}, tc.protectedRange)
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
                    candidatePos = [tc.domain.objective.groundPos, 0];
                    % Generate a random position for the agent based on
                    % existing agent positions
                    if ii == 1
                        while agentsCrowdObjective(tc.domain.objective, candidatePos, mean(tc.domain.dimensions) / 2)
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
                    if agentsCrowdObjective(tc.domain.objective, candidatePos, mean(tc.domain.dimensions) / 2)
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

                    % Initialize candidate agent collision geometry
                    candidateGeometry = rectangularPrism;
                    candidateGeometry = candidateGeometry.initialize([candidatePos - tc.collisionRanges(ii) * ones(1, 3); candidatePos + tc.collisionRanges(ii) * ones(1, 3)], REGION_TYPE.COLLISION, sprintf("Agent %d collision volume", ii));

                    % Initialize candidate agent sensor model
                    sensor = sigmoidSensor;
                    sensor = sensor.initialize(2.5, 3, NaN, NaN, deg2rad(15), 3);

                    % Initialize candidate agent
                    newAgent = tc.agents{ii}.initialize(candidatePos, zeros(1,3), 0, 0, candidateGeometry, sensor, @gradientAscent, tc.comRange, ii, sprintf("Agent %d", ii)); 

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
            [tc.testClass, f] = tc.testClass.initialize(tc.domain, tc.domain.objective, tc.agents, tc.timestep, tc.partitoningFreq, tc.maxIter, tc.obstacles);
        end
        function misim_run(tc)
            % randomly create obstacles
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
                    if obstacleCoversObjective(tc.domain.objective, tc.obstacles{ii})
                        continue;
                    end

                    % Make sure that the obstacles aren't too close to the
                    % sensing objective
                    if obstacleCrowdsObjective(tc.domain.objective, tc.obstacles{ii}, tc.protectedRange)
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
                    candidatePos = [tc.domain.objective.groundPos, 0];
                    % Generate a random position for the agent based on
                    % existing agent positions
                    if ii == 1
                        while agentsCrowdObjective(tc.domain.objective, candidatePos, mean(tc.domain.dimensions) / 2)
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
                    if agentsCrowdObjective(tc.domain.objective, candidatePos, mean(tc.domain.dimensions) / 2)
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

                    % Initialize candidate agent collision geometry
                    candidateGeometry = rectangularPrism;
                    candidateGeometry = candidateGeometry.initialize([candidatePos - tc.collisionRanges(ii) * ones(1, 3); candidatePos + tc.collisionRanges(ii) * ones(1, 3)], REGION_TYPE.COLLISION, sprintf("Agent %d collision volume", ii));
                    
                    % Initialize candidate agent sensor model
                    sensor = sigmoidSensor;
                    sensor = sensor.initialize(2.5, 3, NaN, NaN, deg2rad(15), 3);

                    % Initialize candidate agent
                    newAgent = tc.agents{ii}.initialize(candidatePos, zeros(1,3), 0, 0, candidateGeometry, sensor, @gradientAscent, tc.comRange, ii, sprintf("Agent %d", ii));
                    
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
            [tc.testClass, f] = tc.testClass.initialize(tc.domain, tc.domain.objective, tc.agents, tc.timestep, tc.partitoningFreq, tc.maxIter, tc.obstacles);

            % Run simulation loop
            [tc.testClass, f] = tc.testClass.run(f);
        end
        function test_basic_partitioning(tc)
            % place agents a fixed distance +/- X from the domain's center
            d = 1;

            % make basic domain
            tc.domain = tc.domain.initialize([zeros(1, 3); 10 * ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(@(x, y) mvnpdf([x(:), y(:)], tc.domain.center(1:2)), tc.domain, tc.discretizationStep);
        
            % Initialize agent collision geometry
            geometry1 = rectangularPrism;
            geometry2 = geometry1;
            geometry1 = geometry1.initialize([tc.domain.center + [d, 0, 0] - tc.collisionRanges(1) * ones(1, 3); tc.domain.center + [d, 0, 0] + tc.collisionRanges(1) * ones(1, 3)], REGION_TYPE.COLLISION, sprintf("Agent %d collision volume", 1));
            geometry2 = geometry2.initialize([tc.domain.center - [d, 0, 0] - tc.collisionRanges(1) * ones(1, 3); tc.domain.center - [d, 0, 0] + tc.collisionRanges(1) * ones(1, 3)], REGION_TYPE.COLLISION, sprintf("Agent %d collision volume", 2));
            
            % Initialize agent sensor model
            sensor = sigmoidSensor;
            sensor = sensor.initialize(2.5, 3, NaN, NaN, deg2rad(15), 3);
    
            % Initialize agents
            tc.agents = {agent; agent};
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center + [d, 0, 0], zeros(1,3), 0, 0, geometry1, sensor, @gradientAscent, 3*d, 1, sprintf("Agent %d", 1));
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center - [d, 0, 0], zeros(1,3), 0, 0, geometry2, sensor, @gradientAscent, 3*d, 2, sprintf("Agent %d", 2));

            % Optional third agent along the +Y axis
            % geometry3 = rectangularPrism;
            % geometry3 = geometry3.initialize([tc.domain.center - [0, d, 0] - tc.collisionRanges(1) * ones(1, 3); tc.domain.center - [0, d, 0] + tc.collisionRanges(1) * ones(1, 3)], REGION_TYPE.COLLISION, sprintf("Agent %d collision volume", 3));
            % tc.agents{3} = agent;
            % tc.agents{3} = tc.agents{3}.initialize(tc.domain.center - [0, d, 0], zeros(1, 3), 0, 0, geometry3, sensor, @gradientAscent, 3*d, 3, sprintf("Agent %d", 3));

            % Initialize the simulation
            [tc.testClass, f] = tc.testClass.initialize(tc.domain, tc.domain.objective, tc.agents, tc.timestep, tc.partitoningFreq, tc.maxIter);
        end
    end
end