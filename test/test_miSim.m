classdef test_miSim < matlab.unittest.TestCase
    properties (Access = private)
        % System under test
        testClass = miSim;

        % Debug
        makeVideo = true; % disable video writing for big performance increase
        makePlots = true; % disable plotting for big performance increase (also disables video)
        plotCommsGeometry = false; % disable plotting communications geometries

        % Sim
        maxIter = 50;
        timestep = 0.05;

        % Domain
        domain = rectangularPrism; % domain geometry
        minDimension = 10;
        minAlt = 1; % minimum allowed agent altitude

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
        initialStepSize = 0.2; % gradient ascent step size at the first iteration. Decreases linearly to 0 based on maxIter.
        minAgents = 3; % Minimum number of agents to be randomly generated
        maxAgents = 4; % Maximum number of agents to be randomly generated
        agents = cell(0, 1);
        
        % Collision
        minCollisionRange = 0.1; % Minimum randomly generated collision geometry size
        maxCollisionRange = 0.5; % Maximum randomly generated collision geometry size
        collisionRanges = NaN;

        % Sensing
        betaDistMin = 3;
        betaDistMax = 15;
        betaTiltMin = 3;
        betaTiltMax = 15;
        alphaDistMin = 2.5;
        alphaDistMax = 3;
        alphaTiltMin = 15; % degrees
        alphaTiltMax = 30; % degrees
        sensor = sigmoidSensor;

        % Communications
        minCommsRange = 3; % Minimum randomly generated collision geometry size
        maxCommsRange = 5; % Maximum randomly generated collision geometry size
        commsRanges = NaN;

        % Constraints
        barrierGain = 100;
        barrierExponent = 3;
    end

    % Setup for each test
    methods (TestMethodSetup)
        % Generate a random domain
        function tc = setDomain(tc)
            % random integer-dimensioned cubic domain
            tc.domain = tc.domain.initializeRandom(REGION_TYPE.DOMAIN, "Domain", tc.minDimension);
            % Random bivariate normal PDF objective
            tc.domain.objective = tc.domain.objective.initializeRandomMvnpdf(tc.domain, tc.discretizationStep, tc.protectedRange);
        end
        % Instantiate agents
        function tc = setAgents(tc)
            % Agents will be initialized under different parameters in individual test cases
            % Instantiate a random number of agents according to parameters
            for ii = 1:randi([tc.minAgents, tc.maxAgents])
                tc.agents{ii, 1} = agent;
            end

            % Random collision ranges for each agent
            tc.collisionRanges = tc.minCollisionRange + rand(size(tc.agents, 1), 1) * (tc.maxCollisionRange - tc.minCollisionRange);
        
            % Random commuunications ranges for each agent
            tc.commsRanges = tc.minCommsRange + rand(size(tc.agents, 1), 1) * (tc.maxCommsRange - tc.minCommsRange);
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
                    % Instantiate a rectangular prism obstacle inside the domain
                    tc.obstacles{ii} = rectangularPrism;
                    tc.obstacles{ii} = tc.obstacles{ii}.initializeRandom(REGION_TYPE.OBSTACLE, sprintf("Obstacle %d", ii), tc.minObstacleSize, tc.maxObstacleSize, tc.domain, tc.minAlt);
    
                    % Check if the obstacle collides with an existing obstacle
                    if ~tc.obstacleCollisionCheck(tc.obstacles(1:(ii - 1)), tc.obstacles{ii})
                        badCandidate = false;
                    end

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
                            candidatePos(3) = tc.minAlt  + rand * 3; % place agents at decent altitudes for sensing
                        end
                    else
                        candidatePos = tc.agents{randi(ii - 1)}.pos + sign(randn([1, 3])) .* (rand(1, 3) .* tc.comRange/sqrt(2));
                        candidatePos(3) = tc.minAlt  + rand * 3; % place agents at decent altitudes for sensing
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
                    candidateGeometry = candidateGeometry.initialize([candidatePos - tc.collisionRanges(ii) * ones(1, 3); candidatePos + tc.collisionRanges(ii) * ones(1, 3)], REGION_TYPE.COLLISION);

                    % Initialize candidate agent sensor model
                    tc.sensor = tc.sensor.initialize(tc.alphaDistMin + rand * (tc.alphaDistMax - tc.alphaDistMin), tc.betaDistMin + rand * (tc.betaDistMax - tc.betaDistMin), tc.alphaTiltMin + rand * (tc.alphaTiltMax - tc.alphaTiltMin), tc.betaTiltMin + rand * (tc.betaTiltMax - tc.betaTiltMin));

                    % Initialize candidate agent
                    newAgent = tc.agents{ii}.initialize(candidatePos, candidateGeometry, tc.sensor, tc.comRange, tc.maxIter, tc.initialStepSize); 

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
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo);
        end
        function misim_run(tc)
            % randomly create obstacles
            nGeom = tc.minNumObstacles + randi(tc.maxNumObstacles - tc.minNumObstacles);
            tc.obstacles = cell(nGeom, 1);

            % Iterate over obstacles to initialize
            for ii = 1:size(tc.obstacles, 1)
                badCandidate = true;
                while badCandidate
                    % Instantiate a rectangular prism obstacle inside the domain
                    tc.obstacles{ii} = rectangularPrism;
                    tc.obstacles{ii} = tc.obstacles{ii}.initializeRandom(REGION_TYPE.OBSTACLE, sprintf("Obstacle %d", ii), tc.minObstacleSize, tc.maxObstacleSize, tc.domain, tc.minAlt);
    
                    % Check if the obstacle collides with an existing obstacle
                    if ~tc.obstacleCollisionCheck(tc.obstacles(1:(ii - 1)), tc.obstacles{ii})
                        badCandidate = false;
                    end
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
                            candidatePos(3) = min([tc.domain.maxCorner(3) * 0.95, tc.minAlt + rand * (tc.alphaDistMax * (1.1)  - 0.5)]); % place agents at decent altitudes for sensing
                        end
                    else
                        candidatePos = tc.agents{randi(ii - 1)}.pos + sign(randn([1, 3])) .* (rand(1, 3) .* tc.comRange/sqrt(2));
                        candidatePos(3) = min([tc.domain.maxCorner(3) * 0.95, tc.minAlt + rand * (tc.alphaDistMax * (1.1)  - 0.5)]); % place agents at decent altitudes for sensing
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
                    % candidateGeometry = rectangularPrism;
                    % candidateGeometry = candidateGeometry.initialize([candidatePos - tc.collisionRanges(ii) * ones(1, 3); candidatePos + tc.collisionRanges(ii) * ones(1, 3)], REGION_TYPE.COLLISION);
                    candidateGeometry = spherical;
                    candidateGeometry = candidateGeometry.initialize(candidatePos, tc.collisionRanges(ii), REGION_TYPE.COLLISION);

                    % Initialize candidate agent sensor model
                    tc.sensor = tc.sensor.initialize(tc.alphaDistMin + rand * (tc.alphaDistMax - tc.alphaDistMin), tc.betaDistMin + rand * (tc.betaDistMax - tc.betaDistMin), tc.alphaTiltMin + rand * (tc.alphaTiltMax - tc.alphaTiltMin), tc.betaTiltMin + rand * (tc.betaTiltMax - tc.betaTiltMin));

                    % Initialize candidate agent
                    newAgent = tc.agents{ii}.initialize(candidatePos, candidateGeometry, tc.sensor, tc.comRange, tc.maxIter, tc.initialStepSize);
                    
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
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo);

            % Write out parameters
            tc.testClass.writeParams();

            % Run simulation loop
            tc.testClass = tc.testClass.run();
        end
        function test_basic_partitioning(tc)
            % place agents a fixed distance +/- X from the domain's center
            d = 1;

            % Initialize agent collision geometry
            tc.agents = {agent; agent; agent};
            geometry1 = spherical;
            geometry2 = geometry1;
            geometry3 = geometry1;
            geometry1 = geometry1.initialize(tc.domain.center + [d, 0, 0], tc.collisionRanges(1), REGION_TYPE.COLLISION);
            geometry2 = geometry2.initialize(tc.domain.center - [d, 0, 0], tc.collisionRanges(2), REGION_TYPE.COLLISION);
            geometry3 = geometry3.initialize(tc.domain.center - [0, d, 0], tc.collisionRanges(3), REGION_TYPE.COLLISION);
            
            % Initialize agent sensor model with fixed parameters
            tc.sensor = tc.sensor.initialize(tc.domain.maxCorner(3) / 2, 9, 22.5, 9);

            % Initialize agents
            tc.commsRanges = 3 * d * ones(size(tc.agents));
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center + [d, 0, 0], geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize);
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center - [d, 0, 0], geometry2, tc.sensor, tc.commsRanges(2), tc.maxIter, tc.initialStepSize);
            tc.agents{3} = tc.agents{3}.initialize(tc.domain.center - [0, d, 0], geometry3, tc.sensor, tc.commsRanges(3), tc.maxIter, tc.initialStepSize);

            % Initialize the simulation
            tc.obstacles = cell(0, 1);
            tc.makePlots = false;
            tc.makeVideo = false;
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo);
        
            centerIdx = floor(size(tc.testClass.partitioning, 1) / 2);
            tc.verifyEqual(tc.testClass.partitioning(centerIdx, centerIdx:(centerIdx + 2)), [2, 3, 1]); % all three near center
            tc.verifyLessThan(sum(tc.testClass.partitioning == 1, 'all'), sum(tc.testClass.partitioning == 0, 'all')); % more non-assignments than partition 1 assignments
            tc.verifyLessThan(sum(tc.testClass.partitioning == 2, 'all'), sum(tc.testClass.partitioning == 1, 'all')); % more partition 1 assignments than partition 2 assignments
            tc.verifyLessThan(sum(tc.testClass.partitioning == 3, 'all'), sum(tc.testClass.partitioning == 2, 'all')); % more partition 3 assignments than partition 2 assignments
            tc.verifyEqual(unique(tc.testClass.partitioning), [0; 1; 2; 3;]);
        end
        function test_single_partition(tc)
            % Initialize agent collision geometry
            tc.agents = {agent};
            geometry1 = spherical;
            geometry1 = geometry1.initialize([tc.domain.center(1:2), 3], tc.collisionRanges(1), REGION_TYPE.COLLISION);
            
            % Initialize agent sensor model with fixed parameters
            tc.sensor = tc.sensor.initialize(tc.minDimension / 2, 3, 20, 3);

            % Initialize agents
            tc.agents{1} = tc.agents{1}.initialize([tc.domain.center(1:2), 3], geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize);

            % Initialize the simulation
            tc.obstacles = cell(0, 1);
            tc.makePlots = false;
            tc.makeVideo = false;
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo);
            close(tc.testClass.fPerf);

            tc.verifyEqual(unique(tc.testClass.partitioning), [0; 1]);
            tc.verifyLessThan(sum(tc.testClass.partitioning == 1, 'all'), sum(tc.testClass.partitioning == 0, 'all'));
        end
        function test_single_agent_gradient_ascent(tc)
            % make basic domain
            tc.minDimension = 10; % domain size
            tc.domain = tc.domain.initialize([zeros(1, 3);tc.minDimension* ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([7, 6]), tc.domain, tc.discretizationStep, tc.protectedRange);
        
            % Initialize agent collision geometry
            tc.agents = {agent};
            geometry1 = rectangularPrism;
            geometry1 = geometry1.initialize([[tc.domain.center(1:2)-tc.domain.dimensions(1)/4, 3] - tc.collisionRanges(1) * ones(1, 3); [tc.domain.center(1:2)-tc.domain.dimensions(1)/4, 3] + tc.collisionRanges(1) * ones(1, 3)], REGION_TYPE.COLLISION);
            
            % Initialize agent sensor model with fixed parameters
            tc.sensor = tc.sensor.initialize(tc.minDimension / 2, 3, 20, 3);

            % Initialize agents
            tc.maxIter = 75;
            tc.agents{1} = tc.agents{1}.initialize([tc.domain.center(1:2)-tc.domain.dimensions(1)/4, 3], geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize);

            % Initialize the simulation
            tc.obstacles = cell(0, 1);
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo);
            
            % Run the simulation
            tc.testClass = tc.testClass.run();end
        function test_collision_avoidance(tc)
            % No obstacles
            % Fixed agent initial conditions
            % Exaggerated large collision geometries to test CA
            % make basic domain
            tc.minDimension = 10; % domain size
            tc.domain = tc.domain.initialize([zeros(1, 3);tc.minDimension* ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([3, 7]), tc.domain, tc.discretizationStep, tc.protectedRange);
        
            % Initialize agent collision geometry
            tc.agents = {agent; agent};
            tc.collisionRanges = 1.5 * ones(size(tc.agents));
            d = [2.5, 0, 0];
            geometry1 = spherical;
            geometry2 = spherical;
            geometry1 = geometry1.initialize(tc.domain.center + d, tc.collisionRanges(1), REGION_TYPE.COLLISION);
            geometry2 = geometry2.initialize(tc.domain.center - d, tc.collisionRanges(2), REGION_TYPE.COLLISION);
            
            % Initialize agent sensor model with fixed parameters
            tc.sensor = tc.sensor.initialize(tc.minDimension / 2, 3, 15, 3);

            % Initialize agents
            tc.maxIter = 25;
            tc.commsRanges = 5 * ones(size(tc.agents));
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center + d, geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize);
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center - d, geometry2, tc.sensor, tc.commsRanges(2), tc.maxIter, tc.initialStepSize);

            % Initialize the simulation
            tc.obstacles = cell(0, 1);
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo);
            
            % Run the simulation
            tc.testClass.run();
        end
        function test_obstacle_avoidance(tc)
            % Right now, the communications constraint is violated here

            % Fixed single obstacle
            % Fixed two agents initial conditions
            % Exaggerated large collision geometries
            % make basic domain
            tc.minDimension = 10; % domain size
            tc.domain = tc.domain.initialize([zeros(1, 3);tc.minDimension* ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([8, 5.2195]), tc.domain, tc.discretizationStep, tc.protectedRange);
        
            % Initialize agent collision geometry
            tc.agents = {agent; agent;};
            tc.collisionRanges = 1.1 * ones(size(tc.agents));
            d = [3, 0, 0];

            yOffset = 1;
            % choice of 0 leads to the agents getting stuck attempting to go around the obstacle on both sides
            % choice of 1 leads to one agent easily going around while the other gets stuck and the communications link is broken

            geometry1 = spherical;
            geometry2 = geometry1;
            geometry1 = geometry1.initialize(tc.domain.center - d + [0, tc.collisionRanges(1) * 1.1 - yOffset, 0], tc.collisionRanges(1), REGION_TYPE.COLLISION);
            geometry2 = geometry2.initialize(tc.domain.center - d - [0, tc.collisionRanges(2) * 1.1 + yOffset, 0], tc.collisionRanges(2), REGION_TYPE.COLLISION);
            
            % Initialize agent sensor model with fixed parameters
            tc.sensor = tc.sensor.initialize(tc.minDimension / 2, 3, 15, 3);
            
            % Initialize obstacles
            obstacleLength = 1;
            tc.obstacles{1} = rectangularPrism;
            tc.obstacles{1} = tc.obstacles{1}.initialize([tc.domain.center(1:2) - obstacleLength, tc.minAlt; tc.domain.center(1:2) + obstacleLength, tc.domain.maxCorner(3)], REGION_TYPE.OBSTACLE, "Obstacle 1");

            % Initialize agents
            tc.commsRanges = (2 * tc.collisionRanges(1) + obstacleLength) * 0.9 * ones(size(tc.agents)); % defined such that they cannot go around the obstacle on both sides
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center - d + [0, tc.collisionRanges(1) * 1.1 - yOffset, 0], geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize);
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center - d - [0, tc.collisionRanges(2)  *1.1 + yOffset, 0], geometry2, tc.sensor, tc.commsRanges(2), tc.maxIter, tc.initialStepSize);
            
            % Initialize the simulation
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo);
            
            % Run the simulation
            tc.testClass.run();
        end
        function test_communications_constraint(tc)
            % No obstacles
            % Fixed two agents initial conditions
            % Negligible collision geometries
            % Non-standard domain with two objectives that will try to pull the
            % agents apart
            tc.minDimension = 10; % domain size
            tc.domain = tc.domain.initialize([zeros(1, 3);tc.minDimension* ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([2, 8; 8, 8]), tc.domain, tc.discretizationStep, tc.protectedRange);
        
            % Initialize agent collision geometry
            tc.agents = {agent; agent;};
            tc.collisionRanges = .25 * ones(size(tc.agents));
            d = [1, 0, 0];
            geometry1 = spherical;
            geometry2 = geometry1;
            geometry1 = geometry1.initialize(tc.domain.center + d, tc.collisionRanges(1), REGION_TYPE.COLLISION);
            geometry2 = geometry2.initialize(tc.domain.center - d, tc.collisionRanges(2), REGION_TYPE.COLLISION);
            
            % Initialize agent sensor model
            tc.sensor = sigmoidSensor;
            tc.sensor = tc.sensor.initialize(tc.minDimension / 2, 3, 15, 3);
            
            % Initialize obstacles
            tc.obstacles = {};
            
            % Initialize agents
            tc.maxIter = 50;
            tc.commsRanges = 4 * ones(size(tc.agents)); % defined such that they cannot reach their objective without breaking connectivity
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center + d, geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize);
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center - d, geometry2, tc.sensor, tc.commsRanges(2), tc.maxIter, tc.initialStepSize);

            % Initialize the simulation
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo);
            
            % Run the simulation
            tc.testClass = tc.testClass.run();
        end
        function test_obstacle_permits_comms_LOS(tc)
            % Fixed single obstacle
            % Fixed two agents initial conditions
            % Exaggerated large communications radius
            % make basic domain
            tc.minDimension = 10; % domain size
            tc.domain = tc.domain.initialize([zeros(1, 3); tc.minDimension* ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([8, 5]), tc.domain, tc.discretizationStep, tc.protectedRange);
        
            % Initialize agent collision geometry
            tc.agents = {agent; agent;};
            tc.collisionRanges = .25 * ones(size(tc.agents));
            d = 2;
            geometry1 = spherical;
            geometry2 = geometry1;
            geometry1 = geometry1.initialize(tc.domain.center - [d, 0, 0], tc.collisionRanges(1), REGION_TYPE.COLLISION);
            geometry2 = geometry2.initialize(tc.domain.center - [0, d, 0], tc.collisionRanges(2), REGION_TYPE.COLLISION);
            
            % Initialize agent sensor model
            tc.sensor = tc.sensor.initialize(tc.minDimension / 2, 3, 15, 3);

            % Initialize agents
            tc.maxIter = 125;
            tc.commsRanges = 5 * ones(size(tc.agents));
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center - [d, 0, 0], geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize);
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center - [0, d, 0], geometry2, tc.sensor, tc.commsRanges(2), tc.maxIter, tc.initialStepSize);
            
            % Initialize obstacles
            obstacleLength = 1.5;
            tc.obstacles{1} = rectangularPrism;
            tc.obstacles{1} = tc.obstacles{1}.initialize([tc.domain.center(1:2) - obstacleLength, 0; tc.domain.center(1:2) + obstacleLength, tc.domain.maxCorner(3)], REGION_TYPE.OBSTACLE, "Obstacle 1");

            % Initialize the simulation
            tc.minAlt = 0;
            tc.makePlots = false;
            tc.makeVideo = false;
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo);

            % Communications link should be established
            tc.assertEqual(tc.testClass.adjacency, logical(true(2)));
        end
        function test_LNA_case_1(tc)
            % based on example in meeting 
            % no obstacles
            % fixed 5 agents initial conditions
            % unit communicaitons radius
            % negligible collision radius
            % make basic domain
            tc.minDimension = 10; % domain size
            tc.domain = tc.domain.initialize([zeros(1, 3);tc.minDimension* ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([8, 5]), tc.domain, tc.discretizationStep, tc.protectedRange);
        
            % Initialize agent collision geometry
            tc.agents = {agent; agent; agent; agent; agent;};
            tc.collisionRanges = .01 * ones(size(tc.agents));
            d = 1;
            geometry5 = spherical;
            geometry1 = geometry5.initialize(tc.domain.center + [d, 0, 0], tc.collisionRanges(1), REGION_TYPE.COLLISION);
            geometry2 = geometry5.initialize(tc.domain.center, tc.collisionRanges(2), REGION_TYPE.COLLISION);
            geometry3 = geometry5.initialize(tc.domain.center + [-d, d, 0], tc.collisionRanges(3), REGION_TYPE.COLLISION);
            geometry4 = geometry5.initialize(tc.domain.center + [-2*d, d, 0], tc.collisionRanges(4), REGION_TYPE.COLLISION);
            geometry5 = geometry5.initialize(tc.domain.center + [0, d, 0], tc.collisionRanges(5), REGION_TYPE.COLLISION);
            
            % Initialize agent sensor model
            tc.sensor = tc.sensor.initialize(tc.minDimension / 2, 3, 15, 3);

            % Initialize agents
            tc.maxIter = 125;
            tc.commsRanges = ones(size(tc.agents));
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center + [d, 0, 0], geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize);
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center, geometry2, tc.sensor, tc.commsRanges(2), tc.maxIter, tc.initialStepSize);
            tc.agents{3} = tc.agents{3}.initialize(tc.domain.center + [-d, d, 0], geometry3, tc.sensor, tc.commsRanges(3), tc.maxIter, tc.initialStepSize);
            tc.agents{4} = tc.agents{4}.initialize(tc.domain.center + [-2*d, d, 0], geometry4, tc.sensor, tc.commsRanges(4), tc.maxIter, tc.initialStepSize);
            tc.agents{5} = tc.agents{5}.initialize(tc.domain.center + [0, d, 0], geometry5, tc.sensor, tc.commsRanges(5), tc.maxIter, tc.initialStepSize);

            % Initialize the simulation
            tc.minAlt = 0;
            tc.makePlots = false;
            tc.makeVideo = false;
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo);

            % Constraint adjacency matrix defined by LNA should be as follows
            tc.assertEqual(tc.testClass.constraintAdjacencyMatrix, logical( ...
                [ 1, 1, 0, 0, 0; ...
                  1, 1, 0, 0, 1; ...
                  0, 0, 1, 1, 1;
                  0, 0, 1, 1, 0;
                  0, 1, 1, 0, 1;]));
        end
        function test_LNA_case_2(tc)
            % based on example in paper Asynchronous Local Construction of Bounded-Degree Network Topologies Using Only Neighborhood Information
            % No obstacles
            % Fixed 7 agents initial conditions
            % unitary communicaitons radius
            % negligible collision radius
            % make basic domain
            tc.minDimension = 10; % domain size
            tc.domain = tc.domain.initialize([zeros(1, 3); tc.minDimension* ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([8, 5]), tc.domain, tc.discretizationStep, tc.protectedRange);
        
            % Initialize agent collision geometry
            tc.agents = {agent; agent; agent; agent; agent; agent; agent;};
            tc.collisionRanges = .01 * ones(size(tc.agents));
            d = 1;
            geometry7 = spherical;
            geometry1 = geometry7.initialize(tc.domain.center + [-0.9 * d/sqrt(2), 0.9 * d/sqrt(2), 0], tc.collisionRanges(1), REGION_TYPE.COLLISION);
            geometry2 = geometry7.initialize(tc.domain.center + [-0.5 * d, 0.25 * d, 0], tc.collisionRanges(2), REGION_TYPE.COLLISION);
            geometry3 = geometry7.initialize(tc.domain.center + [0.9 * d, 0, 0], tc.collisionRanges(3), REGION_TYPE.COLLISION);
            geometry4 = geometry7.initialize(tc.domain.center + [0.9 * d/sqrt(2), -0.9 * d/sqrt(2), 0], tc.collisionRanges(4), REGION_TYPE.COLLISION);
            geometry5 = geometry7.initialize(tc.domain.center + [0, 0.9 * d, 0], tc.collisionRanges(5), REGION_TYPE.COLLISION);
            geometry6 = geometry7.initialize(tc.domain.center, tc.collisionRanges(6), REGION_TYPE.COLLISION);
            geometry7 = geometry7.initialize(tc.domain.center + [d/2, d/2, 0], tc.collisionRanges(7), REGION_TYPE.COLLISION);
            
            % Initialize agent sensor model
            tc.sensor = tc.sensor.initialize(tc.minDimension / 2, 3, 15, 3);

            % Initialize agents
            tc.maxIter = 125;
            tc.commsRanges = d * ones(size(tc.agents));
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center + [-0.9 * d/sqrt(2), 0.9 * d/sqrt(2), 0], geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize);
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center + [-0.5 * d, 0.25 * d, 0], geometry2, tc.sensor, tc.commsRanges(2), tc.maxIter, tc.initialStepSize);
            tc.agents{3} = tc.agents{3}.initialize(tc.domain.center + [0.9 * d, 0, 0], geometry3, tc.sensor, tc.commsRanges(3), tc.maxIter, tc.initialStepSize);
            tc.agents{4} = tc.agents{4}.initialize(tc.domain.center + [0.9 * d/sqrt(2), -0.9 * d/sqrt(2), 0], geometry4, tc.sensor, tc.commsRanges(4), tc.maxIter, tc.initialStepSize);
            tc.agents{5} = tc.agents{5}.initialize(tc.domain.center + [0, 0.9 * d, 0], geometry5, tc.sensor, tc.commsRanges(5), tc.maxIter, tc.initialStepSize);
            tc.agents{6} = tc.agents{6}.initialize(tc.domain.center, geometry6, tc.sensor, tc.commsRanges(6), tc.maxIter, tc.initialStepSize);
            tc.agents{7} = tc.agents{7}.initialize(tc.domain.center + [d/2, d/2, 0], geometry7, tc.sensor, tc.commsRanges(7), tc.maxIter, tc.initialStepSize);

            % Initialize the simulation
            tc.minAlt = 0;
            tc.makePlots = false;
            tc.makeVideo = false;
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo);

            % Constraint adjacency matrix defined by LNA should be as follows
            tc.assertEqual(tc.testClass.constraintAdjacencyMatrix, logical( ...
                [ 1, 1, 0, 0, 0, 0, 0; ...
                  1, 1, 0, 0, 1, 0, 0; ...
                  0, 0, 1, 1, 0, 0, 0;
                  0, 0, 1, 1, 0, 1, 0;
                  0, 1, 0, 0, 1, 1, 0;
                  0, 0, 0, 1, 1, 1, 1;
                  0, 0, 0, 0, 0, 1, 1; ]));
        end
    end

    methods
        function c = obstacleCollisionCheck(~, obstacles, obstacle)
            % Check if the obstacle intersects with any other obstacles
            c = false;
            for ii = 1:size(obstacles, 1)
                if geometryIntersects(obstacles{ii}, obstacle)
                    c = true;
                end
            end
        end
    end
end
