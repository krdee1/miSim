classdef test_miSim < matlab.unittest.TestCase
    properties (Access = private)
        % System under test
        testClass = miSim;

        % Debug
        makeVideo = true; % disable video writing for big performance increase
        makePlots = true; % disable plotting for big performance increase (also disables video)
        plotCommsGeometry = false; % disable plotting communications geometries

        % Sim
        maxIter = 250;
        timestep = 0.05
        partitoningFreq = 5;

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
        minAgents = 4; % Minimum number of agents to be randomly generated
        maxAgents = 6; % Maximum number of agents to be randomly generated
        sensingLength = 0.05; % length parameter used by sensing function
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

        % Communications
        comRange = 8; % Maximum range between agents that forms a communications link
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
                    sensor = sigmoidSensor;
                    sensor = sensor.initialize(tc.alphaDistMin + rand * (tc.alphaDistMax - tc.alphaDistMin), tc.betaDistMin + rand * (tc.betaDistMax - tc.betaDistMin), NaN, NaN, tc.alphaTiltMin + rand * (tc.alphaTiltMax - tc.alphaTiltMin), tc.betaTiltMin + rand * (tc.betaTiltMax - tc.betaTiltMin));

                    % Initialize candidate agent
                    newAgent = tc.agents{ii}.initialize(candidatePos, zeros(1,3), 0, 0, candidateGeometry, sensor, @gradientAscent, tc.comRange); 

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
            tc.testClass = tc.testClass.initialize(tc.domain, tc.domain.objective, tc.agents, tc.minAlt, tc.timestep, tc.partitoningFreq, tc.maxIter, tc.obstacles, tc.makeVideo);
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
                    sensor = sigmoidSensor;
                    sensor = sensor.initialize(tc.alphaDistMin + rand * (tc.alphaDistMax - tc.alphaDistMin), tc.betaDistMin + rand * (tc.betaDistMax - tc.betaDistMin), NaN, NaN, tc.alphaTiltMin + rand * (tc.alphaTiltMax - tc.alphaTiltMin), tc.betaTiltMin + rand * (tc.betaTiltMax - tc.betaTiltMin));

                    % Initialize candidate agent
                    newAgent = tc.agents{ii}.initialize(candidatePos, zeros(1,3), 0, 0, candidateGeometry, sensor, @gradientAscent, tc.comRange);
                    
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
            tc.testClass = tc.testClass.initialize(tc.domain, tc.domain.objective, tc.agents, tc.minAlt, tc.timestep, tc.partitoningFreq, tc.maxIter, tc.obstacles, tc.makeVideo);

            % Run simulation loop
            tc.testClass = tc.testClass.run();
        end
        function test_basic_partitioning(tc)
            % place agents a fixed distance +/- X from the domain's center
            d = 1;

            % make basic domain
            tc.domain = tc.domain.initialize([zeros(1, 3); 10 * ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(@(x, y) mvnpdf([x(:), y(:)], tc.domain.center(1:2)), tc.domain, tc.discretizationStep, tc.protectedRange);
        
            % Initialize agent collision geometry
            dh = [0,0,-1]; % bias agent altitude from domain center
            geometry1 = rectangularPrism;
            geometry2 = geometry1;
            geometry1 = geometry1.initialize([tc.domain.center + dh + [d, 0, 0] - tc.collisionRanges(1) * ones(1, 3); tc.domain.center + dh + [d, 0, 0] + tc.collisionRanges(1) * ones(1, 3)], REGION_TYPE.COLLISION);
            geometry2 = geometry2.initialize([tc.domain.center + dh - [d, 0, 0] - tc.collisionRanges(1) * ones(1, 3); tc.domain.center + dh - [d, 0, 0] + tc.collisionRanges(1) * ones(1, 3)], REGION_TYPE.COLLISION);
            
            % Initialize agent sensor model
            sensor = sigmoidSensor;
            % Homogeneous sensor model parameters
            sensor = sensor.initialize(2.75, 9, NaN, NaN, 22.5, 9);
            % Heterogeneous sensor model parameters
            % sensor = sensor.initialize(tc.alphaDistMin + rand * (tc.alphaDistMax - tc.alphaDistMin), tc.betaDistMin + rand * (tc.betaDistMax - tc.betaDistMin), NaN, NaN, tc.alphaTiltMin + rand * (tc.alphaTiltMax - tc.alphaTiltMin), tc.betaTiltMin + rand * (tc.betaTiltMax - tc.betaTiltMin));

            % Plot sensor parameters (optional)
            % f = sensor.plotParameters();

            % Initialize agents
            tc.agents = {agent; agent};
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center + dh + [d, 0, 0], zeros(1,3), 0, 0, geometry1, sensor, @gradientAscent, 3*d);
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center + dh - [d, 0, 0], zeros(1,3), 0, 0, geometry2, sensor, @gradientAscent, 3*d);

            % Optional third agent along the +Y axis
            geometry3 = rectangularPrism;
            geometry3 = geometry3.initialize([tc.domain.center + dh - [0, d, 0] - tc.collisionRanges(1) * ones(1, 3); tc.domain.center + dh - [0, d, 0] + tc.collisionRanges(1) * ones(1, 3)], REGION_TYPE.COLLISION);
            tc.agents{3} = agent;
            tc.agents{3} = tc.agents{3}.initialize(tc.domain.center + dh - [0, d, 0], zeros(1, 3), 0, 0, geometry3, sensor, @gradientAscent, 3*d);

            % Initialize the simulation
            tc.testClass = tc.testClass.initialize(tc.domain, tc.domain.objective, tc.agents, tc.minAlt, tc.timestep, tc.partitoningFreq, tc.maxIter, cell(0, 1), tc.makeVideo);
            close(tc.testClass.fPerf);
        end
        function test_single_partition(tc)
            % make basic domain
            l = 10; % domain size
            tc.domain = tc.domain.initialize([zeros(1, 3); l * ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(@(x, y) mvnpdf([x(:), y(:)], tc.domain.center(1:2) + rand(1, 2) * 6 - 3), tc.domain, tc.discretizationStep, tc.protectedRange);
        
            % Initialize agent collision geometry
            geometry1 = rectangularPrism;
            geometry1 = geometry1.initialize([[tc.domain.center(1:2), 3] - tc.collisionRanges(1) * ones(1, 3); [tc.domain.center(1:2), 3] + tc.collisionRanges(1) * ones(1, 3)], REGION_TYPE.COLLISION);
            
            % Initialize agent sensor model
            sensor = sigmoidSensor;
            % Homogeneous sensor model parameters
            % sensor = sensor.initialize(2.5666, 5.0807, NaN, NaN, 20.8614, 13); % 13
            alphaDist = l/2; % half of domain length/width
            sensor = sensor.initialize(alphaDist, 3, NaN, NaN, 20, 3);

            % Plot sensor parameters (optional)
            f = sensor.plotParameters();

            % Initialize agents
            tc.agents = {agent};
            tc.agents{1} = tc.agents{1}.initialize([tc.domain.center(1:2), 3], zeros(1,3), 0, 0, geometry1, sensor, @gradientAscent, 3);

            % Initialize the simulation
            tc.testClass = tc.testClass.initialize(tc.domain, tc.domain.objective, tc.agents, tc.minAlt, tc.timestep, tc.partitoningFreq, tc.maxIter, cell(0, 1), tc.makeVideo);
            close(tc.testClass.fPerf);
        end
        function test_single_partition_basic_GA(tc)
            % make basic domain
            l = 10; % domain size
            tc.domain = tc.domain.initialize([zeros(1, 3); l * ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(@(x, y) mvnpdf([x(:), y(:)], [2, 8]), tc.domain, tc.discretizationStep, tc.protectedRange);
        
            % Initialize agent collision geometry
            geometry1 = rectangularPrism;
            geometry1 = geometry1.initialize([[tc.domain.center(1:2)-tc.domain.dimensions(1)/3, 3] - tc.collisionRanges(1) * ones(1, 3); [tc.domain.center(1:2)-tc.domain.dimensions(1)/3, 3] + tc.collisionRanges(1) * ones(1, 3)], REGION_TYPE.COLLISION);
            
            % Initialize agent sensor model
            sensor = sigmoidSensor;
            % Homogeneous sensor model parameters
            % sensor = sensor.initialize(2.5666, 5.0807, NaN, NaN, 20.8614, 13); % 13
            alphaDist = l/2; % half of domain length/width
            sensor = sensor.initialize(alphaDist, 3, NaN, NaN, 20, 3);

            % Plot sensor parameters (optional)
            f = sensor.plotParameters();

            % Initialize agents
            tc.agents = {agent};
            tc.agents{1} = tc.agents{1}.initialize([tc.domain.center(1:2)-tc.domain.dimensions(1)/3, 3], zeros(1,3), 0, 0, geometry1, sensor, @gradientAscent, 3, "", true);

            % Initialize the simulation
            tc.testClass = tc.testClass.initialize(tc.domain, tc.domain.objective, tc.agents, tc.minAlt, tc.timestep, tc.partitoningFreq, tc.maxIter, cell(0, 1), tc.makeVideo);
            
            % Run the simulation
            tc.testClass.run();
        end
        function test_collision_avoidance(tc)
            % No obstacles
            % Fixed agent initial conditions
            % Exaggerated large collision geometries to test CA
            % make basic domain
            l = 10; % domain size
            tc.domain = tc.domain.initialize([zeros(1, 3); l * ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(@(x, y) mvnpdf([x(:), y(:)], [3, 7]), tc.domain, tc.discretizationStep, tc.protectedRange);
        
            % Initialize agent collision geometry
            radius = 1.5;
            d = [2.5, 0, 0];
            geometry1 = spherical;
            geometry2 = spherical;
            geometry1 = geometry1.initialize(tc.domain.center + d, radius, REGION_TYPE.COLLISION);
            geometry2 = geometry2.initialize(tc.domain.center - d, radius, REGION_TYPE.COLLISION);
            
            % Initialize agent sensor model
            sensor = sigmoidSensor;
            % Homogeneous sensor model parameters
            % sensor = sensor.initialize(2.5666, 5.0807, NaN, NaN, 20.8614, 13); % 13
            alphaDist = l/2; % half of domain length/width
            sensor = sensor.initialize(alphaDist, 3, NaN, NaN, 15, 3);

            % Initialize agents
            tc.agents = {agent; agent};
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center + d, zeros(1,3), 0, 0, geometry1, sensor, @gradientAscent, 3);
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center - d, zeros(1,3), 0, 0, geometry2, sensor, @gradientAscent, 3);

            % Initialize the simulation
            tc.testClass = tc.testClass.initialize(tc.domain, tc.domain.objective, tc.agents, tc.minAlt, tc.timestep, tc.partitoningFreq, tc.maxIter, cell(0, 1), tc.makeVideo, tc.makePlots);
            
            % Run the simulation
            tc.testClass.run();
        end
        function test_obstacle_avoidance(tc)
            % Fixed single obstacle
            % Fixed two agents initial conditions
            % Exaggerated large collision geometries
            % make basic domain
            l = 10; % domain size
            tc.domain = tc.domain.initialize([zeros(1, 3); l * ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(@(x, y) mvnpdf([x(:), y(:)], [8, 5]), tc.domain, tc.discretizationStep, tc.protectedRange);
        
            % Initialize agent collision geometry
            radius = 1.1;
            d = [3, 0, 0];
            geometry1 = spherical;
            geometry2 = geometry1;
            geometry1 = geometry1.initialize(tc.domain.center - d + [0, radius * 1.5, 0], radius, REGION_TYPE.COLLISION);
            geometry2 = geometry2.initialize(tc.domain.center - d - [0, radius * 1.5, 0], radius, REGION_TYPE.COLLISION);
            
            % Initialize agent sensor model
            sensor = sigmoidSensor;
            alphaDist = l/2; % half of domain length/width
            sensor = sensor.initialize(alphaDist, 3, NaN, NaN, 15, 3);

            % Initialize agents
            tc.agents = {agent; agent;};
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center - d + [0, radius * 1.5, 0], zeros(1,3), 0, 0, geometry1, sensor, @gradientAscent, 10);
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center - d - [0, radius * 1.5, 0], zeros(1,3), 0, 0, geometry2, sensor, @gradientAscent, 10);
            
            % Initialize obstacles
            obstacleLength = 1;
            tc.obstacles{1} = rectangularPrism;
            tc.obstacles{1} = tc.obstacles{1}.initialize([tc.domain.center(1:2) - obstacleLength, tc.minAlt; tc.domain.center(1:2) + obstacleLength, tc.domain.maxCorner(3)], REGION_TYPE.OBSTACLE, "Obstacle 1");

            % Initialize the simulation
            tc.testClass = tc.testClass.initialize(tc.domain, tc.domain.objective, tc.agents, tc.minAlt, tc.timestep, tc.partitoningFreq, 100, tc.obstacles, tc.makeVideo);
            
            % Run the simulation
            tc.testClass.run();
        end
        function test_obstacle_blocks_comms_LOS(tc)
            % Fixed single obstacle
            % Fixed two agents initial conditions
            % Exaggerated large communications radius
            % make basic domain
            l = 10; % domain size
            tc.domain = tc.domain.initialize([zeros(1, 3); l * ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(@(x, y) mvnpdf([x(:), y(:)], [8, 5]), tc.domain, tc.discretizationStep, tc.protectedRange);
        
            % Initialize agent collision geometry
            radius = .25;
            d = 2;
            geometry1 = spherical;
            geometry2 = geometry1;
            geometry1 = geometry1.initialize(tc.domain.center - [d, 0, 0], radius, REGION_TYPE.COLLISION);
            geometry2 = geometry2.initialize(tc.domain.center - [0, d, 0], radius, REGION_TYPE.COLLISION);
            
            % Initialize agent sensor model
            sensor = sigmoidSensor;
            alphaDist = l/2; % half of domain length/width
            sensor = sensor.initialize(alphaDist, 3, NaN, NaN, 15, 3);

            % Initialize agents
            commsRadius = 5;
            tc.agents = {agent; agent;};
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center - [d, 0, 0], zeros(1,3), 0, 0, geometry1, sensor, @gradientAscent, commsRadius);
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center - [0, d, 0], zeros(1,3), 0, 0, geometry2, sensor, @gradientAscent, commsRadius);
            
            % Initialize obstacles
            obstacleLength = 1.5;
            tc.obstacles{1} = rectangularPrism;
            tc.obstacles{1} = tc.obstacles{1}.initialize([tc.domain.center(1:2) - obstacleLength, 0; tc.domain.center(1:2) + obstacleLength, tc.domain.maxCorner(3)], REGION_TYPE.OBSTACLE, "Obstacle 1");

            % Initialize the simulation
            tc.testClass = tc.testClass.initialize(tc.domain, tc.domain.objective, tc.agents, 0, tc.timestep, tc.partitoningFreq, 125, tc.obstacles, false, false);

            % No communications link should be established
            tc.assertEqual(tc.testClass.adjacency, logical(eye(2)));
        end
        function test_LNA_example_case(tc)
            % No obstacles
            % Fixed 5 agents initial conditions
            % unitary communicaitons radius
            % negligible collision radius
            % make basic domain
            l = 10; % domain size
            tc.domain = tc.domain.initialize([zeros(1, 3); l * ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(@(x, y) mvnpdf([x(:), y(:)], [8, 5]), tc.domain, tc.discretizationStep, tc.protectedRange);
        
            % Initialize agent collision geometry
            radius = .01;
            d = 1;
            geometry1 = spherical;
            geometry2 = geometry1;
            geometry3 = geometry2;
            geometry4 = geometry3;
            geometry5 = geometry4;
            geometry1 = geometry1.initialize(tc.domain.center + [d, 0, 0], radius, REGION_TYPE.COLLISION);
            geometry2 = geometry2.initialize(tc.domain.center, radius, REGION_TYPE.COLLISION);
            geometry3 = geometry2.initialize(tc.domain.center + [-d, d, 0], radius, REGION_TYPE.COLLISION);
            geometry4 = geometry2.initialize(tc.domain.center + [-2*d, d, 0], radius, REGION_TYPE.COLLISION);
            geometry5 = geometry2.initialize(tc.domain.center + [0, d, 0], radius, REGION_TYPE.COLLISION);
            
            % Initialize agent sensor model
            sensor = sigmoidSensor;
            alphaDist = l/2; % half of domain length/width
            sensor = sensor.initialize(alphaDist, 3, NaN, NaN, 15, 3);

            % Initialize agents
            commsRadius = d;
            tc.agents = {agent; agent; agent; agent; agent;};
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center + [d, 0, 0], zeros(1,3), 0, 0, geometry1, sensor, @gradientAscent, commsRadius);
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center, zeros(1,3), 0, 0, geometry2, sensor, @gradientAscent, commsRadius);
            tc.agents{3} = tc.agents{3}.initialize(tc.domain.center + [-d, d, 0], zeros(1,3), 0, 0, geometry3, sensor, @gradientAscent, commsRadius);
            tc.agents{4} = tc.agents{4}.initialize(tc.domain.center + [-2*d, d, 0], zeros(1,3), 0, 0, geometry4, sensor, @gradientAscent, commsRadius);
            tc.agents{5} = tc.agents{5}.initialize(tc.domain.center + [0, d, 0], zeros(1,3), 0, 0, geometry5, sensor, @gradientAscent, commsRadius);

            % TODO
            % make @gradientAscent always the choice
            % Build collision geometry initialization into agent initialization?
            
            % Initialize the simulation
            tc.testClass = tc.testClass.initialize(tc.domain, tc.domain.objective, tc.agents, 0, tc.timestep, tc.partitoningFreq, 125, tc.obstacles, false, false);

            % Constraint adjacency matrix defined by LNA should be as follows
            tc.assertEqual(tc.testClass.constraintAdjacencyMatrix, logical( ...
                [ 1, 1, 0, 0, 0; ...
                  1, 1, 0, 0, 1; ...
                  0, 0, 1, 1, 1;
                  0, 0, 1, 0, 0;
                  0, 1, 1, 0, 0; ]));
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