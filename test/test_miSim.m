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
        timestep = 0.1;

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
        initialMaxAngleStepSize = 0.1; % angular step size (degrees) for tilt/azimuth gradient ascent per timestep.
        minAgents = 7; % Minimum number of agents to be randomly generated
        maxAgents = 7; % Maximum number of agents to be randomly generated
        useDoubleIntegrator = false;
        dampingCoeff = 2;
        agents = cell(0, 1);
        
        % Collision
        minCollisionRange = 0.1; % Minimum randomly generated collision geometry size
        maxCollisionRange = 0.5; % Maximum randomly generated collision geometry size
        collisionRanges = NaN;

        % Sensing
        sensor = sigmoidSensor;
        % sigmoidSensor
        betaDistMin = 3;
        betaDistMax = 15;
        betaTiltMin = 3;
        betaTiltMax = 15;
        alphaDistMin = 2.5;
        alphaDistMax = 3;
        alphaTiltMin = 15; % degrees
        alphaTiltMax = 30; % degrees
        opticalPartitioningMin = 1e-6;
        % rfSensor
        P_TX = 1e-3; % Transmit power (Watts)
        T_0 = 300; % Temperature (K)
        BW = 20e6; % Bandwidth (Hz)
        f_c = 3e9; % Center frequency (Hz)
        G_RX_dBi = 3; % Receiving Antenna Gain (dBi)
        beamwidthExponent = 16;
        lossExponent = 2;
        sinrPartitioningMin = 50;

        % Communications
        useFixedTopology = false;
        optimizeSensorPointing = false;
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
        function miSim_initialization(tc)
            % Test flag: set true to exercise the SINR-aware routing
            % overlay on this random scenario. Routing requires SINR comms, so
            % the flag switches the whole SINR stack on (agents keep their
            % default 0.1 W txPower); the permissive -30 dB threshold keeps
            % random geometries fully connected.
            useRoutingTopology = true;

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
                        candidatePos = tc.agents{randi(ii - 1)}.pos + sign(randn([1, 3])) .* (rand(1, 3) .* tc.commsRanges(ii)/sqrt(2));
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
                        if norm(tc.agents{jj}.pos - candidatePos) <= min(tc.commsRanges([ii, jj]))
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
                    candidateGeometry = spherical;
                    candidateGeometry = candidateGeometry.initialize(candidatePos, tc.collisionRanges(ii), REGION_TYPE.COLLISION);

                    % Initialize candidate agent sensor model
                    tc.sensor = tc.sensor.initialize(tc.alphaDistMin + rand * (tc.alphaDistMax - tc.alphaDistMin), tc.betaDistMin + rand * (tc.betaDistMax - tc.betaDistMin), tc.alphaTiltMin + rand * (tc.alphaTiltMax - tc.alphaTiltMin), tc.betaTiltMin + rand * (tc.betaTiltMax - tc.betaTiltMin));

                    % Initialize candidate agent
                    newAgent = tc.agents{ii}.initialize(candidatePos, candidateGeometry, tc.sensor, tc.commsRanges(ii), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize); 

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

                    % Make sure candidate clears domain floor
                    if newAgent.pos(3) - newAgent.collisionGeometry.radius <= tc.minAlt
                        violation = true;
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
            if useRoutingTopology
                tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing, true, -30.0, 2.0, 290, 2.4e9, 20e6, true);
            else
                tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing);
            end
        end
        function miSim_run_rf_sensor(tc)
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
                        candidatePos = tc.agents{randi(ii - 1)}.pos + sign(randn([1, 3])) .* (rand(1, 3) .* tc.commsRanges(ii)/sqrt(2));
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
                        if norm(tc.agents{jj}.pos - candidatePos) <= min(tc.commsRanges([ii, jj]))
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
                    tc.sensor = rfSensor;
                    tilt = 0; azimuth = 0;
                    tc.sensor = tc.sensor.initialize(tc.P_TX * 1 + rand * 4, tc.T_0, tc.BW, tc.f_c, tc.G_RX_dBi, tc.beamwidthExponent + randi(100), tilt, azimuth, tc.lossExponent);

                    % Initialize candidate agent
                    newAgent = tc.agents{ii}.initialize(candidatePos, candidateGeometry, tc.sensor, tc.commsRanges(ii), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
                    
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

                    % Make sure candidate clears domain floor
                    if newAgent.pos(3) - newAgent.collisionGeometry.radius <= tc.minAlt
                        violation = true;
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
            tc.optimizeSensorPointing = true;
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing);

            % Write out initialization state
            tc.testClass.writeInits();

            % Run simulation loop
            tc.testClass = tc.testClass.run();
        end
        function miSim_run(tc)
            % Test flag: set true to exercise the min-max-workload routing
            % overlay on this random scenario. Routing requires SINR comms, so
            % the flag switches the whole SINR stack on (agents keep their
            % default 0.1 W txPower); the permissive -30 dB threshold keeps
            % random geometries fully connected.
            useRoutingTopology = true;

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
                        candidatePos = tc.agents{randi(ii - 1)}.pos + sign(randn([1, 3])) .* (rand(1, 3) .* tc.commsRanges(ii)/sqrt(2));
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
                        if norm(tc.agents{jj}.pos - candidatePos) <= min(tc.commsRanges([ii, jj]))
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
                    newAgent = tc.agents{ii}.initialize(candidatePos, candidateGeometry, tc.sensor, tc.commsRanges(ii), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
                    
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

                    % Make sure candidate clears domain floor
                    if newAgent.pos(3) - newAgent.collisionGeometry.radius <= tc.minAlt
                        violation = true;
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
            if useRoutingTopology
                tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing, true, -30.0, 2.0, 290, 2.4e9, 20e6, true);
            else
                tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing);
            end

            % Write out initialization state
            tc.testClass.writeInits();

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
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center + [d, 0, 0], geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center - [d, 0, 0], geometry2, tc.sensor, tc.commsRanges(2), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
            tc.agents{3} = tc.agents{3}.initialize(tc.domain.center - [0, d, 0], geometry3, tc.sensor, tc.commsRanges(3), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);

            % Initialize the simulation
            tc.obstacles = cell(0, 1);
            tc.makePlots = false;
            tc.makeVideo = false;
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing);
        
            centerIdx = floor(size(tc.testClass.partitioning, 1) / 2);
            tc.verifyEqual(tc.testClass.partitioning(centerIdx, centerIdx:(centerIdx + 2)), [2, 3, 1]); % all three near center
            tc.verifyLessThan(sum(tc.testClass.partitioning == 1, "all"), sum(tc.testClass.partitioning == 0, "all")); % more non-assignments than partition 1 assignments
            tc.verifyLessThan(sum(tc.testClass.partitioning == 2, "all"), sum(tc.testClass.partitioning == 1, "all")); % more partition 1 assignments than partition 2 assignments
            tc.verifyLessThan(sum(tc.testClass.partitioning == 3, "all"), sum(tc.testClass.partitioning == 2, "all")); % more partition 3 assignments than partition 2 assignments
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
            tc.agents{1} = tc.agents{1}.initialize([tc.domain.center(1:2), 3], geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);

            % Initialize the simulation
            tc.obstacles = cell(0, 1);
            tc.makePlots = false;
            tc.makeVideo = false;
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing);
            close(tc.testClass.fPerf);

            tc.verifyEqual(unique(tc.testClass.partitioning), [0; 1]);
            tc.verifyLessThan(sum(tc.testClass.partitioning == 1, "all"), sum(tc.testClass.partitioning == 0, "all"));
        end
        function test_single_agent_gradient_ascent(tc)
            % make basic domain
            tc.minDimension = 10; % domain size
            tc.domain = tc.domain.initialize([zeros(1, 3);tc.minDimension* ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([7, 6]), tc.domain, tc.discretizationStep, tc.protectedRange, tc.opticalPartitioningMin, [7, 6]);
        
            % Initialize agent collision geometry
            tc.agents = {agent};
            geometry1 = spherical;
            geometry1 = geometry1.initialize([tc.domain.center(1:2)-tc.domain.dimensions(1)/4, 3], tc.collisionRanges(1), REGION_TYPE.COLLISION);
            
            % Initialize agent sensor model with fixed parameters
            tc.sensor = tc.sensor.initialize(tc.minDimension / 2, 3, 20, 3);

            % Initialize agents
            tc.maxIter = 75;
            tc.agents{1} = tc.agents{1}.initialize([tc.domain.center(1:2)-tc.domain.dimensions(1)/4, 3], geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);

            % Initialize the simulation
            tc.obstacles = cell(0, 1);
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing);
            
            % Run the simulation
            tc.testClass = tc.testClass.run();
        end
        function test_single_agent_gradient_ascent_tilted(tc)
            % make basic domain
            tc.minDimension = 10; % domain size
            tc.domain = tc.domain.initialize([zeros(1, 3);tc.minDimension* ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([7, 6]), tc.domain, tc.discretizationStep, tc.protectedRange, tc.opticalPartitioningMin, [7, 6]);
        
            % Initialize agent collision geometry
            tc.agents = {agent};
            geometry1 = spherical;
            geometry1 = geometry1.initialize([tc.domain.center(1:2)-tc.domain.dimensions(1)/4, 3], tc.collisionRanges(1), REGION_TYPE.COLLISION);
            
            % Initialize agent sensor model with fixed parameters
            tc.sensor = tc.sensor.initialize(tc.minDimension / 2, 3, 20, 3, 25, 155);

            % Initialize agents
            tc.maxIter = 75;
            tc.agents{1} = tc.agents{1}.initialize([tc.domain.center(1:2)-tc.domain.dimensions(1)/4, 3], geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);

            % Initialize the simulation
            tc.obstacles = cell(0, 1);
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing);
            
            % Run the simulation
            tc.testClass = tc.testClass.run();
        end
        function test_single_agent_gradient_ascent_tilted_RF_sensor(tc)
            % make basic domain
            tc.minDimension = 10; % domain size
            tc.domain = tc.domain.initialize([zeros(1, 3);tc.minDimension* ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([8, 3], reshape([15, 4; 4, 15], [1 2 2])), tc.domain, tc.discretizationStep, tc.protectedRange, tc.sinrPartitioningMin, [7, 6]);
        
            % Initialize agent collision geometry
            tc.agents = {agent};
            geometry1 = spherical;
            geometry1 = geometry1.initialize([tc.domain.center(1:2)-tc.domain.dimensions(1)/4, 3], tc.collisionRanges(1), REGION_TYPE.COLLISION);

            tc.sensor = rfSensor;
            tc.sensor = tc.sensor.initialize(tc.P_TX, tc.T_0, tc.BW, tc.f_c, tc.G_RX_dBi, tc.beamwidthExponent, 45, 45, tc.lossExponent);

            % Initialize agents
            tc.maxIter = 75;
            tc.agents{1} = tc.agents{1}.initialize([tc.domain.center(1:2)-tc.domain.dimensions(1)/4, 3], geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);

            % Initialize the simulation
            tc.obstacles = cell(0, 1);
            tc.minAlt = 0.5;
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing);
            
            % Run the simulation
            tc.testClass = tc.testClass.run();
        end
        function test_single_agent_gradient_ascent_sensor_pointing(tc)
            % make basic domain
            tc.minDimension = 10; % domain size
            tc.domain = tc.domain.initialize([zeros(1, 3);tc.minDimension* ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([7, 6]), tc.domain, tc.discretizationStep, tc.protectedRange, tc.opticalPartitioningMin, [7, 6]);
        
            % Initialize agent collision geometry
            tc.agents = {agent};
            geometry1 = spherical;
            geometry1 = geometry1.initialize([tc.domain.center(1:2)-tc.domain.dimensions(1)/4, 3], tc.collisionRanges(1), REGION_TYPE.COLLISION);
            
            % Initialize agent sensor model with fixed parameters
            tc.sensor = tc.sensor.initialize(tc.minDimension / 2, 3, 20, 3);

            % Initialize agents
            tc.maxIter = 75;
            tc.agents{1} = tc.agents{1}.initialize([tc.domain.center(1:2)-tc.domain.dimensions(1)/4, 3], geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);

            % Initialize the simulation
            tc.optimizeSensorPointing = true;
            tc.obstacles = cell(0, 1);
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing);

            % Run the simulation
            tc.testClass = tc.testClass.run();
        end
        function test_single_agent_gradient_ascent_rf_sensor_pointing(tc)
            % make basic domain
            tc.minDimension = 10; % domain size
            tc.domain = tc.domain.initialize([zeros(1, 3);tc.minDimension* ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([7, 6]), tc.domain, tc.discretizationStep, tc.protectedRange, tc.sinrPartitioningMin, [7, 6]);
        
            % Initialize agent collision geometry
            tc.agents = {agent};
            geometry1 = spherical;
            geometry1 = geometry1.initialize([tc.domain.center(1:2)-tc.domain.dimensions(1)/4, 3], tc.collisionRanges(1), REGION_TYPE.COLLISION);
            
            % Initialize agent sensor model

            tc.sensor = rfSensor;
            tc.sensor = tc.sensor.initialize(tc.P_TX, tc.T_0, tc.BW, tc.f_c, tc.G_RX_dBi, tc.beamwidthExponent, 0, 0, tc.lossExponent);

            % Initialize agents
            tc.maxIter = 75;
            tc.agents{1} = tc.agents{1}.initialize([tc.domain.center(1:2)-tc.domain.dimensions(1)/4, 3], geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);

            % Initialize the simulation
            tc.optimizeSensorPointing = true;
            tc.obstacles = cell(0, 1);
            tc.minAlt = 0.5;
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing);
            
            % Run the simulation
            tc.testClass = tc.testClass.run();
        end
        function test_collision_avoidance(tc)
            % No obstacles
            % Fixed agent initial conditions
            % Exaggerated large collision geometries to test CA
            % make basic domain
            tc.minDimension = 10; % domain size
            tc.domain = tc.domain.initialize([zeros(1, 3);tc.minDimension* ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([3, 7]), tc.domain, tc.discretizationStep, tc.protectedRange, tc.opticalPartitioningMin, [3, 7]);
        
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
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center + d, geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center - d, geometry2, tc.sensor, tc.commsRanges(2), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);

            % Initialize the simulation
            tc.obstacles = cell(0, 1);
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing);
            
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
            tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([8, 5.2195]), tc.domain, tc.discretizationStep, tc.protectedRange, tc.opticalPartitioningMin, [8, 5.2195]);
        
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
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center - d + [0, tc.collisionRanges(1) * 1.1 - yOffset, 0], geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center - d - [0, tc.collisionRanges(2)  *1.1 + yOffset, 0], geometry2, tc.sensor, tc.commsRanges(2), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
            
            % Initialize the simulation
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing);
            
            % Run the simulation
            tc.testClass.run();
        end
        function test_communications_constraint_SINR_threshold(tc)
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
            tc.commsRanges = 4 * ones(size(tc.agents)); % unused in SINR mode; kept for parity with the fixed-radius variant
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center + d, geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center - d, geometry2, tc.sensor, tc.commsRanges(2), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);

            % SINR communications model parameters. With isotropic links and only
            % two agents (no interference) the link is noise-limited:
            %   SINR = Ptx / ((4*pi*fc/c)^2 * d^n * kB*T*B)
            % Pick the threshold so the SINR connectivity boundary sits at an
            % effective range of ~4 m, mirroring the commsRange used by the
            % fixed-radius variant of this test (so the two agents still cannot
            % reach their objectives without breaking connectivity).
            useSinrComms     = true;
            txPower          = [0.01, 0.01];     % transmit power per agent (W)
            pathLossExponent = 2.0;
            ambientTemp      = 290;     % K
            centerFreq       = 2.4e9;   % Hz
            bandwidth        = 20e6;    % Hz

            effRange         = 4.0;     % desired effective comms range (m)
            K_pl   = (4 * pi * centerFreq / 3e8)^2;            % free-space path-loss reference
            noiseW = 1.380649e-23 * ambientTemp * bandwidth;   % thermal noise (W)
            sinrThreshold = 10 * log10(mean(txPower) / (K_pl * effRange^pathLossExponent * noiseW)); % dB

            tc.agents{1}.txPower = txPower(1);
            tc.agents{2}.txPower = txPower(2);

            % Initialize the simulation in SINR comms mode
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing, useSinrComms, sinrThreshold, pathLossExponent, ambientTemp, centerFreq, bandwidth);

            % Run the simulation
            tc.testClass = tc.testClass.run();
        end
        function test_communications_constraint_SINR_threshold_3_agents(tc)
            % No obstacles
            % Fixed three agents initial conditions
            % Negligible collision geometries
            % Non-standard domain with two objectives that will try to pull the
            % agents apart
            tc.minDimension = 10; % domain size
            tc.domain = tc.domain.initialize([zeros(1, 3);tc.minDimension* ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % make basic sensing objective
            tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([2, 8; 8, 8]), tc.domain, tc.discretizationStep, tc.protectedRange);

            % Initialize agent collision geometry
            tc.agents = {agent; agent; agent;};
            tc.collisionRanges = .25 * ones(size(tc.agents));
            d = [2.0, 0, 0];  % 2 m collinear spacing (above the 1 m path-loss floor)
            geometry1 = spherical;
            geometry2 = geometry1;
            geometry3 = geometry1;
            geometry1 = geometry1.initialize(tc.domain.center - d, tc.collisionRanges(1), REGION_TYPE.COLLISION);
            geometry2 = geometry2.initialize(tc.domain.center, tc.collisionRanges(2), REGION_TYPE.COLLISION);
            geometry3 = geometry3.initialize(tc.domain.center + d, tc.collisionRanges(3), REGION_TYPE.COLLISION);

            % Initialize agent sensor model
            tc.sensor = sigmoidSensor;
            tc.sensor = tc.sensor.initialize(tc.minDimension / 2, 3, 15, 3);

            % Initialize obstacles
            tc.obstacles = {};

            % Initialize agents
            tc.maxIter = 50;
            tc.commsRanges = 4 * ones(size(tc.agents)); % unused in SINR mode; kept for parity with the fixed-radius variant
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center - d, geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center, geometry2, tc.sensor, tc.commsRanges(2), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
            tc.agents{3} = tc.agents{3}.initialize(tc.domain.center + d, geometry3, tc.sensor, tc.commsRanges(3), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);

            % SINR communications model parameters. Three collinear, equally
            % spaced agents with homogeneous transmit power form an
            % INTERFERENCE-limited network (co-agent interference swamps thermal
            % noise), unlike the two-agent noise-limited case. Two structural
            % facts set the connectivity boundaries:
            %   * A chain link (1-2 or 2-3) is limited by the MIDDLE agent, which
            %     receives one end while the other end interferes at the SAME
            %     range: signal = interference  ->  0 dB.
            %   * The end-to-end (1,3) link has its transmitter at 2x the range
            %     while the middle agent interferes at 1x:  (1/2)^n  ->  -6.02 dB.
            % Homogeneous power is required for the symmetry: unequal power would
            % push one chain link's middle-receiver SINR below 0 dB and break it.
            % Placing the threshold between the two boundaries (a couple dB above
            % the -6 dB direct link, a few dB below the 0 dB chain links) makes
            % exactly the two adjacent links {1-2, 2-3} feasible and rejects 1-3,
            % giving the desired 1 -> 2 -> 3 line. The +3 dB CBF padding then
            % still lands below the 0 dB chain binding, so the QP starts feasible.
            useSinrComms     = true;
            txPower          = [0.1, 0.1, 0.1];   % homogeneous transmit power (W)
            pathLossExponent = 2.0;
            ambientTemp      = 290;     % K
            centerFreq       = 2.4e9;   % Hz
            bandwidth        = 20e6;    % Hz

            endBindingDb  = 10 * log10(0.5^pathLossExponent);  % (1,3) link SINR: (1/2)^n = -6.02 dB
            sinrThreshold = endBindingDb + 2.0;                % ~ -4.02 dB: 2 dB above the rejected
                                                               % 1-3 link, ~4 dB below the 0 dB chain

            tc.agents{1}.txPower = txPower(1);
            tc.agents{2}.txPower = txPower(2);
            tc.agents{3}.txPower = txPower(3);

            % Initialize the simulation in SINR comms mode
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing, useSinrComms, sinrThreshold, pathLossExponent, ambientTemp, centerFreq, bandwidth);

            % The initial SINR topology must be exactly the 1 -> 2 -> 3 line:
            % two links (1-2 and 2-3), with the direct 1-3 link rejected.
            tc.assertEqual(tc.testClass.constraintAdjacencyMatrix, logical( ...
                [ 1, 1, 0; ...
                  1, 1, 1; ...
                  0, 1, 1;]));

            % Run the simulation
            tc.testClass = tc.testClass.run();
        end
        function test_routing_topology_convergence_demo(tc)
            % Demonstration of the Capacity-Aware Lesser Sink Neighbor
            % (CALSN) topology + routing algorithm: four agents with
            % sigmoid sensors in a cube domain converge on a single
            % sensing objective while the communication links evolve.
            %
            % Agents start scattered in a quadrilateral near one corner;
            % the objective sits in the opposite corner. Base stations are
            % agents 1 and 4. As the fleet migrates and contracts around
            % the objective, pairwise SINRs — and with them the Shannon
            % capacities, base-station potentials, parent choices, and
            % capacity-proportional flow splits — change every step, so
            % the network-graph tile shows the maintained (dotted) and
            % flow-carrying (solid, arrowed, flow-labelled) links
            % re-routing dynamically. validate() enforces the connected-
            % topology invariant on every step of the run.
            tc.minDimension = 10; % cube domain edge length
            tc.domain = tc.domain.initialize([zeros(1, 3);tc.minDimension* ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % Single sensing objective in the far corner
            tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([7, 7]), tc.domain, tc.discretizationStep, tc.protectedRange);

            % Four agents scattered (non-collinear) near the (low-x, low-y)
            % corner, co-altitude at z = 2. Base 1 trails the fleet and
            % base 4 leads it, with the relays staggered between: relay 3
            % starts too far from base 1 for a direct link to be worth
            % using, so its initial basin-1 route is MULTI-HOP via relay 2
            % and re-parents to the base directly as the fleet contracts.
            tc.agents = {agent; agent; agent; agent;};
            tc.collisionRanges = .25 * ones(size(tc.agents));
            positions = [1.0, 2.0, 2; ...
                         3.0, 3.5, 2; ...
                         5.5, 3.0, 2; ...
                         5.0, 5.5, 2;];

            % Initialize agent sensor model (sigmoid)
            tc.sensor = sigmoidSensor;
            tc.sensor = tc.sensor.initialize(tc.minDimension / 2, 3, 15, 3);

            % Initialize obstacles
            tc.obstacles = {};

            % Initialize agents: 50 steps is long enough to watch the
            % fleet converge on the objective and settle (the gradient
            % step decays linearly to zero over maxIter)
            tc.maxIter = 80;
            tc.commsRanges = 4 * ones(size(tc.agents)); % unused in SINR mode
            for aa = 1:4
                geometry = spherical;
                geometry = geometry.initialize(positions(aa, :), tc.collisionRanges(aa), REGION_TYPE.COLLISION);
                tc.agents{aa} = tc.agents{aa}.initialize(positions(aa, :), geometry, tc.sensor, tc.commsRanges(aa), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
                tc.agents{aa}.txPower = 0.1;
            end

            % SINR communications model. The threshold must sit >= 3 dB
            % below the worst link CALSN may select, because the CBF holds
            % every maintained link at threshold + 3 dB in BOTH directions:
            % a link selected inside that 3 dB band is born with a violated
            % barrier and can deadlock the QP (agents hold position, the
            % geometry never changes, and the same link is re-selected
            % forever). The staggered start's weakest candidate link (the
            % long 1-4 base-to-base diagonal) sits at ~ -9 dB and the
            % weakest link CALSN actually selects at ~ -4 dB, so -14 dB
            % leaves the -11 dB CBF floor a comfortable margin.
            useSinrComms       = true;
            pathLossExponent   = 2.0;
            ambientTemp        = 290;     % K
            centerFreq         = 2.4e9;   % Hz
            bandwidth          = 20e6;    % Hz
            sinrThreshold      = -14.0;   % dB
            useRoutingTopology = true;

            % Initialize the simulation with the CALSN topology selector
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing, useSinrComms, sinrThreshold, pathLossExponent, ambientTemp, centerFreq, bandwidth, useRoutingTopology);

            % Initial routing invariants: bases at zero potential, relays
            % finite; maintained topology one connected component; both
            % generated units delivered to the bases with nothing unrouted
            phi = tc.testClass.routingPotentials;
            tc.assertEqual(phi([1, 4]), [0; 0]);
            tc.assertTrue(all(isfinite(phi([2, 3]))) && all(phi([2, 3]) > 0));
            tc.assertEqual(max(conncomp(graph(tc.testClass.constraintAdjacencyMatrix))), 1);
            F = tc.testClass.routingFlows;
            for aa = [2, 3]
                tc.assertEqual(sum(F(aa, :)) - sum(F(:, aa)), 1.0, "AbsTol", 1e-9);
            end
            bases = [1, 4];
            tc.assertEqual(sum(F(:, bases), "all") - sum(F(bases, :), "all"), 2.0, "AbsTol", 1e-9);
            tc.assertTrue(all(tc.testClass.routingUnrouted == 0));

            % Run the simulation (per-step CALSN re-solve; validate()
            % checks feasibility- and maintained-graph connectivity at
            % every iteration)
            tc.testClass = tc.testClass.run();

            % Convergence: total sensing performance improved over the run
            tc.assertGreaterThan(tc.testClass.performance(end), tc.testClass.performance(2));

            % Dynamic evolution: the maintained topology changed at least
            % once relative to the initial configuration
            hist = tc.testClass.constraintAdjacencyHist;
            changed = false;
            for kk = 2:size(hist, 3)
                if any(hist(:, :, kk) ~= hist(:, :, 1), "all")
                    changed = true;
                    break;
                end
            end
            tc.assertTrue(changed, "Maintained topology never changed during the run");

            % Final routing state is still a valid delivery tree: one
            % connected component, conservation at the generating UAVs,
            % both units delivered, nothing unrouted
            tc.assertEqual(max(conncomp(graph(tc.testClass.constraintAdjacencyMatrix))), 1);
            F = tc.testClass.routingFlows;
            for aa = [2, 3]
                tc.assertEqual(sum(F(aa, :)) - sum(F(:, aa)), 1.0, "AbsTol", 1e-9);
            end
            tc.assertEqual(sum(F(:, bases), "all") - sum(F(bases, :), "all"), 2.0, "AbsTol", 1e-9);
            tc.assertTrue(all(tc.testClass.routingUnrouted == 0));
        end
        function test_routing_topology_convergence_demo_rfSensor(tc)
            % Demonstration of the Capacity-Aware Lesser Sink Neighbor
            % (CALSN) topology + routing algorithm: four agents with
            % sigmoid sensors in a cube domain converge on a single
            % sensing objective while the communication links evolve.
            %
            % Agents start scattered in a quadrilateral near one corner;
            % the objective sits in the opposite corner. Base stations are
            % agents 1 and 4. As the fleet migrates and contracts around
            % the objective, pairwise SINRs — and with them the Shannon
            % capacities, base-station potentials, parent choices, and
            % capacity-proportional flow splits — change every step, so
            % the network-graph tile shows the maintained (dotted) and
            % flow-carrying (solid, arrowed, flow-labelled) links
            % re-routing dynamically. validate() enforces the connected-
            % topology invariant on every step of the run.
            tc.minDimension = 10; % cube domain edge length
            tc.domain = tc.domain.initialize([zeros(1, 3);tc.minDimension* ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");

            % Single sensing objective in the far corner
            tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([7, 7]), tc.domain, tc.discretizationStep, tc.protectedRange);

            % Four agents scattered (non-collinear) near the (low-x, low-y)
            % corner, co-altitude at z = 2. Base 1 trails the fleet and
            % base 4 leads it, with the relays staggered between: relay 3
            % starts too far from base 1 for a direct link to be worth
            % using, so its initial basin-1 route is MULTI-HOP via relay 2
            % and re-parents to the base directly as the fleet contracts.
            tc.agents = {agent; agent; agent; agent;};
            tc.collisionRanges = .25 * ones(size(tc.agents));
            positions = [1.0, 2.0, 2; ...
                         3.0, 3.5, 2; ...
                         5.5, 3.0, 2; ...
                         5.0, 5.5, 2;];

            % Initialize agent sensor model (sigmoid)
            tc.sensor = rfSensor;
            tilt = 0; azimuth = 0;
            tc.sensor = tc.sensor.initialize(tc.P_TX * 1 + rand * 4, tc.T_0, tc.BW, tc.f_c, tc.G_RX_dBi, tc.beamwidthExponent + randi(100), tilt, azimuth, tc.lossExponent);

            % Initialize obstacles
            tc.obstacles = {};

            % Initialize agents: 50 steps is long enough to watch the
            % fleet converge on the objective and settle (the gradient
            % step decays linearly to zero over maxIter)
            tc.maxIter = 80;
            tc.commsRanges = 4 * ones(size(tc.agents)); % unused in SINR mode
            for aa = 1:4
                geometry = spherical;
                geometry = geometry.initialize(positions(aa, :), tc.collisionRanges(aa), REGION_TYPE.COLLISION);
                tc.agents{aa} = tc.agents{aa}.initialize(positions(aa, :), geometry, tc.sensor, tc.commsRanges(aa), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
                tc.agents{aa}.txPower = 0.1;
            end

            % SINR communications model. The threshold must sit >= 3 dB
            % below the worst link CALSN may select, because the CBF holds
            % every maintained link at threshold + 3 dB in BOTH directions:
            % a link selected inside that 3 dB band is born with a violated
            % barrier and can deadlock the QP (agents hold position, the
            % geometry never changes, and the same link is re-selected
            % forever). The staggered start's weakest candidate link (the
            % long 1-4 base-to-base diagonal) sits at ~ -9 dB and the
            % weakest link CALSN actually selects at ~ -4 dB, so -14 dB
            % leaves the -11 dB CBF floor a comfortable margin.
            useSinrComms       = true;
            pathLossExponent   = 2.0;
            ambientTemp        = 290;     % K
            centerFreq         = 2.4e9;   % Hz
            bandwidth          = 20e6;    % Hz
            sinrThreshold      = -14.0;   % dB
            useRoutingTopology = true;

            % Initialize the simulation with the CALSN topology selector
            tc.optimizeSensorPointing = true;
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing, useSinrComms, sinrThreshold, pathLossExponent, ambientTemp, centerFreq, bandwidth, useRoutingTopology);

            % Initial routing invariants: bases at zero potential, relays
            % finite; maintained topology one connected component; both
            % generated units delivered to the bases with nothing unrouted
            phi = tc.testClass.routingPotentials;
            tc.assertEqual(phi([1, 4]), [0; 0]);
            tc.assertTrue(all(isfinite(phi([2, 3]))) && all(phi([2, 3]) > 0));
            tc.assertEqual(max(conncomp(graph(tc.testClass.constraintAdjacencyMatrix))), 1);
            F = tc.testClass.routingFlows;
            for aa = [2, 3]
                tc.assertEqual(sum(F(aa, :)) - sum(F(:, aa)), 1.0, "AbsTol", 1e-9);
            end
            bases = [1, 4];
            tc.assertEqual(sum(F(:, bases), "all") - sum(F(bases, :), "all"), 2.0, "AbsTol", 1e-9);
            tc.assertTrue(all(tc.testClass.routingUnrouted == 0));

            % Run the simulation (per-step CALSN re-solve; validate()
            % checks feasibility- and maintained-graph connectivity at
            % every iteration)
            tc.testClass = tc.testClass.run();

            % Convergence: total sensing performance improved over the run
            tc.assertGreaterThan(tc.testClass.performance(end), tc.testClass.performance(2));

            % Dynamic evolution: the maintained topology changed at least
            % once relative to the initial configuration
            hist = tc.testClass.constraintAdjacencyHist;
            changed = false;
            for kk = 2:size(hist, 3)
                if any(hist(:, :, kk) ~= hist(:, :, 1), "all")
                    changed = true;
                    break;
                end
            end
            tc.assertTrue(changed, "Maintained topology never changed during the run");

            % Final routing state is still a valid delivery tree: one
            % connected component, conservation at the generating UAVs,
            % both units delivered, nothing unrouted
            tc.assertEqual(max(conncomp(graph(tc.testClass.constraintAdjacencyMatrix))), 1);
            F = tc.testClass.routingFlows;
            for aa = [2, 3]
                tc.assertEqual(sum(F(aa, :)) - sum(F(:, aa)), 1.0, "AbsTol", 1e-9);
            end
            tc.assertEqual(sum(F(:, bases), "all") - sum(F(bases, :), "all"), 2.0, "AbsTol", 1e-9);
            tc.assertTrue(all(tc.testClass.routingUnrouted == 0));
        end
        function test_communications_constraint_fixed_radius(tc)
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
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center + d, geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center - d, geometry2, tc.sensor, tc.commsRanges(2), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);

            % Initialize the simulation
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing);

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
            tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([8, 5]), tc.domain, tc.discretizationStep, tc.protectedRange, tc.opticalPartitioningMin, [8, 5]);
        
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
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center - [d, 0, 0], geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center - [0, d, 0], geometry2, tc.sensor, tc.commsRanges(2), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
            
            % Initialize obstacles
            obstacleLength = 1.5;
            tc.obstacles{1} = rectangularPrism;
            tc.obstacles{1} = tc.obstacles{1}.initialize([tc.domain.center(1:2) - obstacleLength, 0; tc.domain.center(1:2) + obstacleLength, tc.domain.maxCorner(3)], REGION_TYPE.OBSTACLE, "Obstacle 1");

            % Initialize the simulation
            tc.minAlt = 0;
            tc.makePlots = false;
            tc.makeVideo = false;
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing);

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
            tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([8, 5]), tc.domain, tc.discretizationStep, tc.protectedRange, tc.opticalPartitioningMin, [8, 5]);
        
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
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center + [d, 0, 0], geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center, geometry2, tc.sensor, tc.commsRanges(2), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
            tc.agents{3} = tc.agents{3}.initialize(tc.domain.center + [-d, d, 0], geometry3, tc.sensor, tc.commsRanges(3), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
            tc.agents{4} = tc.agents{4}.initialize(tc.domain.center + [-2*d, d, 0], geometry4, tc.sensor, tc.commsRanges(4), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
            tc.agents{5} = tc.agents{5}.initialize(tc.domain.center + [0, d, 0], geometry5, tc.sensor, tc.commsRanges(5), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);

            % Initialize the simulation
            tc.minAlt = 0;
            tc.makePlots = false;
            tc.makeVideo = false;
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing);

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
            tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([8, 5]), tc.domain, tc.discretizationStep, tc.protectedRange, tc.opticalPartitioningMin, [8, 5]);
        
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
            tc.agents{1} = tc.agents{1}.initialize(tc.domain.center + [-0.9 * d/sqrt(2), 0.9 * d/sqrt(2), 0], geometry1, tc.sensor, tc.commsRanges(1), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
            tc.agents{2} = tc.agents{2}.initialize(tc.domain.center + [-0.5 * d, 0.25 * d, 0], geometry2, tc.sensor, tc.commsRanges(2), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
            tc.agents{3} = tc.agents{3}.initialize(tc.domain.center + [0.9 * d, 0, 0], geometry3, tc.sensor, tc.commsRanges(3), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
            tc.agents{4} = tc.agents{4}.initialize(tc.domain.center + [0.9 * d/sqrt(2), -0.9 * d/sqrt(2), 0], geometry4, tc.sensor, tc.commsRanges(4), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
            tc.agents{5} = tc.agents{5}.initialize(tc.domain.center + [0, 0.9 * d, 0], geometry5, tc.sensor, tc.commsRanges(5), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
            tc.agents{6} = tc.agents{6}.initialize(tc.domain.center, geometry6, tc.sensor, tc.commsRanges(6), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);
            tc.agents{7} = tc.agents{7}.initialize(tc.domain.center + [d/2, d/2, 0], geometry7, tc.sensor, tc.commsRanges(7), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);

            % Initialize the simulation
            tc.minAlt = 0;
            tc.makePlots = false;
            tc.makeVideo = false;
            tc.testClass = tc.testClass.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, tc.makePlots, tc.makeVideo, tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing);

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
        function miSim_initializeFromInits(tc)
            % Build a minimal valid simulation, write it to a matfile, reload
            % via initializeFromInits, assert round-trip consistency, then
            % delete the file.  No plotting or video at any stage.

            % Obstacles
            nGeom = tc.minNumObstacles + randi(tc.maxNumObstacles - tc.minNumObstacles);
            tc.obstacles = cell(nGeom, 1);
            for ii = 1:nGeom
                badCandidate = true;
                while badCandidate
                    tc.obstacles{ii} = rectangularPrism;
                    tc.obstacles{ii} = tc.obstacles{ii}.initializeRandom(REGION_TYPE.OBSTACLE, ...
                        sprintf("Obstacle %d", ii), tc.minObstacleSize, tc.maxObstacleSize, ...
                        tc.domain, tc.minAlt);
                    if ~tc.obstacleCollisionCheck(tc.obstacles(1:(ii - 1)), tc.obstacles{ii})
                        badCandidate = false;
                    end
                end
            end

            % Agents
            nAgents = tc.minAgents;
            tc.agents = cell(nAgents, 1);
            tc.collisionRanges = tc.minCollisionRange + rand(nAgents, 1) * (tc.maxCollisionRange - tc.minCollisionRange);
            tc.commsRanges = tc.minCommsRange + rand(nAgents, 1) * (tc.maxCommsRange - tc.minCommsRange);

            for ii = 1:nAgents
                initInvalid = true;
                while initInvalid
                    if ii == 1
                        candidatePos = tc.domain.random();
                        candidatePos(3) = tc.minAlt + rand * 3;
                        while agentsCrowdObjective(tc.domain.objective, candidatePos, mean(tc.domain.dimensions) / 2)
                            candidatePos = tc.domain.random();
                            candidatePos(3) = tc.minAlt + rand * 3;
                        end
                    else
                        candidatePos = tc.agents{randi(ii - 1)}.pos + sign(randn([1, 3])) .* (rand(1, 3) .* tc.commsRanges(ii) / sqrt(2));
                        candidatePos(3) = tc.minAlt + rand * 3;
                    end

                    if ~tc.domain.contains(candidatePos), continue; end
                    if agentsCrowdObjective(tc.domain.objective, candidatePos, mean(tc.domain.dimensions) / 2), continue; end

                    % Connectivity check
                    connections = false(1, ii - 1);
                    for jj = 1:(ii - 1)
                        if norm(tc.agents{jj}.pos - candidatePos) <= min(tc.commsRanges([ii, jj]))
                            connections(jj) = true;
                            for kk = 1:size(tc.obstacles, 1)
                                if tc.obstacles{kk}.containsLine(tc.agents{jj}.pos, candidatePos)
                                    connections(jj) = false;
                                end
                            end
                        end
                    end
                    if ii ~= 1 && ~any(connections), continue; end

                    geom = spherical;
                    geom = geom.initialize(candidatePos, tc.collisionRanges(ii), REGION_TYPE.COLLISION);
                    tc.sensor = sigmoidSensor;
                    tc.sensor = tc.sensor.initialize( ...
                        tc.alphaDistMin + rand * (tc.alphaDistMax - tc.alphaDistMin), ...
                        tc.betaDistMin  + rand * (tc.betaDistMax  - tc.betaDistMin), ...
                        tc.alphaTiltMin + rand * (tc.alphaTiltMax - tc.alphaTiltMin), ...
                        tc.betaTiltMin  + rand * (tc.betaTiltMax  - tc.betaTiltMin));
                    newAgent = agent;
                    newAgent = newAgent.initialize(candidatePos, geom, tc.sensor, tc.commsRanges(ii), tc.maxIter, tc.initialStepSize, tc.initialMaxAngleStepSize);

                    % Domain / obstacle / agent collision checks
                    violation = false;
                    for jj = 1:size(newAgent.collisionGeometry.vertices, 1)
                        if ~tc.domain.contains(newAgent.collisionGeometry.vertices(jj, 1:3))
                            violation = true; break;
                        end
                    end
                    if violation, continue; end
                    for kk = 1:size(tc.obstacles, 1)
                        if geometryIntersects(tc.obstacles{kk}, newAgent.collisionGeometry)
                            violation = true; break;
                        end
                    end
                    if violation, continue; end
                    for kk = 1:(ii - 1)
                        if geometryIntersects(tc.agents{kk}.collisionGeometry, newAgent.collisionGeometry)
                            violation = true; break;
                        end
                    end
                    if newAgent.pos(3) - newAgent.collisionGeometry.radius <= tc.minAlt
                        violation = true;
                    end
                    if violation, continue; end

                    initInvalid = false;
                    tc.agents{ii} = newAgent;
                end
            end

            % Initialize first sim (no plots / video)
            sim1 = miSim;
            sim1 = sim1.initialize(tc.domain, tc.agents, tc.barrierGain, tc.barrierExponent, ...
                tc.minAlt, tc.timestep, tc.maxIter, tc.obstacles, false, false, ...
                tc.useDoubleIntegrator, tc.dampingCoeff, tc.useFixedTopology, tc.optimizeSensorPointing);

            % Write inits and build file path
            sim1.writeInits();
            initsFile = fullfile(matlab.project.rootProject().RootFolder, "sandbox", ...
                strcat(sim1.artifactName, "_miSimInits.mat"));

            % Load via initializeFromInits
            sim2 = miSim;
            sim2 = sim2.initializeFromInits(initsFile);

            % Assertions
            tc.assertEqual(size(sim2.agents, 1), size(sim1.agents, 1));
            tc.assertEqual(sim2.maxIter, sim1.maxIter);
            tc.assertEqual(sim2.barrierGain, sim1.barrierGain);

            % Cleanup
            delete(initsFile);
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
