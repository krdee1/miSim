classdef results < matlab.unittest.TestCase
    properties (Constant, Access = private)
        seed = 1;
        domainSize = [150, 150, 100]; % fixed domain size [X, Y, Z]
    end

    properties (Access = private)
        % System under test
        testClass = miSim;

        %% Diagnostic Parameters
        % No effect on simulation dynamics
        makeVideo = false; % disable video writing for big performance increase
        makePlots = true; % disable plotting for big performance increase (also disables video)
        plotCommsGeometry = false; % disable plotting communications geometries

        %% Scenario Reinitialization
        % Number of extra reinitializations per test case (3 n-values x 4 configs = 12).
        % Order: n3/A_1_alpha, n3/A_1_beta, n3/A_2_alpha, n3/B_1_beta,
        %        n5/A_1_alpha, ..., n6/B_1_beta
        % Set inspectScenarios = true to pause after init for manual review.
        % At the keyboard prompt, type the number of reinits needed into
        % the variable 'reinitCount', then 'dbcont' to continue.
        inspectScenarios = false;
        reinit = zeros(1, 12);

        %% Fixed Test Parameters
        useFixedTopology = true; % No lesser neighbor, fixed network instead
        discretizationStep = 0.5;
        protectedRange = 5;
        collisionRadius = 5;
        sensorPerformanceMinimum = 0.005;
        comRange = 20;
        maxIter = 400;
        initialStepSize = 1;
        % Each row: [minX minY minZ maxX maxY maxZ]
        obstacleCorners = [results.domainSize(1)/2, results.domainSize(2)*5/8, 0, results.domainSize(1)*5/8, results.domainSize(2), 35;
                           results.domainSize(1)/3, 0, 0, results.domainSize(1)/2, results.domainSize(2)*3/8, 40];
        barrierGain = 1;
        barrierExponent = 1;
        timestep = 0.5;
        dampingCoeff = 2;
    end

    properties (TestParameter)
        %% Test Iterations
        % Specific parameter combos to run iterations on
        n = struct('n3', 3, 'n5', 5, 'n6', 6); % number of agents
        config = results.makeConfigs();
    end

    methods (TestClassSetup)
        function setSeed(tc)
            rng(tc.seed);
        end
    end

    methods (Static, Access = public)
        function c = makeConfigs()
            rng(results.seed);
            abMin = 6; % alpha*beta >= 6 ensures membership(0) = tanh(3) >= 0.995
            alphaDist = rand(1, 2) .* [75, 40];
            betaDist = abMin ./ alphaDist + rand(1, 2) .* (20 - abMin ./ alphaDist);
            alphaTilt = 10 + rand(1, 2) .* [20, 20];
            betaTilt = abMin ./ alphaTilt + rand(1, 2) .* (50 - abMin ./ alphaTilt);
            sensors = struct('alphaDist', num2cell(alphaDist), 'alphaTilt', num2cell(alphaTilt), 'betaDist', num2cell(betaDist), 'betaTilt', num2cell(betaTilt));
            % sensor1 = sigmoidSensor;
            % sensor2 = sigmoidSensor;
            % sensor1 = sensor1.initialize(sensors(1).alphaDist, sensors(1).betaDist, sensors(1).alphaTilt, sensors(1).betaTilt);
            % sensor2 = sensor2.initialize(sensors(2).alphaDist, sensors(2).betaDist, sensors(2).alphaTilt, sensors(2).betaTilt);
            % sensor1.plotParameters;
            % sensor2.plotParameters;
            c = struct('A_1_alpha', struct('objectivePos', [3, 1] / 4 .* results.domainSize(1:2), 'sensor', sensors(1), 'doubleIntegrator', false), ...
                        'A_1_beta',  struct('objectivePos', [3, 1] / 4 .* results.domainSize(1:2), 'sensor', sensors(1), 'doubleIntegrator', true), ...
                        'A_2_alpha', struct('objectivePos', [3, 1] / 4 .* results.domainSize(1:2), 'sensor', sensors(2), 'doubleIntegrator', false), ...
                        'B_1_beta',  struct('objectivePos', [[3, 1] / 4 .* results.domainSize(1:2); [3, 1] / 4 .* results.domainSize(1:2) + 12.5 .* [-1, 1] ./ sqrt(2)], 'sensor', sensors(1), 'doubleIntegrator', true));
        end
    end

    methods (Test)
        function plot1_runs(tc, n, config)
            % if n == 5 && config.doubleIntegrator == true
            %     tc.makePlots = true;
            % else
            %     tc.makePlots = false;
            % end
            % Compute test case index for reinit lookup
            nKeys = fieldnames(tc.n);
            configKeys = fieldnames(tc.config);
            nIdx = find(cellfun(@(k) tc.n.(k) == n, nKeys));
            configIdx = find(cellfun(@(k) isequal(tc.config.(k), config), configKeys));
            testIdx = (nIdx - 1) * numel(configKeys) + configIdx;

            % Determine number of reinitializations
            reinitCount = tc.reinit(testIdx);

            for reroll = 0:reinitCount

            % Set up fixed-size domain
            minAlt = tc.domainSize(3)/10 + rand * 1/10 * tc.domainSize(3);
            % Place sensing objective(s) at fixed positions from config
            objectiveMu = config.objectivePos;
            numDist = size(objectiveMu, 1);
            objectiveSigma = [];
            for ii = 1:numDist
                sig = [200, 140; 140, 280];
                if ~mod(ii, 2)
                    sig = rot90(sig, 2);
                end
                sig = reshape(sig, [1, 2, 2]);
                objectiveSigma = cat(1, objectiveSigma, sig);
            end
            tc.testClass.domain = tc.testClass.domain.initialize([zeros(1, 3); tc.domainSize], REGION_TYPE.DOMAIN, "Domain");
            tc.testClass.domain.objective = tc.testClass.domain.objective.initialize(objectiveFunctionWrapper(objectiveMu, objectiveSigma), tc.testClass.domain, tc.discretizationStep, tc.protectedRange, tc.sensorPerformanceMinimum, objectiveMu, objectiveSigma);
            
            % Initialize agents
            agents = cell(n, 1);
            [agents{:}] = deal(agent);

            % Initialize sensor model
            sensorModel = sigmoidSensor;
            sensorModel = sensorModel.initialize(config.sensor.alphaDist, config.sensor.betaDist, config.sensor.alphaTilt, config.sensor.betaTilt);

            % Initialize fixed obstacles from corner coordinates
            nObs = size(tc.obstacleCorners, 1);
            obstacles = cell(nObs, 1);
            for jj = 1:nObs
                corners = [tc.obstacleCorners(jj, 1:3); tc.obstacleCorners(jj, 4:6)];
                obstacles{jj} = rectangularPrism;
                obstacles{jj} = obstacles{jj}.initialize(corners, REGION_TYPE.OBSTACLE, sprintf("Obstacle %d", jj));
            end

            % Place agents in small-x, large-y quadrant (opposite objectives)
            % with chain topology: each agent connected only to its neighbors
            midXY = (tc.testClass.domain.minCorner(1:2) + tc.testClass.domain.maxCorner(1:2)) / 2;
            quadrantSize = tc.testClass.domain.maxCorner(1:2) / 2;
            margin = quadrantSize / 6;
            agentBounds = [tc.testClass.domain.minCorner(1) + margin(1), ...
                           midXY(2) + margin(2); ...
                           midXY(1) - margin(1), ...
                           tc.testClass.domain.maxCorner(2) - margin(2)];
            % Find a fixed altitude where sensor performance passes at ALL
            % corners of the placement bounds (worst-case XY)
            corners = [agentBounds(1,1), agentBounds(1,2);
                       agentBounds(2,1), agentBounds(1,2);
                       agentBounds(1,1), agentBounds(2,2);
                       agentBounds(2,1), agentBounds(2,2)];
            agentAlt = tc.testClass.domain.maxCorner(3) - tc.collisionRadius;
            while agentAlt > minAlt + 2 * tc.collisionRadius
                worstPerf = inf;
                for cc = 1:4
                    p = sensorModel.sensorPerformance([corners(cc,:), agentAlt], [corners(cc,:), 0]);
                    worstPerf = min(worstPerf, p);
                end
                if worstPerf >= tc.sensorPerformanceMinimum * 10
                    break;
                end
                agentAlt = agentAlt - 1;
            end
            chainSpacingMin = 0.7 * tc.comRange;
            chainSpacingMax = 0.9 * tc.comRange;
            collisionGeometry = spherical;
            for jj = 1:n
                retry = true;
                while retry
                    retry = false;

                    if jj == 1
                        % First agent: random XY within bounds, fixed altitude
                        agentPos = [agentBounds(1, :) + (agentBounds(2, :) - agentBounds(1, :)) .* rand(1, 2), agentAlt];
                    else
                        % Place at 0.7-0.9 * comRange in XY from previous agent, same altitude
                        dir = randn(1, 2);
                        dir = dir / norm(dir);
                        r = chainSpacingMin + rand * (chainSpacingMax - chainSpacingMin);
                        agentPos = [agents{jj-1}.pos(1:2) + r * dir, agentAlt];
                    end

                    % Check within placement bounds (XY only, Z is fixed)
                    if any(agentPos(1:2) <= agentBounds(1, :)) || any(agentPos(1:2) >= agentBounds(2, :))
                        retry = true;
                        continue;
                    end

                    % Check sensor performance threshold; lower altitude if it fails
                    if sensorModel.sensorPerformance(agentPos, [agentPos(1:2), 0]) < tc.sensorPerformanceMinimum * 10
                        agentAlt = max(agentAlt - tc.collisionRadius, minAlt + 1.1 * tc.collisionRadius);
                        agentPos(3) = agentAlt;
                        % If we've hit the floor and still failing, widen XY search
                        if agentAlt <= minAlt + 2 * tc.collisionRadius
                            agentBounds = [tc.testClass.domain.minCorner(1) + tc.collisionRadius, ...
                                           tc.testClass.domain.minCorner(2) + tc.collisionRadius; ...
                                           tc.testClass.domain.maxCorner(1) - tc.collisionRadius, ...
                                           tc.testClass.domain.maxCorner(2) - tc.collisionRadius];
                        end
                        retry = true;
                        continue;
                    end

                    % Must be within comRange of previous agent (chain link)
                    if jj > 1 && norm(agents{jj-1}.pos - agentPos) >= tc.comRange
                        retry = true;
                        continue;
                    end

                    % Must be BEYOND comRange of all non-adjacent agents (sparsity)
                    % for kk = 1:(jj - 2)
                    %     if norm(agents{kk}.pos - agentPos) < tc.comRange
                    %         retry = true;
                    %         break;
                    %     end
                    % end
                    % if retry, continue; end

                    % No collision with any existing agent
                    for kk = 1:(jj - 1)
                        if norm(agents{kk}.pos - agentPos) < agents{kk}.collisionGeometry.radius + tc.collisionRadius
                            retry = true;
                            break;
                        end
                    end
                    if retry, continue; end

                    % No collision with any obstacle
                    for kk = 1:nObs
                        P = min(max(agentPos, obstacles{kk}.minCorner), obstacles{kk}.maxCorner);
                        d = agentPos - P;
                        if dot(d, d) <= tc.collisionRadius^2
                            retry = true;
                            break;
                        end
                    end
                end

                % Initialize agent
                collisionGeometry = collisionGeometry.initialize(agentPos, tc.collisionRadius, REGION_TYPE.COLLISION, sprintf("Agent %d Collision Region", jj));
                agents{jj} = agents{jj}.initialize(agentPos, collisionGeometry, sensorModel, tc.comRange, tc.maxIter, tc.initialStepSize, sprintf("Agent %d", jj), tc.plotCommsGeometry);
            end

            % Randomly shuffle agents to vary index-based topology
            agents = agents(randperm(numel(agents)));

            end % reroll loop

            % Inspect scenario if enabled
            if tc.inspectScenarios
                tc.testClass = tc.testClass.initialize(tc.testClass.domain, agents, tc.barrierGain, tc.barrierExponent, minAlt, tc.timestep, tc.maxIter, obstacles, tc.makePlots, tc.makeVideo, config.doubleIntegrator, tc.dampingCoeff, tc.useFixedTopology);
                fprintf("Test %d (n=%d, config=%s): reinit=%d. Inspect plot.\n", testIdx, n, configKeys{configIdx}, reinitCount);
                fprintf("To add reinits, update tc.reinit(%d) and rerun.\n", testIdx);
                keyboard;
                tc.testClass = tc.testClass.teardown();
                return;
            end

            % Set up simulation
            tc.testClass = tc.testClass.initialize(tc.testClass.domain, agents, tc.barrierGain, tc.barrierExponent, minAlt, tc.timestep, tc.maxIter, obstacles, tc.makePlots, tc.makeVideo, config.doubleIntegrator, tc.dampingCoeff, tc.useFixedTopology);
            
            % Save simulation parameters to output file
            tc.testClass.writeInits();

            % Run
            tc.testClass = tc.testClass.run();

            % Cleanup
            tc.testClass = tc.testClass.teardown();
            close all;
        end
        function AIIbeta_plots_3_4(tc)
            % test-specific parameters
            maxIters = 400;

            configs = results.makeConfigs();
            c = configs.A_2_alpha;

            % Set up fixed-size domain
            minAlt = tc.domainSize(3)/10 + rand * 1/10 * tc.domainSize(3);
            tc.testClass.domain = tc.testClass.domain.initialize([zeros(1, 3); tc.domainSize], REGION_TYPE.DOMAIN, "Domain");
            
            % Set objective
            objectiveMu = [tc.domainSize(1) * 2 / 3, tc.domainSize(2) * 3 / 4];
            objectiveSigma = reshape([215, 100; 100, 175], [1, 2, 2]);
            tc.testClass.domain.objective = tc.testClass.domain.objective.initialize(objectiveFunctionWrapper(objectiveMu, objectiveSigma), tc.testClass.domain, tc.discretizationStep, tc.protectedRange, tc.sensorPerformanceMinimum, objectiveMu, objectiveSigma);
            
            % Set agent initial states (fully connected network of 4)
            centerPos = [tc.domainSize(1) / 4, tc.domainSize(2) / 4];
            d = tc.collisionRadius + (tc.comRange - tc.collisionRadius) / 4;
            agentsPos = centerPos + [1, 1; 1, -1; -1, -1; -1, 1] / sqrt(2) * d;
            agentAlt = minAlt * 1.5;
            agentsPos = [agentsPos, agentAlt * ones(4, 1) + rand * 5 - 2.5];

            agents = {agent, agent, agent, agent};
            cg = spherical;
            sensorModel = sigmoidSensor;
            sensorModel = sensorModel.initialize(c.sensor.alphaDist, c.sensor.betaDist, c.sensor.alphaTilt, c.sensor.betaTilt);
            agents{1} = agents{1}.initialize(agentsPos(1, :), cg.initialize(agentsPos(1, :), tc.collisionRadius, REGION_TYPE.COLLISION, "Agent 1 Collision Geometry"), sensorModel, tc.comRange, maxIters, tc.initialStepSize, "Agent 1", false);
            agents{2} = agents{2}.initialize(agentsPos(2, :), cg.initialize(agentsPos(2, :), tc.collisionRadius, REGION_TYPE.COLLISION, "Agent 2 Collision Geometry"), sensorModel, tc.comRange, maxIters, tc.initialStepSize, "Agent 2", false);
            agents{3} = agents{3}.initialize(agentsPos(3, :), cg.initialize(agentsPos(3, :), tc.collisionRadius, REGION_TYPE.COLLISION, "Agent 3 Collision Geometry"), sensorModel, tc.comRange, maxIters, tc.initialStepSize, "Agent 3", false);
            agents{4} = agents{4}.initialize(agentsPos(4, :), cg.initialize(agentsPos(4, :), tc.collisionRadius, REGION_TYPE.COLLISION, "Agent 4 Collision Geometry"), sensorModel, tc.comRange, maxIters, tc.initialStepSize, "Agent 4", false);
            
            obstacles = cell(1, 1);
            obstacles{1} = rectangularPrism;
            obstacles{1} = obstacles{1}.initialize([0, tc.domainSize(2)/2, 0; tc.domainSize(1) * 0.4, tc.domainSize(2), 40],REGION_TYPE.OBSTACLE, "Obstacle 1");

            % Set up simulation
            tc.testClass = tc.testClass.initialize(tc.testClass.domain, agents, tc.barrierGain, tc.barrierExponent, minAlt, tc.timestep, maxIters, obstacles, tc.makePlots, tc.makeVideo, c.doubleIntegrator, tc.dampingCoeff, tc.useFixedTopology);
            
            % Save simulation parameters to output file
            tc.testClass.writeInits();

            % Run
            tc.testClass = tc.testClass.run();

            % Cleanup
            tc.testClass = tc.testClass.teardown();
        end
    end

    methods
        function c = obstacleCollisionCheck(~, obstacles, obstacle)
            % Check if the obstacle intersects with any other obstacles
            c = false;
            for ii = 1:size(obstacles, 1)
                if geometryIntersects(obstacles{ii}, obstacle)
                    c = true;
                    return;
                end
            end
        end

    end
end