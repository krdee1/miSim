classdef results < matlab.unittest.TestCase
    properties (Constant, Access = private)
        seed = 1;
    end

    properties (Access = private)
        % System under test
        testClass = miSim;

        %% Diagnostic Parameters
        % No effect on simulation dynamics
        makeVideo = false; % disable video writing for big performance increase
        makePlots = false; % disable plotting for big performance increase (also disables video)
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
        minDimension = 50; % minimum domain size
        maxDimension = 100; % maximum domain size
        discretizationStep = 0.1;
        protectedRange = 5;
        collisionRadius = 5;
        sensorPerformanceMinimum = 0.005;
        comRange = 20;
        maxIter = 250;
        initialStepSize = 1;
        numObstacles = 3;
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

    methods (TestMethodSetup)
        % % Generate a random domain for each test
        % function tc = setDomain(tc)
        %     tc.testClass.domain = rectangularPrism;
        %     % random integer-dimensioned cubic domain
        %     tc.testClass.domain = tc.testClass.domain.initializeRandom(REGION_TYPE.DOMAIN, "Domain", tc.minDimension);
        %     % Random bivariate normal PDF objective
        %     tc.testClass.domain.objective = tc.testClass.domain.objective.initializeRandomMvnpdf(tc.testClass.domain, tc.discretizationStep, tc.protectedRange);
        % end
    end

    methods (Static, Access = private)
        function c = makeConfigs()
            rng(results.seed);
            abMin = 6; % alpha*beta >= 6 ensures membership(0) = tanh(3) >= 0.995
            alphaDist = rand(1, 2) .* [100, 100];
            betaDist = abMin ./ alphaDist + rand(1, 2) .* (20 - abMin ./ alphaDist);
            alphaTilt = 10 + rand(1, 2) .* [20, 20];
            betaTilt = abMin ./ alphaTilt + rand(1, 2) .* (50 - abMin ./ alphaTilt);
            sensors = struct('alphaDist', num2cell(alphaDist), 'alphaTilt', num2cell(alphaTilt), 'betaDist', num2cell(betaDist), 'betaTilt', num2cell(betaTilt));
            sensor1 = sigmoidSensor;
            sensor2 = sigmoidSensor;
            sensor1 = sensor1.initialize(sensors(1).alphaDist, sensors(1).betaDist, sensors(1).alphaTilt, sensors(1).betaTilt);
            sensor2 = sensor2.initialize(sensors(2).alphaDist, sensors(2).betaDist, sensors(2).alphaTilt, sensors(2).betaTilt);
            sensor1.plotParameters;
            sensor2.plotParameters;
            c = struct('A_1_alpha', struct('numDist', 1, 'sensor', sensors(1), 'doubleIntegrator', false), ...
                        'A_1_beta',  struct('numDist', 1, 'sensor', sensors(1), 'doubleIntegrator', true), ...
                        'A_2_alpha', struct('numDist', 1, 'sensor', sensors(2), 'doubleIntegrator', false), ...
                        'B_1_beta',  struct('numDist', 2, 'sensor', sensors(1), 'doubleIntegrator', true));
        end
    end

    methods (Test)
        function plot1_runs(tc, n, config)
        % OVERRIDES
        % function plot1_runs(tc)
            % n = 3;
            % config = struct('numDist', 1, 'sensor', struct('alphaDist', 100, 'alphaTilt', 2, 'betaDist', 10, 'betaTilt', 0.5), 'doubleIntegrator', false);
           
            % Compute test case index for reinit lookup
            nKeys = fieldnames(tc.n);
            configKeys = fieldnames(tc.config);
            nIdx = find(cellfun(@(k) tc.n.(k) == n, nKeys));
            configIdx = find(cellfun(@(k) isequal(tc.config.(k), config), configKeys));
            testIdx = (nIdx - 1) * numel(configKeys) + configIdx;

            % Determine number of reinitializations
            reinitCount = tc.reinit(testIdx);

            for reroll = 0:reinitCount

            % Set up random cube domain
            minAlt = tc.minDimension(1) * rand() * 0.5;
            tc.testClass.domain = tc.testClass.domain.initializeRandom(REGION_TYPE.DOMAIN, "Domain", tc.minDimension, tc.maxDimension, tc.testClass.domain, minAlt);
            % Place sensing objective(s)
            objectiveMu = [];
            objectiveSigma = [];
            for ii = 1:config.numDist
                mu = tc.testClass.domain.minCorner;
                while tc.testClass.domain.distance(mu) < tc.protectedRange * 1.01
                    mu = tc.testClass.domain.random();
                end
                notPosDef = true;
                while notPosDef
                    sig = reshape(sort(rand(1, 4) * min(tc.testClass.domain.dimensions(1:2))), [1, 2, 2]);
                    sig(1, 2, 1) = max([sig(1, 1, 2), sig(1, 2, 1)]);
                    sig(1, 1, 2) = sig(1, 2, 1);
                    [~, notPosDef] = chol(squeeze(sig));
                end
                objectiveMu = [objectiveMu; mu(1:2)];
                objectiveSigma = cat(1, objectiveSigma, sig);
            end
            tc.testClass.domain.objective = tc.testClass.domain.objective.initialize(objectiveFunctionWrapper(objectiveMu, objectiveSigma), tc.testClass.domain, tc.discretizationStep, tc.protectedRange, tc.sensorPerformanceMinimum, objectiveMu, objectiveSigma);

            % Initialize agents
            agents = cell(n, 1);
            [agents{:}] = deal(agent);

            % Initialize sensor model
            sensorModel = sigmoidSensor;
            sensorModel = sensorModel.initialize(config.sensor.alphaDist, config.sensor.betaDist, config.sensor.alphaTilt, config.sensor.betaTilt);

            % Place agents in a quadrant that contains no objective peaks
            midXY = (tc.testClass.domain.minCorner(1:2) + tc.testClass.domain.maxCorner(1:2)) / 2;
            occupied = false(2, 2);
            for ii = 1:size(objectiveMu, 1)
                occupied(1 + (objectiveMu(ii, 1) >= midXY(1)), ...
                         1 + (objectiveMu(ii, 2) >= midXY(2))) = true;
            end
            freeQ = find(~occupied);
            if isempty(freeQ)
                qi = 1;
            else
                qi = freeQ(randi(numel(freeQ)));
            end
            [xi, yi] = ind2sub([2, 2], qi);
            xLim = [tc.testClass.domain.minCorner(1), midXY(1), tc.testClass.domain.maxCorner(1)];
            yLim = [tc.testClass.domain.minCorner(2), midXY(2), tc.testClass.domain.maxCorner(2)];
            agentBounds = [max(xLim(xi),   tc.testClass.domain.minCorner(1) + tc.collisionRadius), ...
                           max(yLim(yi),   tc.testClass.domain.minCorner(2) + tc.collisionRadius), ...
                           minAlt + tc.collisionRadius; ...
                           min(xLim(xi+1), tc.testClass.domain.maxCorner(1) - tc.collisionRadius), ...
                           min(yLim(yi+1), tc.testClass.domain.maxCorner(2) - tc.collisionRadius), ...
                           tc.testClass.domain.maxCorner(3) - tc.collisionRadius];
            collisionGeometry = spherical;
            for jj = 1:n
                retry = true;
                while retry
                    retry = false;

                    if jj == 1
                        % First agent: uniform random within placement bounds
                        agentPos = agentBounds(1, :) + (agentBounds(2, :) - agentBounds(1, :)) .* rand(1, 3);
                    else
                        % Sample near centroid of existing agents to maximize
                        % probability of being within comRange of all others
                        positions = cell2mat(cellfun(@(x) x.pos, agents(1:(jj-1)), 'UniformOutput', false));
                        centroid = mean(positions, 1);
                        maxSpread = max(vecnorm(positions - centroid, 2, 2));
                        safeRadius = tc.comRange - maxSpread;

                        if safeRadius > 2 * tc.collisionRadius
                            % Uniform random within guaranteed-connected sphere
                            dir = randn(1, 3);
                            dir = dir / norm(dir);
                            r = safeRadius * rand()^(1/3);
                            agentPos = centroid + r * dir;
                        else
                            % Safe sphere too small; sample within comms sphere
                            % of random existing agent (comRange check below)
                            baseIdx = randi(jj - 1);
                            agentPos = agents{baseIdx}.commsGeometry.random();
                        end
                    end

                    % Check within placement bounds
                    if any(agentPos <= agentBounds(1, :)) || any(agentPos >= agentBounds(2, :))
                        retry = true;
                        continue;
                    end

                    % Check sensor performance threshold
                    if sensorModel.sensorPerformance(agentPos, [agentPos(1:2), 0]) < tc.sensorPerformanceMinimum * 10
                        retry = true;
                        continue;
                    end

                    % Check within comRange of ALL existing agents (complete graph)
                    for kk = 1:(jj - 1)
                        if norm(agents{kk}.pos - agentPos) >= tc.comRange
                            retry = true;
                            break;
                        end
                    end
                    if retry, continue; end

                    % Check collision with ALL existing agents
                    for kk = 1:(jj - 1)
                        if norm(agents{kk}.pos - agentPos) < agents{kk}.collisionGeometry.radius + tc.collisionRadius
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

            % Add random obstacles (each limited to 1/4 domain size in X and Y)
            obstacles = cell(tc.numObstacles, 1);
            [obstacles{:}] = deal(rectangularPrism);

            % Define target region for obstacles (between agents and objective)
            agentExtent = max(cell2mat(cellfun(@(x) x.pos(1:2), agents, "UniformOutput", false))) + max(cellfun(@(x) x.collisionGeometry.radius, agents));
            objExtent = tc.testClass.domain.objective.groundPos - tc.testClass.domain.objective.protectedRange;
            obsMin = zeros(1, 2);
            obsMax = zeros(1, 2);
            for dim = 1:2
                if agentExtent(dim) < objExtent(dim)
                    obsMin(dim) = agentExtent(dim);
                    obsMax(dim) = objExtent(dim);
                else
                    obsMin(dim) = tc.testClass.domain.minCorner(dim);
                    obsMax(dim) = tc.testClass.domain.maxCorner(dim);
                end
            end
            maxObsSize = 3 * tc.collisionRadius * ones(1, 3);

            for jj = 1:size(obstacles, 1)
                retry = true;
                while retry
                    retry = false;

                    % Generate random anchor point, then random size up to 3x collision radius
                    anchor = [obsMin + rand(1, 2) .* (obsMax - obsMin), minAlt];
                    obsSize = rand(1, 3) .* maxObsSize;
                    corners = [anchor; anchor + obsSize];

                    % Initialize obstacle using proposed coordinates
                    obstacles{jj} = obstacles{jj}.initialize(corners, REGION_TYPE.OBSTACLE, sprintf("Obstacle %d", jj));

                    % Make sure the obstacle doesn't crowd the objective
                    for kk = 1:size(tc.testClass.domain.objective.groundPos, 1)
                        if ~retry && obstacles{jj}.distance([tc.testClass.domain.objective.groundPos(kk, 1:2), minAlt]) <= tc.testClass.domain.objective.protectedRange
                            retry = true;
                            continue;
                        end
                    end

                    % Check if the obstacle collides with an existing obstacle
                    if ~retry && jj > 1 && tc.obstacleCollisionCheck(obstacles(1:(jj - 1)), obstacles{jj})
                        retry = true;
                        continue;
                    end

                    % Check if the obstacle collides with an agent
                    if ~retry
                        for kk = 1:size(agents, 1)
                            P = min(max(agents{kk}.pos, obstacles{jj}.minCorner), obstacles{jj}.maxCorner);
                            d = agents{kk}.pos - P;
                            if dot(d, d) <= agents{kk}.collisionGeometry.radius^2
                                retry = true;
                                break;
                            end
                        end
                    end

                    if retry
                        continue;
                    end
                end
            end

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