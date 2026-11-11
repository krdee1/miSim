classdef test_permutations < matlab.unittest.TestCase
    % Parameter sweep over scenario configurations for 5-agent runs with
    % SINR communications and the CALSN topology. The test parameters
    % below combine exhaustively into 72 tests:
    %   2 dynamics x 3 objectives x 2 models x 2 reaches x 3 obstacle layouts
    %
    % Each run writes <name>_miSimInits.mat and <name>_miSimHist.mat into
    % sandbox/perm_<suite start time>/, where <name> spells out the
    % permutation, e.g. N5_double_obj2_service_longNarrow_close4.
    %
    % A test passes when the run completes with validate() holding at
    % every step (connected feasibility and CALSN graphs, no obstacle
    % penetration), the initial CALSN state is valid, and no two agents'
    % collision spheres ever overlap. Performance is saved, not asserted.
    %
    % Agent starting states are kept separate from the swept parameters
    % (agentConfigs) so that specific agent-count cases can be added later
    % as their own test method calling runScenario, without re-running the
    % full sweep.

    properties (TestParameter)
        dynamics = struct( ...
            "single", struct("name", "single", "useDoubleIntegrator", false), ...
            "double", struct("name", "double", "useDoubleIntegrator", true));

        % Gaussian objectives in the upper-right quadrant, one row per peak.
        % cov rows are [varX, varY, covXY] in m^2. Round peaks use sigma = 15 m;
        % with peaks 40-50 m apart (~3 sigma) each stays visually distinct.
        % One peak each in "two" and "three" is elongated to sigma = 25 m
        % along one axis: in "two", the (170, 140) peak along the (1, 2)
        % direction, across the line joining the peaks; in "three", the
        % (150, 125) peak along X. Peaks are summed PDFs, so every peak
        % carries the same total importance and an elongated peak is lower.
        objective = struct( ...
            "one",   struct("name", "obj1", "mu", [150, 150], ...
                            "cov", [225, 225, 0]), ...
            "two",   struct("name", "obj2", "mu", [130, 160; 170, 140], ...
                            "cov", [225, 225, 0; 305, 545, 160]), ...
            "three", struct("name", "obj3", "mu", [125, 165; 170, 160; 150, 125], ...
                            "cov", [225, 225, 0; 225, 225, 0; 625, 225, 0]));

        model = struct( ...
            "sensing", "sensing", ... % sigmoidSensor
            "service", "service");    % rfSensor

        reach = struct( ...
            "shortWide", "shortWide", ...
            "longNarrow", "longNarrow");

        % Obstacles between the start corner and the objectives, rows of
        % [minCorner, maxCorner]. Heights are mixed: full-height (200 m)
        % walls must be flown around; partial-height blocks (75-125 m) top
        % out inside the agents' ~80-130 m operating band, so they can be
        % climbed over at some cost in coverage.
        obstacles = struct( ...
            ... % two blocks ~30 m apart near the diagonal
            "sparse2", struct("name", "sparse2", "corners", [ ...
                 60,  85, 0,  80, 105, 200; ...
                110,  75, 0, 130,  95, 110]), ...
            ... % a 15 m gap astride the diagonal near (100, 100)
            "close2", struct("name", "close2", "corners", [ ...
                 75,  95, 0,  95, 115, 200; ...
                110,  75, 0, 130,  95,  90]), ...
            ... % a partial block and a full-height wall with a ~14 m diagonal
            ... % gap between their corners, a full-height block on the
            ... % diagonal, and a partial block inside the objective area. The
            ... % gap was ~6 m, narrower than an agent's 10 m collision sphere,
            ... % and stranded 1-2 agents at its mouth in nearly every run;
            ... % swapping heights alone didn't help (agents fly below 125 m),
            ... % widening the gap did.
            "close4", struct("name", "close4", "corners", [ ...
                 70,  90, 0,  88, 108, 125; ...
                 98,  62, 0, 116,  80, 200; ...
                100, 100, 0, 115, 115, 200; ...
                140, 140, 0, 150, 150,  75]));
    end

    properties (Access = private)
        % Agent starting states. Agents 1 and N are the CALSN base stations,
        % placed at the extremes of the cluster. All starts lie in the
        % lower-left corner, outside the central 100 x 100 square, at about
        % mid-domain altitude.
        agentConfigs = struct( ...
            "five", struct("name", "N5", "pos", [ ...
                12, 42, 100; ...
                22, 28, 108; ...
                32, 38,  96; ...
                38, 20, 104; ...
                44, 12, 100]));

        % Output folder (under sandbox/) shared by all runs of one suite run
        outputFolder = "";

        % Simulation
        maxIter = 150;
        timestep = 1; % s
        initialStepSize = 4; % m per step, decays linearly to 0 (~300 m total over 150 steps)
        % Gimbal rate: deg per step, decays linearly to 0 (~19 deg of total
        % tilt change over 150 steps). Kept small so agents improve coverage
        % by moving rather than by aiming narrow beams at the objective from
        % far away (at 2 deg/step, service/longNarrow agents stood off
        % 50-100 m and aimed).
        initialMaxAngleStepSize = 0.25;

        % Domain and objective
        domainCorners = [0, 0, 0; 200, 200, 200];
        discretizationStep = 2; % m; also the finite-difference step in agent.run
        protectedRange = 1;
        sensingPartitionMin = 0; % sigmoid performance, unitless
        servicePartitionMin = -Inf; % SINR, dB

        % Agents
        collisionRadius = 5; % m
        commsRange = 50; % m; unused in SINR mode, required by agent.initialize
        commsTxPower = 0.1; % W, UAV-UAV links

        % Sigmoid sensors. Agents settle roughly 40-50 m below alphaDist,
        % where the distance falloff starts to outweigh the wider footprint.
        % In 5-agent runs: shortWide ~65-90 m, longNarrow ~110-135 m.
        % betas are gentle so that the gradient is nonzero (not underflowed)
        % across the whole domain.
        sigmoidParams = struct( ...
            "shortWide",  struct("alphaDist", 130, "alphaTilt", 40), ...
            "longNarrow", struct("alphaDist", 160, "alphaTilt", 15));
        betaDist = 0.2;
        betaTilt = 0.2;

        % RF transmitters. The service objective sums SINR in dB, and for a
        % cos^n beam with path-loss exponent a the best altitude over a
        % ground point at horizontal range r is z = r * sqrt(n / a),
        % independent of power. Beam width therefore sets altitude: in
        % 5-agent runs shortWide (~17 deg) settles ~30-65 m and longNarrow
        % (~6 deg) ~75-115 m. A genuinely wide beam would sit on the floor.
        % Power sets the interference/noise balance instead. Fraction of the
        % grid where interference exceeds noise, at the start / over the
        % objective: shortWide ~53% / ~30%, longNarrow ~17% / ~24%. The
        % narrow beam's split barely moves with power because its off-axis
        % gain falls off so steeply.
        rfParams = struct( ...
            "shortWide",  struct("P_TX", 3e-4, "beamwidthExponent", 16), ...  % ~17 deg half-power half-angle
            "longNarrow", struct("P_TX", 1e-1, "beamwidthExponent", 128));    % ~6 deg
        rfTemperature = 300; % K
        rfBandwidth = 20e6; % Hz
        rfCenterFreq = 3e9; % Hz
        rfRxGain = 3; % dBi
        rfLossExponent = 2;

        % Communications (SINR model) and CALSN. The starting cluster's
        % weakest pair (the base-to-base diagonal) is at -13.3 dB and the
        % other pairs are between -12.2 and +1.4 dB. initialize() asserts
        % that every selected link is at least threshold + 3 dB.
        sinrThreshold = -15; % dB
        pathLossExponent = 2;
        ambientTemp = 290; % K
        centerFreq = 2.4e9; % Hz
        bandwidth = 20e6; % Hz

        % Constraints
        minAlt = 25; % m
        % Linear barrier with gain * timestep = 0.5: each step may close at
        % most half of the remaining collision/obstacle margin, which keeps
        % the discrete-time CBF safe with 4 m steps. (Gain 100 with exponent 3,
        % tuned for the 10 m test domains, lets agents step through 5 m spheres.)
        barrierGain = 0.5;
        barrierExponent = 1;
        dampingCoeff = 2;

        % Diagnostics
        makePlots = false;
        makeVideo = false;
        seed = 1;
    end

    methods (TestClassSetup)
        function createOutputFolder(tc)
            tc.outputFolder = "perm_" + string(datetime("now", "Format", "yyyy_MM_dd_HH_mm_ss"));
            mkdir(fullfile(matlab.project.rootProject().RootFolder, "sandbox", tc.outputFolder));
        end
    end

    methods (TestMethodSetup)
        function rngSetup(tc)
            rng(tc.seed);
        end
    end

    methods (Test)
        function permutation(tc, dynamics, objective, model, reach, obstacles)
            tc.runScenario(tc.agentConfigs.five, dynamics, objective, model, reach, obstacles);
        end
    end

    methods (Access = private)
        function runScenario(tc, agentConfig, dynamics, objective, model, reach, obstacles)
            name = sprintf("%s_%s_%s_%s_%s_%s", agentConfig.name, dynamics.name, objective.name, model, reach, obstacles.name);

            domain = tc.buildDomain(objective, model);
            sensor = tc.buildSensor(model, reach);
            agents = tc.buildAgents(agentConfig, sensor);
            obs = tc.buildObstacles(obstacles);

            sim = miSim;
            sim = sim.initialize(domain, agents, tc.barrierGain, tc.barrierExponent, tc.minAlt, tc.timestep, tc.maxIter, obs, ...
                tc.makePlots, tc.makeVideo, dynamics.useDoubleIntegrator, tc.dampingCoeff, false, true, ...
                true, tc.sinrThreshold, tc.pathLossExponent, tc.ambientTemp, tc.centerFreq, tc.bandwidth, true);
            sim.artifactName = tc.outputFolder + "/" + name;

            % Initial CALSN state: bases at zero potential, every other
            % agent's potential finite, one connected maintained graph,
            % nothing unrouted
            n = numel(agents);
            phi = sim.routingPotentials;
            tc.assertEqual(phi([1, n]), [0; 0]);
            tc.assertTrue(all(isfinite(phi)));
            tc.assertEqual(max(conncomp(graph(sim.constraintAdjacencyMatrix))), 1);
            tc.assertTrue(all(sim.routingUnrouted == 0));

            % Run; validate() checks connectivity and obstacle clearance at
            % every step and errors (failing the test) on a violation
            sim.writeInits();
            sim = sim.run();
            tc.assertEqual(max(conncomp(graph(sim.constraintAdjacencyMatrix))), 1);
            sim.teardown();

            % Agent-agent clearance over the whole trajectory, with the
            % same 1e-3 m^2 tolerance validate() uses for obstacles
            hist = load(fullfile(matlab.project.rootProject().RootFolder, "sandbox", sim.artifactName + "_miSimHist.mat")).out;
            for ii = 1:(n - 1)
                for jj = (ii + 1):n
                    d2 = sum((hist.agent(ii).pos - hist.agent(jj).pos).^2, 2);
                    rSum = hist.agent(ii).collisionRadius + hist.agent(jj).collisionRadius;
                    tc.verifyGreaterThanOrEqual(min(d2), rSum^2 - 1e-3, ...
                        sprintf("Agents %d and %d collision spheres overlap", ii, jj));
                end
            end
        end

        function domain = buildDomain(tc, objective, model)
            domain = rectangularPrism;
            domain = domain.initialize(tc.domainCorners, REGION_TYPE.DOMAIN, "Domain");

            mu = objective.mu;
            c = objective.cov; % [varX, varY, covXY] per peak
            sigma = zeros(size(mu, 1), 2, 2);
            sigma(:, 1, 1) = c(:, 1);
            sigma(:, 2, 2) = c(:, 2);
            sigma(:, 1, 2) = c(:, 3);
            sigma(:, 2, 1) = c(:, 3);
            if model == "sensing"
                partitionMin = tc.sensingPartitionMin;
            else
                partitionMin = tc.servicePartitionMin;
            end
            domain.objective = domain.objective.initialize(objectiveFunctionWrapper(mu, sigma), domain, ...
                tc.discretizationStep, tc.protectedRange, partitionMin, mu, sigma);
        end

        function sensor = buildSensor(tc, model, reach)
            if model == "sensing"
                p = tc.sigmoidParams.(reach);
                sensor = sigmoidSensor;
                sensor = sensor.initialize(p.alphaDist, tc.betaDist, p.alphaTilt, tc.betaTilt, 0, 0);
            else
                p = tc.rfParams.(reach);
                sensor = rfSensor;
                sensor = sensor.initialize(p.P_TX, tc.rfTemperature, tc.rfBandwidth, tc.rfCenterFreq, tc.rfRxGain, ...
                    p.beamwidthExponent, 0, 0, tc.rfLossExponent);
            end
        end

        function agents = buildAgents(tc, agentConfig, sensor)
            n = size(agentConfig.pos, 1);
            agents = cell(n, 1);
            for ii = 1:n
                pos = agentConfig.pos(ii, :);
                geometry = spherical;
                geometry = geometry.initialize(pos, tc.collisionRadius, REGION_TYPE.COLLISION, sprintf("Agent %d Collision Geometry", ii));
                agents{ii} = agent;
                agents{ii} = agents{ii}.initialize(pos, geometry, sensor, tc.commsRange, tc.maxIter, tc.initialStepSize, ...
                    tc.initialMaxAngleStepSize, sprintf("Agent %d", ii), false, tc.commsTxPower);
            end
        end

        function obs = buildObstacles(~, obstacles)
            corners = obstacles.corners;
            obs = cell(size(corners, 1), 1);
            for ii = 1:size(corners, 1)
                obs{ii} = rectangularPrism;
                obs{ii} = obs{ii}.initialize([corners(ii, 1:3); corners(ii, 4:6)], REGION_TYPE.OBSTACLE, sprintf("Obstacle %d", ii));
            end
        end
    end
end
