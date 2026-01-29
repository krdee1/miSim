classdef parametricTestSuite < matlab.unittest.TestCase
    properties (Access = private)
        % System under test
        testClass = miSim;
        domain = rectangularPrism;

        % RNG control
        seed = 1;

        %% Diagnostic Parameters
        % No effect on simulation dynamics
        makeVideo = true; % disable video writing for big performance increase
        makePlots = true; % disable plotting for big performance increase (also disables video)
        plotCommsGeometry = false; % disable plotting communications geometries
        protectedRange = 0;

        %% Test iterations
        csvPath = fullfile(matlab.project.rootProject().RootFolder, "test", "testIterations.csv");
    end

    methods (TestMethodSetup)
        function rngSetup(tc)
            % Allow for controlling the random seed for reproducibility
            rng(tc.seed);
        end
    end

    methods (Static)
        function params = readIterationsCsv(csvPath)
            arguments (Input)
                csvPath (1, 1) string;
            end
            arguments (Output)
                params (1, 1) struct;
            end

            % File input validation
            assert(isfile(csvPath), "%s is not a valid filepath.");
            assert(endsWith(csvPath, ".csv"), "%s is not a CSV file.");

            % Read file
            csv = readtable(csvPath, "TextType", "String", "NumHeaderLines", 0, "VariableNamingRule", "Preserve");
            csv.Properties.VariableNames = ["timestep", "maxIter", "minAlt", "discretizationStep", "protectedRange", "sensorPerformanceMinimum", "initialStepSize", "barrierGain", "barrierExponent", "numObstacles", "numAgents", "collisionRadius", "comRange", "alphaDist", "betaDist", "alphaTilt", "betaTilt"];

            for ii = 1:size(csv.Properties.VariableNames, 2)
                csv.(csv.Properties.VariableNames{ii}) = cell2mat(cellfun(@(x) str2num(x), csv.(csv.Properties.VariableNames{ii}), "UniformOutput", false));
            end
        
            % Put params into standard structure
            params = struct("timestep", csv.timestep, "maxIter", csv.maxIter, "minAlt", csv.minAlt, "discretizationStep", csv.discretizationStep, ...
                            "protectedRange", csv.protectedRange, "sensorPerformanceMinimum", csv.sensorPerformanceMinimum, "initialStepSize", csv.initialStepSize, ...
                            "barrierGain", csv.barrierGain, "barrierExponent", csv.barrierExponent, "numObstacles", csv.numObstacles,...
                            "numAgents", csv.numAgents, "collisionRadius", csv.collisionRadius, "comRange", csv.comRange, "alphaDist", csv.alphaDist, ...
                            "betaDist", csv.betaDist, "alphaTilt", csv.alphaTilt, "betaTilt", csv.betaTilt);
        end
    end

    methods (Test)
        % Test cases
        function csv_parametric_tests(tc)
            % Read in parameters to iterate over
            params = tc.readIterationsCsv(tc.csvPath);

            % Test case setup
            l = 10; % domain size
            sensorModel = sigmoidSensor;
            collisionGeometry = spherical;

            % Iterate over test cases defined in CSV
            for ii = 1:size(params.timestep, 1)
                % Set up square domain
                tc.domain = tc.domain.initialize([zeros(1, 3); l * ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");
                tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([.75 * l, 0.75 * l]), tc.domain, params.discretizationStep(ii), params.protectedRange(ii), params.sensorPerformanceMinimum(ii));
                
                % Initialize agents
                agents = cell(params.numAgents(ii), 1);
                [agents{:}] = deal(agent);

                % Initialize sensor model
                sensorModel = sensorModel.initialize(params.alphaDist(ii, 1), params.betaDist(ii, 1), params.alphaTilt(ii, 1), params.betaTilt(ii, 1));

                % Place first agent randomly in the quadrant opposite the objective
                % not too close to the domain boundaries
                bounds = [params.collisionRadius(ii, 1) * ones(1, 2), params.collisionRadius(ii, 1) + params.minAlt(ii); l / 2 * ones(1, 2), l - params.collisionRadius(ii, 1)];
                agentPos = bounds(1, :) + (bounds(2, :) - bounds(1, :)) .* rand(1, 3);
                
                % Keep trying new positions until the greatest possible
                % sensor performance clears the threshold (meaning this
                % agent has the ability to make a partition)
                while sensorModel.sensorPerformance(agentPos, [agentPos(1:2), 0]) < params.sensorPerformanceMinimum(ii)
                    agentPos = bounds(1, :) + (bounds(2, :) - bounds(1, :)) .* rand(1, 3);
                end
                
                % Initialize agent
                collisionGeometry = collisionGeometry.initialize(agentPos, params.collisionRadius(ii, 1), REGION_TYPE.COLLISION, "Agent 1 Collision Region");
                agents{1} = agents{1}.initialize(agentPos, collisionGeometry, sensorModel, params.comRange(ii, 1), params.maxIter(ii), params.initialStepSize(ii), "Agent 1", tc.plotCommsGeometry);

                % Set up remaining agents in random (valid) locations
                for jj = 2:size(agents, 1)
                    % Initialize sensor model
                    sensorModel = sensorModel.initialize(params.alphaDist(ii, jj), params.betaDist(ii, jj), params.alphaTilt(ii, jj), params.betaTilt(ii, jj));
                    
                    % Base next agent's location on random previous agent's location
                    baseAgentIdx = randi(jj - 1);
                    retry = true;
                    while retry
                        agentPos = agents{baseAgentIdx}.commsGeometry.random();
                        retry = false;

                        % Check that altitude clears minimum
                        if agentPos(3) <= params.minAlt(ii) + params.collisionRadius(ii, jj)
                            retry = true;
                        end

                        % Check that the agent's greatest sensor
                        % performance clears the threshold for partitioning
                        if ~retry && sensorModel.sensorPerformance(agentPos, [agentPos(1:2), 0]) < params.sensorPerformanceMinimum(ii)
                            retry = true;
                        end

                        % Check that candidate position is well inside the domain
                        bounds = [params.collisionRadius(ii, jj) * ones(1, 2), max([params.collisionRadius(ii, jj), params.minAlt(ii)]); l / 2 * ones(1, 2), l - params.collisionRadius(ii, jj)];
                        if ~retry && ~isequal(agentPos < bounds, [false, false, false; true, true, true])
                            retry = true;
                        end

                        % Check that candidate position does not collide with existing agents
                        for kk = 1:(jj - 1)
                            if ~retry && norm(agents{kk}.pos - agentPos, 2) < agents{kk}.collisionGeometry.radius + params.collisionRadius(ii, jj)
                                retry = true;
                                break;
                            end
                        end
                    end

                    % Initialize agent
                    collisionGeometry = collisionGeometry.initialize(agentPos, params.collisionRadius(ii, jj), REGION_TYPE.COLLISION, sprintf("Agent %d Collision Region", jj));
                    agents{jj} = agents{jj}.initialize(agentPos, collisionGeometry, sensorModel, params.comRange(ii, jj), params.maxIter(ii), params.initialStepSize(ii), sprintf("Agent %d", jj), tc.plotCommsGeometry);
                end

                % randomly shuffle agents to make the network more interesting (probably)
                agents = agents(randperm(numel(agents))); 

                % Set up obstacles
                obstacles = cell(params.numObstacles(ii), 1);
                [obstacles{:}] = deal(rectangularPrism);

                % Define ranges to permit obstacles (relies on certain
                % assumptions about agent and objective placement)
                bounds = [max(cell2mat(cellfun(@(x) x.pos(1:2), agents, "UniformOutput", false))) + max(cellfun(@(x) x.collisionGeometry.radius, agents)); ...
                          tc.domain.objective.groundPos - tc.domain.objective.protectedRange];

                for jj = 1:size(obstacles, 1)
                    % randomly place obstacles in at least the X or Y or X
                    % and Y range defined by the objective region and the 
                    % agents initial region
                    retry = true;
                    while retry
                        retry = false;

                        % candidate corners for obstacle
                        corners = [sort(tc.domain.maxCorner(1, 1:2) .* rand(2), 1, "ascend"), [params.minAlt(ii); params.minAlt(ii) + rand * (tc.domain.maxCorner(3) - params.minAlt(ii))]];

                        % Check X falls into bucket in at least one vertex
                        if ~retry && ~(any(corners(:, 1) > bounds(1, 1) & corners(:, 1) < bounds(2, 1)) || any(corners(:, 2) > bounds(1, 2) & corners(:, 2) < bounds(2, 2)))
                            retry = true;
                        end

                        % Initialize obstacle using proposed coordinates
                        if ~retry
                            obstacles{jj} = obstacles{jj}.initialize(corners, REGION_TYPE.OBSTACLE, sprintf("Obstacle %d", jj));
                        end


                        % Make sure the obstacle doesn't crowd the objective
                        if ~retry && obstacles{jj}.distance([tc.domain.objective.groundPos, params.minAlt(ii)]) <= tc.domain.objective.protectedRange
                            retry = true;
                        end

                        % Check if the obstacle collides with an existing obstacle
                        if ~retry && jj > 1 && tc.obstacleCollisionCheck(obstacles(1:(jj - 1)), obstacles{jj})
                            retry = true;
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

                % Set up simulation
                tc.testClass = tc.testClass.initialize(tc.domain, agents, params.barrierGain(ii), params.barrierExponent(ii), params.minAlt(ii), params.timestep(ii), params.maxIter(ii), obstacles, tc.makePlots, tc.makeVideo);
    
                % Save simulation parameters to output file
                tc.testClass.writeInits();
    
                % Run
                tc.testClass = tc.testClass.run();
    
                % Cleanup
                tc.testClass = tc.testClass.teardown();
            end
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