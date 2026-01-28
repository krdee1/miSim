classdef parametricTestSuite < matlab.unittest.TestCase
    properties (Access = private)
        % System under test
        testClass = miSim;
        domain = rectangularPrism;
        obstacles = cell(1, 0);

        % RNG control
        seed = 1234567890;

        %% Diagnostic Parameters
        % No effect on simulation dynamics
        makeVideo = true; % disable video writing for big performance increase
        makePlots = true; % disable plotting for big performance increase (also disables video)
        plotCommsGeometry = false; % disable plotting communications geometries
        protectedRange = 0;

        %% Test iterations
        csvPath = fullfile(matlab.project.rootProject().RootFolder, 'test', 'testIterations.csv');
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
            assert(endsWith(csvPath, '.csv'), "%s is not a CSV file.");

            % Read file
            csv = readtable(csvPath, 'TextType', 'string', 'NumHeaderLines', 0, "VariableNamingRule", "Preserve");
            csv.Properties.VariableNames = ["timestep", "maxIter", "minAlt", "discretizationStep", "sensorPerformanceMinimum", "initialStepSize", "barrierGain", "barrierExponent", "numAgents", "collisionRadius", "comRange", "alphaDist", "betaDist", "alphaTilt", "betaTilt"];

            for ii = 1:size(csv.Properties.VariableNames, 2)
                csv.(csv.Properties.VariableNames{ii}) = cell2mat(cellfun(@(x) str2num(x), csv.(csv.Properties.VariableNames{ii}), 'UniformOutput', false));
            end
        
            % Put params into standard structure
            params = struct('timestep', csv.timestep, 'maxIter', csv.maxIter, 'minAlt', csv.minAlt, 'discretizationStep', csv.discretizationStep, ...
                            'sensorPerformanceMinimum', csv.sensorPerformanceMinimum, 'initialStepSize', csv.initialStepSize, 'barrierGain', csv.barrierGain, 'barrierExponent', csv.barrierExponent, ...
                            'numAgents', csv.numAgents, 'collisionRadius', csv.collisionRadius, 'comRange', csv.comRange, 'alphaDist', csv.alphaDist, 'betaDist', csv.betaDist, 'alphaTilt', csv.alphaTilt, 'betaTilt', csv.betaTilt);
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
                tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([.75 * l, 0.75 * l]), tc.domain, params.discretizationStep(ii), tc.protectedRange, params.sensorPerformanceMinimum(ii));
                
                % Initialize agents
                agents = cell(params.numAgents(ii), 1);
                [agents{:}] = deal(agent);

                % Initialize sensor model
                sensorModel = sensorModel.initialize(params.alphaDist(ii, 1), params.betaDist(ii, 1), params.alphaTilt(ii, 1), params.betaTilt(ii, 1));

                % Place first agent randomly in the quadrant opposite the objective
                % not too close to the domain boundaries
                bounds = [params.collisionRadius(ii, 1) * ones(1, 2), max([params.collisionRadius(ii, 1), params.minAlt(ii)]); l / 2 * ones(1, 2), l - params.collisionRadius(ii, 1)];
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

                        % Check that the agent's greatest sensor
                        % performance clears the threshold for partitioning
                        if sensorModel.sensorPerformance(agentPos, [agentPos(1:2), 0]) < params.sensorPerformanceMinimum(ii)
                            retry = true;
                            continue;
                        end

                        % Check that candidate position is well inside the domain
                        bounds = [params.collisionRadius(ii, jj) * ones(1, 2), max([params.collisionRadius(ii, jj), params.minAlt(ii)]); l / 2 * ones(1, 2), l - params.collisionRadius(ii, jj)];
                        if ~isequal(agentPos < bounds, [false, false, false; true, true, true])
                            retry = true;
                            continue;
                        end

                        % Check that candidate position does not collide with existing agents
                        for kk = 1:(jj - 1)
                            if norm(agents{kk}.pos - agentPos, 2) < agents{kk}.collisionGeometry.radius + params.collisionRadius(ii, jj)
                                retry = true;
                                continue;
                            end
                        end
                        retry = false;
                    end

                    % Initialize agent
                    collisionGeometry = collisionGeometry.initialize(agentPos, params.collisionRadius(ii, jj), REGION_TYPE.COLLISION, sprintf("Agent %d Collision Region", jj));
                    agents{jj} = agents{jj}.initialize(agentPos, collisionGeometry, sensorModel, params.comRange(ii, jj), params.maxIter(ii), params.initialStepSize(ii), sprintf("Agent %d", jj), tc.plotCommsGeometry);
                end

                % Set up simulation
                agents = agents(randperm(numel(agents))); % randomly shuffle agents to make the network more interesting (probably)
                tc.testClass = tc.testClass.initialize(tc.domain, agents, params.barrierGain(ii), params.barrierExponent(ii), params.minAlt(ii), params.timestep(ii), params.maxIter(ii), tc.obstacles, tc.makePlots, tc.makeVideo);
    
                % Save simulation parameters to output file
                tc.testClass.writeParams();
    
                % Run
                tc.testClass = tc.testClass.run();
    
                % Cleanup
                tc.testClass = tc.testClass.teardown();
            end
            
        end
    end
end