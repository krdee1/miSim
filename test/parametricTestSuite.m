classdef parametricTestSuite < matlab.unittest.TestCase
    properties (Access = private)
        % System under test
        testClass = miSim;
        domain = rectangularPrism;
        obstacles = cell(1, 0);

        %% Diagnostic Parameters
        % No effect on simulation dynamics
        makeVideo = true; % disable video writing for big performance increase
        makePlots = true; % disable plotting for big performance increase (also disables video)
        plotCommsGeometry = false; % disable plotting communications geometries
        protectedRange = 0;

        %% Test iterations
        csvPath = fullfile(matlab.project.rootProject().RootFolder, 'test', 'testIterations.csv');
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
            csv = readtable(csvPath);
        
            % Put params into standard structure
            params = struct('timestep', csv.timestep, 'maxIter', csv.maxIter, 'minAlt', csv.minAlt, 'discretizationStep', csv.discretizationStep, ...
                            'sensorPerformanceMinimum', csv.sensorPerformanceMinimum, 'collisionRadius', csv.collisionRadius, 'alphaDist', csv.alphaDist, 'betaDist', csv.betaDist, ...
                            'alphaTilt', csv.alphaTilt, 'betaTilt', csv.betaTilt, 'comRange', csv.comRange, 'initialStepSize', csv.initialStepSize, 'barrierGain', csv.barrierGain, 'barrierExponent', csv.barrierExponent);
        end
    end

    methods (Test, ParameterCombination = "exhaustive")
        % Test cases
        function single_agent_gradient_ascent(tc)
            % Read in parameters to iterate over
            params = tc.readIterationsCsv(tc.csvPath);

            % Test case setup
            l = 10;
            sensorModel = sigmoidSensor;
            agentPos = [l/4, l/4, l/4];
            collisionGeometry = spherical;
            agents = {agent};

            for ii = 1:size(params.timestep, 1)
                % Set up square domain
                tc.domain = tc.domain.initialize([zeros(1, 3); l * ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");
                tc.domain.objective = tc.domain.objective.initialize(objectiveFunctionWrapper([.75 * l, 0.75 * l]), tc.domain, params.discretizationStep(ii), tc.protectedRange, params.sensorPerformanceMinimum(ii));
                
                % Set up agent
                sensorModel = sensorModel.initialize(params.alphaDist(ii), params.betaDist(ii), params.alphaTilt(ii), params.betaTilt(ii));
                collisionGeometry = collisionGeometry.initialize(agentPos, params.collisionRadius(ii), REGION_TYPE.COLLISION, "Agent 1 Collision Region");
                agents{1} = agents{1}.initialize(agentPos, collisionGeometry, sensorModel, params.comRange(ii), params.maxIter(ii), params.initialStepSize(ii), "Agent 1", tc.plotCommsGeometry);
    
                % Set up simulation
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