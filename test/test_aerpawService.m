classdef test_aerpawService < matlab.unittest.TestCase
    % Runs the 2-UAV wireless-service AERPAW scenarios in miSim, through the
    % same CSV path the AERPAW controller uses on MATLAB
    % (miSim.initializeFromCsv). The CSVs are written by
    % aerpaw/scripts/makeServiceScenarios.m: the paper's layouts L1-L3 with
    % densities U1-U3, UAV 1 wide beam, UAV 2 narrow beam, nadir antennas,
    % single integrator, SINR backhaul, 100 m ceiling.
    %
    % Results go to sandbox/aerpawService_<timestamp>/<scenario>_miSim{Inits,Hist}.mat.
    % miSim.validate() checks connectivity and obstacle clearance at every
    % step; this test also checks the final graph and the agent-agent
    % clearance over the whole trajectory.
    %
    % Run all three:
    %   runtests("test/test_aerpawService.m")
    properties (TestParameter)
        scenario = struct("L1U1", "L1U1", "L2U2", "L2U2", "L3U3", "L3U3");
    end

    properties (Access = private)
        suiteTimestamp string = "";
    end

    methods (TestClassSetup)
        function setTimestamp(tc)
            tc.suiteTimestamp = string(datetime("now", "Format", "yyyy_MM_dd_HH_mm_ss"));
        end
    end

    methods (Test)
        function service(tc, scenario)
            root = matlab.project.rootProject().RootFolder;
            csv = fullfile(root, "aerpaw", "config", "scenario_service_" + scenario + ".csv");
            folder = "aerpawService_" + tc.suiteTimestamp;
            if ~isfolder(fullfile(root, "sandbox", folder))
                mkdir(fullfile(root, "sandbox", folder));
            end

            sim = miSim;
            sim = sim.initializeFromCsv(csv);
            tc.assertTrue(all(cellfun(@(a) isa(a.sensorModel, "rfSensor"), sim.agents)));
            n = numel(sim.agents);
            tc.assertEqual(max(conncomp(graph(sim.constraintAdjacencyMatrix))), 1);

            sim.artifactName = folder + "/" + scenario;
            sim.writeInits();
            sim = sim.run();
            tc.assertEqual(max(conncomp(graph(sim.constraintAdjacencyMatrix))), 1);
            sim.teardown();

            hist = load(fullfile(root, "sandbox", sim.artifactName + "_miSimHist.mat")).out;
            inits = load(fullfile(root, "sandbox", sim.artifactName + "_miSimInits.mat"));
            for ii = 1:(n - 1)
                for jj = (ii + 1):n
                    d2 = sum((hist.agent(ii).pos - hist.agent(jj).pos).^2, 2);
                    rSum = hist.agent(ii).collisionRadius + hist.agent(jj).collisionRadius;
                    tc.verifyGreaterThanOrEqual(min(d2), rSum^2 - 1e-3, ...
                        sprintf("Agents %d and %d collision spheres overlap", ii, jj));
                end
            end
            perf = hist.perf / inits.objectiveIntegral;
            fprintf("%s: normalized C* %.2f dB at start, %.2f dB at the end (peak %.2f)\n", ...
                scenario, perf(find(~isnan(perf), 1)), perf(end), max(perf));
        end
    end
end
