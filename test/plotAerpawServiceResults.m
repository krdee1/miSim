function R = plotAerpawServiceResults(folder, opts)
% plotAerpawServiceResults  Plots of the 2-UAV wireless-service AERPAW
% scenarios simulated in miSim by test/test_aerpawService.m.
%
%   R = plotAerpawServiceResults()                   % newest sandbox/aerpawService_*
%   R = plotAerpawServiceResults(folder, Export=false)
%
% Three figures per scenario (L1U1, L2U2, L3U3), styled and saved by
% util/exportThesisFig.m in opts.OutDir, with no title (the LaTeX caption
% gives it):
%   aerpawservice_perf_<scn>
%       Service coverage vs time: each UAV's contribution (labeled by its
%       antenna) and the fleet C*, normalized by objectiveIntegral so the
%       fleet curve is the density-weighted mean user SINR in dB.
%   aerpawservice_margins_<scn>
%       Smallest margin in the fleet vs time per constraint class:
%       geofence (walls, ceiling, minimum altitude, obstacles; m),
%       collision (gap between safety spheres; m), backhaul (weaker
%       direction of the maintained link minus the SINR threshold; dB).
%   aerpawservice_traj_<scn>
%       Top view: normalized user density, obstacles labeled with their
%       heights, the UAV tracks with start (circle) and end (square).
%
% The scenario (density, obstacles, radii, link model) is rebuilt from the
% CSV that was simulated, aerpaw/config/scenario_service_<scn>.csv.
arguments
    folder (1, 1) string = newestFolder()
    opts.Scenarios (1, :) string = ["L1U1", "L2U2", "L3U3"]
    opts.OutDir (1, 1) string = fullfile(matlab.project.rootProject().RootFolder, "sandbox", ...
        "figures_" + string(datetime("now", "Format", "yyyy_MM_dd")) + "_aerpawService")
    opts.Export (1, 1) logical = true
end
root = matlab.project.rootProject().RootFolder;
if opts.Export && ~isfolder(opts.OutDir)
    mkdir(opts.OutDir);
end
colors = okabeIto();
antennaNames = @(rf) arrayfun(@(d) string(sprintf("$\\delta_n = %d$", d)), rf);

R = struct("scenario", {}, "t", {}, "perf", {}, "agentPerf", {}, "geofence", {}, "collision", {}, "backhaul", {});
for ss = 1:numel(opts.Scenarios)
    scn = opts.Scenarios(ss);
    csv = fullfile(root, "aerpaw", "config", "scenario_service_" + scn + ".csv");
    p = readScenarioCsv(csv);
    sim = miSim;
    sim = sim.initializeFromCsv(csv);
    h = load(fullfile(folder, scn + "_miSimHist.mat")).out;
    in = load(fullfile(folder, scn + "_miSimInits.mat"));
    nA = numel(h.agent);
    X = cat(3, h.agent.pos);              % N x 3 x nA, rows k = 0..N-1
    N = size(X, 1);
    t = (0:N - 1) * p.timestep;

    % Performance: stored column k + 1 is measured at position row k
    agentPerf = cell2mat(arrayfun(@(a) a.perf(2:N), h.agent(:), "UniformOutput", false)) / in.objectiveIntegral;
    perf = h.perf(2:N) / in.objectiveIntegral;
    tp = t(1:N - 1);

    % Margins
    r = p.collisionRadius;
    obsMin = reshape(p.obstacleMin, 3, [])';
    obsMax = reshape(p.obstacleMax, 3, [])';
    geofence = Inf(1, N);
    for ii = 1:nA
        Xi = X(:, :, ii);
        walls = [Xi(:, 1) - p.domainMin(1), p.domainMax(1) - Xi(:, 1), ...
                 Xi(:, 2) - p.domainMin(2), p.domainMax(2) - Xi(:, 2), ...
                 Xi(:, 3) - p.minAlt, p.domainMax(3) - Xi(:, 3)] - r(ii);
        obs = zeros(N, size(obsMin, 1));
        for oo = 1:size(obsMin, 1)
            q = min(max(Xi, obsMin(oo, :)), obsMax(oo, :));
            obs(:, oo) = vecnorm(Xi - q, 2, 2) - r(ii);
        end
        geofence = min(geofence, min([walls, obs], [], 2)');
    end
    collision = Inf(1, N);
    backhaul = Inf(1, N);
    for ii = 1:(nA - 1)
        for jj = (ii + 1):nA
            collision = min(collision, vecnorm(X(:, :, ii) - X(:, :, jj), 2, 2)' - (r(ii) + r(jj)));
            for kk = 1:N
                adj = h.constraintAdjacency(:, :, min(kk, size(h.constraintAdjacency, 3)));
                if adj(ii, jj)
                    Pk = squeeze(X(kk, :, :))';
                    g = min(sim.sinrLink(Pk, ii, jj), sim.sinrLink(Pk, jj, ii));
                    backhaul(kk) = min(backhaul(kk), 10 * log10(g) - p.sinrThreshold);
                end
            end
        end
    end
    fprintf("%s: final C* %.2f dB (UAV 1 %.2f, UAV 2 %.2f); min margins: geofence %.3f m, collision %.2f m, backhaul %.2f dB\n", ...
        scn, perf(end), agentPerf(1, end), agentPerf(2, end), min(geofence), min(collision), min(backhaul));
    R(ss) = struct("scenario", scn, "t", t, "perf", perf, "agentPerf", agentPerf, ...
        "geofence", geofence, "collision", collision, "backhaul", backhaul);

    % --- Performance over time
    f = figure("Name", "Service " + scn);
    ax = axes(f);
    hold(ax, "on");
    names = "UAV " + (1:nA)' + " (" + antennaNames(p.beamwidthExponent(:)) + ")";
    for ii = 1:nA
        plot(ax, tp, agentPerf(ii, :), "Color", colors(ii + 1, :), "SeriesIndex", "none", "DisplayName", names(ii));
    end
    plot(ax, tp, perf, "k", "SeriesIndex", "none", "DisplayName", "Fleet");
    xlabel(ax, "Time [s]");
    ylabel(ax, "$C^\star$ [dB]");
    ylim(ax, [max(-60, min([perf, agentPerf(:)'])), max(perf) + 2]);
    xlim(ax, [0, tp(end)]);
    grid(ax, "on");
    legend(ax, "Location", "southeast");
    finish(f, "aerpawservice_perf_" + lower(scn), opts);

    % --- Margins over time
    f = figure("Name", "Margins " + scn);
    ax = axes(f);
    hold(ax, "on");
    plot(ax, t, geofence, "Color", colors(1, :), "SeriesIndex", "none", "DisplayName", "Geofence [m]");
    plot(ax, t, collision, "Color", colors(2, :), "SeriesIndex", "none", "DisplayName", "Collision [m]");
    plot(ax, t, backhaul, "Color", colors(3, :), "SeriesIndex", "none", "DisplayName", "Backhaul [dB]");
    yline(ax, 0, "k:", "HandleVisibility", "off");
    xlabel(ax, "Time [s]");
    ylabel(ax, "Minimum margin");
    xlim(ax, [0, t(end)]);
    grid(ax, "on");
    legend(ax, "Location", "northeast");
    finish(f, "aerpawservice_margins_" + lower(scn), opts);

    % --- Top view
    f = figure("Name", "Tracks " + scn);
    ax = axes(f);
    obj = sim.domain.objective;
    imagesc(ax, obj.X(1, :), obj.Y(:, 1), obj.values);
    set(ax, "YDir", "normal");
    hold(ax, "on");
    axis(ax, "equal");
    colormap(ax, "parula");
    cb = colorbar(ax);
    cb.Label.String = "Normalized $\phi$";
    cb.Label.Interpreter = "latex";
    for oo = 1:size(obsMin, 1)
        full = obsMax(oo, 3) >= p.domainMax(3);
        rectangle(ax, "Position", [obsMin(oo, 1:2), obsMax(oo, 1:2) - obsMin(oo, 1:2)], ...
            "FaceColor", (full * 0.25 + ~full * 0.7) * [1, 1, 1], "EdgeColor", "k");
        text(ax, mean([obsMin(oo, 1), obsMax(oo, 1)]), mean([obsMin(oo, 2), obsMax(oo, 2)]), ...
            sprintf("%g", obsMax(oo, 3)), "HorizontalAlignment", "center", "Color", full * [1, 1, 1], "FontSize", 7);
    end
    for ii = 1:nA
        plot(ax, X(:, 1, ii), X(:, 2, ii), "-", "Color", colors(ii + 1, :), "SeriesIndex", "none", "DisplayName", names(ii));
        plot(ax, X(1, 1, ii), X(1, 2, ii), "o", "Color", colors(ii + 1, :), "MarkerFaceColor", "w", "SeriesIndex", "none", "HandleVisibility", "off");
        plot(ax, X(end, 1, ii), X(end, 2, ii), "s", "Color", colors(ii + 1, :), "MarkerFaceColor", colors(ii + 1, :), "SeriesIndex", "none", "HandleVisibility", "off");
    end
    xlim(ax, [p.domainMin(1), p.domainMax(1)]);
    ylim(ax, [p.domainMin(2), p.domainMax(2)]);
    xlabel(ax, "$x$ [m]");
    ylabel(ax, "$y$ [m]");
    legend(ax, "Location", "southeast");
    finish(f, "aerpawservice_traj_" + lower(scn), opts);
end
end

function finish(f, name, opts)
if opts.Export
    exportThesisFig(f, name, 3/4, Recolor=false, OutDir=opts.OutDir);
end
end

function folder = newestFolder()
d = dir(fullfile(matlab.project.rootProject().RootFolder, "sandbox", "aerpawService_*"));
d = d([d.isdir]);
assert(~isempty(d), "No sandbox/aerpawService_* results folder; run test/test_aerpawService.m first");
[~, k] = max([d.datenum]);
folder = fullfile(d(k).folder, d(k).name);
end
