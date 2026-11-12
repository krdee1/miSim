function R = plotPermutationResults(resultsFolder, opts)
% plotPermutationResults  Thesis plots for a test_permutations result set.
%
%   R = plotPermutationResults()                 % newest sandbox/perm_* folder
%   R = plotPermutationResults("sandbox/perm_2026_11_11_15_12_16")
%   R = plotPermutationResults(folder, Export=false, SensingCase="N5_...")
%
% Figures, styled and saved by util/exportThesisFig.m in opts.OutDir as
% vector PDFs, except sweepobjectives, which is a 400 dpi PNG. Names follow the thesis figures/ convention (lowercase, no
% separators), ready for \includegraphics{figures/<name>}:
%   sweepperfsensing, sweepperfservice
%       Normalized system performance vs time, all runs of each modality.
%   sweepcaseperfsensing, sweepcaseperfservice
%       System and per-UAV performance vs time for the chosen case.
%   sweepcasemarginssensing, sweepcasemarginsservice
%       Per-step minimum margin of each constraint class for the chosen case.
%   sweepmarginsboxsensing, sweepmarginsboxservice
%       Box plot of each UAV's HNCBF margin (min over its constituents) at
%       every step, per class, all runs of each modality.
%   sweepobstacles
%       1x3 3-D views of the domain with each obstacle layout.
%   sweepobjectives
%       1x3 top-down views of each objective with white iso-curves (0.1, 0.5,
%       0.9 of the peak), and the UAVs' starting positions as red circles of
%       their collision radius. The first panel labels
%       the starting altitudes via leader lines (the starts are identical
%       across panels).
%
% Performance is normalized the same way as the SPAWC paper plots
% (plots_1_2.m, plots_3_4.m): the simulator's total, sum(f .* S) over the
% grid, divided by objectiveIntegral = sum(f), where f is the objective
% scaled to a peak of 1. For sensing this is sensor quality averaged over the
% objective, in [0, 1]; for service it is objective-weighted SINR in dB.
% Per-UAV values use the same divisor and sum to the system value.
%
% Constraint values are physical margins, 0 at the constraint boundary:
%   Domain (geofence walls + obstacles, incl. the min-altitude floor) and
%   Collision: clearance in m beyond the agents' collision spheres.
%   Communications: SINR in dB above the link threshold sinrThreshold
%   (0 = link lost), maintained links only. The CBF itself holds maintained
%   links 3 dB higher (paddingDb in @miSim/constrainMotion.m), so values
%   between 0 and 3 dB are connected links inside that safety padding.
% Each class is stored per UAV as its HNCBF value, the min over that UAV's
% constituents in these physical units: geofence walls and obstacles
% (eq:obstaclenbf), every pair it belongs to (eq:collisionnbf), and both
% directions of each of its maintained links (eq:linknbf). The minimum is
% taken over margins in m (or dB), not over the raw h values, which mix
% linear and squared distances.
%
% Returns a table R with one row per run (parsed permutation, time vector,
% system and per-UAV performance, and per-step per-UAV HNCBF margins by
% class, each nA x steps).
    arguments
        resultsFolder (1, 1) string = newestResults();
        opts.OutDir (1, 1) string = "";
        opts.SensingCase (1, 1) string = "N5_double_obj2_sensing_longNarrow_close4";
        opts.ServiceCase (1, 1) string = "N5_double_obj2_service_longNarrow_close4";
        opts.Export (1, 1) logical = true;
    end
    if opts.OutDir == ""
        opts.OutDir = fullfile(resultsFolder, "figures");
    end
    if opts.Export && ~isfolder(opts.OutDir)
        mkdir(opts.OutDir);
    end

    R = loadResults(resultsFolder);
    fprintf("Loaded %d runs from %s\n", height(R), resultsFolder);
    sanityChecks(R);

    models = ["sensing", "service"];
    caseNames = [opts.SensingCase, opts.ServiceCase];
    perfLabel = ["Performance (normalized)", "Performance (normalized, dB)"];
    palette = okabeIto();

    % Plot 1: system performance vs time, all runs of each modality
    for mm = 1:2
        f = figure("Name", "Performance, all " + models(mm) + " runs");
        ax = axes(f); hold(ax, "on"); grid(ax, "on");
        rows = find(R.model == models(mm));
        objKeys = ["obj1", "obj2", "obj3"];
        hLegend = gobjects(1, 3);
        for rr = rows'
            k = find(objKeys == R.objective(rr));
            h = plot(ax, R.t{rr}, R.perf{rr}, "Color", palette(k, :));
            if ~isgraphics(hLegend(k))
                hLegend(k) = h;
            end
        end
        legend(ax, hLegend, ["1 objective", "2 objectives", "3 objectives"], "Location", "best");
        xlabel(ax, "Time (s)"); ylabel(ax, perfLabel(mm));
        title(ax, "System performance, all " + models(mm) + " runs");
        if opts.Export
            % thin lines: 36 overlaid runs per figure
            exportThesisFig(f, "sweepperf" + models(mm), 3/4, "Recolor", false, "LineWidth", 0.75, "OutDir", opts.OutDir);
        end
    end

    % Plot 2: system and per-UAV performance for the chosen cases
    for mm = 1:2
        rr = caseRow(R, caseNames(mm));
        f = figure("Name", "Performance, " + caseNames(mm));
        ax = axes(f); hold(ax, "on"); grid(ax, "on");
        nA = size(R.agentPerf{rr}, 1);
        for aa = 1:nA
            plot(ax, R.t{rr}, R.agentPerf{rr}(aa, :));
        end
        hSys = plot(ax, R.t{rr}, R.perf{rr}, "k-", "LineWidth", 3);
        hSys.SeriesIndex = "none"; % keep black/thick through exportThesisFig
        legend(ax, ["UAV " + (1:nA), "System"], "Location", "best");
        xlabel(ax, "Time (s)"); ylabel(ax, perfLabel(mm));
        title(ax, caseTitle(R(rr, :)));
        if opts.Export
            exportThesisFig(f, "sweepcaseperf" + models(mm), 3/4, "OutDir", opts.OutDir);
        end
    end

    % Plot 3: per-step minimum margin of each constraint class, chosen cases
    classes = ["Domain", "Collision", "Communications"];
    for mm = 1:2
        rr = caseRow(R, caseNames(mm));
        f = figure("Name", "Constraint margins, " + caseNames(mm));
        ax = axes(f); hold(ax, "on"); grid(ax, "on");
        plot(ax, R.t{rr}, min(R.domain{rr}, [], 1, "omitnan"));
        plot(ax, R.t{rr}, min(R.collision{rr}, [], 1, "omitnan"));
        plot(ax, R.t{rr}, min(R.comms{rr}, [], 1, "omitnan"));
        hZero = yline(ax, 0, "k--");
        hZero.SeriesIndex = "none";
        hZero.HandleVisibility = "off";
        legend(ax, classes + [" (m)", " (m)", " (dB)"], "Location", "best");
        xlabel(ax, "Time (s)"); ylabel(ax, "Minimum margin (m; dB for comms)");
        title(ax, caseTitle(R(rr, :)));
        if opts.Export
            exportThesisFig(f, "sweepcasemargins" + models(mm), 3/4, "OutDir", opts.OutDir);
        end
    end

    % Plot 4: every UAV's HNCBF margin at every step, per class, one figure
    % per modality
    for mm = 1:2
        rows = R.model == models(mm);
        vals = {R.domain(rows), R.collision(rows), R.comms(rows)};
        y = []; x = [];
        fprintf("%s:\n", models(mm));
        for cc = 1:3
            v = cell2mat(cellfun(@(m) m(:), vals{cc}, "UniformOutput", false));
            v = v(~isnan(v));
            y = [y; v]; %#ok<AGROW>
            x = [x; cc * ones(numel(v), 1)]; %#ok<AGROW>
            fprintf("  %-15s n = %7d  min %8.3f  mean %8.2f  max %8.2f\n", classes(cc), numel(v), min(v), mean(v), max(v));
        end
        f = figure("Name", "Constraint margins, all " + models(mm) + " runs");
        ax = axes(f);
        % MATLAB box chart (box = quartiles, line = median) with whiskers
        % spanning the full min-max range and no outlier markers. boxchart has
        % no whisker-length option, so its outlier markers and 1.5 IQR
        % whiskers are hidden and min-max whiskers are drawn with matching lines.
        b = boxchart(ax, x, y, "MarkerStyle", "none");
        whiskerColor = b.WhiskerLineColor;
        b.WhiskerLineStyle = "none";
        hold(ax, "on");
        capHalf = b.BoxWidth / 4;
        for cc = 1:3
            v = y(x == cc);
            q = quantile(v, [0.25, 0.75]);
            lo = min(v); hi = max(v);
            hw = plot(ax, [cc, cc, NaN, cc, cc, NaN, cc - capHalf, cc + capHalf, NaN, cc - capHalf, cc + capHalf], ...
                          [lo, q(1), NaN, q(2), hi, NaN, lo, lo, NaN, hi, hi], ...
                      "Color", whiskerColor, "LineWidth", b.LineWidth);
            hw.SeriesIndex = "none"; % keep box-chart styling through exportThesisFig
        end
        hold(ax, "off");
        xticks(ax, 1:3); xticklabels(ax, classes); xlim(ax, [0.5, 3.5]);
        grid(ax, "on"); % after boxchart, which resets the axes
        ylabel(ax, "Per-UAV HNCBF margin (m; dB for comms)");
        title(ax, sprintf("HNCBF margins, every UAV and step, all %d %s runs", nnz(rows), models(mm)));
        if opts.Export
            exportThesisFig(f, "sweepmarginsbox" + models(mm), 3/4, "OutDir", opts.OutDir);
        end
    end
    % Scenario overview: obstacle layouts (3-D) and objectives with UAV starts
    scenarioFigures(R, resultsFolder, opts);

    if opts.Export
        fprintf("Saved figures to %s\n", opts.OutDir);
    end
end

function folder = newestResults()
    root = matlab.project.rootProject().RootFolder;
    d = dir(fullfile(root, "sandbox", "perm_*"));
    d = d([d.isdir]);
    assert(~isempty(d), "No sandbox/perm_* result folders found");
    [~, i] = max([d.datenum]);
    folder = string(fullfile(d(i).folder, d(i).name));
end

function R = loadResults(folder)
    files = dir(fullfile(folder, "*_miSimHist.mat"));
    assert(~isempty(files), "No _miSimHist.mat files in %s", folder);
    n = numel(files);
    [name, nAgents, dynamics, objective, model, reach, obstacles] = deal(strings(n, 1));
    [t, perf, agentPerf, domain, collision, comms] = deal(cell(n, 1));
    for k = 1:n
        name(k) = erase(string(files(k).name), "_miSimHist.mat");
        tok = regexp(name(k), "^N(\d+)_(single|double)_(obj\d)_(sensing|service)_(shortWide|longNarrow)_(\w+)$", "tokens", "once");
        assert(~isempty(tok), "Unrecognized result name %s", name(k));
        [nAgents(k), dynamics(k), objective(k), model(k), reach(k), obstacles(k)] = tok{:};

        out = load(fullfile(folder, files(k).name)).out;
        in = load(fullfile(folder, name(k) + "_miSimInits.mat"));

        % Element 1 of perf and of each barrier column is a t = 0
        % placeholder (perf starts at 0; teardown prepends a zero barrier
        % column). Element k >= 2 is measured at t = (k - 2) * timestep.
        keep = 2:numel(out.perf);
        t{k} = (keep - 2) * in.timestep;
        perf{k} = out.perf(keep) / in.objectiveIntegral; % SPAWC normalization
        agentPerf{k} = cell2mat(arrayfun(@(a) a.perf(keep), out.agent(:), "UniformOutput", false)) / in.objectiveIntegral;
        [domain{k}, collision{k}, comms{k}] = barrierMargins(out.barriers(:, keep), in);
    end
    R = table(name, nAgents, dynamics, objective, model, reach, obstacles, t, perf, agentPerf, domain, collision, comms);
end

function [domain, collision, comms] = barrierMargins(B, in)
    % Row layout written by @miSim/constrainMotion.m:
    %   1. collision: nAA rows, upper-triangle pairs in column-major order,
    %      h = d^2 - (ri + rj)^2
    %   2. obstacles: nA * nO rows, reshape(hObs, [], 1) (agent fastest),
    %      h = d^2 - ri^2; nO includes the min-altitude floor obstacle
    %   3. geofence: 6 rows per agent (x/y/z min/max), h = clearance - ri
    %   4. comms: 2 * nAA rows (upper triangle, then lower triangle),
    %      h = SINR - gammaCbf (linear), NaN for unmaintained links
    nA = in.numAgents;
    nAA = nA * (nA - 1) / 2;
    nO = in.numObstacles + (in.minAlt > 0);
    r = in.collisionRadius(:);
    assert(size(B, 1) == nAA + nA * nO + 6 * nA + 2 * nAA, ...
        "Barrier rows (%d) do not match the expected layout (%d)", size(B, 1), nAA + nA * nO + 6 * nA + 2 * nAA);

    %
    % Each constituent is first converted to a physical margin, then reduced
    % to the per-UAV HNCBF value of its class (min over that UAV's
    % constituents), so every output is nA x steps.
    [ii, jj] = find(triu(true(nA), 1)); % column-major upper triangle
    rSum = r(ii) + r(jj);
    pairOwner = (1:nA)' == ii' | (1:nA)' == jj'; % nA x nAA: UAVs in each pair
    collisionPairs = sqrt(clampKeepNaN(B(1:nAA, :) + rSum.^2, 0)) - rSum;
    collision = ownerMin(collisionPairs, pairOwner);

    idx = nAA + (1:(nA * nO));
    rObs = repmat(r, nO, 1);
    obstacle = sqrt(clampKeepNaN(B(idx, :) + rObs.^2, 0)) - rObs;
    obstacleOwner = (1:nA)' == repmat(1:nA, 1, nO); % agent index varies fastest

    idx = nAA + nA * nO + (1:(6 * nA));
    geofence = B(idx, :);
    geofenceOwner = (1:nA)' == repelem(1:nA, 6); % 6 consecutive walls per agent

    % Geofencing HNCBF (eq:obstaclenbf): walls and obstacles together
    domain = ownerMin([obstacle; geofence], [obstacleOwner, geofenceOwner]);

    % Recover SINR from h (gammaCbf includes paddingDb from constrainMotion),
    % then reference it to the user-specified link threshold
    gammaCbf = 10^((in.sinrThreshold + 3.0) / 10);
    idx = nAA + nA * nO + 6 * nA + (1:(2 * nAA));
    commsDirs = 10 * log10(clampKeepNaN(B(idx, :) + gammaCbf, realmin)) - in.sinrThreshold;
    % The lower-triangle block is in its own column-major order, so its pairs
    % differ from the upper block's. Both directions of a maintained link
    % count toward both endpoints' HNCBF (eq:linknbf).
    [iiL, jjL] = find(tril(true(nA), -1));
    lowerOwner = (1:nA)' == iiL' | (1:nA)' == jjL';
    comms = ownerMin(commsDirs, [pairOwner, lowerOwner]);
end

function M = ownerMin(C, owner)
    % Per-UAV minimum over constituents. C is constituents x steps, owner is
    % nA x constituents. Unlogged (NaN) entries are ignored; a UAV with no
    % logged constituent at a step stays NaN.
    M = NaN(size(owner, 1), size(C, 2));
    for a = 1:size(owner, 1)
        if any(owner(a, :))
            M(a, :) = min(C(owner(a, :), :), [], 1, "omitnan");
        end
    end
end

function x = clampKeepNaN(x, lo)
    % max(x, lo) without max()'s NaN-dropping: unlogged entries (unmaintained
    % links, steps where the safety filter returned early) must stay NaN
    x(x < lo) = lo;
end

function sanityChecks(R)
    worstSum = 0;
    for k = 1:height(R)
        worstSum = max(worstSum, max(abs(sum(R.agentPerf{k}, 1) - R.perf{k}), [], "all"));
    end
    fprintf("Per-UAV performance sums to system within %.2e\n", worstSum);
    classes = ["domain", "collision", "comms"];
    for cc = classes
        m = cellfun(@(x) min(x, [], "all", "omitnan"), R.(cc));
        [worst, k] = min(m);
        fprintf("Min %-9s margin over all runs: %8.4f (%s)\n", cc, worst, R.name(k));
        bad = R.name(m < -1e-3);
        if ~isempty(bad)
            fprintf("  %d runs below -1e-3: %s\n", numel(bad), strjoin(bad, ", "));
        end
    end
end

function rr = caseRow(R, name)
    rr = find(R.name == name, 1);
    assert(~isempty(rr), "Case %s not found in the result set", name);
end

function s = caseTitle(row)
    dyn = containers.Map(["single", "double"], ["single integrator", "double integrator"]);
    obj = containers.Map(["obj1", "obj2", "obj3"], ["1 objective", "2 objectives", "3 objectives"]);
    rch = containers.Map(["shortWide", "longNarrow"], ["short range, wide", "long range, narrow"]);
    s = sprintf("%s: %s, %s, %s, %s", upper(extractBefore(row.model, 2)) + extractAfter(row.model, 1), ...
        dyn(row.dynamics), obj(row.objective), rch(row.reach), row.obstacles);
end

function scenarioFigures(R, folder, opts)
    % Scenario overview figures, rebuilt from the saved _miSimInits files of
    % one representative run per obstacle layout / objective configuration.
    obsKeys = ["sparse2", "close2", "close4"];
    obsTitles = ["2 sparse obstacles", "2 close obstacles", "4 close obstacles"];
    objKeys = ["obj1", "obj2", "obj3"];
    objTitles = ["1 objective", "2 objectives", "3 objectives"];
    obstacleColor = [0.85, 0.1, 0.1];
    startColor = [0.9, 0, 0];
    leaderColor = [0.7, 0.7, 0.7]; % visible on the dark plot and the white legend

    % 1x3 3-D views of the obstacle layouts
    f = figure("Name", "Obstacle layouts");
    tl = tiledlayout(f, 1, 3, "TileSpacing", "compact", "Padding", "compact");
    for kk = 1:3
        in = loadInits(R, folder, "obstacles", obsKeys(kk));
        ax = nexttile(tl); hold(ax, "on");
        drawBoxEdges(ax, in.domainMin, in.domainMax);
        for oo = 1:size(in.obsMinCorners, 1)
            drawBox(ax, in.obsMinCorners(oo, :), in.obsMaxCorners(oo, :), obstacleColor);
        end
        axis(ax, "equal"); grid(ax, "on"); view(ax, -37.5, 25);
        xlim(ax, [in.domainMin(1), in.domainMax(1)]); ylim(ax, [in.domainMin(2), in.domainMax(2)]); zlim(ax, [in.domainMin(3), in.domainMax(3)]);
        xlabel(ax, "X (m)"); ylabel(ax, "Y (m)"); zlabel(ax, "Z (m)");
        title(ax, obsTitles(kk));
        uniformFontScale(ax);
    end
    if opts.Export
        exportThesisFig(f, "sweepobstacles", 0.36, "Recolor", false, "OutDir", opts.OutDir);
    end

    % 1x3 top-down views of the objectives with UAV starting positions
    f = figure("Name", "Objectives and starting positions");
    tl = tiledlayout(f, 1, 3, "TileSpacing", "compact", "Padding", "compact");
    th = linspace(0, 2 * pi, 64);
    for kk = 1:3
        in = loadInits(R, folder, "objective", objKeys(kk));
        nObj = size(in.objectivePos, 1);
        sigma = reshape(in.objectiveSigma, [nObj, 2, 2]); % guard against singleton squeeze on load
        [X, Y] = meshgrid(in.domainMin(1):in.discretizationStep:in.domainMax(1), in.domainMin(2):in.discretizationStep:in.domainMax(2));
        F = objectiveFunctionWrapper(in.objectivePos, sigma);
        Z = reshape(F(X, Y), size(X));
        Z = Z ./ max(Z, [], "all");
        ax = nexttile(tl); hold(ax, "on");
        contourf(ax, X, Y, Z, 0:0.05:1, "LineStyle", "none");
        colormap(ax, parula); clim(ax, [0, 1]);
        contour(ax, X, Y, Z, [0.1, 0.5, 0.9], "Color", "w", "LineWidth", 0.75); % iso-curves

        % Starting positions: red circle of each UAV's collision radius
        P = in.pos;
        for aa = 1:size(P, 1)
            r = in.collisionRadius(aa);
            hc = plot(ax, P(aa, 1) + r * cos(th), P(aa, 2) + r * sin(th), "-", "Color", startColor, "LineWidth", 0.75);
            hc.SeriesIndex = "none";
        end

        % Starting altitudes, labeled in a column to the right of the start
        % cluster (ordered by Y so the leader lines don't cross). The starts
        % are the same in every panel, so only the first is labeled.
        if kk == 1
            [~, order] = sort(P(:, 2), "descend");
            labelX = max(P(:, 1)) + 28;
            labelY = linspace(max(P(:, 2)) + 48, min(P(:, 2)) - 2, size(P, 1));
            for ll = 1:numel(order)
                aa = order(ll);
                u = [labelX, labelY(ll)] - P(aa, 1:2);
                u = u / norm(u);
                from = P(aa, 1:2) + in.collisionRadius(aa) * u; % leader starts on the circle
                hl = plot(ax, [from(1), labelX - 1], [from(2), labelY(ll)], "-", "Color", leaderColor, "LineWidth", 0.75);
                hl.SeriesIndex = "none";
                % sans-serif, slightly larger than the 10 pt figure text;
                % 'KeepFont' exempts it from exportThesisFig's LaTeX/size reset
                text(ax, labelX, labelY(ll), sprintf("%.0f m", P(aa, 3)), ...
                    "Color", "w", "HorizontalAlignment", "left", "VerticalAlignment", "middle", ...
                    "Interpreter", "none", "FontName", "Helvetica", "FontSize", 11, "Tag", "KeepFont");
            end

            % Legend for the markers and annotations. Proxy series keep their
            % style (SeriesIndex "none") through exportThesisFig.
            hUav = plot(ax, NaN, NaN, "o", "Color", startColor, "MarkerSize", 6, "LineWidth", 0.75);
            hAlt = plot(ax, NaN, NaN, "-", "Color", leaderColor, "LineWidth", 0.75);
            [hUav.SeriesIndex, hAlt.SeriesIndex] = deal("none");
            lgd = legend(ax, [hUav, hAlt], ["Initial UAV position", "Initial altitude"], "Orientation", "horizontal");
            lgd.Layout.Tile = "south"; % inside a panel it would cover the objective
        end
        axis(ax, "equal"); box(ax, "on");
        xlim(ax, [in.domainMin(1), in.domainMax(1)]); ylim(ax, [in.domainMin(2), in.domainMax(2)]);
        xlabel(ax, "X (m)"); ylabel(ax, "Y (m)");
        title(ax, objTitles(kk));
        uniformFontScale(ax);
    end
    cb = colorbar(ax);
    cb.Layout.Tile = "east";
    cb.Label.String = "Objective (normalized)";
    if opts.Export
        % PNG (400 dpi): as a vector PDF, the filled-contour bands show white
        % hairline seams between them in LaTeX-built PDFs
        exportThesisFig(f, "sweepobjectives", 0.38, "Recolor", false, "Raster", true, "OutDir", opts.OutDir);
    end
end

function uniformFontScale(ax)
    % exportThesisFig sets every FontSize to the same value, but labels and
    % titles are rescaled by these multipliers when the axes FontSize is set,
    % and the order of those updates varies between tiles. Pinning them to 1
    % gives every tile identical 10 pt text.
    ax.LabelFontSizeMultiplier = 1;
    ax.TitleFontSizeMultiplier = 1;
end

function in = loadInits(R, folder, field, key)
    rr = find(R.(field) == key, 1);
    assert(~isempty(rr), "No run with %s = %s", field, key);
    in = load(fullfile(folder, R.name(rr) + "_miSimInits.mat"));
end

function h = drawBox(ax, lo, hi, color)
    [vx, vy, vz] = ndgrid([lo(1), hi(1)], [lo(2), hi(2)], [lo(3), hi(3)]);
    faces = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
    h = patch(ax, "Vertices", [vx(:), vy(:), vz(:)], "Faces", faces, ...
        "FaceColor", color, "FaceAlpha", 0.6, "EdgeColor", 0.6 * color);
end

function drawBoxEdges(ax, lo, hi)
    % 12 edges of the box: corner pairs that differ in exactly one coordinate
    [ix, iy, iz] = ndgrid(1:2, 1:2, 1:2);
    idx = [ix(:), iy(:), iz(:)];
    c = [lo; hi];
    for p = 1:8
        for q = (p + 1):8
            if nnz(idx(p, :) ~= idx(q, :)) == 1
                p1 = [c(idx(p, 1), 1), c(idx(p, 2), 2), c(idx(p, 3), 3)];
                p2 = [c(idx(q, 1), 1), c(idx(q, 2), 2), c(idx(q, 3), 3)];
                h = plot3(ax, [p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], "k-", "LineWidth", 0.5);
                h.SeriesIndex = "none"; % thin frame, untouched by exportThesisFig
            end
        end
    end
end
