function R = plotPermutationResults(resultsFolder, opts)
% plotPermutationResults  Thesis plots for a test_permutations result set.
%
%   R = plotPermutationResults()                 % newest sandbox/perm_* folder
%   R = plotPermutationResults("sandbox/perm_2026_11_11_15_12_16")
%   R = plotPermutationResults(folder, Export=false, SensingCase="N5_...")
%
% Figures (vector PDFs in opts.OutDir via exportThesisFig):
%   perm_perf_sensing / perm_perf_service
%       Normalized system performance vs time, all runs of each modality.
%   perm_case_perf_sensing / perm_case_perf_service
%       System and per-UAV performance vs time for one chosen case.
%   perm_case_constraints_sensing / perm_case_constraints_service
%       Per-step minimum margin of each constraint class for the chosen case.
%   perm_constraint_box
%       Box plot of every margin value at every step across all runs, per class.
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
%
% Returns a table R with one row per run (parsed permutation, time vector,
% system and per-UAV performance, and per-step margins by class).
    arguments
        resultsFolder (1, 1) string = newestResults();
        opts.OutDir (1, 1) string = "";
        opts.SensingCase (1, 1) string = "N5_double_obj2_sensing_longNarrow_close4";
        opts.ServiceCase (1, 1) string = "N5_double_obj2_service_longNarrow_close4";
        opts.Export (1, 1) logical = true;
    end
    if opts.OutDir == ""
        opts.OutDir = fullfile(resultsFolder, "plots");
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
            exportThesisFig(f, "perm_perf_" + models(mm), 3/4, "Recolor", false, "LineWidth", 0.75, "OutDir", opts.OutDir);
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
        hSys = plot(ax, R.t{rr}, R.perf{rr}, "k-", "LineWidth", 2.5);
        hSys.SeriesIndex = "none"; % keep black/thick through exportThesisFig
        legend(ax, ["UAV " + (1:nA), "System"], "Location", "best");
        xlabel(ax, "Time (s)"); ylabel(ax, perfLabel(mm));
        title(ax, caseTitle(R(rr, :)));
        if opts.Export
            exportThesisFig(f, "perm_case_perf_" + models(mm), 3/4, "LineWidth", 1.25, "OutDir", opts.OutDir);
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
            exportThesisFig(f, "perm_case_constraints_" + models(mm), 3/4, "LineWidth", 1.25, "OutDir", opts.OutDir);
        end
    end

    % Plot 4: every margin value at every step, all runs, per class
    vals = {cellfun(@(x) x(:), R.domain, "UniformOutput", false), ...
            cellfun(@(x) x(:), R.collision, "UniformOutput", false), ...
            cellfun(@(x) x(:), R.comms, "UniformOutput", false)};
    y = []; g = strings(0, 1);
    for cc = 1:3
        v = vertcat(vals{cc}{:});
        v = v(~isnan(v));
        y = [y; v]; %#ok<AGROW>
        g = [g; repmat(classes(cc), numel(v), 1)]; %#ok<AGROW>
        fprintf("%-15s n = %7d  min %8.3f  mean %8.2f  max %8.2f\n", classes(cc), numel(v), min(v), mean(v), max(v));
    end
    gc = categorical(g, classes);
    f = figure("Name", "Constraint margins, all runs");
    ax = axes(f);
    % MATLAB box chart (box = quartiles, line = median) with whiskers spanning
    % the full min-max range and no outlier markers. boxchart has no
    % whisker-length option, so its outlier markers and 1.5 IQR whiskers are
    % hidden and min-max whiskers are drawn with matching lines.
    x = double(gc);
    b = boxchart(ax, x, y, "MarkerStyle", "none");
    whiskerColor = b.WhiskerLineColor;
    b.WhiskerLineStyle = "none"; % drop the 1.5 IQR whiskers; min-max ones drawn below
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
    ylabel(ax, "Margin (m; dB for comms)");
    title(ax, sprintf("Constraint margins, all %d runs, every step", height(R)));
    if opts.Export
        exportThesisFig(f, "perm_constraint_box", 3/4, "OutDir", opts.OutDir);
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

    [ii, jj] = find(triu(true(nA), 1)); % column-major upper triangle
    rSum = r(ii) + r(jj);
    collision = sqrt(clampKeepNaN(B(1:nAA, :) + rSum.^2, 0)) - rSum;

    idx = nAA + (1:(nA * nO));
    rObs = repmat(r, nO, 1);
    obstacle = sqrt(clampKeepNaN(B(idx, :) + rObs.^2, 0)) - rObs;

    idx = nAA + nA * nO + (1:(6 * nA));
    geofence = B(idx, :);
    domain = [obstacle; geofence];

    % Recover SINR from h (gammaCbf includes paddingDb from constrainMotion),
    % then reference it to the user-specified link threshold
    gammaCbf = 10^((in.sinrThreshold + 3.0) / 10);
    idx = nAA + nA * nO + 6 * nA + (1:(2 * nAA));
    comms = 10 * log10(clampKeepNaN(B(idx, :) + gammaCbf, realmin)) - in.sinrThreshold;
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
