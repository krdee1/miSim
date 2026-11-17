function R = plotAerpawResults(aerpawRoot, opts)
% plotAerpawResults  Thesis plots comparing AERPAW flights with miSim runs.
%
%   R = plotAerpawResults()                        % all three experiments
%   R = plotAerpawResults(root, Scenarios="walls", Export=false)
%
% For each AERPAW experiment (walls, columns, lock), compares the miSim
% simulation of the scenario (<root>/<scn>/sim_results/miSim) with the
% flight (<root>/<scn>/flight_results). Four figures per experiment, styled
% and saved by util/exportThesisFig.m in opts.OutDir as vector PDFs, with no
% title (the LaTeX caption gives it):
%   aerpawperf<scn>
%       Normalized performance vs time of each UAV (labeled by its sensor)
%       and of the system. miSim solid, AERPAW dashed.
%   aerpawmargins<scn>
%       Minimum HNCBF margin over all UAVs vs time per constraint class
%       (domain/obstacle, collision, communications), in m. miSim solid,
%       AERPAW dashed.
%   aerpawbox<scn>
%       Box plot of every UAV's HNCBF margin at every step, per class, one
%       box per source (quartile boxes, min-max whiskers).
%   aerpawsinr<scn>
%       Flight only: SINR measured by the receiving UAV's radio (logged as
%       "snr"; raw samples and a 1 s median) and the UAV-UAV distance from
%       the GPS logs (right axis) vs time. Restricted to the guidance window,
%       first to last GUIDANCE_TOGGLE in the controller log (no takeoff,
%       flyout, return or landing), with t = 0 at guidance start.
%
% Data and alignment. The flight logs record no performance or margins, so
% both are recomputed from positions with the same code for both sources:
%   miSim:  out.agent(i).pos, rows k = 0..N-1.
%   AERPAW: each UAV's local ENU position as reported to the controller at
%           every guidance iteration ("Position from client c" in the
%           controller log; the reading right after GUIDANCE_TOGGLE is k = 0,
%           "(k/N)" readings follow each move). These are already in the
%           scenario frame and match miSim's iterations one to one.
% Time is algorithm time, t = k * timestep (the flight took ~5-6 s of wall
% clock per iteration). The scenario (objective, sensors, obstacles, radii)
% is rebuilt from aerpaw/config/scenario_<scn>.csv, the file flown; the
% miSim inits files do not store the objective covariance.
%
% Performance: each UAV's sum(S .* F) over its partition cell, exactly as in
% agent.run, divided by objectiveIntegral (SPAWC normalization, as in
% plotPermutationResults). Replaying the miSim positions reproduces the
% stored out.agent(i).perf, which is checked.
%
% Margins, physical m, 0 at the constraint boundary, per UAV (min over its
% constituents):
%   Domain/obstacle: geofence walls and ceiling, the min-altitude floor and
%       every obstacle, clearance beyond the collision sphere.
%   Collision: gap between collision spheres, over every pair it belongs to.
%   Communications: comRange - distance over its maintained links (0 = link
%       lost). The CBF holds links at 0.9 * comRange (constrainMotion), so
%       values from 0 to 0.1 * comRange are inside that padding.
% The miSim values are checked against the stored barrier values.
%
% Returns a struct array R, one element per experiment, with t and, per
% source, perf (system), agentPerf (nA x steps) and the domain, collision and
% comms margins (nA x steps).
    arguments
        aerpawRoot (1, 1) string = "/home/kdee/Desktop/EECS296/AERPAW";
        opts.Scenarios (1, :) string = ["walls", "columns", "lock"];
        opts.OutDir (1, 1) string = "";
        opts.Export (1, 1) logical = true;
    end
    if opts.OutDir == ""
        opts.OutDir = fullfile(aerpawRoot, "figures");
    end
    if opts.Export && ~isfolder(opts.OutDir)
        mkdir(opts.OutDir);
    end

    sources = ["miSim", "AERPAW"];
    styles = ["-", "--"];
    classes = ["Domain/obstacle", "Collision", "Communications"];
    palette = okabeIto();

    for ss = 1:numel(opts.Scenarios)
        scn = opts.Scenarios(ss);
        sc = loadScenario(aerpawRoot, scn);
        N = size(sc.pos{1}, 1);
        t = (0:N - 1) * sc.params.timestep;
        fprintf("== %s: %d UAVs, %d steps\n", scn, sc.nA, N);

        r = struct("scenario", scn, "t", t);
        for src = 1:2
            agentPerf = replayPerformance(sc, sc.pos{src}) / sc.objectiveIntegral;
            [domain, collision, comms] = margins(sc, sc.pos{src});
            r.(sources(src)) = struct("perf", sum(agentPerf, 1), "agentPerf", agentPerf, ...
                "domain", domain, "collision", collision, "comms", comms);
        end
        checkAgainstMiSim(sc, r.miSim);
        for src = 1:2
            m = r.(sources(src));
            fprintf("  %-6s final perf %.4f; min margins (m): domain %.2f, collision %.2f, comms %.2f\n", ...
                sources(src), m.perf(end), min(m.domain, [], "all"), min(m.collision, [], "all"), min(m.comms, [], "all"));
        end
        r.radio = radioData(aerpawRoot, scn);
        R(ss) = r; %#ok<AGROW>

        % Plot 1: per-UAV and system performance, miSim solid, AERPAW dashed
        f = figure("Name", "Performance, " + scn);
        ax = axes(f); hold(ax, "on"); grid(ax, "on");
        colors = [palette(1:sc.nA, :); 0, 0, 0];
        names = [sc.sensorLabels, "System"];
        for src = 1:2
            m = r.(sources(src));
            for aa = 1:sc.nA
                plot(ax, t, m.agentPerf(aa, :), styles(src), "Color", colors(aa, :), "HandleVisibility", "off");
            end
            hSys = plot(ax, t, m.perf, styles(src), "Color", colors(end, :), "LineWidth", 3, "HandleVisibility", "off");
            hSys.SeriesIndex = "none"; % keep black and thick through exportThesisFig
        end
        legendKey(ax, colors, names, styles, sources);
        xlim(ax, t([1, end]));
        xlabel(ax, "Time (s)"); ylabel(ax, "Performance (normalized)");
        if opts.Export
            exportThesisFig(f, "aerpawperf" + scn, 3/4, "Recolor", false, "OutDir", opts.OutDir);
        end

        % Plot 2: minimum margin over all UAVs per class, miSim solid, AERPAW dashed
        f = figure("Name", "Constraint margins, " + scn);
        ax = axes(f); hold(ax, "on"); grid(ax, "on");
        fields = ["domain", "collision", "comms"];
        for src = 1:2
            m = r.(sources(src));
            for cc = 1:3
                plot(ax, t, min(m.(fields(cc)), [], 1), styles(src), "Color", palette(cc, :), "HandleVisibility", "off");
            end
        end
        hZero = yline(ax, 0, "k:", "HandleVisibility", "off");
        hZero.SeriesIndex = "none";
        legendKey(ax, palette(1:3, :), classes, styles, sources);
        xlim(ax, t([1, end]));
        xlabel(ax, "Time (s)"); ylabel(ax, "Minimum HNCBF margin (m)");
        if opts.Export
            exportThesisFig(f, "aerpawmargins" + scn, 3/4, "Recolor", false, "OutDir", opts.OutDir);
        end

        % Plot 3: every UAV's margin at every step per class, one box per source
        f = figure("Name", "Constraint margin distribution, " + scn);
        ax = axes(f);
        offsets = [-0.2, 0.2];
        hBox = gobjects(1, 2);
        yAll = [];
        for src = 1:2
            m = r.(sources(src));
            x = []; y = [];
            for cc = 1:3
                v = m.(fields(cc))(:);
                x = [x; (cc + offsets(src)) * ones(numel(v), 1)]; %#ok<AGROW>
                y = [y; v]; %#ok<AGROW>
            end
            hBox(src) = minMaxBoxchart(ax, x, y, "BoxWidth", 0.35, "BoxFaceColor", palette(src, :));
            yAll = [yAll; y]; %#ok<AGROW>
        end
        xticks(ax, 1:3); xticklabels(ax, classes); xlim(ax, [0.5, 3.5]);
        yLo = 0;
        if min(yAll) < 0
            % Real violations: show them rather than clip at the boundary
            warning("plotAerpawResults:negativeMargin", "%s: margin down to %.3f m, axis extended below 0", scn, min(yAll));
            yLo = floor(min(yAll));
        end
        ylim(ax, [yLo, 10 * ceil(max(yAll) / 10)]);
        grid(ax, "on");
        legend(ax, hBox, sources, "Location", "best");
        ylabel(ax, "HNCBF margin (m)");
        if opts.Export
            exportThesisFig(f, "aerpawbox" + scn, 3/4, "Recolor", false, "OutDir", opts.OutDir);
        end

        % Plot 4: measured link SINR and UAV-UAV distance over the guidance window
        rd = r.radio;
        fprintf("  radio: window %.1f s, %d SINR samples, SINR min/median/max %.1f/%.1f/%.1f dB, distance %.1f-%.1f m\n", ...
            rd.tEnd, numel(rd.t), min(rd.sinr), median(rd.sinr), max(rd.sinr), min(rd.dist), max(rd.dist));
        for uu = 1:2
            fprintf("  radio: %s = client %d, clock offset %+.1f s (GPS vs reported position median %.2f m)\n", ...
                rd.uav(uu), rd.client(uu), rd.clockOffset(uu), rd.matchResidual(uu));
        end
        fprintf("  radio: GPS vs controller-log distance at each iteration, max |diff| %.2f m (median %.2f m)\n", ...
            max(rd.distCheck), median(rd.distCheck));
        f = figure("Name", "SINR and distance, " + scn);
        ax = axes(f); hold(ax, "on");
        yyaxis(ax, "left");
        hRaw = plot(ax, rd.t, rd.sinr, ".", "Color", 0.5 * palette(1, :) + 0.5, "MarkerSize", 2);
        hRaw.SeriesIndex = "none"; % keep the dots small through exportThesisFig
        hMed = plot(ax, rd.tMed, rd.sinrMed, "-", "Color", palette(1, :));
        ylabel(ax, "SINR (dB)");
        ax.YAxis(1).Color = palette(1, :);
        yyaxis(ax, "right");
        hDist = plot(ax, rd.tDist, rd.dist, "-", "Color", palette(2, :));
        ylabel(ax, "Distance (m)");
        ax.YAxis(2).Color = palette(2, :);
        xlim(ax, [0, rd.tEnd]);
        grid(ax, "on");
        xlabel(ax, "Time (s)");
        legend(ax, [hRaw, hMed, hDist], ["SINR", "SINR (1 s median)", "Distance"], "Location", "best");
        if opts.Export
            exportThesisFig(f, "aerpawsinr" + scn, 3/4, "Recolor", false, "OutDir", opts.OutDir);
        end
    end

    if opts.Export
        fprintf("Saved figures to %s\n", opts.OutDir);
    end
end

function sc = loadScenario(root, scn)
    % Scenario definition from the flown CSV, miSim history, and the
    % per-iteration flight positions from the controller log
    repo = matlab.project.rootProject().RootFolder;
    p = readScenarioCsv(fullfile(repo, "aerpaw", "config", "scenario_" + scn + ".csv"));
    sc.params = p;
    sc.nA = numel(p.initialPositions) / 3;

    % Objective, built as in aerpaw/results/resultsAnalysis.m
    domain = rectangularPrism;
    domain = domain.initialize([p.domainMin; p.domainMax], REGION_TYPE.DOMAIN, "Domain");
    if length(p.objectiveVar) > 4 && length(p.objectivePos) > 2
        sigma = permute(reshape(p.objectiveVar, [length(p.objectiveVar) / 4, 2, 2]), [3, 1, 2]);
        mu = reshape(p.objectivePos, [length(p.objectivePos) / 2, 2])';
    else
        sigma = reshape(p.objectiveVar, [1, 2, 2]);
        mu = p.objectivePos;
    end
    domain.objective = domain.objective.initialize(objectiveFunctionWrapper(mu, sigma), domain, ...
        p.discretizationStep, p.protectedRange, p.sensorPerformanceMinimum);
    sc.objective = domain.objective;
    sc.objectiveIntegral = sum(sc.objective.values, "all");

    % Agents (only the sensor model and position are used)
    sc.agents = cell(sc.nA, 1);
    for ii = 1:sc.nA
        sensor = sigmoidSensor;
        sensor = sensor.initialize(p.alphaDist(ii), p.betaDist(ii), p.alphaTilt(ii), p.betaTilt(ii));
        sc.agents{ii} = agent;
        sc.agents{ii}.sensorModel = sensor;
    end
    isLong = p.alphaDist(:)' == max(p.alphaDist);
    sc.sensorLabels = strings(1, sc.nA);
    sc.sensorLabels(isLong) = "Long range, narrow FOV";
    sc.sensorLabels(~isLong) = "Short range, wide FOV";
    sc.radius = p.collisionRadius(:);
    sc.comRange = p.comRange(:);
    sc.obsMin = reshape(p.obstacleMin, 3, [])';
    sc.obsMax = reshape(p.obstacleMax, 3, [])';

    % miSim run
    d = dir(fullfile(root, scn, "sim_results", "miSim", "*_miSimHist.mat"));
    assert(isscalar(d), "Expected one miSim history in %s", scn);
    sc.hist = load(fullfile(d.folder, d.name)).out;
    in = load(fullfile(d.folder, erase(d.name, "Hist.mat") + "Inits.mat"));
    assert(abs(in.objectiveIntegral - sc.objectiveIntegral) < 1e-6 * sc.objectiveIntegral, ...
        "%s: objective rebuilt from the CSV does not match the miSim run", scn);
    N = size(sc.hist.agent(1).pos, 1);
    Pm = zeros(N, 3, sc.nA);
    for ii = 1:sc.nA
        Pm(:, :, ii) = sc.hist.agent(ii).pos;
    end

    % Flight: per-iteration positions from the controller log
    c = dir(fullfile(root, scn, "flight_results", "controller_*", "*_controller_log.txt"));
    assert(isscalar(c), "Expected one controller log in %s", scn);
    Pa = flightPositions(fullfile(c.folder, c.name), sc.nA);
    assert(size(Pa, 1) >= N, "%s: flight log has %d iterations, miSim %d", scn, size(Pa, 1), N);
    assert(all(vecnorm(Pa(1, :, :) - Pm(1, :, :), 2, 2) < 3, "all"), "%s: flight and miSim starts differ", scn);
    sc.pos = {Pm, Pa(1:N, :, :)};
end

function P = flightPositions(logPath, nA)
    % (iterations x 3 x nA) positions reported at each guidance iteration.
    % k = 0 is the reading right after GUIDANCE_TOGGLE (unprefixed), k >= 1
    % the "(k/N)" readings that follow each move.
    lines = string(splitlines(fileread(logPath)));
    first = find(contains(lines, "Sent GUIDANCE_TOGGLE"), 1);
    assert(~isempty(first), "No GUIDANCE_TOGGLE in %s", logPath);
    lines = lines(first:end);
    % The prefix is "(k/N)" in the loop, a clock time for the k = 0 reading
    tok = regexp(lines, "\] (\(\d+/\d+\)|\d\d:\d\d:\d\d) Position from client (\d+): ([^,]+),([^,]+),(\S+)", "tokens", "once");
    hit = ~cellfun(@isempty, tok);
    tok = vertcat(tok{hit});
    k = str2double(regexp(tok(:, 1), "(?<=\()\d+", "match", "once"));
    k(isnan(k)) = 0; % unprefixed reading
    client = str2double(tok(:, 2));
    xyz = str2double(tok(:, 3:5));
    P = NaN(max(k) + 1, 3, nA);
    for rr = 1:numel(k)
        P(k(rr) + 1, :, client(rr)) = xyz(rr, :);
    end
    assert(~any(isnan(P), "all"), "Missing position readings in %s", logPath);
end

function agentPerf = replayPerformance(sc, P)
    % Each UAV's sum(S .* F) over its partition cell at every step, as in
    % agent.run (partition by best sensor, nadir-pointing sigmoid sensors)
    N = size(P, 1);
    obj = sc.objective;
    agents = sc.agents;
    agentPerf = zeros(sc.nA, N);
    for k = 1:N
        for ii = 1:sc.nA
            agents{ii}.pos = P(k, :, ii);
        end
        part = agents{1}.partition(agents, obj);
        for ii = 1:sc.nA
            m = part == ii;
            S = agents{ii}.sensorModel.sensorPerformance(agents{ii}.pos, [obj.X(m), obj.Y(m), zeros(nnz(m), 1)]);
            agentPerf(ii, k) = sum(S(:) .* obj.values(m));
        end
    end
end

function [domain, collision, comms] = margins(sc, P)
    % Per-UAV HNCBF margins in m (nA x steps), 0 at the constraint boundary
    p = sc.params;
    N = size(P, 1);
    [domain, collision, comms] = deal(Inf(sc.nA, N));
    for ii = 1:sc.nA
        X = P(:, :, ii);
        r = sc.radius(ii);
        walls = [X(:, 1) - p.domainMin(1), p.domainMax(1) - X(:, 1), ...
                 X(:, 2) - p.domainMin(2), p.domainMax(2) - X(:, 2), ...
                 X(:, 3) - p.minAlt, p.domainMax(3) - X(:, 3)] - r;
        obs = zeros(N, size(sc.obsMin, 1));
        for oo = 1:size(sc.obsMin, 1)
            q = min(max(X, sc.obsMin(oo, :)), sc.obsMax(oo, :)); % closest point of the box
            obs(:, oo) = vecnorm(X - q, 2, 2) - r;
        end
        domain(ii, :) = min([walls, obs], [], 2)';
    end
    % Every pair is a maintained link in these 2-UAV runs (checked against
    % the miSim constraint adjacency in checkAgainstMiSim)
    for ii = 1:(sc.nA - 1)
        for jj = (ii + 1):sc.nA
            d = vecnorm(P(:, :, ii) - P(:, :, jj), 2, 2)';
            gap = d - (sc.radius(ii) + sc.radius(jj));
            link = min(sc.comRange([ii, jj])) - d;
            collision([ii, jj], :) = min(collision([ii, jj], :), [gap; gap]);
            comms([ii, jj], :) = min(comms([ii, jj], :), [link; link]);
        end
    end
end

function checkAgainstMiSim(sc, m)
    % Replayed performance and geometric margins against the values the
    % miSim run stored. Stored column k + 1 is measured at position row k
    % (column 1 is a placeholder), and the final position row has none.
    h = sc.hist;
    N = size(m.agentPerf, 2);
    stored = cell2mat(arrayfun(@(a) a.perf(2:N), h.agent(:), "UniformOutput", false));
    replay = m.agentPerf(:, 1:N - 1) * sc.objectiveIntegral;
    errPerf = max(abs(replay - stored), [], "all") / max(abs(stored), [], "all");
    assert(errPerf < 1e-9, "Performance replay differs from the miSim run (rel. err %.3g)", errPerf);

    adj = h.constraintAdjacency;
    assert(all(adj(~repmat(eye(sc.nA, "logical"), 1, 1, size(adj, 3))), "all"), ...
        "Not every pair is a maintained link; margins() assumes a full link set");

    % Stored rows: collision pairs, nA * (obstacles + floor) obstacle rows
    % (agent fastest), 6 * nA wall rows, comms pairs
    B = h.barriers(:, 2:N);
    nAA = nchoosek(sc.nA, 2);
    nO = size(sc.obsMin, 1) + 1;
    assert(size(h.barriers, 1) == nAA + sc.nA * nO + 6 * sc.nA + nAA, "Unexpected barrier row layout");
    rSum = sc.radius(1) + sc.radius(2);
    errColl = max(abs(sqrt(B(1, :) + rSum^2) - rSum - m.collision(1, 1:N - 1)));
    rComms = 0.9 * min(sc.comRange);
    errComms = max(abs(min(sc.comRange) - sqrt(rComms^2 - B(end, :)) - m.comms(1, 1:N - 1)));
    errDom = 0;
    for ii = 1:sc.nA
        r = sc.radius(ii);
        obsRows = nAA + (ii:sc.nA:sc.nA * nO);
        wallRows = nAA + sc.nA * nO + (ii - 1) * 6 + (1:6);
        stored = min([sqrt(B(obsRows, :) + r^2) - r; B(wallRows, :)], [], 1);
        errDom = max(errDom, max(abs(stored - m.domain(ii, 1:N - 1))));
    end
    fprintf("  check vs miSim run: perf rel. err %.2g; margin err (m) domain %.2g, collision %.2g, comms %.2g\n", ...
        errPerf, errDom, errColl, errComms);
    assert(max([errDom, errColl, errComms]) < 1e-6, "Geometric margins differ from the stored miSim barriers");
end

function rd = radioData(root, scn)
    % Receiver SINR and GPS UAV-UAV distance during the guidance window,
    % with time in s from guidance start. The window runs from the first
    % "Sent GUIDANCE_TOGGLE" (guidance on) to the last (guidance off) in the
    % controller log, which excludes takeoff, flyout, return and landing.
    % Parsing uses aerpaw/results: readRadioLogs, readGpsLogs, timeBinMedian.
    %
    % Clocks. Each UAV VM stamps its GPS and radio logs with its own clock,
    % which is 8-12 s off the controller's (and up to ~3 s off the other
    % UAV's). Each UAV's offset is estimated by matching its GPS track to the
    % positions it reported to the controller ("Position from client c"),
    % and every UAV-side timestamp is shifted onto the controller clock
    % before windowing. The receiver's radio log is assumed to share its own
    % VM's clock (same machine as its GPS log).
    fr = fullfile(root, scn, "flight_results");
    c = dir(fullfile(fr, "controller_*", "*_controller_log.txt"));
    lines = string(splitlines(fileread(fullfile(c.folder, c.name))));
    stamp = @(L) datetime(extractBetween(L, "[", "]"), "InputFormat", "yyyy-MM-dd HH:mm:ss.SSSSSS");
    toggles = stamp(lines(contains(lines, "Sent GUIDANCE_TOGGLE")));
    assert(numel(toggles) >= 2, "%s: guidance start/end not found in the controller log", scn);
    ts0 = toggles(1); ts1 = toggles(end);
    rd.tEnd = seconds(ts1 - ts0);

    % Positions each client reported, on the controller clock (whole flight)
    tok = regexp(lines, "^\[([^\]]+)\] (\(\d+/\d+\)|\d\d:\d\d:\d\d) Position from client (\d+): ([^,]+),([^,]+),(\S+)", "tokens", "once");
    tok = vertcat(tok{~cellfun(@isempty, tok)});
    tPos = seconds(datetime(tok(:, 1), "InputFormat", "yyyy-MM-dd HH:mm:ss.SSSSSS") - ts0);
    client = str2double(tok(:, 3));
    xyz = str2double(tok(:, 4:6));

    % GPS tracks in local ENU (origin as in plotGpsLogs), each matched to its
    % client and shifted onto the controller clock
    repo = matlab.project.rootProject().RootFolder;
    yaml = fileread(fullfile(repo, "aerpaw", "config", "client1.yaml"));
    lla0 = [str2double(regexp(yaml, "lat:\s*([-\d.]+)", "tokens", "once")), ...
            str2double(regexp(yaml, "lon:\s*([-\d.]+)", "tokens", "once")), 0];
    uavDirs = dir(fullfile(fr, "uav*_*"));
    uavDirs = uavDirs([uavDirs.isdir]);
    assert(numel(uavDirs) == 2, "%s: expected two UAV log folders", scn);
    clients = unique(client)';
    lags = -30:0.1:30;
    [gpsT, gpsP] = deal(cell(1, 2));
    [offset, residual] = deal(zeros(1, 2));
    for uu = 1:2
        csv = dir(fullfile(uavDirs(uu).folder, uavDirs(uu).name, "*_gps_log.csv"));
        G = readGpsLogs(fullfile(csv.folder, csv.name));
        ts = G.Timestamp; ts.TimeZone = '';
        [t, ia] = unique(seconds(ts - ts0));
        P = lla2enu([G.Latitude(ia), G.Longitude(ia), G.Altitude(ia)], lla0, "flat");
        % Best (client, offset): median position error over the whole flight
        best = [Inf, NaN, NaN];
        for cc = clients
            m = client == cc;
            for lag = lags
                e = median(vecnorm(interp1(t, P, tPos(m) + lag) - xyz(m, :), 2, 2), "omitnan");
                if e < best(1)
                    best = [e, cc, lag];
                end
            end
        end
        gpsT{uu} = t - best(3); % controller clock
        gpsP{uu} = P;
        offset(uu) = best(3);
        residual(uu) = best(1);
        rd.client(uu) = best(2);
    end
    assert(rd.client(1) ~= rd.client(2), "%s: both GPS logs matched the same client", scn);
    rd.uav = string({uavDirs.name});
    rd.clockOffset = offset; % UAV clock minus controller clock, s
    rd.matchResidual = residual;

    % SINR at the receiver. Only one UAV receives (the other transmits);
    % readRadioLogs drops TDM guard-slot and self-reception rows. Values
    % below -200 dB are sentinels written before the flowgraph locks, as in
    % plotRadioLogs.
    isRx = arrayfun(@(d) ~isempty(dir(fullfile(d.folder, d.name, "*_snr_log.txt"))), uavDirs);
    assert(nnz(isRx) == 1, "%s: expected one receiving UAV", scn);
    radio = readRadioLogs(fullfile(uavDirs(isRx).folder, uavDirs(isRx).name));
    t = seconds(radio.Timestamp - ts0) - offset(isRx); % controller clock
    keep = t >= 0 & t <= rd.tEnd & ~isnan(radio.SNR) & radio.SNR >= -200;
    rd.t = t(keep);
    rd.sinr = radio.SNR(keep);
    [rd.tMed, rd.sinrMed] = timeBinMedian(rd.t, rd.sinr, 1);

    % Distance on a 1 s grid over the window
    distAt = @(t) vecnorm(interp1(gpsT{1}, gpsP{1}, t(:), "linear") - interp1(gpsT{2}, gpsP{2}, t(:), "linear"), 2, 2);
    rd.tDist = (0:floor(rd.tEnd))';
    rd.dist = distAt(rd.tDist);

    % Check: GPS distance at each in-window controller reading against the
    % distance between the positions the clients reported
    inWin = tPos >= 0 & tPos <= rd.tEnd;
    p1 = xyz(inWin & client == rd.client(1), :);
    p2 = xyz(inWin & client == rd.client(2), :);
    t1 = tPos(inWin & client == rd.client(1));
    n = min(size(p1, 1), size(p2, 1));
    rd.distCheck = abs(distAt(t1(1:n)) - vecnorm(p1(1:n, :) - p2(1:n, :), 2, 2));
end

function legendKey(ax, colors, names, styles, sources)
    % Legend of solid color entries (one per series) plus gray style keys
    % (one per data source), drawn as NaN dummy lines
    h = gobjects(1, size(colors, 1) + numel(styles));
    for cc = 1:size(colors, 1)
        h(cc) = plot(ax, NaN, NaN, "-", "Color", colors(cc, :));
    end
    for ss = 1:numel(styles)
        h(size(colors, 1) + ss) = plot(ax, NaN, NaN, styles(ss), "Color", 0.45 * [1, 1, 1]);
    end
    legend(ax, h, [names, sources], "Location", "best");
end
