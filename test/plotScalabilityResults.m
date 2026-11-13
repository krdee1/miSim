function S = plotScalabilityResults(resultsFolder, opts)
% plotScalabilityResults  Thesis plots for the test_permutations scalability study.
%
%   S = plotScalabilityResults()                  % newest sandbox/scale_* folder
%   S = plotScalabilityResults("sandbox/scale_2026_11_13_14_00_02")
%   S = plotScalabilityResults(folder, Export=false)
%
% The study runs the double/obj2/longNarrow/close4 scenario for N = 3..8
% UAVs, several trials per N from random valid starts (test_permutations,
% scalability method). For each model this plots the distribution over
% trials of the final normalized system performance against N, as a box
% chart with min-max whiskers (util/minMaxBoxchart.m). The figures carry no
% title (the LaTeX caption gives it) and are saved by util/exportThesisFig.m
% in opts.OutDir as vector PDFs:
%   scalabilitysensing, scalabilityservice
%
% Performance uses the same SPAWC normalization as plotPermutationResults:
% the simulator's total divided by objectiveIntegral, the sum of the
% peak-normalized objective. Final = the last step of the run.
%
% Returns a table S with one row per run (N, trial, model, final performance).
    arguments
        resultsFolder (1, 1) string = newestResults();
        opts.OutDir (1, 1) string = "";
        opts.Export (1, 1) logical = true;
    end
    if opts.OutDir == ""
        opts.OutDir = fullfile(resultsFolder, "figures");
    end
    if opts.Export && ~isfolder(opts.OutDir)
        mkdir(opts.OutDir);
    end

    S = loadResults(resultsFolder);
    fprintf("Loaded %d runs from %s\n", height(S), resultsFolder);

    models = ["sensing", "service"];
    perfLabel = ["Final performance (normalized)", "Final performance (normalized, dB)"];
    for mm = 1:2
        rows = S(S.model == models(mm), :);
        Ns = unique(rows.N)';
        fprintf("%s:\n  %3s %6s %10s %10s %10s\n", models(mm), "N", "runs", "min", "median", "max");
        for n = Ns
            v = rows.finalPerf(rows.N == n);
            fprintf("  %3d %6d %10.4f %10.4f %10.4f\n", n, numel(v), min(v), median(v), max(v));
            if numel(v) < max(groupcounts(rows.N))
                warning("plotScalabilityResults:fewRuns", "%s, N = %d has only %d run(s), fewer than other N", models(mm), n, numel(v));
            end
        end

        f = figure("Name", "Scalability, " + models(mm));
        ax = axes(f);
        minMaxBoxchart(ax, rows.N, rows.finalPerf);
        xticks(ax, Ns); xlim(ax, [Ns(1) - 0.5, Ns(end) + 0.5]);
        grid(ax, "on"); % after boxchart, which resets the axes
        xlabel(ax, "Number of UAVs"); ylabel(ax, perfLabel(mm));
        if opts.Export
            exportThesisFig(f, "scalability" + models(mm), 3/4, "OutDir", opts.OutDir);
        end
    end

    if opts.Export
        fprintf("Saved figures to %s\n", opts.OutDir);
    end
end

function folder = newestResults()
    root = matlab.project.rootProject().RootFolder;
    d = dir(fullfile(root, "sandbox", "scale_*"));
    d = d([d.isdir]);
    assert(~isempty(d), "No sandbox/scale_* result folders found");
    [~, i] = max([d.datenum]);
    folder = string(fullfile(d(i).folder, d(i).name));
end

function S = loadResults(folder)
    files = dir(fullfile(folder, "*_miSimHist.mat"));
    assert(~isempty(files), "No _miSimHist.mat files in %s", folder);
    n = numel(files);
    [name, model] = deal(strings(n, 1));
    [N, trial, finalPerf] = deal(zeros(n, 1));
    for k = 1:n
        name(k) = erase(string(files(k).name), "_miSimHist.mat");
        tok = regexp(name(k), "^N(\d+)_trial(\d+)_double_obj2_(sensing|service)_longNarrow_close4$", "tokens", "once");
        assert(~isempty(tok), "Unrecognized result name %s", name(k));
        N(k) = str2double(tok{1});
        trial(k) = str2double(tok{2});
        model(k) = tok{3};

        out = load(fullfile(folder, files(k).name)).out;
        in = load(fullfile(folder, name(k) + "_miSimInits.mat"));
        finalPerf(k) = out.perf(end) / in.objectiveIntegral; % SPAWC normalization
    end
    S = sortrows(table(N, trial, model, finalPerf, name), ["model", "N", "trial"]);
end
