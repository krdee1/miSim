function [f, R] = plotRadioLogs(resultsPath)
    arguments (Input)
        resultsPath (1, 1) string;
    end
    arguments (Output)
        f (1, 1) matlab.ui.Figure;
        R cell;
    end

    logDirs = dir(resultsPath);
    logDirs = logDirs(3:end);
    logDirs = logDirs([logDirs.isdir] == 1);

    R = cell(size(logDirs));
    for ii = 1:size(logDirs, 1)
        R{ii} = readRadioLogs(fullfile(logDirs(ii).folder, logDirs(ii).name));
    end

    % Discard rows where any non-NaN dB metric is below -200 (sentinel values)
    for ii = 1:numel(R)
        snr = R{ii}.SNR;
        pwr = R{ii}.Power;
        bad = (snr < -200 & ~isnan(snr)) | (pwr < -200 & ~isnan(pwr));
        R{ii}(bad, :) = [];
    end

    % Build legend labels and color map for up to 4 UAVs
    nUAV = numel(R);
    colors = lines(nUAV * nUAV);
    styles = ["-o", "-s", "-^", "-d", "-v", "-p", "-h", "-<", "->", "-+", "-x", "-*"];

    metricNames = ["SNR", "Power", "Quality"];
    yLabels     = ["SNR (dB)", "Power (dB)", "Quality"];

    f = figure;
    tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    for mi = 1:numel(metricNames)
        ax = nexttile(tl);
        hold(ax, 'on');
        grid(ax, 'on');

        legendEntries = string.empty;
        ci = 1;
        for rxIdx = 1:nUAV
            tbl = R{rxIdx};
            txIDs = unique(tbl.TxUAVID);
            for ti = 1:numel(txIDs)
                txID = txIDs(ti);
                rows = tbl(tbl.TxUAVID == txID, :);
                vals = rows.(metricNames(mi));

                % Skip if all NaN for this metric
                if all(isnan(vals))
                    continue;
                end

                si = mod(ci - 1, numel(styles)) + 1;
                plot(ax, rows.Timestamp, vals, styles(si), ...
                    'Color', colors(ci, :), 'MarkerSize', 3, 'LineWidth', 1);
                legendEntries(end+1) = sprintf("TX %d → RX %d", txID, tbl.RxUAVID(1)); %#ok<AGROW>
                ci = ci + 1;
            end
        end

        ylabel(ax, yLabels(mi));
        if mi == numel(metricNames)
            xlabel(ax, 'Time');
        end
        legend(ax, legendEntries, 'Location', 'best');
        hold(ax, 'off');
    end

    title(tl, 'Radio Channel Metrics');
end