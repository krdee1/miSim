function [f, fDist, R] = plotRadioLogs(resultsPath, G, tLim)
    arguments (Input)
        resultsPath (1, 1) string;
        G cell = {};
        tLim (1, 2) datetime = [datetime(-Inf, 'ConvertFrom', 'datenum'), datetime(Inf, 'ConvertFrom', 'datenum')];
    end
    arguments (Output)
        f (1, 1) matlab.ui.Figure;
        fDist (1, 1) matlab.ui.Figure;
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

    % Compute path loss from Power (post-processing)
    % Power = 20*log10(peak_mag) - rxGain; path loss = txGain - rxGain - Power
    txGain_dB = 76;   % from startchannelsounderTXGRC.sh GAIN_TX
    rxGain_dB = 30;   % from startchannelsounderRXGRC.sh GAIN_RX
    for ii = 1:numel(R)
        R{ii}.PathLoss = txGain_dB - rxGain_dB - R{ii}.Power;
        R{ii}.FreqOffset = R{ii}.FreqOffset / 1e6; % Hz to MHz
    end

    % Build legend labels and color map for up to 4 UAVs
    nUAV = numel(R);
    colors = lines(nUAV * nUAV);
    styles = ["-o", "-s", "-^", "-d", "-v", "-p", "-h", "-<", "->", "-+", "-x", "-*"];

    metricNames = ["SNR", "Power", "Quality", "PathLoss", "NoiseFloor", "FreqOffset"];
    yLabels     = ["SNR (dB)", "Power (dB)", "Quality", "Path Loss (dB)", "Noise Floor (dB)", "Freq Offset (MHz)"];
    nMetrics    = numel(metricNames);

    % --- Time-based figure ---
    f = figure;
    tl = tiledlayout(nMetrics + 1, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    % Distance vs time tile (first)
    ax = nexttile(tl);
    hold(ax, 'on'); grid(ax, 'on');
    legendEntries = string.empty;
    ci = 1;
    if ~isempty(G)
        for rxIdx = 1:nUAV
            tbl = R{rxIdx};
            txIDs = unique(tbl.TxUAVID);
            for ti = 1:numel(txIDs)
                txID = txIDs(ti);
                rows = tbl(tbl.TxUAVID == txID, :);
                rows = rows(rows.Timestamp >= tLim(1) & rows.Timestamp <= tLim(2), :);
                if isempty(rows), continue; end
                [~, ia] = unique(rows.Timestamp);
                [radioPt, dist] = pairDist(rows(ia, :), G);
                if isempty(dist) || all(isnan(dist)), continue; end
                valid = ~isnan(dist);
                si = mod(ci - 1, numel(styles)) + 1;
                plot(ax, datetime(radioPt(valid), 'ConvertFrom', 'posixtime'), dist(valid), ...
                    styles(si), 'Color', colors(ci, :), 'MarkerSize', 3, 'LineWidth', 0.5);
                legendEntries(end+1) = sprintf("TX %d → RX %d", txID, rows.RxUAVID(1)); %#ok<AGROW>
                ci = ci + 1;
            end
        end
    end
    ylabel(ax, 'Distance (m)');
    xlabel(ax, 'Time');
    legend(ax, legendEntries, 'Location', 'best');
    hold(ax, 'off');

    for mi = 1:nMetrics
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
                rows = rows(rows.Timestamp >= tLim(1) & rows.Timestamp <= tLim(2), :);
                vals = rows.(metricNames(mi));
                valid = ~isnan(vals);
                rows = rows(valid, :);
                vals = vals(valid);

                if isempty(rows)
                    continue;
                end

                si = mod(ci - 1, numel(styles)) + 1;
                plot(ax, rows.Timestamp, vals, styles(si), ...
                    'Color', colors(ci, :), 'MarkerSize', 3, 'LineWidth', 0.5);
                legendEntries(end+1) = sprintf("TX %d → RX %d", txID, tbl.RxUAVID(1)); %#ok<AGROW>

                % Median per 1/3-second time bin
                [t_med, v_med] = timeBinMedian(posixtime(rows.Timestamp), vals, 1/3);
                plot(ax, datetime(t_med, 'ConvertFrom', 'posixtime'), v_med, '-', ...
                    'Color', 'r', 'LineWidth', 2);
                legendEntries(end+1) = sprintf("TX %d → RX %d (median)", txID, tbl.RxUAVID(1)); %#ok<AGROW>
                ci = ci + 1;
            end
        end

        ylabel(ax, yLabels(mi));
        if mi == nMetrics
            xlabel(ax, 'Time');
        end
        legend(ax, legendEntries, 'Location', 'best');
        hold(ax, 'off');
    end

    title(tl, 'Radio Channel Metrics');

    % --- Distance-based figure ---
    fDist = figure;

    if isempty(G)
        return;
    end

    tl2 = tiledlayout(nMetrics + 1, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    % Distance vs time tile (first)
    ax = nexttile(tl2);
    hold(ax, 'on'); grid(ax, 'on');
    legendEntries = string.empty;
    ci = 1;
    for rxIdx = 1:nUAV
        tbl = R{rxIdx};
        txIDs = unique(tbl.TxUAVID);
        for ti = 1:numel(txIDs)
            txID = txIDs(ti);
            rows = tbl(tbl.TxUAVID == txID, :);
            rows = rows(rows.Timestamp >= tLim(1) & rows.Timestamp <= tLim(2), :);
            if isempty(rows), continue; end
            [~, ia] = unique(rows.Timestamp);
            [radioPt, dist] = pairDist(rows(ia, :), G);
            if isempty(dist) || all(isnan(dist)), continue; end
            valid = ~isnan(dist);
            si = mod(ci - 1, numel(styles)) + 1;
            plot(ax, datetime(radioPt(valid), 'ConvertFrom', 'posixtime'), dist(valid), ...
                styles(si), 'Color', colors(ci, :), 'MarkerSize', 3, 'LineWidth', 0.5);
            legendEntries(end+1) = sprintf("TX %d → RX %d", txID, rows.RxUAVID(1)); %#ok<AGROW>
            ci = ci + 1;
        end
    end
    ylabel(ax, 'Distance (m)');
    xlabel(ax, 'Time');
    legend(ax, legendEntries, 'Location', 'best');
    hold(ax, 'off');

    for mi = 1:nMetrics
        ax = nexttile(tl2);
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

                if isempty(rows)
                    continue;
                end

                rows = rows(rows.Timestamp >= tLim(1) & rows.Timestamp <= tLim(2), :);
                if isempty(rows)
                    continue;
                end

                vals = rows.(metricNames(mi));
                valid = ~isnan(vals);
                rows = rows(valid, :);
                vals = vals(valid);

                if isempty(rows)
                    continue;
                end

                [radioPt, dist] = pairDist(rows, G);
                if isempty(dist) || all(isnan(dist)), continue; end

                % Drop points where GPS interpolation returned NaN
                validDist = ~isnan(dist);
                rowTs = radioPt(validDist);
                dist  = dist(validDist);
                vals  = vals(validDist);

                si = mod(ci - 1, numel(styles)) + 1;
                scatter(ax, dist, vals, 9, colors(ci, :), strrep(styles(si), "-", ""), 'filled');
                legendEntries(end+1) = sprintf("TX %d → RX %d", txID, rows.RxUAVID(1)); %#ok<AGROW>

                % Median per 1/3-second time bin, plotted against median distance
                [~, dv_med] = timeBinMedian(rowTs, [dist, vals], 1/3);
                [d_med, si_sort] = sort(dv_med(:, 1));
                v_med = dv_med(si_sort, 2);
                plot(ax, d_med, v_med, '-', 'Color', 'r', 'LineWidth', 2);
                legendEntries(end+1) = sprintf("TX %d → RX %d (median)", txID, rows.RxUAVID(1)); %#ok<AGROW>
                ci = ci + 1;
            end
        end

        ylabel(ax, yLabels(mi));
        if mi == nMetrics
            xlabel(ax, 'Distance (m)');
        end
        legend(ax, legendEntries, 'Location', 'best');
        hold(ax, 'off');
    end

    title(tl2, 'Radio Channel Metrics vs Distance');
end


function [radioPt, dist] = pairDist(rows, G)
    % Interpolate GPS-based inter-UAV distance at each row's timestamp.
    radioPt = []; dist = [];
    txGpsIdx = double(rows.TxUAVID(1)) + 1;
    rxGpsIdx = double(rows.RxUAVID(1)) + 1;
    if txGpsIdx > numel(G) || rxGpsIdx > numel(G), return; end
    Gtx = G{txGpsIdx};
    Grx = G{rxGpsIdx};
    if ~ismember('East', Gtx.Properties.VariableNames) || ...
       ~ismember('East', Grx.Properties.VariableNames), return; end
    txTs = Gtx.Timestamp; txTs.TimeZone = '';
    rxTs = Grx.Timestamp; rxTs.TimeZone = '';
    txPt = posixtime(txTs);
    rxPt = posixtime(rxTs);
    radioPt = posixtime(rows.Timestamp);
    validTx = ~isnan(Gtx.East);
    validRx = ~isnan(Grx.East);
    txE = interp1(txPt(validTx), Gtx.East(validTx),  radioPt, 'linear', NaN);
    txN = interp1(txPt(validTx), Gtx.North(validTx), radioPt, 'linear', NaN);
    txU = interp1(txPt(validTx), Gtx.Up(validTx),    radioPt, 'linear', NaN);
    rxE = interp1(rxPt(validRx), Grx.East(validRx),  radioPt, 'linear', NaN);
    rxN = interp1(rxPt(validRx), Grx.North(validRx), radioPt, 'linear', NaN);
    rxU = interp1(rxPt(validRx), Grx.Up(validRx),    radioPt, 'linear', NaN);
    dist = vecnorm([txE - rxE, txN - rxN, txU - rxU], 2, 2);
end
