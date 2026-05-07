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

    % --- Time-based figure ---
    f = figure;
    tl = tiledlayout(numel(metricNames), 1, 'TileSpacing', 'compact', 'Padding', 'compact');

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
                rows = rows(rows.Timestamp >= tLim(1) & rows.Timestamp <= tLim(2), :);
                vals = rows.(metricNames(mi));

                if isempty(rows) || all(isnan(vals))
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

    % --- Distance-based figure ---
    fDist = figure;

    if isempty(G)
        return;
    end

    tl2 = tiledlayout(numel(metricNames), 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    for mi = 1:numel(metricNames)
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
                if all(isnan(vals))
                    continue;
                end

                % Map 0-based UAV IDs to 1-based GPS cell indices
                txGpsIdx = double(txID) + 1;
                rxGpsIdx = double(rows.RxUAVID(1)) + 1;

                if txGpsIdx > numel(G) || rxGpsIdx > numel(G)
                    continue;
                end

                Gtx = G{txGpsIdx};
                Grx = G{rxGpsIdx};

                if ~ismember('East', Gtx.Properties.VariableNames) || ...
                   ~ismember('East', Grx.Properties.VariableNames)
                    continue;
                end

                % Strip timezone before posixtime so radio and GPS timestamps
                % are treated on the same scale (both are AERPAW wall-clock time)
                txTs = Gtx.Timestamp; txTs.TimeZone = '';
                rxTs = Grx.Timestamp; rxTs.TimeZone = '';
                txPt = posixtime(txTs);
                rxPt = posixtime(rxTs);
                radioPt = posixtime(rows.Timestamp);

                % Interpolate GPS positions at radio measurement times.
                % Exclude NaN ENU entries (outside algorithm flight range).
                validTx = ~isnan(Gtx.East);
                validRx = ~isnan(Grx.East);

                txE = interp1(txPt(validTx), Gtx.East(validTx),  radioPt, 'linear', NaN);
                txN = interp1(txPt(validTx), Gtx.North(validTx), radioPt, 'linear', NaN);
                txU = interp1(txPt(validTx), Gtx.Up(validTx),    radioPt, 'linear', NaN);
                rxE = interp1(rxPt(validRx), Grx.East(validRx),  radioPt, 'linear', NaN);
                rxN = interp1(rxPt(validRx), Grx.North(validRx), radioPt, 'linear', NaN);
                rxU = interp1(rxPt(validRx), Grx.Up(validRx),    radioPt, 'linear', NaN);

                dist = vecnorm([txE - rxE, txN - rxN, txU - rxU], 2, 2);

                if all(isnan(dist))
                    continue;
                end

                si = mod(ci - 1, numel(styles)) + 1;
                scatter(ax, dist, vals, 9, colors(ci, :), strrep(styles(si), "-", ""), 'filled');
                legendEntries(end+1) = sprintf("TX %d → RX %d", txID, rows.RxUAVID(1)); %#ok<AGROW>
                ci = ci + 1;
            end
        end

        ylabel(ax, yLabels(mi));
        if mi == numel(metricNames)
            xlabel(ax, 'Distance (m)');
        end
        legend(ax, legendEntries, 'Location', 'best');
        hold(ax, 'off');
    end

    title(tl2, 'Radio Channel Metrics vs Distance');
end
