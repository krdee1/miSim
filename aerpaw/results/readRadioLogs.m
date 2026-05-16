function R = readRadioLogs(logPath)
    arguments (Input)
        logPath (1, 1) string {isfolder(logPath)};
    end
    arguments (Output)
        R (:, 8) table;
    end

    % Extract receiving UAV ID from directory name (e.g. "uav0_..." → 0)
    [~, dirName] = fileparts(logPath);
    rxID = int32(sscanf(dirName, 'uav%d'));

    metrics = ["quality", "snr", "power", "noisefloor", "freqoffset"];
    logs = dir(logPath);
    logs = logs(endsWith({logs(:).name}, metrics + "_log.txt"));

    R = table(datetime.empty(0,1), zeros(0,1,'int32'), zeros(0,1,'int32'), zeros(0,1), zeros(0,1), zeros(0,1), zeros(0,1), zeros(0,1), ...
        'VariableNames', ["Timestamp", "TxUAVID", "RxUAVID", "SNR", "Power", "Quality", "NoiseFloor", "FreqOffset"]);

    for ii = 1:numel(logs)
        filepath = fullfile(logs(ii).folder, logs(ii).name);

        % Determine which metric this file contains
        metric = "";
        for m = 1:numel(metrics)
            if endsWith(logs(ii).name, metrics(m) + "_log.txt")
                metric = metrics(m);
                break;
            end
        end

        fid = fopen(filepath, 'r');
        % Skip header lines: some files have 2 tail-error lines + 1 column
        % header ("tx_uav_id,value"), others start with data immediately.
        % Read until a line that looks like a data record, then rewind to it.
        dataPattern = '^\[\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+\] [-\d]';
        linePos = ftell(fid);
        while true
            line = fgetl(fid);
            if ~ischar(line)
                break;  % EOF
            end
            if ~isempty(regexp(line, dataPattern, 'once'))
                fseek(fid, linePos, 'bof');  % rewind to start of this line
                break;
            end
            linePos = ftell(fid);
        end
        data = textscan(fid, '[%26c] %d,%f');
        fclose(fid);

        ts = datetime(cellstr(data{1}), 'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSSSSS');
        txId = int32(data{2});
        val = data{3};

        n = numel(ts);
        t = table(ts, txId, repmat(rxID, n, 1), NaN(n,1), NaN(n,1), NaN(n,1), NaN(n,1), NaN(n,1), ...
            'VariableNames', ["Timestamp", "TxUAVID", "RxUAVID", "SNR", "Power", "Quality", "NoiseFloor", "FreqOffset"]);

        switch metric
            case "snr",        t.SNR = val;
            case "power",      t.Power = val;
            case "quality",    t.Quality = val;
            case "noisefloor", t.NoiseFloor = val;
            case "freqoffset", t.FreqOffset = val;
        end

        R = [R; t]; %#ok<AGROW>
    end

    R = sortrows(R, "Timestamp");

    % Reconstruct per-measurement timestamps within GNURadio processing batches.
    % The flowgraph accumulates one full PN sequence (4095 chips at samp_rate/sps)
    % per measurement, but outputs the whole batch simultaneously with a single
    % wall-clock timestamp. We reassign timestamps by counting backward from the
    % batch processing time at the known PN period interval.
    pn_period = 4095 / (2e6 / 16);  % 32.76 ms per PN correlation period

    for txId = unique(R.TxUAVID)'
        rows = find(R.TxUAVID == txId);
        if numel(rows) < 2, continue; end

        dt = seconds(diff(R.Timestamp(rows)));
        break_pos = [1; find(dt > 0.5) + 1];
        end_pos   = [break_pos(2:end) - 1; numel(rows)];

        for b = 1:numel(break_pos)
            idx      = rows(break_pos(b) : end_pos(b));
            batch_ts = posixtime(R.Timestamp(idx));
            t_ref    = max(batch_ts);

            % Multiple metric files share the same processing timestamp for
            % each PN period, so group by unique original timestamp rather
            % than treating every row as a separate PN period.
            [~, ~, group_id] = unique(batch_ts);
            n_groups = max(group_id);
            new_ts   = t_ref - (n_groups - 1 : -1 : 0)' * pn_period;

            for g = 1:n_groups
                R.Timestamp(idx(group_id == g)) = ...
                    datetime(new_ts(g), 'ConvertFrom', 'posixtime');
            end
        end
    end

    % Remove rows during defined guard period between TDM shifts
    R(R.TxUAVID == -1, :) = [];

    % Remove self-reception rows (TX == RX)
    R(R.TxUAVID == R.RxUAVID, :) = [];
end