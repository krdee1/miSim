function R = readRadioLogs(logPath)
    arguments (Input)
        logPath (1, 1) string {isfolder(logPath)};
    end

    arguments (Output)
        R (:, 6) table;
    end

    % Extract receiving UAV ID from directory name (e.g. "uav0_..." → 0)
    [~, dirName] = fileparts(logPath);
    rxID = int32(sscanf(dirName, 'uav%d'));

    metrics = ["quality", "snr", "power"];
    logs = dir(logPath);
    logs = logs(endsWith({logs(:).name}, metrics + "_log.txt"));

    R = table(datetime.empty(0,1), zeros(0,1,'int32'), zeros(0,1,'int32'), zeros(0,1), zeros(0,1), zeros(0,1), ...
        'VariableNames', ["Timestamp", "TxUAVID", "RxUAVID", "SNR", "Power", "Quality"]);

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
        % Skip 3 lines: 2 junk (tail errors) + 1 header (tx_uav_id,value)
        for k = 1:3
            fgetl(fid);
        end
        data = textscan(fid, '[%26c] %d,%f');
        fclose(fid);

        ts = datetime(data{1}, 'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSSSSS');
        txId = int32(data{2});
        val = data{3};

        n = numel(ts);
        t = table(ts, txId, repmat(rxID, n, 1), NaN(n,1), NaN(n,1), NaN(n,1), ...
            'VariableNames', ["Timestamp", "TxUAVID", "RxUAVID", "SNR", "Power", "Quality"]);

        switch metric
            case "snr",     t.SNR = val;
            case "power",   t.Power = val;
            case "quality", t.Quality = val;
        end

        R = [R; t]; %#ok<AGROW>
    end

    R = sortrows(R, "Timestamp");

    % Remove rows during defined guard period between TDM shifts
    R(R.TxUAVID == -1, :) = [];

    % Remove self-reception rows (TX == RX)
    R(R.TxUAVID == R.RxUAVID, :) = [];
end