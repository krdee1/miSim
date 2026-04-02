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

    % Remove rows during defined guard period between TDM shifts
    R(R.TxUAVID == -1, :) = [];

    % Remove self-reception rows (TX == RX)
    R(R.TxUAVID == R.RxUAVID, :) = [];
end