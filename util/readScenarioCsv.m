function scenario = readScenarioCsv(csvPath)
    arguments (Input)
        csvPath (1, 1) string;
    end
    arguments (Output)
        scenario struct;
    end

    % File input validation
    assert(isfile(csvPath), "%s is not a valid filepath.", csvPath);
    assert(endsWith(csvPath, ".csv"), "%s is not a CSV file.", csvPath);

    % Read the first two lines directly — avoids readtable's quoting
    % requirement that '"' must immediately follow ',' (no leading space).
    fid = fopen(csvPath, 'r');
    headerLine = fgetl(fid);
    dataLine   = fgetl(fid);
    fclose(fid);

    assert(ischar(headerLine) && ischar(dataLine), ...
        "CSV must have a header row and at least one data row.");

    % Parse header: simple comma split + trim (no quoting expected in names)
    headers = strtrim(strsplit(headerLine, ','));

    % Parse data row: comma split that respects double-quoted fields
    fields = splitQuotedCSV(dataLine);

    assert(numel(fields) == numel(headers), ...
        "CSV data row has %d fields but header has %d columns.", ...
        numel(fields), numel(headers));

    % Build output struct: strip outer quotes, trim whitespace, convert to numeric.
    % str2num handles scalar ("5") and vector ("1.0, 2.0, 3.0") fields alike.
    % Empty fields ("") become [] via str2num('') == [].
    scenario = struct();
    for ii = 1:numel(headers)
        raw = strtrim(stripQuotes(fields{ii}));
        scenario.(headers{ii}) = str2num(raw); %#ok<ST2NM>
    end
end

% -------------------------------------------------------------------------
function fields = splitQuotedCSV(line)
% Split a CSV row by commas, respecting double-quoted fields.
% Fields may have leading/trailing whitespace around quotes.
    fields = {};
    inQuote = false;
    start = 1;
    for ii = 1:length(line)
        if line(ii) == '"'
            inQuote = ~inQuote;
        elseif line(ii) == ',' && ~inQuote
            fields{end+1} = line(start:ii-1); %#ok<AGROW>
            start = ii + 1;
        end
    end
    fields{end+1} = line(start:end);
end

% -------------------------------------------------------------------------
function s = stripQuotes(s)
% Trim whitespace then remove a single layer of enclosing double-quotes.
    s = strtrim(s);
    if length(s) >= 2 && s(1) == '"' && s(end) == '"'
        s = s(2:end-1);
    end
end
