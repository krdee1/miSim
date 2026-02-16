function [G] = readGpsCsv(csvPath)
arguments (Input)
    csvPath (1, 1) string {isfile(csvPath)};
end

arguments (Output)
    G (:, 10) table;
end

G = readtable(csvPath, "ReadVariableNames", false);

% first column is just index, meaningless, toss it
G = G(:, 2:end);

% switch to the correct LLA convention (lat, lon, alt)
tmp = G(:, 2);
G(:, 2) = G(:, 1);
G(:, 1) = tmp;

% Split pitch, yaw, roll data read in as one string per timestep into separate columns
PYR = cell2mat(cellfun(@(x) str2num(strip(strip(x, "left", "("), "right", ")")), table2cell(G(:, 5)), "UniformOutput", false)); %#ok<ST2NM>
% Reinsert to original table
G = [G(:, 1:3), table(PYR(:, 1), VariableNames="Pitch"), table(PYR(:, 2), VariableNames="Yaw"), table(PYR(:, 3), VariableNames="Roll"), G(:, 6:end)];

% Clean up datetime entry
G = [table(datetime(G{:,8}, "InputFormat","yyyy-MM-dd HH:mm:ss.SSS", "TimeZone","America/New_York")), G(:, [1:7, 9:10])];

% Fix variable names
G.Properties.VariableNames = ["Timestamp", "Latitude", "Longitude", "Altitude", "Pitch", "Yaw", "Roll", "Voltage", "GPS Status", "Satellites"];
G.Properties.VariableTypes = ["datetime", "double", "double", "double", "double", "double", "double", "double", "double", "double"];
G.Properties.VariableUnits = ["yyyy-MM-dd HH:mm:ss.SSS (UTC+5)", "deg", "deg", "m", "deg", "deg", "deg", "Volts", "", ""];

end