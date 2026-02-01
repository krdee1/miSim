function targets = loadTargetsFromYaml(filename)
%LOADTARGETSFROMYAML Load target coordinates from YAML config file
%   targets = loadTargetsFromYaml(filename) reads the targets section
%   from a YAML config file and returns an Nx3 matrix of [x, y, z] coordinates.

targets = [];
fid = fopen(filename, 'r');
if fid == -1
    error('Could not open file: %s', filename);
end

inTargets = false;
while ~feof(fid)
    line = fgetl(fid);
    if ~ischar(line)
        break;
    end

    % Check if we've entered the targets section
    if contains(line, 'targets:')
        inTargets = true;
        continue;
    end

    % If we hit another top-level key, exit targets section
    if inTargets && ~isempty(line) && line(1) ~= ' ' && line(1) ~= '#'
        break;
    end

    % Parse target entries: "  - [x, y, z]"
    if inTargets
        % Extract numbers from array format
        match = regexp(line, '\[\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\]', 'tokens');
        if ~isempty(match)
            coords = str2double(match{1});
            targets = [targets; coords]; %#ok<AGROW>
        end
    end
end

fclose(fid);
end
