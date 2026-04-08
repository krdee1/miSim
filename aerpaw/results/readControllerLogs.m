function T2 = readControllerLogs(filepath)
    arguments (Input)
        filepath (1, 1) string;
    end
    arguments (Output)
        T2 table;
    end
    assert(isfile(filepath), "File not found at %s", filepath);

    T = readtable(filepath, 'VariableNamingRule', 'preserve');
    s = split(T.(T.Properties.VariableNames{1}), ']');
    s2 = strip(s(startsWith(s(:, 2), " ("), 1), 'left', '[');
    d = datetime(s2, "InputFormat", "yyyy-MM-dd HH:mm:ss.SSSSSS")';
    it = s(startsWith(s(:, 2), " ("), 2);
    it = str2double(strip(strip(it, 'left'), 'left', '('));
    T.Var3 = strip(append(T.Var3, " ", T.Var4, " ", T.Var5, " ", T.Var6, " ", T.Var7));
    T.Var4 = []; T.Var5 = []; T.Var6 = []; T.Var7 = [];
    msg = T.(T.Properties.VariableNames{2});
    msg = msg(startsWith(s(:, 2), " ("), :);
    s3 = split(msg, ') ');
    s3 = s3(:, 2);
    msg = append(s3, T.Var3(startsWith(s(:, 2), " (")));
    T2 = table(it, d', msg, 'VariableNames', ["iteration", "timestamp", "message"]);
    % T.Var1 = datetime(strip(strip(append(T.Var1, " ", T.Var2), 'left', '['), 'right', ']'), "InputFormat", "yyyy-MM-dd HH:mm:ss.SSSSSS");
    % T.Var2 = [];
    % T.Var3 = strip(append(T.Var3, " ", T.Var4, " ", T.Var5, " ", T.Var6, " ", string(T.Var7), " ", T.Var8, " ", T.Var9));
    % T.Var4 = []; T.Var5 = []; T.Var6 = []; T.Var7 = []; T.Var8 = []; T.Var9 = [];
    % T.Properties.VariableNames{1} = 'timestamp';
    % T.Properties.VariableNames{2} = 'message';

    % T(ismissing(T.message), :) = [];
end