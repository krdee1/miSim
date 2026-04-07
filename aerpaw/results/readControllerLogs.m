function T = readControllerLogs(filepath)
    arguments (Input)
        filepath (1, 1) string;
    end
    arguments (Output)
        T table;
    end
    assert(isfile(filepath), "File not found at %s", filepath);

    T = readtable(filepath);
    T.Var1 = datetime(strip(strip(append(T.Var1, " ", T.Var2), 'left', '['), 'right', ']'), "InputFormat", "yyyy-MM-dd HH:mm:ss.SSSSSS");
    T.Var2 = [];
    T.Var3 = strip(append(T.Var3, " ", T.Var4, " ", T.Var5, " ", T.Var6, " ", string(T.Var7), " ", T.Var8, " ", T.Var9));
    T.Var4 = []; T.Var5 = []; T.Var6 = []; T.Var7 = []; T.Var8 = []; T.Var9 = [];
    T.Properties.VariableNames{1} = 'timestamp';
    T.Properties.VariableNames{2} = 'message';

    T(ismissing(T.message), :) = [];
end