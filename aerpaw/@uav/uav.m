classdef uav
    properties
        label = "";

        % Communications
        port;
        timeout;

        % State
        opMode = OPMODE.INVALID;
        pos = NaN(1, 3);

        % Commanding (not for codegen)
        commandOpMode = OPMODE.INVALID;
        commandPos = NaN(1, 3);
    end

    methods (Access = public)
        obj = initialize(obj, port, timeout, label)
        obj = onYourMarks(obj);
    end
end