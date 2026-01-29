function u = sendTarget(~, u, pos)
    arguments (Input)
        ~
        u (1, 1) {mustBeA(u, 'uav')};
        pos (1, 3) double;
    end
    arguments (Output)
        u (1, 1) {mustBeA(u, 'uav')};
    end

    % Branch here depending on environment
    if coder.target("MATLAB")
        % Simulation - update target position
        u.commandPos = pos;

    elseif coder.target("C++")
        % Codegen - TX starting position to UAV
        coder.cinclude("udp_comm.h");
        coder.ceval("sendTarget", coder.rref(pos));
    end



end