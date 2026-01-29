function obj = run(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'scenario')};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'scenario')};
    end
    
    count = 0;

    while true
        %% UAV operations (simulated only)
        if coder.target("MATLAB")
            % Iterate over UAVs
            for ii = 1:size(obj.uavs, 1)
                % Determine behavior from UAV opMode
                if obj.uavs{ii}.opMode == OPMODE.INVALID
                    error("Invalid UAV opMode");
                elseif obj.uavs{ii}.opMode == OPMODE.INITIALIZED
                    % Waiting for starting position to be sent by the controller
                    if all(~(isnan(obj.uavs{ii}.commandPos)))
                        % Teleport to starting position
                        obj.uavs{ii}.pos = obj.uavs{ii}.commandPos;

                        % reset to no command
                        obj.uavs{ii}.commandPos = NaN(1, 3);

                        % Acknowledge command receipt to controller by reporting a new opMode
                        obj.uavs{ii}.commandOpMode = OPMODE.SET;
                    end
                else
                    error("Unexpected UAV opMode");
                end
            end
        elseif coder.target("C++")
            coder.cinclude('udp_comm.h');
            % Iterate over UAVs
            for ii = 1:size(obj.uavs, 1)
                mode = uint8(0);
                coder.ceval('recvOpMode', coder.wref(mode));
            end
        end


        %% Server operations
        % Compute guidance for UAVs and issue commands
        if obj.opMode == OPMODE.INVALID
            error("Invalid controller opMode");
        elseif obj.opMode == OPMODE.INITIALIZED
            % On the first iteration, command UAVs to go to their starting positions
            commandPos = obj.inits.pos;
        elseif obj.opMode == OPMODE.SET
            % begin experiment once the controller and UAVs are ready
            if unique(cellfun(@(x) x.opMode.id, obj.uavs)) == OPMODE.SET.id
                keyboard
            end
        else
            error("Unexpected controller opMode");
        end

        % Transmit commands
        % Command drones to their starting positions
        for ii = 1:size(obj.uavs, 1)
            obj.uavs{ii} = obj.sendTarget(obj.uavs{ii}, commandPos(ii, 1:3));
        end
        keyboard
            


        % tally iterations
        count = count + 1;
    end
end