function obj = onYourMarks(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "uav")};
    end
    
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "uav")};
    end

    % Receive initial position data
    initialPosition = read(obj.udp, 3, "double");

    % Acknowledge message receipt

    % teleport to desired position
    obj.pos = initialPosition;

    keyboard

end