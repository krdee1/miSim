classdef MESSAGE_TYPE < uint8
    enumeration
        TARGET  (1)  % Server->Client: target coordinates follow (3 doubles)
        ACK     (2)  % Client->Server: command received
        READY   (3)  % Both: ready for next command / mission complete
        RTL              (4)  % Server->Client: return to launch
        LAND             (5)  % Server->Client: land now
        GUIDANCE_TOGGLE  (6)  % Server->Client: toggle guidance mode on/off
        REQUEST_POSITION (7)  % Server->Client: respond with current ENU position
        POSITION         (8)  % Client->Server: current ENU position (3 doubles)
    end
end
