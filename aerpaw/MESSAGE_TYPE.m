classdef MESSAGE_TYPE < uint8
    enumeration
        TARGET  (1)  % Server->Client: target coordinates follow (3 doubles)
        ACK     (2)  % Client->Server: command received
        READY   (3)  % Both: ready for next command / mission complete
        RTL     (4)  % Server->Client: return to launch
        LAND    (5)  % Server->Client: land now
    end
end
