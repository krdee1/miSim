function t = buildSensorType()
% buildSensorType  Sensor class the compiled AERPAW controller is built for.
%
%   0  sigmoidSensor (sensing coverage)
%   1  rfSensor      (wireless service coverage)
%
% MATLAB Coder fixes the class of every agent's sensorModel at build time,
% so one compiled controller supports one sensor class. Set this before
% generating code; the controller refuses to start if the scenario CSV's
% sensorType column (0 when absent) does not match.
%
% The MATLAB (uncompiled) path ignores it and builds whatever the CSV asks for.
t = 1;
end
