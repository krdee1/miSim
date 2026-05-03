function value = sensorPerformance(obj, agentPos, targetPos)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "sigmoidSensor")};
        agentPos (1, 3) double;
        targetPos (:, 3) double;
    end
    arguments (Output)
        value (:, 1) double;
    end

    % Unit vectors from agent to each target
    diffs = targetPos - agentPos;
    d = vecnorm(diffs, 2, 2);
    dirs = diffs ./ d;

    % Boresight unit vector: tilt=0 → nadir [0,0,-1]; azimuth 0=+Y, 90=+X clockwise
    boresight = [sind(obj.tilt)*sind(obj.azimuth), sind(obj.tilt)*cosd(obj.azimuth), -cosd(obj.tilt)];

    % Angular offset from boresight to each target direction
    angularOffset = acosd(dirs * boresight');

    % Membership functions
    mu_d = obj.distanceMembership(d);
    mu_t = obj.tiltMembership(angularOffset);

    value = mu_d .* mu_t; % assume pan membership is always 1
end