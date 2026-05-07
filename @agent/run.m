function obj = run(obj, domain, partitioning, timestepIndex, index, useDoubleIntegrator, dampingCoeff, dt, optimizeSensorPointing)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "agent")};
        domain (1, 1) {mustBeGeometry};
        partitioning (:, :) double;
        timestepIndex (1, 1) double;
        index (1, 1) double;
        useDoubleIntegrator (1, 1) logical = false;
        dampingCoeff (1, 1) double = 2.0;
        dt (1, 1) double = 1.0;
        optimizeSensorPointing (1, 1) logical = false;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "agent")};
    end

    % Always update lastPos/lastVel so constrainMotion evaluates barriers at
    % the correct (most recent) position, even when this agent has no partition.
    obj.lastPos = obj.pos;
    if useDoubleIntegrator
        obj.lastVel = obj.vel;
    end

    % Collect objective function values across partition
    partitionMask = partitioning == index;
    if ~any(partitionMask(:))
        % This agent has no partition, maintain current state
        return;
    end
    objectiveValues = domain.objective.values(partitionMask); % f(omega) on W_n

    % Compute sensor performance on partition
    maskedX = domain.objective.X(partitionMask);
    maskedY = domain.objective.Y(partitionMask);

    if optimizeSensorPointing
        % Stash actual current sensor model tilt/azimuth before messing with it
        % in these following hypotheticals
        tilt = obj.sensorModel.tilt;
        azimuth = obj.sensorModel.azimuth;
    end

    % Compute agent performance at the current position and each delta position +/- X, Y, Z, tilt, azimuth
    deltaPos = domain.objective.discretizationStep; % smallest possible step size that gets different results
    if optimizeSensorPointing
        deltaAngle = atan2d(domain.objective.discretizationStep, obj.pos(3)); % smallest possible angle derived from smallest possible step size and current height
    end
    deltaApplicator = [0, 0, 0, 0, 0; 1, 0, 0, 0, 0; -1, 0, 0, 0, 0; 0, 1, 0, 0, 0; 0, -1, 0, 0, 0; 0, 0, 1, 0, 0; 0, 0, -1, 0, 0; 0, 0, 0, 1, 0; 0, 0, 0, -1, 0; 0, 0, 0, 0, 1; 0, 0, 0, 0, -1;]; % none, +X, -X, +Y, -Y, +Z, -Z, +tilt, -tilt, +azimuth, -azimuth
    C_delta = NaN(size(deltaApplicator, 1), 1); % agent performance at delta steps in each direction
    for ii = 1:size(deltaApplicator, 1)
        if ~optimizeSensorPointing && ii > 7; break; end
        % Apply delta to position
        pos = obj.pos + deltaPos * deltaApplicator(ii, 1:3);
        if optimizeSensorPointing
            % Apply delta to tilt and azimuth
            obj.sensorModel.tilt = tilt + deltaAngle * deltaApplicator(ii, 4);
            obj.sensorModel.azimuth = azimuth + deltaAngle * deltaApplicator(ii, 5);
        end
        
        % Compute performance values on partition
        sensorValues = obj.sensorModel.sensorPerformance(pos, [maskedX, maskedY, zeros(size(maskedX))]); % S_n(omega, P_n) on W_n

        % Rearrange data into image arrays
        F = NaN(size(partitionMask));
        F(partitionMask) = objectiveValues;
        S = NaN(size(partitionMask));
        S(partitionMask) = sensorValues;

        % Compute agent performance
        C = S .* F;
        C_delta(ii) = sum(C(~isnan(C)));
    end

    if optimizeSensorPointing
        % Reset sensor model to actual tilt and azimuth angles
        obj.sensorModel.tilt = tilt;
        obj.sensorModel.azimuth = azimuth;
    end

    % Store agent performance at current time and place
    if coder.target('MATLAB')
        obj.performance(timestepIndex + 1) = C_delta(1);
    end

    % Compute gradient by finite central differences
    gradC = [(C_delta(2)-C_delta(3))/(2*deltaPos), (C_delta(4)-C_delta(5))/(2*deltaPos), (C_delta(6)-C_delta(7))/(2*deltaPos)];
    if optimizeSensorPointing
        gradC(4) = (C_delta(8) -C_delta(9)) /(2*deltaAngle);
        gradC(5) = (C_delta(10)-C_delta(11))/(2*deltaAngle);
    end

    % Compute scaling factor
    targetPosRate = obj.initialStepSize - obj.stepDecayRate * timestepIndex; % slow down as you get closer
    gradPosNorm = norm(gradC(1:3));

    % Compute unconstrained next state
    if useDoubleIntegrator
        % Double-integrator: gradient produces desired acceleration with damping
        if gradPosNorm < 1e-100
            a_gradient = zeros(1, 5);
        else
            % Scale so steady-state step ≈ targetRate (matching SI behavior)
            a_gradient = (targetPosRate * dampingCoeff / (gradPosNorm * dt)) * gradC;
        end
        % Semi-implicit Euler: unconditionally stable for any dampingCoeff and dt
        obj.vel = (obj.vel + a_gradient(1:3) * dt) / (1 + dampingCoeff * dt);
        obj.pos = obj.lastPos + obj.vel * dt;
    else
        % Single-integrator: gradient directly sets position step
        if gradPosNorm >= 1e-100
            obj.pos = obj.pos + (targetPosRate / gradPosNorm) * gradC(1:3);
        end
    end

    % Update tilt and azimuth, saturating at the decaying maximum allowed step size
    if optimizeSensorPointing
        maxAngleStep = obj.initialMaxAngleStepSize - obj.angleStepDecayRate * timestepIndex;
        obj.sensorModel.tilt    = obj.sensorModel.tilt    + sign(gradC(4)) * min(abs(gradC(4)), maxAngleStep);
        obj.sensorModel.azimuth = obj.sensorModel.azimuth + sign(gradC(5)) * min(abs(gradC(5)), maxAngleStep);
    end

    % Reinitialize collision geometry in the new position
    d = obj.pos - obj.collisionGeometry.center;
    if isa(obj.collisionGeometry, "rectangularPrism")
        obj.collisionGeometry = obj.collisionGeometry.initialize([obj.collisionGeometry.minCorner; obj.collisionGeometry.maxCorner] + d, obj.collisionGeometry.tag, obj.collisionGeometry.label);
    elseif isa(obj.collisionGeometry, "spherical")
        obj.collisionGeometry = obj.collisionGeometry.initialize(obj.collisionGeometry.center + d, obj.collisionGeometry.radius, obj.collisionGeometry.tag, obj.collisionGeometry.label);
    else
        error("?");
    end
end