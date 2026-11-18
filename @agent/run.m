function obj = run(obj, domain, partitioning, timestepIndex, index, useDoubleIntegrator, dampingCoeff, dt, optimizeSensorPointing, otherAgents)
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
        otherAgents (:, 1) cell = cell();
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

    if isa(obj.sensorModel, "rfSensor")
        % Extract other agents' sensor models and positions once, outside the delta loop.
        % Mask the full-grid RSS caches (filled by partition()) down to this agent's
        % partition subset so sensorPerformance can reuse them for all perturbations.
        otherSensorsPos = cell2mat(cellfun(@(x) x.pos, otherAgents, "UniformOutput", false));
        otherSensors    = cellfun(@(x) x.sensorModel, otherAgents, "UniformOutput", false);

        % Full-grid received power and noise of every other agent, for the
        % fleet coverage C* that the service gradient ascends (see below).
        % partition() leaves full-grid caches; compute on a copy if missing so
        % otherSensors keeps its own cache for the masking that follows.
        fullTargets = [domain.objective.X(:), domain.objective.Y(:), zeros(numel(domain.objective.X), 1)];
        fullWeights = domain.objective.values(:);
        otherRssFull = zeros(numel(fullWeights), numel(otherSensors));
        otherNoise = zeros(1, numel(otherSensors));
        for kk = 1:numel(otherSensors)
            otherFull = otherSensors{kk};
            if isempty(otherFull.rssCache)
                [~, ~, otherFull] = otherFull.sensorPerformance(otherSensorsPos(kk, :), fullTargets);
            end
            assert(numel(otherFull.rssCache) == numel(fullWeights), "Other agent's RSS cache does not cover the full grid");
            otherRssFull(:, kk) = otherFull.rssCache;
            otherNoise(kk) = otherFull.N;
        end

        partitionIndices = find(partitionMask);
        for kk = 1:numel(otherSensors)
            if ~isempty(otherSensors{kk}.rssCache)
                otherSensors{kk}.rssCache = otherSensors{kk}.rssCache(partitionIndices);
            end
        end
        % Pre-mask this agent's own full-grid cache to the partition subset.
        % Used for ii==1 (current position, no perturbation) to avoid recomputing.
        baseSensorModel = obj.sensorModel;
        if ~isempty(obj.sensorModel.rssCache)
            baseSensorModel.rssCache = obj.sensorModel.rssCache(partitionIndices);
        end
    end

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
    fullCoverage = NaN(size(deltaApplicator, 1), 1); % fleet coverage C* at delta steps (rfSensor only)
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
        if isa(obj.sensorModel, "sigmoidSensor")
            sensorValues = obj.sensorModel.sensorPerformance(pos, [maskedX, maskedY, zeros(size(maskedX))]); % S_n(omega, P_n) on W_n
        elseif isa(obj.sensorModel, "rfSensor")
            if ii == 1
                sensorModelForDelta = baseSensorModel; % reuse partition-step cache; no recompute needed
            else
                sensorModelForDelta = obj.sensorModel.clearRssCache;
            end
            [sensorValues, ~, ~, ~] = sensorModelForDelta.sensorPerformance(pos, [maskedX, maskedY, zeros(size(maskedX))], otherSensorsPos, otherSensors);

            % Fleet coverage C* with this agent at the perturbed state (position,
            % and tilt/azimuth set on obj.sensorModel above): every grid point
            % served by its best-SINR agent, SINR as in rfSensor/sensorPerformance
            ownFull = obj.sensorModel.clearRssCache;
            [~, ~, ownFull] = ownFull.sensorPerformance(pos, fullTargets);
            Sall = max([ownFull.rssCache, otherRssFull], realmin);
            noise = [ownFull.N, otherNoise];
            total = sum(Sall, 2);
            bestSinr = max(10 * log10(Sall ./ (noise + total - Sall)), [], 2);
            fullCoverage(ii) = sum(bestSinr .* fullWeights);
        else
            error("?");
        end

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

    % Objective whose gradient the agent ascends. For sensing it is the
    % agent's own-cell score C_delta: one agent's sensor quality doesn't
    % depend on the others' positions, and the boundary terms cancel where
    % neighbors tie, so the own-cell gradient equals the gradient of the
    % system coverage. For service it must be the fleet coverage C*: this
    % agent's transmitter interferes in every other agent's cell, so moving it
    % changes their SINR too, and ascending only the own-cell score drops that
    % interference term (agents ascend selfishly and the system total can
    % fall). C* uses best-SINR association over the whole grid and does not
    % apply the partition minimum. The logged performance stays C_delta(1).
    % Fix identified by Reza Barzegaran.
    if isa(obj.sensorModel, "rfSensor")
        gradSource = fullCoverage;
    else
        gradSource = C_delta;
    end

    % Compute gradient by finite central differences
    gradC = [(gradSource(2)-gradSource(3))/(2*deltaPos), (gradSource(4)-gradSource(5))/(2*deltaPos), (gradSource(6)-gradSource(7))/(2*deltaPos)];
    if optimizeSensorPointing
        gradC(4) = (gradSource(8) -gradSource(9)) /(2*deltaAngle);
        gradC(5) = (gradSource(10)-gradSource(11))/(2*deltaAngle);
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