function obj = run(obj, domain, partitioning, timestepIndex, index, agents)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'agent')};
        domain (1, 1) {mustBeGeometry};
        partitioning (:, :) double;
        timestepIndex (1, 1) double;
        index (1, 1) double;
        agents (:, 1) {mustBeA(agents, 'cell')};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'agent')};
    end

    % Collect objective function values across partition
    partitionMask = partitioning == index;
    objectiveValues = domain.objective.values(partitionMask); % f(omega) on W_n

    % Compute sensor performance on partition
    maskedX = domain.objective.X(partitionMask);
    maskedY = domain.objective.Y(partitionMask);

    % Compute agent performance at the current position and each delta position +/- X, Y, Z
    delta = domain.objective.discretizationStep; % smallest possible step size that gets different results
    deltaApplicator = [0, 0, 0; 1, 0, 0; -1, 0, 0; 0, 1, 0; 0, -1, 0; 0, 0, 1; 0, 0, -1]; % none, +X, -X, +Y, -Y, +Z, -Z
    C_delta = NaN(7, 1); % agent performance at delta steps in each direction
    for ii = 1:7
        % Apply delta to position
        pos = obj.pos + delta * deltaApplicator(ii, 1:3);
        
        % Compute performance values on partition
        if ii < 5
            % Compute sensing performance
            sensorValues = obj.sensorModel.sensorPerformance(pos, obj.pan, obj.tilt, [maskedX, maskedY, zeros(size(maskedX))]); % S_n(omega, P_n) on W_n
            % Objective performance does not change for 0, +/- X, Y steps.
            % Those values are computed once before the loop and are only
            % recomputed when +/- Z steps are applied
        else
            % Redo partitioning for Z stepping only
            partitioning = obj.partition(agents, domain.objective);

            % Recompute partiton-derived performance values for objective
            partitionMask = partitioning == index;
            objectiveValues = domain.objective.values(partitionMask); % f(omega) on W_n

            % Recompute partiton-derived performance values for sensing
            maskedX = domain.objective.X(partitionMask);
            maskedY = domain.objective.Y(partitionMask);
            sensorValues = obj.sensorModel.sensorPerformance(pos, obj.pan, obj.tilt, [maskedX, maskedY, zeros(size(maskedX))]); % S_n(omega, P_n) on W_n
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

    % Store agent performance at current time and place
    obj.performance(timestepIndex + 1) = C_delta(1);

    % Compute gradient by finite central differences
    gradC = [(C_delta(2)-C_delta(3))/(2*delta), (C_delta(4)-C_delta(5))/(2*delta), (C_delta(6)-C_delta(7))/(2*delta)];

    % Compute scaling factor
    targetRate = 0.2 - 0.0008 * timestepIndex; % slow down as you get closer
    rateFactor = targetRate / norm(gradC); 

    % Compute unconstrained next position
    pNext = obj.pos + rateFactor * gradC;

    % Move to next position
    obj.lastPos = obj.pos;
    obj.pos = pNext;

    % Reinitialize collision geometry in the new position
    d = obj.pos - obj.collisionGeometry.center;
    if isa(obj.collisionGeometry, 'rectangularPrism')
        obj.collisionGeometry = obj.collisionGeometry.initialize([obj.collisionGeometry.minCorner; obj.collisionGeometry.maxCorner] + d, obj.collisionGeometry.tag, obj.collisionGeometry.label);
    elseif isa(obj.collisionGeometry, 'spherical')
        obj.collisionGeometry = obj.collisionGeometry.initialize(obj.collisionGeometry.center + d, obj.collisionGeometry.radius, obj.collisionGeometry.tag, obj.collisionGeometry.label);
    else
        error("?");
    end
end