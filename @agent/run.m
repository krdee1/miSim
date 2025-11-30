function obj = run(obj, domain, partitioning)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'agent')};
        domain (1, 1) {mustBeGeometry};
        partitioning (:, :) double;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'agent')};
    end

    % Collect objective function values across partition
    partitionMask = partitioning == obj.index;
    objectiveValues = domain.objective.values(partitionMask); % f(omega) on W_n

    % Compute sensor performance across partition
    maskedX = domain.objective.X(partitionMask);
    maskedY = domain.objective.Y(partitionMask);
    sensorValues = obj.sensorModel.sensorPerformance(obj.pos, obj.pan, obj.tilt, [maskedX, maskedY, zeros(size(maskedX))]); % S_n(omega, P_n) on W_n

    % Find agent's performance
    obj.performance = [obj.performance; sum(objectiveValues .* sensorValues, 'all')];

    %%
    % Put the values back into the form of the partition
    F = NaN(size(partitionMask));
    F(partitionMask) = objectiveValues;
    S = NaN(size(partitionMask));
    S(partitionMask) = sensorValues;

    % Compute gradient on agent's performance
    [gradSensorPerformanceX, gradSensorPerformanceY] = gradient(S, domain.objective.discretizationStep); % grad S_n
    [gradObjectiveX, gradObjectiveY] = gradient(F, domain.objective.discretizationStep); % grad f

    gradS = cat(3, gradSensorPerformanceX, gradSensorPerformanceY, zeros(size(gradSensorPerformanceX))); % grad S_n
    gradF = cat(3, gradObjectiveX, gradObjectiveY, zeros(size(gradObjectiveX))); % grad f

    % grad(s*f) = grad(f) * s + f * grad(s) - product rule (f scalar field, s vector field)
    gradC = S .* gradF + F .* gradS; % second term provides altitude
    % normalize in x3 dimension and find the direction which maximizes ascent
    nGradC = vecnorm(gradC, 2, 3);
    [xNextIdx, yNextIdx] = find(nGradC == max(nGradC, [], 'all')); % find direction of steepest increase
    pNext = [floor(mean(unique(domain.objective.X(:, xNextIdx)))), floor(mean(unique(domain.objective.Y(yNextIdx, :)))), obj.pos(3)]; % have to do some unfortunate rounding here soemtimes

    vDir = (pNext - obj.pos)./norm(pNext - obj.pos, 2);
    nextPos = obj.pos + vDir * 0.2;    

    % Move to next position
    % (dynamics not modeled at this time)
    obj.lastPos = obj.pos;
    obj.pos = nextPos;

    % Calculate movement
    d = obj.pos - obj.collisionGeometry.center;

    % Reinitialize collision geometry in the new position
    obj.collisionGeometry = obj.collisionGeometry.initialize([obj.collisionGeometry.minCorner; obj.collisionGeometry.maxCorner] + d, obj.collisionGeometry.tag, obj.collisionGeometry.label);
end