function obj = run(obj, domain, partitioning, t)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'agent')};
        domain (1, 1) {mustBeGeometry};
        partitioning (:, :) double;
        t (1, 1) double;
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

    % Put the values back into the form of the partition to enable basic operations on this data
    F = NaN(size(partitionMask));
    F(partitionMask) = objectiveValues;
    S = NaN(size(partitionMask));
    S(partitionMask) = sensorValues;

    % Find agent's performance
    C = S.* F; % try gradient on this directly
    obj.performance = [obj.performance  sum(C(~isnan(C)))];

    % Compute gradient on agent's performance
    [gradSensorPerformanceX, gradSensorPerformanceY] = gradient(S, domain.objective.discretizationStep); % grad S_n
    [gradObjectiveX, gradObjectiveY] = gradient(F, domain.objective.discretizationStep); % grad f

    gradS = cat(3, gradSensorPerformanceX, gradSensorPerformanceY, zeros(size(gradSensorPerformanceX))); % grad S_n
    gradF = cat(3, gradObjectiveX, gradObjectiveY, zeros(size(gradObjectiveX))); % grad f

    [gradCX, gradCY] = gradient(C, domain.objective.discretizationStep); % grad C;
    gradC = cat(3, gradCX, gradCY, zeros(size(gradCX))); % temp zeros for gradCZ
    nGradC = vecnorm(gradC, 2, 3);

    if obj.debug
        ii = 8;
        hold(obj.debugFig.Children(1).Children(ii), "on");
        imagesc(obj.debugFig.Children(1).Children(ii), F./max(F, [], 'all'));
        hold(obj.debugFig.Children(1).Children(ii), "off");
        ii = ii - 1;
        hold(obj.debugFig.Children(1).Children(ii), "on");
        imagesc(obj.debugFig.Children(1).Children(ii), S./max(S, [], 'all'));
        hold(obj.debugFig.Children(1).Children(ii), "off");
        ii = ii - 1;
        hold(obj.debugFig.Children(1).Children(ii), "on");
        imagesc(obj.debugFig.Children(1).Children(ii), vecnorm(gradF, 2, 3)./max(vecnorm(gradF, 2, 3), [], 'all'));
        hold(obj.debugFig.Children(1).Children(ii), "off");
        ii = ii - 1;
        hold(obj.debugFig.Children(1).Children(ii), "on");
        imagesc(obj.debugFig.Children(1).Children(ii), vecnorm(gradS, 2, 3)./max(vecnorm(gradS, 2, 3), [], 'all'));
        hold(obj.debugFig.Children(1).Children(ii), "off");
        ii = ii - 1;
        hold(obj.debugFig.Children(1).Children(ii), "on");
        imagesc(obj.debugFig.Children(1).Children(ii), S .* vecnorm(gradF, 2, 3)./max(vecnorm(gradF, 2, 3), [], 'all'));
        hold(obj.debugFig.Children(1).Children(ii), "off");
        ii = ii - 1;
        hold(obj.debugFig.Children(1).Children(ii), "on");
        imagesc(obj.debugFig.Children(1).Children(ii), F .* vecnorm(gradS, 2, 3)./max(vecnorm(gradS, 2, 3), [], 'all')./(max(F .* vecnorm(gradS, 2, 3)./max(vecnorm(gradS, 2, 3), [], 'all'))));
        hold(obj.debugFig.Children(1).Children(ii), "off");
        
        ii = ii - 1;
        hold(obj.debugFig.Children(1).Children(ii), "on");
        imagesc(obj.debugFig.Children(1).Children(ii), C./max(C, [], 'all'));
        hold(obj.debugFig.Children(1).Children(ii), "off");
        ii = ii - 1;
        hold(obj.debugFig.Children(1).Children(ii), "on");
        imagesc(obj.debugFig.Children(1).Children(ii), nGradC./max(nGradC, [], 'all'));
        hold(obj.debugFig.Children(1).Children(ii), "off");
        [x, y] = find(nGradC == max(nGradC, [], "all"));

        % just pick one
        r = randi([1, size(x, 1)]);
        x = x(r); y = y(r);
        
        % find objective location in discrete domain
        [~, xIdx] = find(domain.objective.groundPos(1) == domain.objective.X);
        xIdx = unique(xIdx);
        [yIdx, ~] = find(domain.objective.groundPos(2) == domain.objective.Y);
        yIdx = unique(yIdx);
        for ii = 8:-1:1
            hold(obj.debugFig.Children(1).Children(ii), "on");
            % plot GA selection
            scatter(obj.debugFig.Children(1).Children(ii), x, y, 'go');
            scatter(obj.debugFig.Children(1).Children(ii), x, y, 'g+');
            % plot objective center
            scatter(obj.debugFig.Children(1).Children(ii), xIdx, yIdx, 'ro');
            scatter(obj.debugFig.Children(1).Children(ii), xIdx, yIdx, 'r+');
            hold(obj.debugFig.Children(1).Children(ii), "off");
        end
    end

    % grad(s*f) = grad(f) * s + f * grad(s) - product rule (f scalar field, s vector field)
    % gradC = S .* abs(gradF) + F .* abs(gradS); % second term provides altitude
    % normalize in x3 dimension and find the direction which maximizes ascent
    % nGradC = vecnorm(gradC, 2, 3);
    [xNextIdx, yNextIdx] = find(nGradC == max(nGradC, [], 'all')); % find direction of steepest increase
    roundingScale = 10^-log10(domain.objective.discretizationStep);
    pNext = [floor(roundingScale .* mean(unique(domain.objective.X(:, xNextIdx))))./roundingScale, floor(roundingScale .* mean(unique(domain.objective.Y(yNextIdx, :))))./roundingScale, obj.pos(3)]; % have to do some unfortunate rounding here soemtimes

    vDir = (pNext - obj.pos)./norm(pNext - obj.pos, 2);
    rate = 0.2 - 0.004 * t;
    nextPos = obj.pos + vDir * rate;

    % Move to next position
    % (dynamics not modeled at this time)
    obj.lastPos = obj.pos;
    obj.pos = nextPos;

    % Calculate movement
    d = obj.pos - obj.collisionGeometry.center;

    % Reinitialize collision geometry in the new position
    obj.collisionGeometry = obj.collisionGeometry.initialize([obj.collisionGeometry.minCorner; obj.collisionGeometry.maxCorner] + d, obj.collisionGeometry.tag, obj.collisionGeometry.label);
end