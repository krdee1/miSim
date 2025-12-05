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
    
    % Update collision barrier function
    % first part evaluates to +/-1 if the point is outside/inside the collision geometry
    % Second part determines the distance from the point to the boundary of the collision geometry
    obj.barrierFunction = @(x) (1 - 2 * obj.collisionGeometry.contains(x)) * obj.collisionGeometry.distance(x); % x is 1x3
    obj.dBarrierFunction = @(x) obj.collisionGeometry.distanceGradient(x); % x is 1x3

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
    C = S.* F;
    obj.performance = [obj.performance, sum(C(~isnan(C)))];

    % Compute gradient on agent's performance
    [gradCX, gradCY] = gradient(C, domain.objective.discretizationStep); % grad C;
    gradC = cat(3, gradCX, gradCY, zeros(size(gradCX))); % temp zeros for gradCZ
    nGradC = vecnorm(gradC, 2, 3);

    if obj.debug
        % Compute additional component-level values for diagnosing issues
        [gradSensorPerformanceX, gradSensorPerformanceY] = gradient(S, domain.objective.discretizationStep); % grad S_n
        [gradObjectiveX, gradObjectiveY] = gradient(F, domain.objective.discretizationStep); % grad f
        gradS = cat(3, gradSensorPerformanceX, gradSensorPerformanceY, zeros(size(gradSensorPerformanceX))); % grad S_n
        gradF = cat(3, gradObjectiveX, gradObjectiveY, zeros(size(gradObjectiveX))); % grad f

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

    % Use largest grad(C) value to find the direction of the next position
    [xNextIdx, yNextIdx] = find(nGradC == max(nGradC, [], 'all'));
    roundingScale = 10^-log10(domain.objective.discretizationStep);
    pNext = [floor(roundingScale .* mean(unique(domain.objective.X(:, xNextIdx))))./roundingScale, floor(roundingScale .* mean(unique(domain.objective.Y(yNextIdx, :))))./roundingScale, obj.pos(3)]; % have to do some unfortunate rounding here soemtimes

    % Determine next position
    vDir = (pNext - obj.pos)./norm(pNext - obj.pos, 2);
    rate = 0.2 - 0.004 * t;
    nextPos = obj.pos + vDir * rate;

    % Move to next position
    obj.lastPos = obj.pos;
    obj.pos = nextPos;

    % Reinitialize collision geometry in the new position
    d = obj.pos - obj.collisionGeometry.center;
    obj.collisionGeometry = obj.collisionGeometry.initialize([obj.collisionGeometry.minCorner; obj.collisionGeometry.maxCorner] + d, obj.collisionGeometry.tag, obj.collisionGeometry.label);
end