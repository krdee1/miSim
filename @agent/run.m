function obj = run(obj, domain, partitioning, t, index)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'agent')};
        domain (1, 1) {mustBeGeometry};
        partitioning (:, :) double;
        t (1, 1) double;
        index (1, 1) double;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'agent')};
    end

    % Collect objective function values across partition
    partitionMask = partitioning == index;
    objectiveValues = domain.objective.values(partitionMask); % f(omega) on W_n

    % Compute sensor performance across partition
    maskedX = domain.objective.X(partitionMask);
    maskedY = domain.objective.Y(partitionMask);
    zFactor = 10;
    sensorValues = obj.sensorModel.sensorPerformance(obj.pos, obj.pan, obj.tilt, [maskedX, maskedY, zeros(size(maskedX))]); % S_n(omega, P_n) on W_n
    sensorValuesLower = obj.sensorModel.sensorPerformance(obj.pos - [0, 0, zFactor * domain.objective.discretizationStep], obj.pan, obj.tilt, [maskedX, maskedY, zeros(size(maskedX))]); % S_n(omega, P_n - [0, 0, z]) on W_n
    sensorValuesHigher = obj.sensorModel.sensorPerformance(obj.pos + [0, 0, zFactor * domain.objective.discretizationStep], obj.pan, obj.tilt, [maskedX, maskedY, zeros(size(maskedX))]); % S_n(omega, P_n - [0, 0, z]) on W_n

    % Put the values back into the form of the partition to enable basic operations on this data
    F = NaN(size(partitionMask));
    F(partitionMask) = objectiveValues;
    S = NaN(size(partitionMask));
    Slower = S;
    Shigher = S;
    S(partitionMask) = sensorValues;
    Slower(partitionMask) = sensorValuesLower;
    Shigher(partitionMask) = sensorValuesHigher;

    % Find agent's performance
    C = S .* F;
    obj.performance = [obj.performance, sum(C(~isnan(C)))]; % at current Z only
    C = cat(3, Shigher, S, Slower) .* F;

    % Compute gradient on agent's performance
    [gradCX, gradCY, gradCZ] = gradient(C, domain.objective.discretizationStep); % grad C
    gradC = cat(4, gradCX, gradCY, gradCZ);
    nGradC = vecnorm(gradC, 2, 4);

    if obj.debug
        % Compute additional component-level values for diagnosing issues
        [gradSensorPerformanceX, gradSensorPerformanceY] = gradient(S, domain.objective.discretizationStep); % grad S_n
        [gradObjectiveX, gradObjectiveY] = gradient(F, domain.objective.discretizationStep); % grad f
        gradS = cat(3, gradSensorPerformanceX, gradSensorPerformanceY, zeros(size(gradSensorPerformanceX))); % grad S_n
        gradF = cat(3, gradObjectiveX, gradObjectiveY, zeros(size(gradObjectiveX))); % grad f

        ii = 8;
        hold(obj.debugFig.Children(1).Children(ii), "on");
        cla(obj.debugFig.Children(1).Children(ii));
        imagesc(obj.debugFig.Children(1).Children(ii), F./max(F, [], 'all'));
        hold(obj.debugFig.Children(1).Children(ii), "off");
        ii = ii - 1;
        hold(obj.debugFig.Children(1).Children(ii), "on");
        cla(obj.debugFig.Children(1).Children(ii));
        imagesc(obj.debugFig.Children(1).Children(ii), S./max(S, [], 'all'));
        hold(obj.debugFig.Children(1).Children(ii), "off");
        ii = ii - 1;
        hold(obj.debugFig.Children(1).Children(ii), "on");
        cla(obj.debugFig.Children(1).Children(ii));
        imagesc(obj.debugFig.Children(1).Children(ii), vecnorm(gradF, 2, 3)./max(vecnorm(gradF, 2, 3), [], 'all'));
        hold(obj.debugFig.Children(1).Children(ii), "off");
        ii = ii - 1;
        hold(obj.debugFig.Children(1).Children(ii), "on");
        cla(obj.debugFig.Children(1).Children(ii));
        imagesc(obj.debugFig.Children(1).Children(ii), vecnorm(gradS, 2, 3)./max(vecnorm(gradS, 2, 3), [], 'all'));
        hold(obj.debugFig.Children(1).Children(ii), "off");
        ii = ii - 1;
        hold(obj.debugFig.Children(1).Children(ii), "on");
        cla(obj.debugFig.Children(1).Children(ii));
        imagesc(obj.debugFig.Children(1).Children(ii), S .* vecnorm(gradF, 2, 3)./max(vecnorm(gradF, 2, 3), [], 'all'));
        hold(obj.debugFig.Children(1).Children(ii), "off");
        ii = ii - 1;
        hold(obj.debugFig.Children(1).Children(ii), "on");
        cla(obj.debugFig.Children(1).Children(ii));
        imagesc(obj.debugFig.Children(1).Children(ii), F .* vecnorm(gradS, 2, 3)./max(vecnorm(gradS, 2, 3), [], 'all')./(max(F .* vecnorm(gradS, 2, 3)./max(vecnorm(gradS, 2, 3), [], 'all'))));
        hold(obj.debugFig.Children(1).Children(ii), "off");
        
        ii = ii - 1;
        hold(obj.debugFig.Children(1).Children(ii), "on");
        cla(obj.debugFig.Children(1).Children(ii));
        imagesc(obj.debugFig.Children(1).Children(ii), C./max(C, [], 'all'));
        hold(obj.debugFig.Children(1).Children(ii), "off");
        ii = ii - 1;
        hold(obj.debugFig.Children(1).Children(ii), "on");
        cla(obj.debugFig.Children(1).Children(ii));
        imagesc(obj.debugFig.Children(1).Children(ii), nGradC./max(nGradC, [], 'all'));
        hold(obj.debugFig.Children(1).Children(ii), "off");
        [x, y] = find(nGradC == max(nGradC, [], "all"));

        % just pick one
        r = randi([1, size(x, 1)]);
        x = x(r); y = y(r);

        % switch them
        temp = x;
        x = y;
        y = temp;
        
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
    [xNextIdx, yNextIdx, zNextIdx] = ind2sub(size(nGradC), find(nGradC == max(nGradC, [], 'all')));
    disp(zNextIdx)
    % switch them
    temp = xNextIdx;
    xNextIdx = yNextIdx;
    yNextIdx = temp;

    roundingScale = 10^-log10(domain.objective.discretizationStep);
    zKey = zFactor * [1; 0; -1];
    pNext = [floor(roundingScale .* mean(unique(domain.objective.X(:, xNextIdx))))./roundingScale, floor(roundingScale .* mean(unique(domain.objective.Y(yNextIdx, :))))./roundingScale, obj.pos(3) + zKey(zNextIdx)]; % have to do some unfortunate rounding here sometimes

    % Determine next position
    vDir = (pNext - obj.pos)./norm(pNext - obj.pos, 2);
    rate = 0.1 - 0.0004 * t; % slow down as you get closer, coming to a stop by the end
    nextPos = obj.pos + vDir * rate;

    % Move to next position
    obj.lastPos = obj.pos;
    obj.pos = nextPos;

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