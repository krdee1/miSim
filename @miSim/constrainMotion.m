function [obj] = constrainMotion(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end

    nAgents = size(obj.agents, 1);

    % Compute current velocity and desired control input
    v = zeros(nAgents, 3);        % current velocity (for drift term in DI mode)
    u_desired = zeros(nAgents, 3); % desired control: velocity (SI) or acceleration (DI)
    for ii = 1:nAgents
        if obj.useDoubleIntegrator
            v(ii, :) = obj.agents{ii}.lastVel;
            u_desired(ii, :) = (obj.agents{ii}.vel - obj.agents{ii}.lastVel) / obj.timestep;
        else
            v(ii, :) = (obj.agents{ii}.pos - obj.agents{ii}.lastPos) ./ obj.timestep;
            u_desired(ii, :) = v(ii, :);
        end
    end
    if ~obj.useDoubleIntegrator && (all(isnan(v), "all") || all(v == zeros(nAgents, 3), "all"))
        % Single-integrator: agents are not attempting to move
        return;
    end
    if obj.useDoubleIntegrator && all(u_desired == 0, "all") && all(v == 0, "all")
        % Double-integrator: no desired acceleration and no existing velocity
        return;
    end

    % Initialize QP based on number of agents and obstacles
    kk = 1;
    A = zeros(obj.numBarriers, 3 * nAgents);
    b = zeros(obj.numBarriers, 1);

    % Set up collision avoidance constraints
    h = NaN(nAgents, nAgents);
    h(logical(eye(nAgents))) = 0; % self value is 0
    for ii = 1:(nAgents - 1)
        for jj = (ii + 1):nAgents
            h(ii, jj) = norm(obj.agents{ii}.lastPos - obj.agents{jj}.lastPos)^2 - (obj.agents{ii}.collisionGeometry.radius + obj.agents{jj}.collisionGeometry.radius)^2;
            h(jj, ii) = h(ii, jj);

            A(kk, (3 * ii - 2):(3 * ii)) =  -2 * (obj.agents{ii}.lastPos - obj.agents{jj}.lastPos);
            A(kk, (3 * jj - 2):(3 * jj)) = -A(kk, (3 * ii - 2):(3 * ii));
            % Slack derived from existing params: recovery velocity = max gradient approach velocity.
            % Correction splits between 2 agents, so |A| = 2*r_sum
            r_sum_ij = obj.agents{ii}.collisionGeometry.radius + obj.agents{jj}.collisionGeometry.radius;
            v_max_ij = max(obj.agents{ii}.initialStepSize, obj.agents{jj}.initialStepSize) / obj.timestep;
            hMin = -(4 * r_sum_ij * v_max_ij / obj.barrierGain)^(1 / obj.barrierExponent);
            if norm(A(kk, :)) < 1e-9
                % Agents are coincident: A-row is zero, so b < 0 would make
                % 0 ≤ b unsatisfiable. Fall back to b = 0 (no correction possible).
                b(kk) = 0;
            else
                b(kk) = obj.barrierGain * max(hMin, h(ii, jj))^obj.barrierExponent;
            end
            kk = kk + 1;
        end
    end

    idx = length(h(triu(true(size(h)), 1)));    
    if coder.target('MATLAB')
	obj.barriers(1:idx, obj.timestepIndex) = h(triu(true(size(h)), 1));
    end
    idx = idx + 1;

    hObs = NaN(nAgents, size(obj.obstacles, 1));
    % Set up obstacle avoidance constraints
    for ii = 1:nAgents
        for jj = 1:size(obj.obstacles, 1)
            % find closest position to agent on/in obstacle
            cPos = obj.obstacles{jj}.closestToPoint(obj.agents{ii}.lastPos);

            hObs(ii, jj) = dot(obj.agents{ii}.lastPos - cPos, obj.agents{ii}.lastPos - cPos) - obj.agents{ii}.collisionGeometry.radius^2;

            A(kk, (3 * ii - 2):(3 * ii)) = -2 * (obj.agents{ii}.lastPos - cPos);
            % Floor for single-agent constraint: full correction on one agent, |A| = 2*r_i
            r_i = obj.agents{ii}.collisionGeometry.radius;
            v_max_i = obj.agents{ii}.initialStepSize / obj.timestep;
            hMin = -(2 * r_i * v_max_i / obj.barrierGain)^(1 / obj.barrierExponent);
            b(kk) = obj.barrierGain * max(hMin, hObs(ii, jj))^obj.barrierExponent;

            kk = kk + 1;
        end
    end

    if coder.target('MATLAB')
	obj.barriers(idx:(idx + numel(hObs) - 1), obj.timestepIndex) = reshape(hObs, [], 1);
    end
    idx = idx + numel(hObs);

    % Set up domain constraints (walls and ceiling only)
    % Floor constraint is implicit with an obstacle corresponding to the
    % minimum allowed altitude, but I included it anyways
    h_xMin = 0.0; h_xMax = 0.0; h_yMin = 0.0; h_yMax = 0.0; h_zMin = 0.0; h_zMax = 0.0;
    for ii = 1:nAgents
        % X minimum
        h_xMin = (obj.agents{ii}.lastPos(1) - obj.domain.minCorner(1)) - obj.agents{ii}.collisionGeometry.radius;
        A(kk, (3 * ii - 2):(3 * ii)) = [-1, 0, 0];
        b(kk) = obj.barrierGain * max(0, h_xMin)^obj.barrierExponent;
        kk = kk + 1;

        % X maximum
        h_xMax = (obj.domain.maxCorner(1) - obj.agents{ii}.lastPos(1)) - obj.agents{ii}.collisionGeometry.radius;
        A(kk, (3 * ii - 2):(3 * ii)) = [1, 0, 0];
        b(kk) = obj.barrierGain * max(0, h_xMax)^obj.barrierExponent;
        kk = kk + 1;

        % Y minimum
        h_yMin = (obj.agents{ii}.lastPos(2) - obj.domain.minCorner(2)) - obj.agents{ii}.collisionGeometry.radius;
        A(kk, (3 * ii - 2):(3 * ii)) = [0, -1, 0];
        b(kk) = obj.barrierGain * max(0, h_yMin)^obj.barrierExponent;
        kk = kk + 1;

        % Y maximum
        h_yMax = (obj.domain.maxCorner(2) - obj.agents{ii}.lastPos(2)) - obj.agents{ii}.collisionGeometry.radius;
        A(kk, (3 * ii - 2):(3 * ii)) = [0, 1, 0];
        b(kk) = obj.barrierGain * max(0, h_yMax)^obj.barrierExponent;
        kk = kk + 1;

        % Z minimum — enforce z >= minAlt + radius (not just z >= domain floor + radius)
        h_zMin = (obj.agents{ii}.lastPos(3) - obj.minAlt) - obj.agents{ii}.collisionGeometry.radius;
        A(kk, (3 * ii - 2):(3 * ii)) = [0, 0, -1];
        b(kk) = obj.barrierGain * max(0, h_zMin)^obj.barrierExponent;
        kk = kk + 1;

        % Z maximum
        h_zMax = (obj.domain.maxCorner(3) - obj.agents{ii}.lastPos(3)) - obj.agents{ii}.collisionGeometry.radius;
        A(kk, (3 * ii - 2):(3 * ii)) = [0, 0, 1];
        b(kk) = obj.barrierGain * max(0, h_zMax)^obj.barrierExponent;
        kk = kk + 1;

        if coder.target('MATLAB')
	    obj.barriers(idx:(idx + 5), obj.timestepIndex) = [h_xMin; h_xMax; h_yMin; h_yMax; h_zMin; h_zMax];
        end
	idx = idx + 6;
    end

    % Add communication network constraints
    hComms = NaN(nAgents, nAgents);
    hComms(logical(eye(nAgents))) = 0;
    if ~obj.useSinrComms
        % Fixed-radius comms: keep each maintained pair within the smaller of
        % the two comms radii via the barrier h = r_comms^2 - dist^2.
        for ii = 1:(nAgents - 1)
            for jj = (ii + 1):nAgents
                if obj.constraintAdjacencyMatrix(ii, jj)
                    paddingFactor = 0.9; % Barrier at 90% of actual range; real comms still work beyond this
                    r_comms = paddingFactor * min([obj.agents{ii}.commsGeometry.radius, obj.agents{jj}.commsGeometry.radius]);
                    hComms(ii, jj) = r_comms^2 - norm(obj.agents{ii}.lastPos - obj.agents{jj}.lastPos)^2;

                    A(kk, (3 * ii - 2):(3 * ii)) =  2 * (obj.agents{ii}.lastPos - obj.agents{jj}.lastPos);
                    A(kk, (3 * jj - 2):(3 * jj)) = -A(kk, (3 * ii - 2):(3 * ii));

                    % One-step forward invariance: b = h/dt ensures h cannot
                    % go negative in a single timestep (linear approximation)
                    v_max_ij = max(obj.agents{ii}.initialStepSize, obj.agents{jj}.initialStepSize) / obj.timestep;
                    hMin = -4 * r_comms * v_max_ij * obj.timestep;
                    if norm(A(kk, :)) < 1e-9
                        b(kk) = 0;
                    else
                        b(kk) = max(hMin, hComms(ii, jj)) / obj.timestep;
                    end

                    kk = kk + 1;
                end
            end
        end
    else
        % SINR comms: a link is feasible only if each agent receives the other
        % above threshold, so every maintained pair is held with TWO barrier
        % rows, h = SINR - gammaCbf, one per receiver direction. Each gradient
        % couples the receiver, transmitter, and every interferer, so the QP
        % rows are dense across all agent blocks.
        % Maintain SINR a margin above the connectivity threshold so that
        % discrete-time overshoot of the (convex) SINR does not transiently
        % drop a maintained link below threshold. Mirrors the fixed-radius
        % barrier's 0.9 padding: the CBF holds SINR >= gammaCbf > gammaLin,
        % while connectivity (updateAdjacency) still uses gammaLin.
        paddingDb = 3.0;
        gammaCbf = 10^((obj.sinrThreshold + paddingDb) / 10);
        lastPositions = zeros(nAgents, 3);
        for ll = 1:nAgents
            lastPositions(ll, :) = obj.agents{ll}.lastPos;
        end
        for ii = 1:(nAgents - 1)
            for jj = (ii + 1):nAgents
                if obj.constraintAdjacencyMatrix(ii, jj)
                    % Direction 1: lower-index agent (ii) receives higher (jj).
                    % Upper triangle of hComms stores this direction's barrier.
                    [sinr1, grad1] = obj.sinrLink(lastPositions, ii, jj);
                    hComms(ii, jj) = sinr1 - gammaCbf;
                    % CBF row: A = -grad(h), b = h/dt (one-step forward invariance)
                    for aa = 1:nAgents
                        A(kk, (3 * aa - 2):(3 * aa)) = -grad1(aa, :);
                    end
                    if norm(A(kk, :)) < 1e-9
                        b(kk) = 0;
                    else
                        b(kk) = hComms(ii, jj) / obj.timestep;
                    end
                    kk = kk + 1;

                    % Direction 2: higher-index agent (jj) receives lower (ii).
                    % Lower triangle of hComms stores this direction's barrier.
                    [sinr2, grad2] = obj.sinrLink(lastPositions, jj, ii);
                    hComms(jj, ii) = sinr2 - gammaCbf;
                    for aa = 1:nAgents
                        A(kk, (3 * aa - 2):(3 * aa)) = -grad2(aa, :);
                    end
                    if norm(A(kk, :)) < 1e-9
                        b(kk) = 0;
                    else
                        b(kk) = hComms(jj, ii) / obj.timestep;
                    end
                    kk = kk + 1;
                end
            end
        end
    end

    if coder.target('MATLAB')
        if obj.useSinrComms
            % Both directions are maintained: log the upper triangle
            % (lower-index receiver) then the lower triangle (higher-index
            % receiver), matching the doubled comms-barrier allocation.
            commsLog = [hComms(triu(true(size(hComms)), 1)); hComms(tril(true(size(hComms)), -1))];
        else
            commsLog = hComms(triu(true(size(hComms)), 1));
        end
        obj.barriers(idx:(idx + numel(commsLog) - 1), obj.timestepIndex) = commsLog;
    end

    % Double-integrator: transform QP from velocity to acceleration space.
    % Single-integrator constraint: A * v <= b
    % Double-integrator: A * a <= (b - A * v_current) / dt
    if obj.useDoubleIntegrator
        v_flat = reshape(v', 3 * nAgents, 1);
        b = (b - A * v_flat) / obj.timestep;
    end

    % Solve QP: minimize ||u - u_desired||²
    uhat = reshape(u_desired', 3 * nAgents, 1);
    H = 2 * eye(3 * nAgents);
    f = -2 * uhat;

    % Update solution based on constraints
    if coder.target('MATLAB')
        assert(size(A,2) == size(H,1))
        assert(size(A,1) == size(b,1))
        assert(size(H,1) == length(f))
    end
    opt = optimoptions("quadprog", "Display", "off", "Algorithm", "active-set", "UseCodegenSolver", true);
    x0 = zeros(size(H, 1), 1);
    [uNew, ~, exitflag] = quadprog(H, double(f), A, b, [], [], [], [], x0, opt);
    uNew = reshape(uNew, 3, nAgents)';

    if exitflag < 0
        % Infeasible or other hard failure: hold all agents at current positions
        if coder.target('MATLAB')
            warning("QP infeasible (exitflag=%d), holding positions.", int16(exitflag));
        else
            fprintf("[constrainMotion] QP infeasible (exitflag=%d), holding positions\n", int16(exitflag));
        end
        uNew = zeros(nAgents, 3);
    elseif exitflag == 0
        % Max iterations exceeded: use suboptimal solution already in uNew
        if coder.target('MATLAB')
            warning("QP max iterations exceeded, using suboptimal solution.");
        else
            fprintf("[constrainMotion] QP max iterations exceeded, using suboptimal solution\n");
        end
    end

    % Update agent state using the constrained control input
    for ii = 1:size(uNew, 1)
        if obj.useDoubleIntegrator
            % uNew is constrained acceleration
            obj.agents{ii}.vel = obj.agents{ii}.lastVel + uNew(ii, :) * obj.timestep;
            obj.agents{ii}.pos = obj.agents{ii}.lastPos + obj.agents{ii}.vel * obj.timestep;
        else
            % uNew is constrained velocity
            obj.agents{ii}.pos = obj.agents{ii}.lastPos + uNew(ii, :) * obj.timestep;
        end
    end

    % Here we run this at the simulation level, but in reality there is no
    % parent level, so this would be run independently on each agent.
    % Running at the simulation level is just meant to simplify the
    % simulation

end
