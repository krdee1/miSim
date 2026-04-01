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
    obj.barriers(1:idx, obj.timestepIndex) = h(triu(true(size(h)), 1));
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

    obj.barriers(idx:(idx + numel(hObs) - 1), obj.timestepIndex) = reshape(hObs, [], 1);
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

        obj.barriers(idx:(idx + 5), obj.timestepIndex) = [h_xMin; h_xMax; h_yMin; h_yMax; h_zMin; h_zMax];
        idx = idx + 6;
    end

    % Add communication network constraints
    hComms = NaN(nAgents, nAgents);
    hComms(logical(eye(nAgents))) = 0;
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
    obj.barriers(idx:(idx + length(hComms(triu(true(size(hComms)), 1))) - 1), obj.timestepIndex) = hComms(triu(true(size(hComms)), 1));

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