function [obj] = constrainMotion(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end

    nAgents = size(obj.agents, 1);

    if nAgents < 2
        nAAPairs = 0;
    else
        nAAPairs = nchoosek(nAgents, 2); % unique agent/agent pairs
    end

    % Compute velocity matrix from unconstrained gradient-ascent step
    v = zeros(nAgents, 3);
    for ii = 1:nAgents
        v(ii, :) = (obj.agents{ii}.pos - obj.agents{ii}.lastPos) ./ obj.timestep;
    end
    if all(isnan(v), "all") || all(v == zeros(nAgents, 3), "all")
        % Agents are not attempting to move, so there is no motion to be
        % constrained
        return;
    end

    % Initialize QP based on number of agents and obstacles
    nAOPairs = nAgents * size(obj.obstacles, 1); % unique agent/obstacle pairs
    nADPairs = nAgents * 6; % agents x (4 walls + 1 floor + 1 ceiling)
    nLNAPairs = sum(obj.constraintAdjacencyMatrix, "all") - nAgents;
    total = nAAPairs + nAOPairs + nADPairs + nLNAPairs;
    kk = 1;
    A = zeros(total, 3 * nAgents);
    b = zeros(total, 1);

    % Set up collision avoidance constraints
    h = NaN(nAgents, nAgents);
    h(logical(eye(nAgents))) = 0; % self value is 0
    for ii = 1:(nAgents - 1)
        for jj = (ii + 1):nAgents
            h(ii, jj) = norm(obj.agents{ii}.pos - obj.agents{jj}.pos)^2 - (obj.agents{ii}.collisionGeometry.radius + obj.agents{jj}.collisionGeometry.radius)^2;
            h(jj, ii) = h(ii, jj);

            A(kk, (3 * ii - 2):(3 * ii)) =  -2 * (obj.agents{ii}.pos - obj.agents{jj}.pos);
            A(kk, (3 * jj - 2):(3 * jj)) = -A(kk, (3 * ii - 2):(3 * ii));
            % Slack derived from existing params: recovery velocity = max gradient approach velocity.
            % Correction splits between 2 agents, so |A| = 2*r_sum
            r_sum_ij = obj.agents{ii}.collisionGeometry.radius + obj.agents{jj}.collisionGeometry.radius;
            v_max_ij = max(obj.agents{ii}.initialStepSize, obj.agents{jj}.initialStepSize) / obj.timestep;
            slack = -(4 * r_sum_ij * v_max_ij / obj.barrierGain)^(1 / obj.barrierExponent);
            if norm(A(kk, :)) < 1e-9
                % Agents are coincident: A-row is zero, so b < 0 would make
                % 0 ≤ b unsatisfiable. Fall back to b = 0 (no correction possible).
                b(kk) = 0;
            else
                b(kk) = obj.barrierGain * max(slack, h(ii, jj))^obj.barrierExponent;
            end
            kk = kk + 1;
        end
    end

    hObs = NaN(nAgents, size(obj.obstacles, 1));
    % Set up obstacle avoidance constraints
    for ii = 1:nAgents
        for jj = 1:size(obj.obstacles, 1)
            % find closest position to agent on/in obstacle
            cPos = obj.obstacles{jj}.closestToPoint(obj.agents{ii}.pos);

            hObs(ii, jj) = dot(obj.agents{ii}.pos - cPos, obj.agents{ii}.pos - cPos) - obj.agents{ii}.collisionGeometry.radius^2;

            A(kk, (3 * ii - 2):(3 * ii)) = -2 * (obj.agents{ii}.pos - cPos);
            % Floor for single-agent constraint: full correction on one agent, |A| = 2*r_i
            r_i = obj.agents{ii}.collisionGeometry.radius;
            v_max_i = obj.agents{ii}.initialStepSize / obj.timestep;
            h_floor_i = -(2 * r_i * v_max_i / obj.barrierGain)^(1 / obj.barrierExponent);
            b(kk) = obj.barrierGain * max(h_floor_i, hObs(ii, jj))^obj.barrierExponent;

            kk = kk + 1;
        end
    end

    % Set up domain constraints (walls and ceiling only)
    % Floor constraint is implicit with an obstacle corresponding to the
    % minimum allowed altitude, but I included it anyways
    h_xMin = 0.0; h_xMax = 0.0; h_yMin = 0.0; h_yMax = 0.0; h_zMin = 0.0; h_zMax = 0.0;
    for ii = 1:nAgents
        % X minimum
        h_xMin = (obj.agents{ii}.pos(1) - obj.domain.minCorner(1)) - obj.agents{ii}.collisionGeometry.radius;
        A(kk, (3 * ii - 2):(3 * ii)) = [-1, 0, 0];
        b(kk) = obj.barrierGain * max(0, h_xMin)^obj.barrierExponent;
        kk = kk + 1;

        % X maximum
        h_xMax = (obj.domain.maxCorner(1) - obj.agents{ii}.pos(1)) - obj.agents{ii}.collisionGeometry.radius;
        A(kk, (3 * ii - 2):(3 * ii)) = [1, 0, 0];
        b(kk) = obj.barrierGain * max(0, h_xMax)^obj.barrierExponent;
        kk = kk + 1;

        % Y minimum
        h_yMin = (obj.agents{ii}.pos(2) - obj.domain.minCorner(2)) - obj.agents{ii}.collisionGeometry.radius;
        A(kk, (3 * ii - 2):(3 * ii)) = [0, -1, 0];
        b(kk) = obj.barrierGain * max(0, h_yMin)^obj.barrierExponent;
        kk = kk + 1;

        % Y maximum
        h_yMax = (obj.domain.maxCorner(2) - obj.agents{ii}.pos(2)) - obj.agents{ii}.collisionGeometry.radius;
        A(kk, (3 * ii - 2):(3 * ii)) = [0, 1, 0];
        b(kk) = obj.barrierGain * max(0, h_yMax)^obj.barrierExponent;
        kk = kk + 1;

        % Z minimum
        h_zMin = (obj.agents{ii}.pos(3) - obj.domain.minCorner(3)) - obj.agents{ii}.collisionGeometry.radius;
        A(kk, (3 * ii - 2):(3 * ii)) = [0, 0, -1];
        b(kk) = obj.barrierGain * max(0, h_zMin)^obj.barrierExponent;
        kk = kk + 1;

        % Z maximum
        h_zMax = (obj.domain.maxCorner(3) - obj.agents{ii}.pos(3)) - obj.agents{ii}.collisionGeometry.radius;
        A(kk, (3 * ii - 2):(3 * ii)) = [0, 0, 1];
        b(kk) = obj.barrierGain * max(0, h_zMax)^obj.barrierExponent;
        kk = kk + 1;
    end

    if coder.target('MATLAB')
        % Save off h function values (logging only — not needed in compiled mode)
        obj.h(:, obj.timestepIndex) = [h(triu(true(nAgents), 1)); reshape(hObs, [], 1); h_xMin; h_xMax; h_yMin; h_yMax; h_zMin; h_zMax;];
    end

    % Add communication network constraints
    hComms = NaN(nAgents, nAgents);
    hComms(logical(eye(nAgents))) = 0;
    for ii = 1:(nAgents - 1)
        for jj = (ii + 1):nAgents
            if obj.constraintAdjacencyMatrix(ii, jj)
                hComms(ii, jj) = min([obj.agents{ii}.commsGeometry.radius, obj.agents{jj}.commsGeometry.radius])^2 - norm(obj.agents{ii}.pos - obj.agents{jj}.pos)^2;

                A(kk, (3 * ii - 2):(3 * ii)) =  2 * (obj.agents{ii}.pos - obj.agents{jj}.pos);
                A(kk, (3 * jj - 2):(3 * jj)) = -A(kk, (3 * ii - 2):(3 * ii));
                b(kk) = obj.barrierGain * max(0, hComms(ii, jj))^obj.barrierExponent;

                kk = kk + 1;
            end
        end
    end

    % Solve QP program generated earlier
    vhat = reshape(v', 3 * nAgents, 1);
    H = 2 * eye(3 * nAgents);
    f = -2 * vhat;

    % Update solution based on constraints
    if coder.target('MATLAB')
        assert(size(A,2) == size(H,1))
        assert(size(A,1) == size(b,1))
        assert(size(H,1) == length(f))
    end
    opt = optimoptions("quadprog", "Display", "off", "Algorithm", "active-set", "UseCodegenSolver", true);
    x0 = zeros(size(H, 1), 1);
    [vNew, ~, exitflag, m] = quadprog(H, double(f), A, b, [], [], [], [], x0, opt);
    if coder.target('MATLAB')
        assert(exitflag == 1, sprintf("quadprog failure... %s%s", newline, m.message));
    end
    vNew = reshape(vNew, 3, nAgents)';

    if exitflag <= 0
        if coder.target('MATLAB')
            warning("QP failed, continuing with unconstrained solution...")
        end
        vNew = v;
    end

    % Update the "next position" that was previously set by unconstrained
    % GA using the constrained solution produced here
    for ii = 1:size(vNew, 1)
        obj.agents{ii}.pos = obj.agents{ii}.lastPos + vNew(ii, :) * obj.timestep;
    end

    % Here we run this at the simulation level, but in reality there is no
    % parent level, so this would be run independently on each agent.
    % Running at the simulation level is just meant to simplify the
    % simulation

end