function [obj] = constrainMotion(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end

    if size(obj.agents, 1) < 2
        return;
        % this doesn't work right now with only one agent
    end

    agents = [obj.agents{:}];
    v = reshape(([agents.pos] - [agents.lastPos])./obj.timestep, 3, size(obj.agents, 1))';

    % Initialize QP based on number of agents and obstacles
    nAAPairs = nchoosek(size(obj.agents, 1), 2); % unique agent/agent pairs
    nAOPairs = size(obj.agents, 1) * size(obj.obstacles, 1); % unique agent/obstacle pairs
    nADPairs = size(obj.agents, 1) * 5; % agents x (4 walls + 1 ceiling)
    nLNAPairs = sum(obj.constraintAdjacencyMatrix, 'all') - size(obj.agents, 1);
    total = nAAPairs + nAOPairs + nADPairs + nLNAPairs;
    kk = 1;
    A = zeros(total, 3 * size(obj.agents, 1));
    b = zeros(total, 1);

    % Set up collision avoidance constraints
    h = NaN(size(obj.agents, 1));
    h(logical(eye(size(obj.agents, 1)))) = 0; % self value is 0
    for ii = 1:(size(obj.agents, 1) - 1)
        for jj = (ii + 1):size(obj.agents, 1)
            h(ii, jj) = norm(agents(ii).pos - agents(jj).pos)^2 - (agents(ii).collisionGeometry.radius + agents(jj).collisionGeometry.radius)^2;
            h(jj, ii) = h(ii, jj);
            
            A(kk, (3 * ii - 2):(3 * ii)) =  -2 * (agents(ii).pos - agents(jj).pos);
            A(kk, (3 * jj - 2):(3 * jj)) = -A(kk, (3 * ii - 2):(3 * ii));
            b(kk) = obj.barrierGain * h(ii, jj)^3;
            kk = kk + 1;
        end
    end

    hObs = NaN(size(obj.agents, 1), size(obj.obstacles, 1));
    % Set up obstacle avoidance constraints
    for ii = 1:size(obj.agents, 1)
        for jj = 1:size(obj.obstacles, 1)
            % find closest position to agent on/in obstacle
            cPos = obj.obstacles{jj}.closestToPoint(agents(ii).pos);

            hObs(ii, jj) = dot(agents(ii).pos - cPos, agents(ii).pos - cPos) - agents(ii).collisionGeometry.radius^2;

            A(kk, (3 * ii - 2):(3 * ii)) = -2 * (agents(ii).pos - cPos);
            b(kk) = obj.barrierGain * hObs(ii, jj)^3;
           
            kk = kk + 1;
        end
    end
    
    % Set up domain constraints (walls and ceiling only)
    % Floor constraint is implicit with an obstacle corresponding to the
    % minimum allowed altitude, but I included it anyways
    for ii = 1:size(obj.agents, 1)
        % X minimum
        h_xMin = (agents(ii).pos(1) - obj.domain.minCorner(1)) - agents(ii).collisionGeometry.radius;
        A(kk, (3 * ii - 2):(3 * ii)) = [-1, 0, 0];
        b(kk) = obj.barrierGain * h_xMin^3;
        kk = kk + 1;
    
        % X maximum
        h_xMax = (obj.domain.maxCorner(1) - agents(ii).pos(1)) - agents(ii).collisionGeometry.radius;
        A(kk, (3 * ii - 2):(3 * ii)) = [1, 0, 0];
        b(kk) = obj.barrierGain * h_xMax^3;
        kk = kk + 1;
    
        % Y minimum
        h_yMin = (agents(ii).pos(2) - obj.domain.minCorner(2)) - agents(ii).collisionGeometry.radius;
        A(kk, (3 * ii - 2):(3 * ii)) = [0, -1, 0];
        b(kk) = obj.barrierGain * h_yMin^3;
        kk = kk + 1;
    
        % Y maximum
        h_yMax = (obj.domain.maxCorner(2) - agents(ii).pos(2)) - agents(ii).collisionGeometry.radius;
        A(kk, (3 * ii - 2):(3 * ii)) = [0, 1, 0];
        b(kk) = obj.barrierGain * h_yMax^3;
        kk = kk + 1;
    
        % Z minimum
        h_zMin = (agents(ii).pos(3) - obj.domain.minCorner(3)) - agents(ii).collisionGeometry.radius;
        A(kk, (3 * ii - 2):(3 * ii)) = [0, 0, -1];
        b(kk) = obj.barrierGain * h_zMin^3;
        kk = kk + 1;
    
        % Z maximum
        h_zMax = (obj.domain.maxCorner(2) - agents(ii).pos(2)) - agents(ii).collisionGeometry.radius;
        A(kk, (3 * ii - 2):(3 * ii)) = [0, 0, 1];
        b(kk) = obj.barrierGain * h_zMax^3;
        kk = kk + 1;
    end

    % Add communication network constraints
    hComms = NaN(size(obj.agents, 1));
    hComms(logical(eye(size(obj.agents, 1)))) = 0;
    for ii = 1:(size(obj.agents, 1) - 1)
        for jj = (ii + 1):size(obj.agents, 1)
            if obj.constraintAdjacencyMatrix(ii, jj)
                % d = agents(ii).pos - agents(jj).pos;
                % h = agents(ii).commsGeometry.radius^2 - dot(d,d);
                % 
                % A(kk, (3*ii-2):(3*ii)) =  2*d;
                % A(kk, (3*jj-2):(3*jj)) = -2*d;
                % b(kk) = obj.barrierGain * h^3;
                % 
                % kk = kk + 1;

                hComms(ii, jj) = agents(ii).commsGeometry.radius^2 - norm(agents(ii).pos - agents(jj).pos)^2;

                A(kk, (3 * ii - 2):(3 * ii)) = 2 * (agents(ii).pos - agents(jj).pos);
                A(kk, (3 * jj - 2):(3 * jj)) = -A(kk, (3 * ii - 2):(3 * ii));
                b(kk) = obj.barrierGain * hComms(ii, jj)^3;
                kk = kk + 1;
            end
        end
    end

    % Solve QP program generated earlier
    vhat = reshape(v', 3 * size(obj.agents, 1), 1);
    H = 2 * eye(3 * size(obj.agents, 1));
    f = -2 * vhat;
    
    % Update solution based on constraints
    assert(size(A,2) == size(H,1))
    assert(size(A,1) == size(b,1))
    assert(size(H,1) == length(f))
    opt = optimoptions('quadprog', 'Display', 'off');
    [vNew, ~, exitflag, m] = quadprog(sparse(H), double(f), A, b, [],[], [], [], [], opt);
    assert(exitflag == 1, sprintf('quadprog failure... %s%s', newline, m.message));
    vNew = reshape(vNew, 3, size(obj.agents, 1))';

    if exitflag <= 0
        warning("QP failed, continuing with unconstrained solution...")
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