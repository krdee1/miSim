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
    h = NaN(size(obj.agents, 1));
    h(logical(eye(size(obj.agents, 1)))) = 0; % self value is 0
    nAAPairs = nchoosek(size(obj.agents, 1), 2);
    nAOPairs = size(obj.agents, 1) * size(obj.obstacles, 1);
    kk = 1;
    A = zeros(nAAPairs + nAOPairs, 3 * size(obj.agents, 1));
    b = zeros(nAAPairs + nAOPairs, 1);

    % Set up collision avoidance constraints
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


    % Solve QP program generated earlier
    vhat = reshape(v', 3 * size(obj.agents, 1), 1);
    H = 2 * eye(3 * size(obj.agents, 1));
    f = -2 * vhat;
    
    % Update solution based on constraints
    opt = optimoptions('quadprog', 'Display', 'off');
    [vNew, ~, exitflag] = quadprog(sparse(H), double(f), A, b, [],[], [], [], [], opt);
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