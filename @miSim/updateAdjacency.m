function obj = updateAdjacency(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end

    nAgents = size(obj.agents, 1);

    % Initialize assuming all agents are connected
    A = true(nAgents);

    if ~obj.useSinrComms
        % Fixed-radius model: pair connected iff separation is within the
        % smaller of the two comms radii.
        for ii = 2:nAgents
            for jj = 1:(ii - 1)
                if norm(obj.agents{ii}.pos - obj.agents{jj}.pos) > min([obj.agents{ii}.commsGeometry.radius, obj.agents{jj}.commsGeometry.radius])
                    A(ii, jj) = false; % comm range violation
                    continue;
                end
            end
        end
    else
        % SINR model: pair (ii, jj) connected iff the lower-index agent (jj)
        % receives the higher-index agent's (ii) signal above the SINR
        % threshold, with interference from all other agents at the receiver.
        gammaLin = 10^(obj.sinrThreshold / 10); % dB threshold -> linear
        positions = zeros(nAgents, 3);
        for kk = 1:nAgents
            positions(kk, :) = obj.agents{kk}.pos;
        end
        for ii = 2:nAgents
            for jj = 1:(ii - 1)
                sinr = obj.sinrLink(positions, jj, ii); % receiver jj, transmitter ii
                if sinr < gammaLin
                    A(ii, jj) = false; % SINR below threshold
                    continue;
                end
            end
        end
    end

    obj.adjacency = A & A';
end