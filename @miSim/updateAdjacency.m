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
        % SINR model: a link between a pair is feasible only if BOTH directional
        % SINRs clear the threshold, i.e. each agent receives the other's signal
        % above gammaLin (with interference from all other agents at the
        % receiver). Every ordered pair (permutation) is evaluated; a single
        % failing direction eliminates the undirected link.
        gammaLin = 10^(obj.sinrThreshold / 10); % dB threshold -> linear
        positions = zeros(nAgents, 3);
        for kk = 1:nAgents
            positions(kk, :) = obj.agents{kk}.pos;
        end
        for ii = 2:nAgents
            for jj = 1:(ii - 1)
                sinrLoRx = obj.sinrLink(positions, jj, ii); % receiver jj (lower), transmitter ii
                sinrHiRx = obj.sinrLink(positions, ii, jj); % receiver ii (higher), transmitter jj
                if sinrLoRx < gammaLin || sinrHiRx < gammaLin
                    A(ii, jj) = false; % at least one direction below threshold
                    continue;
                end
            end
        end
    end

    obj.adjacency = A & A';
end