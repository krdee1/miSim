function obj = routeTopology(obj)
% ROUTETOPOLOGY  Select the network topology by solving a min-max-workload
% routing LP (alternative to the lesser-neighbor algorithm).
%
% Every agent produces one unit of sensing data per timestep, all of which
% must reach one of the network endpoints (agents 1 and the largest odd
% index), which output data off-network for free (bottomless sinks). Data
% may split across paths, but relayed data accumulates at intermediate
% nodes, so each node experiences a workload: the share of a timestep it
% spends transmitting and receiving, with per-link effort flow/capacity and
% Shannon capacity c = log2(1 + SINR). The LP minimizes the maximum
% workload over all nodes, since the busiest node bounds system capacity.
%
% Candidate links are the threshold-feasible pairs in obj.adjacency (both
% directional SINRs clear sinrThreshold, per updateAdjacency). After the
% first solve, links carrying negligible flow (< routingFlowThreshold) are
% disallowed and the LP is re-solved so they cannot reappear: feasible but
% impractical links add system complexity for very little gain.
%
% The routed links are a HIGH-QUALITY OVERLAY on top of the low-level
% connectivity topology, not a replacement for it: basic command/telemetry
% traffic must always be routable between any two drones, so the caller
% runs lesserNeighbor first and this method ORs the bulk links into the
% maintained set. Both tiers are held above threshold by the CBF.
%
% Outputs (set on obj):
%   routingAdjacencyMatrix    directed adjacency, (i, j) = bulk link i -> j
%   routingFlows              flow carried by each directed link
%   constraintAdjacencyMatrix existing (lesser-neighbor) links OR the
%                             symmetrized routed links (the CBF maintains
%                             both directions of every link)
%
% Uses linprog (Optimization Toolbox): MATLAB simulation only, no codegen.

    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end

    nAgents = size(obj.agents, 1);

    % Network endpoints: agent 1 and the largest ODD index (nAgents when
    % odd, nAgents-1 when even); with one or two agents only agent 1.
    endpoints = unique([1, 2 * ceil(nAgents / 2) - 1]);
    isEndpoint = false(1, nAgents);
    isEndpoint(endpoints) = true;

    % Trivial network: every node is an endpoint, no bulk routing needed
    % (the lesser-neighbor constraint topology is left untouched)
    if all(isEndpoint)
        obj.routingAdjacencyMatrix = false(nAgents);
        obj.routingFlows = zeros(nAgents);
        return;
    end

    positions = zeros(nAgents, 3);
    for kk = 1:nAgents
        positions(kk, :) = obj.agents{kk}.pos;
    end

    % Candidate directed links: threshold-feasible pairs, excluding
    % endpoint-sourced links (endpoints only sink; allowing them to
    % transmit only enables useless cycles). Capacity is the Shannon
    % spectral efficiency of the transmission direction.
    src = zeros(nAgents^2, 1);
    dst = zeros(nAgents^2, 1);
    cap = zeros(nAgents^2, 1);
    nE = 0;
    for ii = 1:nAgents
        if isEndpoint(ii)
            continue;
        end
        for jj = 1:nAgents
            if jj == ii || ~obj.adjacency(ii, jj)
                continue;
            end
            nE = nE + 1;
            src(nE) = ii;
            dst(nE) = jj;
            cap(nE) = log2(1 + obj.sinrLink(positions, jj, ii)); % tx ii -> rx jj
        end
    end
    src = src(1:nE);
    dst = dst(1:nE);
    cap = cap(1:nE);

    % --- LP over x = [f_1 .. f_nE, W] ------------------------------------
    % min  W + eps*sum(f)   (eps breaks ties toward sparse, cycle-free flow)
    % s.t. flow conservation (per non-endpoint i): out(i) - in(i) = 1
    %      workload (per node i): sum(f_out/c) + sum(f_in/c) - W <= 0
    %      f >= 0
    relays = find(~isEndpoint);
    nRelays = numel(relays);

    fObj = [1e-6 * ones(nE, 1); 1];

    Aeq = zeros(nRelays, nE + 1);
    for rr = 1:nRelays
        Aeq(rr, 1:nE) = double(src == relays(rr))' - double(dst == relays(rr))';
    end
    beq = ones(nRelays, 1);

    Aineq = zeros(nAgents, nE + 1);
    for ii = 1:nAgents
        Aineq(ii, 1:nE) = (double(src == ii)' + double(dst == ii)') ./ cap';
        Aineq(ii, nE + 1) = -1;
    end
    bineq = zeros(nAgents, 1);

    lb = zeros(nE + 1, 1);
    options = optimoptions("linprog", "Display", "off");

    [x, ~, exitflag] = linprog(fObj, Aineq, bineq, Aeq, beq, lb, [], options);
    if exitflag <= 0
        error("Routing LP infeasible: feasible links cannot route all data to an endpoint");
    end

    % Prune trivial links and re-solve with them disallowed, so negligible
    % flows cannot reappear in the final solution
    ub = inf(nE + 1, 1);
    ub(x(1:nE) < obj.routingFlowThreshold) = 0;
    [x, ~, exitflag] = linprog(fObj, Aineq, bineq, Aeq, beq, lb, ub, options);
    if exitflag <= 0
        error("Routing LP infeasible after pruning: routingFlowThreshold eliminated links needed to route all data to an endpoint");
    end

    % Extract the directed topology from the pruned solution
    flows = x(1:nE);
    obj.routingFlows = zeros(nAgents);
    obj.routingAdjacencyMatrix = false(nAgents);
    for ee = 1:nE
        if flows(ee) > 1e-9
            obj.routingFlows(src(ee), dst(ee)) = flows(ee);
            obj.routingAdjacencyMatrix(src(ee), dst(ee)) = true;
        end
    end

    % OR the bulk links into the low-level (lesser-neighbor) constraint
    % topology already on obj: the CBF maintains both tiers, keeping the
    % network connected for basic traffic while holding the routed links.
    % Both directions of every link are maintained (feasibility requires
    % both to clear the threshold anyway).
    obj.constraintAdjacencyMatrix = obj.constraintAdjacencyMatrix | obj.routingAdjacencyMatrix | obj.routingAdjacencyMatrix';
end
