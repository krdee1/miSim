function obj = lesserSinkNeighbor(obj)
% LESSERSINKNEIGHBOR  Capacity-Aware Lesser Sink Neighbor algorithm (CALSN).
%
% Implements the algorithm given in the CALSN algorithm description document
% (ADD; kept outside the repository) as the selectable alternative to
% lesserNeighbor. UAVs 1 and N are base stations; every other UAV generates
% r = 1 unit of data per step, and all data must reach a base station. The
% algorithm outputs both a sparse maintained topology and a routing table.
%
% 1. Base-station potentials phi via distributed Bellman-Ford over the
%    current communication neighborhoods (obj.adjacency), with link cost
%    c_ij = 1/(C_ij + eps), where C_ij = bandwidth * log2(1 + SINR_ij) is
%    the Shannon capacity of direction i -> j.
% 2. Lesser sink neighbors L_i replace LNA's "j < i" rule:
%        L_i = { j in N_i : phi_j < phi_i, or phi_j == phi_i and j < i }.
%    The equal-potential lexicographic tie-break is a documented deviation
%    from the ADD's strict inequality. It makes (phi, index) a total order,
%    so every feasible link between two non-base UAVs has exactly one
%    deciding endpoint (the uphill one).
% 3. LNA-style sparsity: one parent per connected component of the
%    L_i-induced subgraph (only links with positive capacity in at least
%    one direction count as edges), chosen by
%        p = argmin_{j in C} ( phi_j + 1/(C_ij + eps) ).
%    A_control(i,p) = A_control(p,i) = 1 (diagonal = I).
% 4. Connectivity bridge (documented deviation; not in the ADD). Every UAV
%    must stay connected to the whole fleet to exchange positions for
%    guidance, so A_control must be one connected graph. LNA's grouping
%    argument guarantees that whenever every feasible link has a deciding
%    endpoint. The one exception is the base-to-base link: both bases have
%    phi = 0 and select no parents, so nobody maintains it, yet it still
%    merges groups when other UAVs form their components. Steps 1-3
%    therefore leave A_control either connected or split into exactly two
%    pieces, one containing each base. In the split case one bridge is
%    added: of all feasible links between the two pieces (base-to-base
%    included), the one whose weaker direction has the highest SINR, i.e.
%    the most margin under the comms barrier. A warning is issued if even
%    that link is inside the CBF maintenance band (sinrThreshold + 3 dB).
%    The bridge is maintained for connectivity only and carries no flow,
%    so when present it is the one maintained link outside the ADD's
%    invariant A_control(i,j) = 1, i ~= j, iff F(i,j) > 0 or F(j,i) > 0.
% 5. Routing, children first (decreasing phi; equal phi processed in
%    decreasing index order so tie-break children precede their parents):
%    aggregate demand D_i = r + inflows, split capacity-proportionally
%    over parents; saturated links are capped at capacity and the residual
%    redistributed over unsaturated parents; any demand beyond the total
%    parent capacity is reported as unrouted and the UAV marked infeasible.
%
% Outputs (set on obj):
%   constraintAdjacencyMatrix  A_control: undirected maintained topology
%   routingFlows               F: flow on each directed link (units of r)
%   routingFractions           R = F/D: share of the sender's demand, 0..1
%   routingAdjacencyMatrix     F > 0 (directed, child -> parent)
%   routingPotentials          phi (base-station potential per UAV)
%   routingUnrouted            unrouted demand per UAV (zeros when feasible)

    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end

    nAgents = size(obj.agents, 1);
    r = 1.0;         % data generated per non-base UAV per step (abstract unit)
    EPS_CAP = 1e-12; % epsilon in c = 1/(C + eps); avoids division by zero
    TOL = 1e-12;     % residual-demand tolerance for the saturation loop

    isBase = false(nAgents, 1);
    isBase(1) = true;
    isBase(nAgents) = true; % base stations are UAVs 1 and N

    positions = zeros(nAgents, 3);
    for kk = 1:nAgents
        positions(kk, :) = obj.agents{kk}.pos;
    end

    % Directed SINRs and Shannon link capacities over the current
    % communication neighborhoods: S(i, j) and C(i, j) describe sending
    % from i to j (receiver j)
    S = zeros(nAgents);
    C = zeros(nAgents);
    for ii = 1:nAgents
        for jj = 1:nAgents
            if jj ~= ii && obj.adjacency(ii, jj)
                S(ii, jj) = obj.sinrLink(positions, jj, ii);
                C(ii, jj) = obj.bandwidth * log2(1 + S(ii, jj));
            end
        end
    end

    % ---- Base-station potentials (Bellman-Ford, at most N-1 sweeps) -----
    phi = inf(nAgents, 1);
    phi(isBase) = 0;
    for sweep = 1:max(nAgents - 1, 1)
        changed = false;
        for ii = 1:nAgents
            if isBase(ii)
                continue;
            end
            newPhi = inf;
            for jj = 1:nAgents
                if jj == ii || ~obj.adjacency(ii, jj)
                    continue;
                end
                newPhi = min(newPhi, phi(jj) + 1 / (C(ii, jj) + EPS_CAP));
            end
            if newPhi ~= phi(ii)
                phi(ii) = newPhi;
                changed = true;
            end
        end
        if ~changed
            break;
        end
    end

    % ---- Parent selection (one per lesser-sink component) ---------------
    Acontrol = logical(eye(nAgents));
    isParent = false(nAgents); % isParent(i, p): p is a selected parent of i
    for ii = 1:nAgents
        if isBase(ii) || isinf(phi(ii))
            % Base stations select no parents; an infinite potential means
            % no current local route toward a base station
            continue;
        end

        % Lesser sink neighbors (with the equal-potential tie-break)
        lsnBuf = zeros(1, nAgents);
        lsnCount = 0;
        for jj = 1:nAgents
            if jj == ii || ~obj.adjacency(ii, jj)
                continue;
            end
            if phi(jj) < phi(ii) || (phi(jj) == phi(ii) && jj < ii)
                lsnCount = lsnCount + 1;
                lsnBuf(lsnCount) = jj;
            end
        end
        if lsnCount == 0
            continue;
        end
        lsn = lsnBuf(1:lsnCount);

        % Subgraph induced by the lesser sink neighbors in the usable
        % communication graph: only links with positive capacity in at
        % least one direction count as edges (a zero-capacity link cannot
        % carry data, so it must not merge two components)
        subgraphAdjacency = false(lsnCount);
        for aa = 1:lsnCount
            for bb = (aa + 1):lsnCount
                ja = lsn(aa);
                jb = lsn(bb);
                if obj.adjacency(ja, jb) && (C(ja, jb) > 0 || C(jb, ja) > 0)
                    subgraphAdjacency(aa, bb) = true;
                    subgraphAdjacency(bb, aa) = true;
                end
            end
        end

        % One parent per connected component, selected by capacity and
        % potential: p = argmin( phi_j + 1/(C_ij + eps) )
        visited = false(1, lsnCount);
        for aa = 1:lsnCount
            if visited(aa)
                continue;
            end
            component = bfs(subgraphAdjacency, aa);
            visited(component) = true;

            bestMetric = inf;
            bestParent = 0;
            for cc = 1:numel(component)
                jj = lsn(component(cc));
                metric = phi(jj) + 1 / (C(ii, jj) + EPS_CAP);
                if metric < bestMetric
                    bestMetric = metric;
                    bestParent = jj;
                end
            end
            isParent(ii, bestParent) = true;
            Acontrol(ii, bestParent) = true;
            Acontrol(bestParent, ii) = true;
        end
    end

    % ---- Connectivity bridge ---------------------------------------------
    % The unmaintained base-to-base link is the only way steps 1-3 can
    % leave A_control split, and then into exactly two pieces, one per
    % base. Bridge them with the feasible cross link whose weaker direction
    % has the highest SINR. If the pieces are not joined by any feasible
    % link, the feasibility graph itself is disconnected; validate()
    % reports that case.
    pieceOfBase1 = false(1, nAgents);
    pieceOfBase1(bfs(Acontrol, 1)) = true;
    if ~pieceOfBase1(nAgents)
        bestSinr = -inf;
        bestU = 0;
        bestV = 0;
        for uu = find(pieceOfBase1)
            for vv = find(~pieceOfBase1)
                if obj.adjacency(uu, vv)
                    sinrBinding = min(S(uu, vv), S(vv, uu));
                    if sinrBinding > bestSinr
                        bestSinr = sinrBinding;
                        bestU = uu;
                        bestV = vv;
                    end
                end
            end
        end
        if bestU > 0
            Acontrol(bestU, bestV) = true;
            Acontrol(bestV, bestU) = true;
            % CBF maintenance band: constrainMotion holds maintained links
            % at sinrThreshold + 3 dB (paddingDb; keep in sync)
            if bestSinr < 10^((obj.sinrThreshold + 3.0) / 10)
                warning("miSim:bridgeInCbfBand", ...
                    "Connectivity bridge %d-%d has SINR %.1f dB, inside the CBF maintenance band (sinrThreshold %.1f dB + 3 dB)", ...
                    bestU, bestV, 10 * log10(bestSinr), obj.sinrThreshold);
            end
        end
    end

    % ---- Routing (children first: decreasing potential order) -----------
    % Equal potentials are processed in decreasing index order: with the
    % lexicographic tie-break, flow within an equal-potential group goes
    % from higher to lower index, so higher indices must aggregate first.
    F = zeros(nAgents);
    R = zeros(nAgents);
    unrouted = zeros(nAgents, 1);

    nonBase = find(~isBase);
    [~, ord] = sortrows([phi(nonBase), nonBase], [-1, -2]);
    order = nonBase(ord);

    for oo = 1:numel(order)
        ii = order(oo);

        % Aggregated outgoing demand: own data plus everything received
        D = r + sum(F(:, ii));
        parents = find(isParent(ii, :));
        if D == 0 || isempty(parents)
            continue;
        end

        % Capacity-proportional split, saturating full links first and
        % redistributing the remainder across parents with remaining
        % capacity. Each pass either routes all residual demand or
        % saturates at least one parent, so the loop terminates.
        capP = C(ii, parents);
        flows = zeros(1, numel(parents));
        active = true(1, numel(parents));
        residual = D;
        while residual > TOL && any(active)
            totalCap = sum(capP(active));
            if totalCap <= 0
                break;
            end
            alloc = zeros(1, numel(parents));
            alloc(active) = residual .* capP(active) ./ totalCap;
            flows = flows + alloc;
            saturated = active & (flows > capP);
            if any(saturated)
                residual = sum(flows(saturated) - capP(saturated));
                flows(saturated) = capP(saturated);
                active(saturated) = false;
            else
                residual = 0;
            end
        end
        if residual > TOL
            % Demand exceeds the total parent capacity: route as much as
            % possible and report the remainder as unrouted (infeasible)
            unrouted(ii) = residual;
            warning("miSim:routingInfeasible", ...
                "UAV %d demand %.3g exceeds total parent capacity; %.3g units unrouted", ...
                ii, D, residual);
        end

        F(ii, parents) = flows;
        R(ii, parents) = flows ./ D;
    end

    % ---- Outputs ---------------------------------------------------------
    obj.constraintAdjacencyMatrix = Acontrol;
    obj.routingFlows = F;
    obj.routingFractions = R;
    obj.routingAdjacencyMatrix = F > 0;
    obj.routingPotentials = phi;
    obj.routingUnrouted = unrouted;
end

function cComp = bfs(subgraphAdjacency, startIdx)
    % Breadth-first search returning the connected component of startIdx.
    % Pre-allocated queue and component buffer with head/tail pointers to
    % avoid element deletion and dynamic array growth (same pattern as the
    % local bfs in lesserNeighbor.m).
    n = size(subgraphAdjacency, 1);
    visited = false(1, n);

    queue    = zeros(1, n);
    cCompBuf = zeros(1, n);
    qHead = 1;
    qTail = 2;
    queue(1)    = startIdx;
    cCompBuf(1) = startIdx;
    cSize = 1;
    visited(startIdx) = true;

    while qHead < qTail
        current = queue(qHead);
        qHead = qHead + 1;

        neighbors = find(subgraphAdjacency(current, :));
        for kk = 1:numel(neighbors)
            neighbor = neighbors(kk);
            if ~visited(neighbor)
                visited(neighbor) = true;
                cCompBuf(cSize + 1) = neighbor;
                cSize = cSize + 1;
                queue(qTail) = neighbor;
                qTail = qTail + 1;
            end
        end
    end
    cComp = sort(cCompBuf(1:cSize));
end
