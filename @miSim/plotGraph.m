function obj = plotGraph(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end

    % The maintained links are always drawn as a dotted undirected graph.
    % In routing (CALSN) mode the flow-carrying links are overlaid as SOLID
    % DIRECTED edges (arrow = child -> parent, toward a base station) on
    % the same node layout.
    G = graph(obj.constraintAdjacencyMatrix, "omitselfloops");

    % Resolve the target axes for the network graph tile
    if isnan(obj.networkGraphIndex)
        ax = obj.f.CurrentAxes;
    else
        ax = obj.f.Children(1).Children(obj.networkGraphIndex(1));
    end

    % Plot graph object(s)
    hold(ax, "on");
    o = plot(ax, G, "LineStyle", "--", "EdgeColor", "g", "NodeColor", "k", "LineWidth", 2);
    if obj.useRoutingTopology
        % Flow overlay: reuse the base layout's node coordinates and hide
        % the overlay's own nodes/labels so only its edges show. Sizeable
        % arrowheads so the flow direction reads at tile size.
        D = digraph(obj.routingAdjacencyMatrix);
        o = [o; plot(ax, D, "XData", o(1).XData, "YData", o(1).YData, ...
                     "LineStyle", "-", "EdgeColor", "g", "LineWidth", 2, ...
                     "ArrowSize", 12, "Marker", "none", "NodeLabel", {})];
    end
    hold(ax, "off");
    nBase = size(o, 1);
    if ~isnan(obj.networkGraphIndex) && size(obj.networkGraphIndex, 2) > 1
        for ii = 2:size(obj.networkGraphIndex, 2)
            o = [o; copyobj(o(1:nBase), obj.f.Children(1).Children(obj.networkGraphIndex(ii)))];
        end
    end

    % In SINR comms mode, label each edge with the connection's SINR and report
    % the connectivity threshold in the tile title. The fixed-radius model is
    % left unchanged (no edge labels, title stays as set in firstPlotSetup).
    if obj.useSinrComms
        endNodes = G.Edges.EndNodes;
        nAgents = size(obj.agents, 1);
        positions = zeros(nAgents, 3);
        for kk = 1:nAgents
            positions(kk, :) = obj.agents{kk}.pos;
        end
        % Labels are bare numbers (no units) so they stay compact; GraphPlot
        % renders them horizontally, left-to-right. Dotted (maintained) edges
        % are labelled with the binding (worse) directional SINR — a link is
        % feasible only if BOTH directions clear the threshold. Links that
        % also carry flow leave the label to the solid directed edge, which
        % shows the flow value (units of r) sent child -> parent.
        edgeLabels = strings(size(endNodes, 1), 1);
        for ee = 1:size(endNodes, 1)
            lo = endNodes(ee, 1);
            hi = endNodes(ee, 2);
            if obj.useRoutingTopology && (obj.routingAdjacencyMatrix(lo, hi) || obj.routingAdjacencyMatrix(hi, lo))
                edgeLabels(ee) = ""; % solid overlay carries this link's label
            else
                sinrLoRx = obj.sinrLink(positions, lo, hi); % receiver lo (lower index)
                sinrHiRx = obj.sinrLink(positions, hi, lo); % receiver hi (higher index)
                edgeLabels(ee) = sprintf("%.1f", 10 * log10(min(sinrLoRx, sinrHiRx)));
            end
        end
        o(1).EdgeLabel = edgeLabels;
        if obj.useRoutingTopology
            endNodesD = D.Edges.EndNodes;   % each row [child parent]
            flowLabels = strings(size(endNodesD, 1), 1);
            for ee = 1:size(endNodesD, 1)
                flowLabels(ee) = sprintf("%.2f", obj.routingFlows(endNodesD(ee, 1), endNodesD(ee, 2)));
            end
            o(2).EdgeLabel = flowLabels;
        end
        % Threshold goes in the existing single-line title (not a subtitle) so
        % the small tile is not compressed further, and the hover toolbar is
        % disabled so the R2026 "..." menu stops covering the readout.
        title(ax, sprintf("Network Graph (\\geq %.1f dB)", obj.sinrThreshold));
        ax.Toolbar.Visible = "off";
    end

    obj.graphPlot = o;
end
