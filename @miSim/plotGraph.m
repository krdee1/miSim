function obj = plotGraph(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end

    % Form graph from adjacency matrix
    G = graph(obj.constraintAdjacencyMatrix, "omitselfloops");

    % Resolve the target axes for the network graph tile
    if isnan(obj.networkGraphIndex)
        ax = obj.f.CurrentAxes;
    else
        ax = obj.f.Children(1).Children(obj.networkGraphIndex(1));
    end

    % Plot graph object
    hold(ax, "on");
    o = plot(ax, G, "LineStyle", "--", "EdgeColor", "g", "NodeColor", "k", "LineWidth", 2);
    hold(ax, "off");
    if ~isnan(obj.networkGraphIndex) && size(obj.networkGraphIndex, 2) > 1
        for ii = 2:size(obj.networkGraphIndex, 2)
            o = [o; copyobj(o(1), obj.f.Children(1).Children(obj.networkGraphIndex(ii)))];
        end
    end

    % In SINR comms mode, label each edge with the connection's SINR and report
    % the connectivity threshold in the tile title. The fixed-radius model is
    % left unchanged (no edge labels, title stays as set in firstPlotSetup).
    if obj.useSinrComms
        endNodes = G.Edges.EndNodes;   % each row [lo hi]; graph stores lo < hi
        nAgents = size(obj.agents, 1);
        positions = zeros(nAgents, 3);
        for kk = 1:nAgents
            positions(kk, :) = obj.agents{kk}.pos;
        end
        % A link is feasible only if BOTH directions clear the threshold, so
        % label each edge with the binding (worse) directional SINR: the min of
        % the lower-index-receiver and higher-index-receiver links, with
        % interference from all other agents at each receiver. Labels are bare
        % numbers (no units) so they stay compact; GraphPlot renders them
        % horizontally, left-to-right.
        edgeLabels = strings(size(endNodes, 1), 1);
        for ee = 1:size(endNodes, 1)
            lo = endNodes(ee, 1);
            hi = endNodes(ee, 2);
            sinrLoRx = obj.sinrLink(positions, lo, hi); % receiver lo (lower index)
            sinrHiRx = obj.sinrLink(positions, hi, lo); % receiver hi (higher index)
            edgeLabels(ee) = sprintf("%.1f", 10 * log10(min(sinrLoRx, sinrHiRx)));
        end
        o(1).EdgeLabel = edgeLabels;
        % Threshold goes in the existing single-line title (not a subtitle) so
        % the small tile is not compressed further, and the hover toolbar is
        % disabled so the R2026 "..." menu stops covering the readout.
        title(ax, sprintf("Network Graph (\\geq %.1f dB)", obj.sinrThreshold));
        ax.Toolbar.Visible = "off";
    end

    obj.graphPlot = o;
end
