function obj = plotGraph(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end

    nAgents = size(obj.agents, 1);

    % Resolve the target axes for the network graph tile
    if isnan(obj.networkGraphIndex)
        ax = obj.f.CurrentAxes;
    else
        ax = obj.f.Children(1).Children(obj.networkGraphIndex(1));
    end

    % Build the graph to plot. The fixed-radius model is undirected. In SINR
    % mode the link set (lesser-neighbor topology) is kept exactly as-is, but
    % each connection is given an orientation: of the two directional SINRs on
    % a link, the larger one wins, and the edge is directed from that
    % direction's transmitter to its receiver (arrow = transmitter -> receiver).
    if obj.useSinrComms
        positions = zeros(nAgents, 3);
        for kk = 1:nAgents
            positions(kk, :) = obj.agents{kk}.pos;
        end
        D = false(nAgents);   % directed adjacency: D(tx, rx) = true
        for aa = 1:(nAgents - 1)
            for bb = (aa + 1):nAgents
                if obj.constraintAdjacencyMatrix(aa, bb)
                    sinrARx = obj.sinrLink(positions, aa, bb); % aa receives bb
                    sinrBRx = obj.sinrLink(positions, bb, aa); % bb receives aa
                    if sinrARx >= sinrBRx
                        D(bb, aa) = true;  % better as bb -> aa (tx bb, rx aa)
                    else
                        D(aa, bb) = true;  % better as aa -> bb (tx aa, rx bb)
                    end
                end
            end
        end
        G = digraph(D);
    else
        G = graph(obj.constraintAdjacencyMatrix, "omitselfloops");
    end

    % Plot graph object (digraph renders arrowheads automatically)
    hold(ax, "on");
    o = plot(ax, G, "LineStyle", "--", "EdgeColor", "g", "NodeColor", "k", "LineWidth", 2);
    hold(ax, "off");
    if ~isnan(obj.networkGraphIndex) && size(obj.networkGraphIndex, 2) > 1
        for ii = 2:size(obj.networkGraphIndex, 2)
            o = [o; copyobj(o(1), obj.f.Children(1).Children(obj.networkGraphIndex(ii)))];
        end
    end

    % Mark ground-station-connected nodes with an "x" (others keep the default
    % dot). Ground stations attach to the two "router" agents: agent 1 and the
    % agent with the largest ODD index (that is nAgents when odd, else
    % nAgents-1). With one or two agents only agent 1 qualifies.
    routers = unique([1, 2 * ceil(nAgents / 2) - 1]);
    for oo = 1:numel(o)
        highlight(o(oo), routers, "Marker", "x", "MarkerSize", 8);
    end

    % In SINR comms mode, label each directed edge with the better SINR that set
    % its orientation and report the threshold in the tile title. The
    % fixed-radius model is left unchanged (no edge labels, title from setup).
    if obj.useSinrComms
        endNodes = G.Edges.EndNodes;   % each row [tx rx] for a directed edge
        % Label with the SINR of the chosen (better) direction: the receiver is
        % the edge target, the transmitter the edge source. Bare numbers (no
        % units) keep the small tile uncluttered; GraphPlot draws labels
        % horizontally, left-to-right.
        edgeLabels = strings(size(endNodes, 1), 1);
        for ee = 1:size(endNodes, 1)
            sinrLin = obj.sinrLink(positions, endNodes(ee, 2), endNodes(ee, 1)); % rx, tx
            edgeLabels(ee) = sprintf("%.1f", 10 * log10(sinrLin));
        end
        o(1).EdgeLabel = edgeLabels;
        % Threshold in the existing single-line title (not a subtitle) so the
        % small tile isn't compressed, and disable the hover toolbar so the
        % R2026 "..." menu stops covering the readout.
        title(ax, sprintf("Network Graph (\\geq %.1f dB)", obj.sinrThreshold));
        ax.Toolbar.Visible = "off";
    end

    obj.graphPlot = o;
end
