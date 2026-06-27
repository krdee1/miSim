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

    % In SINR comms mode, label each edge with the connection's SINR (dB) and
    % report the connectivity threshold as a subtitle on the tile. The
    % fixed-radius model is left unchanged (no edge labels, no subtitle).
    if obj.useSinrComms
        endNodes = G.Edges.EndNodes;   % each row [lo hi]; graph stores lo < hi
        nAgents = size(obj.agents, 1);
        positions = zeros(nAgents, 3);
        for kk = 1:nAgents
            positions(kk, :) = obj.agents{kk}.pos;
        end
        % Directional convention (matches updateAdjacency/constrainMotion):
        % the lower-index node is the receiver, the higher-index node the
        % transmitter, with interference from all other agents at the receiver.
        edgeLabels = strings(size(endNodes, 1), 1);
        for ee = 1:size(endNodes, 1)
            sinrLin = obj.sinrLink(positions, endNodes(ee, 1), endNodes(ee, 2));
            edgeLabels(ee) = sprintf("%.1f dB", 10 * log10(sinrLin));
        end
        o(1).EdgeLabel = edgeLabels;
        subtitle(ax, sprintf("SINR threshold: %.1f dB", obj.sinrThreshold));
    end

    obj.graphPlot = o;
end
