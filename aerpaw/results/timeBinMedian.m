function [t_med, v_med] = timeBinMedian(t, v, binWidth)
    % Compute median of each column of v within fixed-width time bins.
    %
    %   t        - (N,1) posixtime values
    %   v        - (N,K) data matrix; one column per quantity
    %   binWidth - scalar bin width in seconds
    %
    %   t_med    - (B,1) median time of each non-empty bin
    %   v_med    - (B,K) median of each column per non-empty bin

    edges = (floor(min(t) / binWidth) * binWidth) : binWidth : ...
            (floor(max(t) / binWidth) * binWidth + binWidth);
    bins  = discretize(t, edges);
    nBins = numel(edges) - 1;
    K     = size(v, 2);

    t_all = NaN(nBins, 1);
    v_all = NaN(nBins, K);
    for bi = 1:nBins
        mask = bins == bi;
        if ~any(mask), continue; end
        t_all(bi)   = median(t(mask));
        v_all(bi,:) = median(v(mask,:), 1);
    end

    ok    = ~isnan(t_all);
    t_med = t_all(ok);
    v_med = v_all(ok, :);
end
