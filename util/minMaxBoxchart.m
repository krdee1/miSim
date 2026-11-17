function b = minMaxBoxchart(ax, x, y, opts)
% minMaxBoxchart  Box chart whose whiskers span the full min-max range.
%
%   b = minMaxBoxchart(ax, x, y)
%   b = minMaxBoxchart(ax, x, y, BoxWidth=0.35, BoxFaceColor=[0 0.45 0.7])
%
% MATLAB box chart (box = quartiles, line = median) of y grouped by the
% numeric group positions x, with whiskers from each group's minimum to its
% maximum and no outlier markers. boxchart has no whisker-length option, so
% its outlier markers and 1.5 IQR whiskers are hidden and min-max whiskers
% (with caps) are drawn in matching lines. The whisker lines carry
% SeriesIndex "none" so exportThesisFig keeps the box-chart styling.
%
% BoxWidth (data units, group spacing 1) and BoxFaceColor (also used for
% the box edges and median line) allow several series side by side: call
% once per series with offset positions, e.g. x - 0.2 and x + 0.2.
% Returns the BoxChart object. Leaves the axes with hold off.
    arguments
        ax (1, 1) matlab.graphics.axis.Axes
        x (:, 1) double
        y (:, 1) double
        opts.BoxWidth (1, 1) double {mustBePositive} = 0.5
        opts.BoxFaceColor (1, 3) double = NaN(1, 3)
    end
    hold(ax, "on");
    b = boxchart(ax, x, y, "MarkerStyle", "none", "BoxWidth", opts.BoxWidth);
    if all(~isnan(opts.BoxFaceColor))
        b.BoxFaceColor = opts.BoxFaceColor;
        b.BoxEdgeColor = opts.BoxFaceColor;
    end
    whiskerColor = b.WhiskerLineColor;
    b.WhiskerLineStyle = "none";
    capHalf = b.BoxWidth / 4;
    for g = unique(x)'
        v = y(x == g);
        q = quantile(v, [0.25, 0.75]);
        lo = min(v); hi = max(v);
        hw = plot(ax, [g, g, NaN, g, g, NaN, g - capHalf, g + capHalf, NaN, g - capHalf, g + capHalf], ...
                      [lo, q(1), NaN, q(2), hi, NaN, lo, lo, NaN, hi, hi], ...
                  "Color", whiskerColor, "LineWidth", b.LineWidth, "HandleVisibility", "off");
        hw.SeriesIndex = "none";
    end
    hold(ax, "off");
end
