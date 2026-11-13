function b = minMaxBoxchart(ax, x, y)
% minMaxBoxchart  Box chart whose whiskers span the full min-max range.
%
%   b = minMaxBoxchart(ax, x, y)
%
% MATLAB box chart (box = quartiles, line = median) of y grouped by the
% numeric group positions x, with whiskers from each group's minimum to its
% maximum and no outlier markers. boxchart has no whisker-length option, so
% its outlier markers and 1.5 IQR whiskers are hidden and min-max whiskers
% (with caps) are drawn in matching lines. The whisker lines carry
% SeriesIndex "none" so exportThesisFig keeps the box-chart styling.
% Returns the BoxChart object. Leaves the axes with hold off.
    b = boxchart(ax, x, y, "MarkerStyle", "none");
    whiskerColor = b.WhiskerLineColor;
    b.WhiskerLineStyle = "none";
    hold(ax, "on");
    capHalf = b.BoxWidth / 4;
    for g = unique(x(:))'
        v = y(x == g);
        q = quantile(v, [0.25, 0.75]);
        lo = min(v); hi = max(v);
        hw = plot(ax, [g, g, NaN, g, g, NaN, g - capHalf, g + capHalf, NaN, g - capHalf, g + capHalf], ...
                      [lo, q(1), NaN, q(2), hi, NaN, lo, lo, NaN, hi, hi], ...
                  "Color", whiskerColor, "LineWidth", b.LineWidth);
        hw.SeriesIndex = "none";
    end
    hold(ax, "off");
end
