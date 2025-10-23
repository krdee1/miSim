function f = firstPlotSetup(f)
    if isempty(f.CurrentAxes)
        axes(f);
        axis(f.CurrentAxes, "equal");
        grid(f.CurrentAxes, "on");
        view(f.CurrentAxes, 3);
    end
end