function f = exportThesisFig(f, name, aspect, opts)
    % Restyle a figure for print and export it as a vector PDF sized for the
    % thesis, so it can be included at 1:1 (\includegraphics with no width).
    %
    %   exportThesisFig(f, "sigmoidmodel")                 % full width, 4:3
    %   exportThesisFig(f, "traj", 1, Width=230)           % half-width subfigure
    %   exportThesisFig(f, "traj", 3/4, LineStyles=true)   % distinguishable in grayscale
    %
    % The figure is modified in place.
    arguments (Input)
        f (1, 1) {mustBeA(f, "matlab.ui.Figure")};
        name (1, 1) string;
        aspect (1, 1) double {mustBePositive} = 3/4;
        opts.Width (1, 1) double {mustBePositive} = 469;      % points; \the\hsize
        opts.FontSize (1, 1) double {mustBePositive} = 10;    % \footnotesize in a 12pt doc
        opts.LineWidth (1, 1) double {mustBePositive} = 2; % data lines, points
        opts.AxesLineWidth (1, 1) double {mustBePositive} = 0.75;
        opts.MarkerSize (1, 1) double {mustBePositive} = 4;
        opts.Palette (:, 3) double = okabeIto();              % colorblind- and print-safe
        opts.Recolor (1, 1) logical = true;                   % also override explicitly set colors
        opts.LineStyles (1, 1) logical = false;               % cycle line styles per series
        opts.Raster (1, 1) logical = false;                   % for very dense plots
        opts.OutDir (1, 1) string = matlab.project.rootProject().RootFolder;
    end
    arguments (Output)
        f (1, 1) {mustBeA(f, "matlab.ui.Figure")};
    end

    styles = ["-", "--", ":", "-."];
    nColors = size(opts.Palette, 1);

    % Text
    set(findall(f, '-property', 'Interpreter'), 'Interpreter', 'latex');
    set(findall(f, '-property', 'TickLabelInterpreter'), 'TickLabelInterpreter', 'latex');
    set(findall(f, '-property', 'FontSize'), 'FontSize', opts.FontSize);

    % Axes: frame width, and palette for any series that pick their color automatically
    for ax = findall(f, 'Type', 'axes')'
        ax.LineWidth = opts.AxesLineWidth;
        ax.ColorOrder = opts.Palette;
        if opts.LineStyles
            ax.LineStyleOrder = styles;
        end
    end

    % Data series. SeriesIndex is the series' position in its axes' color
    % order, so it keeps the palette assignment consistent with the legend.
    series = findall(f, '-property', 'SeriesIndex');
    for s = series'
        k = s.SeriesIndex;
        if ~isnumeric(k) || k < 1
            continue;   % series excluded from the color order
        end
        c = opts.Palette(mod(k - 1, nColors) + 1, :);
        % Line-type series have a single Color; bars/areas have Face/EdgeColor
        % instead, and keep their thin default outline
        isLine = isprop(s, 'Color');
        isScatter = isa(s, 'matlab.graphics.chart.primitive.Scatter');

        if isLine || isScatter
            s.LineWidth = opts.LineWidth;
        end
        if isprop(s, 'MarkerSize')
            s.MarkerSize = opts.MarkerSize;
        end
        if opts.LineStyles && isLine && ~strcmp(s.LineStyle, 'none')
            s.LineStyle = styles(mod(k - 1, numel(styles)) + 1);
        end
        if opts.Recolor
            if isLine
                s.Color = c;            % line, stair, errorbar, function plots
            end
            if isprop(s, 'FaceColor') && ~ischar(s.FaceColor)
                s.FaceColor = c;        % bar, area, histogram (leaves 'none'/'flat' alone)
            end
            if isScatter && size(s.CData, 1) <= 1
                s.CData = c;            % single-color scatter (leaves colormapped ones alone)
            end
        end
    end

    % Page size in points so the PDF is exactly Width wide, independent of
    % the on-screen window (which the window manager may resize)
    w = opts.Width;
    h = aspect * w;
    f.PaperUnits = 'points';
    f.PaperPositionMode = 'manual';
    f.PaperPosition = [0, 0, w, h];
    f.PaperSize = [w, h];

    if opts.Raster
        renderer = {'-image', '-r300'};
    else
        renderer = {'-vector'};
    end
    print(f, fullfile(opts.OutDir, name + ".pdf"), '-dpdf', renderer{:});
end
