function f = exportThesisFig(f, name, aspect, opts)
    % Restyle a figure for print and export it as a vector PDF sized for the
    % thesis, so it can be included at 1:1 (\includegraphics with no width).
    %
    %   exportThesisFig(f, "sigmoidmodel")                 % full width, 4:3
    %   exportThesisFig(f, "traj", 1, Width=230)           % half-width subfigure
    %   exportThesisFig(f, "traj", 3/4, LineStyles=true)   % distinguishable in grayscale
    %   exportThesisFig(f, "perf", 0.8, Width=330)         % imagesc plot -> name.png
    %
    % Figures containing images (imagesc, image) are exported as a lossless
    % 400 dpi PNG instead: MATLAB's vector PDF export draws them with a
    % visible diagonal seam. The PNG records its dpi, so LaTeX still places it
    % at Width points. The figure is modified in place.
    arguments (Input)
        f (1, 1) {mustBeA(f, "matlab.ui.Figure")};
        name (1, 1) string;
        aspect (1, 1) double {mustBePositive} = 3/4;
        opts.Width (1, 1) double {mustBePositive} = 469;      % points; \the\hsize
        opts.FontSize (1, 1) double {mustBePositive} = 10;    % \footnotesize in a 12pt doc
        opts.LineWidth (1, 1) double {mustBePositive} = 2; % data lines, points
        opts.AxesLineWidth (1, 1) double {mustBePositive} = 0.75;
        opts.MarkerSize (1, 1) double {mustBePositive} = 4;        % markers on lines
        opts.PointMarkerSize (1, 1) double {mustBePositive} = 16;  % marker-only series, e.g. plot(x, y, "k+")
        opts.PointMarkerLineWidth (1, 1) double {mustBePositive} = 2.5;
        opts.Palette (:, 3) double = okabeIto();              % colorblind- and print-safe
        opts.Recolor (1, 1) logical = true;                   % also override explicitly set colors
        opts.LineStyles (1, 1) logical = false;               % cycle line styles per series
        opts.Raster (1, 1) logical = ~isempty(findall(f, 'Type', 'image')); % PNG; also for very dense plots
        % dpi, for Raster. MATLAB silently writes an all-black PNG when the image is
        % too large (3909 px wide failed, 2750 px worked), so full width stays at 400
        opts.Resolution (1, 1) double {mustBePositive} = 400;
        opts.Colormap = [];                                   % for images/surfaces, e.g. "parula"; [] keeps the figure's
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
        if ~isempty(opts.Colormap)
            colormap(ax, opts.Colormap);
        end
    end

    % Colorbars (image and surface plots) get the same frame as the axes. Their
    % Label isn't reached by findall above, so set it directly.
    for cb = findall(f, 'Type', 'colorbar')'
        cb.LineWidth = opts.AxesLineWidth;
        cb.Label.Interpreter = 'latex';
        cb.Label.FontSize = opts.FontSize;
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

        % Marker-only series (no connecting line) mark individual points; a
        % thick data-line width would fill in markers like + and x
        isPoint = isLine && isprop(s, 'Marker') && strcmp(s.LineStyle, 'none') ...
            && ~strcmp(s.Marker, 'none');

        if isPoint
            s.LineWidth = opts.PointMarkerLineWidth;
            s.MarkerSize = opts.PointMarkerSize;
        else
            if isLine || isScatter
                s.LineWidth = opts.LineWidth;
            end
            if isprop(s, 'MarkerSize')
                s.MarkerSize = opts.MarkerSize;
            end
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
        % PNG rather than '-dpdf -image', which embeds a lossy JPEG
        print(f, fullfile(opts.OutDir, name + ".png"), '-dpng', "-r" + opts.Resolution);
    else
        print(f, fullfile(opts.OutDir, name + ".pdf"), '-dpdf', '-vector');
    end
end
