function f = plotParameters(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
    end
    arguments (Output)
        f (1, 1) {mustBeA(f, "matlab.ui.Figure")};
    end

    % Agent altitude layers and angle sample points
    alt_values = 10.^[1, 2, 3, 4];
    t_values = 0:2.5:87.5;   % 0=nadir (center), <90=near horizon (edge)
    a_values = 0:2.5:360;

    [T, A] = meshgrid(t_values, a_values);   % Naz x Nel
    Ar = deg2rad(A);

    f = figure;
    hold("on");

    for ii = 1:numel(alt_values)
        alt = alt_values(ii);

        % For agent at altitude alt, ground target at tilt T has slant distance:
        D = alt ./ cosd(T);

        % Compute RSS for each (d, t, a) triple
        rss = obj.RSS(D(:), T(:), A(:));
        Fslice = reshape(rss, size(D));

        % Disc geometry: t=0 (nadir) -> center, t~90 (horizon) -> edge
        r = log10(alt) .* T ./ 90;
        X = r .* cos(Ar);
        Y = r .* sin(Ar);
        Z = log10(alt) * ones(size(X));

        hs = surf(X, Y, Z, Fslice);
        hs.EdgeColor = 'none';
        hs.FaceColor = 'interp';
        hs.FaceAlpha = 0.25;
    end

    colormap(turbo);
    colorbar;
    daspect([1 1 0.2]);
    xlabel('X (log_{10} units)'); ylabel('Y (log_{10} units)'); zlabel('log_{10} Altitude (m)');
    set(gca, 'ZDir', 'reverse');
    view(3);
    axis("vis3d");
    grid("on");
    scatter3(0, 0, 0, 'rx');
    hold("off");
end
