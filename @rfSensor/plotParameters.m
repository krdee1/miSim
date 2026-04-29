function f = plotParameters(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
    end
    arguments (Output)
        f (1, 1) {mustBeA(f, "matlab.ui.Figure")};
    end
    
    % Distance and tilt sample points
    d_values = 10.^[1, 2, 3, 4, 5, 6];
    t_values = 0:2.5:180;   % 0=nadir (center), 180=zenith (edge)
    a_values = 0:2.5:360;

    % Make grid of values of distances and tilts
    [d_mg, t_mg, a_mg] = meshgrid(d_values, t_values, a_values);
    d = d_mg(:); t = t_mg(:); a = a_mg(:); % flatten

    % Sample received signal strength (no interference or noise)
    s_x = obj.RSS(d, t, a);
    s_x = reshape(s_x, size(d_mg));

    [T, A] = meshgrid(t_values, a_values);   % Naz x Nel
    Ar = deg2rad(A);

    f = figure;
    hold("on");

    for ii = 1:numel(d_values)
        % Linear radial mapping: t=0 (nadir) -> center, t=180 (zenith) -> edge
        % Radius in log10 units to match Z axis scale
        r = log10(d_values(ii)) .* T ./ 180;
        X = r .* cos(Ar);
        Y = r .* sin(Ar);
        Z = log10(d_values(ii)) * ones(size(X));
    
        % evaluate or extract this slice
        Fslice = squeeze(s_x(:, ii, :))';
    
        % plot as its own surface
        h = surf(X, Y, Z, Fslice);
        h.EdgeColor = 'none';
        h.FaceColor = 'interp';
        h.FaceAlpha = 0.25;
    end

    colormap(turbo);
    colorbar;
    daspect([1 1 0.2])   % Separate Z further for more distinct layers
    xlabel('X (log_{10} units)'); ylabel('Y (log_{10} units)'); zlabel('log_{10} Distance (m)');
    set(gca,'ZDir','reverse');
    view(3); 
    axis("vis3d");
    grid("on");
    scatter3(0, 0, 0, 'rx');
    hold("off");
end