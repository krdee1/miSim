function f = plotParameters(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "rfSensor")};
    end
    arguments (Output)
        f (1, 1) {mustBeA(f, "matlab.ui.Figure")};
    end
    
    % Distance and tilt sample points
    d_values = [0.1, 0.5, 1:1:9, 10:2:19, 20:5:49, 50:10:100];
    t_values = -90:0.5:90;
    a_values = 0:0.5:360;

    % Make grid of values of distances and tilts
    [d_mg, t_mg, a_mg] = meshgrid(d_values, t_values, a_values);
    d = d_mg(:); t = t_mg(:); a = a_mg(:); % flatten

    % Sample received signal strength (no interference or noise)
    s_x = obj.RSS(d, t, a);
    s_x = reshape(s_x, size(d_mg));

    [T, A] = meshgrid(t_values, a_values);   % Naz x Nel
    Tr = deg2rad(T);
    Ar = deg2rad(A);
    
    figure;
    hold("on");

    for ii = 1:numel(d_values)
        % geometry (your "tilt from nadir, stack by distance")
        X = d_values(ii) * cos(Ar) .* sin(Tr);
        Y = d_values(ii) * sin(Ar) .* sin(Tr);
        Z = d_values(ii) * ones(size(X));
    
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
    xlabel('X'); ylabel('Y'); zlabel('Distance (m)');
    set(gca,'ZDir','reverse');
    view(3); 
    axis("vis3d");
    grid("on");
    scatter3(0, 0, 0, 'rx');
    hold("off");
end