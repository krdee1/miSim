function f = objectiveFunctionWrapper(center)
    % Convenience function to generate MVNPDFs at a point
    % Makes it look a lot neater to instantiate and sum these to make
    % composite objectives in particular
    arguments (Input)
        center (1, 2) double;
    end
    arguments (Output)
        f (1, 1) {mustBeA(f, 'function_handle')};
    end

    f = @(x, y) mvnpdf([x(:), y(:)], center);
end