function f = objectiveFunctionWrapper(center, sigma)
    % Convenience function to generate MVNPDFs at a point
    % Makes it look a lot neater to instantiate and sum these to make
    % composite objectives in particular
    arguments (Input)
        center (:, 2) double;
        sigma (:, 2, 2) double = eye(2);
    end
    arguments (Output)
        f (1, 1) {mustBeA(f, "function_handle")};
    end
    
    assert(size(center, 1) == size(sigma, 1));
    f = @(x,y) sum(cell2mat(arrayfun(@(i) mvnpdf([x(:), y(:)], center(i,:), squeeze(sigma(i, :, :))), 1:size(center,1), "UniformOutput", false)), 2);
end