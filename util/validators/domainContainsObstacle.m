function c = domainContainsObstacle(domain, obstacle)
    arguments (Input)
        domain (1, 1) {mustBeGeometry};
        obstacle (1, 1) {mustBeGeometry}; % this could be expanded to handle n obstacles in 1 call
    end
    arguments (Output)
        c (1, 1) logical;
    end

    switch class(domain)
        case 'rectangularPrism'
            switch class(obstacle)
                case 'rectangularPrism'
                    c = all(domain.minCorner <= obstacle.minCorner) && all(domain.maxCorner >= obstacle.maxCorner);
                otherwise
                    error("%s not implemented for obstacles of class %s", coder.mfunctionname, class(domain));
            end
        otherwise
            error("%s not implemented for domains of class %s", coder.mfunctionname, class(domain));
    end
end