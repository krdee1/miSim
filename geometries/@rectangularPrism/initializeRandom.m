function [obj] = initializeRandom(obj, tag, label, minDimension, maxDimension, domain, minAlt)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'rectangularPrism')};
        tag (1, 1) REGION_TYPE = REGION_TYPE.INVALID;
        label (1, 1) string = "";
        minDimension (1, 1) double = 10;
        maxDimension (1, 1) double = 20;
        domain (1, 1) {mustBeGeometry} = rectangularPrism;
        minAlt (1, 1) double = 1;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'rectangularPrism')};
    end
    
    % Produce random bounds based on region type
    if tag == REGION_TYPE.DOMAIN
        % Domain
        L = ceil(minDimension + rand * (maxDimension - minDimension));
        bounds = [zeros(1, 3); L * ones(1, 3)];
    else
        % Obstacle

        % Produce a corners that are contained in the domain
        ii = 0;
        candidateMaxCorner = domain.maxCorner + ones(1, 3);
        candidateMinCorner = domain.minCorner - ones(1, 3);
        % Continue until the domain contains the obstacle without crowding the objective
        while ~domain.contains(candidateMaxCorner) || all(domain.objective.groundPos + domain.objective.protectedRange >= candidateMinCorner(1:2), 2) && all(domain.objective.groundPos - domain.objective.protectedRange <= candidateMaxCorner(1:2), 2)
            if ii == 0 || ii > 10
                candidateMinCorner = domain.random();
                candidateMinCorner(3) = minAlt; % bind to floor (plus minimum altitude constraint)
                ii = 1;
            end

            candidateMaxCorner = candidateMinCorner + minDimension + rand(1, 3) * (maxDimension - minDimension);

            ii = ii + 1;
        end
        
        bounds = [candidateMinCorner; candidateMaxCorner;];
    end

    % Regular initialization
    obj = obj.initialize(bounds, tag, label);
end