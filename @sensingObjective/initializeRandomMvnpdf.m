function obj = initializeRandomMvnpdf(obj, domain, discretizationStep, protectedRange)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'sensingObjective')};
        domain (1, 1) {mustBeGeometry};
        discretizationStep (1, 1) double = 1;
        protectedRange (1, 1) double = 1;
    end
    arguments (Output)
        obj (1, 1) {mustBeA(obj, 'sensingObjective')};
    end

    % Set random objective position
    mu = domain.minCorner;
    while domain.distance(mu) < protectedRange
        mu = domain.random();
    end

    % Set random distribution parameters
    sig = [2 + rand * 2, 1; 1, 2 + rand * 2];

    % Set up random bivariate normal distribution function
    objectiveFunction = objectiveFunctionWrapper(mu(1:2), sig);

    % Regular initialization
    obj = obj.initialize(objectiveFunction, domain, discretizationStep, protectedRange);
end