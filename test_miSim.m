classdef test_miSim < matlab.unittest.TestCase
    properties (Access = private)
        testClass = miSim;
        % Domain
        domain = rectangularPrism;

        % Obstacles
        minNumObstacles = 1;
        maxNumObstacles = 3;
        obstacles = cell(1, 0);
        minObstacleDimension = 1;
        
        % Objective
        objective = sensingObjective;
        objectiveFunction = @(x, y) 0;
        objectiveDiscretizationStep = 0.01;
        protectedRange = 1;

        % Agents
        minAgents = 3;
        maxAgents = 9;
        agents = cell(1, 0);
        
        % Collision
        minCollisionRange = 0.1;
        maxCollisionRange = 0.5;
        collisionRanges = NaN;

        % Communications
        comRange = 5;
    end

    % Setup for each test
    methods (TestMethodSetup)
        % Generate a random domain
        function tc = setDomain(tc)
            % random integer-sized domain ranging from [0, 5] to [0, 25] in all dimensions
            L = ceil(5 + rand * 10 + rand * 10);
            tc.domain = tc.domain.initialize([zeros(1, 3); L * ones(1, 3)], REGION_TYPE.DOMAIN, "Domain");
        end
        % Generate a random sensing objective within that domain
        function tc = setSensingObjective(tc)
            mu = tc.domain.minCorner;
            while tc.domain.interiorDistance(mu) < tc.protectedRange
                mu = tc.domain.random();
            end
            mu(3) = 0;
            assert(tc.domain.contains(mu));
            sig = [2 + rand * 2, 1; 1, 2 + rand * 2];
            tc.objectiveFunction = @(x, y) mvnpdf([x(:), y(:)], mu(1:2), sig);
            tc.objective = tc.objective.initialize(tc.objectiveFunction, tc.domain.footprint, tc.domain.minCorner(3), tc.objectiveDiscretizationStep);
        end
        % Instantiate agents, they will be initialized under different
        % parameters in individual test cases
        function tc = setAgents(tc)
            for ii = 1:randi([tc.minAgents, tc.maxAgents])
                tc.agents{ii, 1} = agent;
            end
            tc.collisionRanges = tc.minCollisionRange + rand(size(tc.agents, 1), 1) * (tc.maxCollisionRange - tc.minCollisionRange);
        end
    end

    methods (Test)
        % Test methods
        function misim_initialization(tc)
            % randomly create 2-3 obstacles
            nGeom = tc.minNumObstacles + randi(tc.maxNumObstacles - tc.minNumObstacles);
            tc.obstacles = cell(nGeom, 1);
            for ii = 1:size(tc.obstacles, 1)
                % Instantiate a rectangular prism obstacle
                tc.obstacles{ii, 1} = rectangularPrism;

                % Randomly come up with dimensions until they
                % fit within the domain
                candidateMinCorner = [-Inf(1, 2), 0];
                candidateMaxCorner = Inf(1, 3);

                % make sure obstacles are not too small in any dimension
                tooSmall = true;
                while tooSmall
                    % make sure the obstacles don't contain the sensing
                    % objective or encroach on it too much
                    obstructs = true;
                    while obstructs
    
                        % Make sure the obstacle is in the domain
                        while any(candidateMinCorner < tc.domain.minCorner) 
                            candidateMinCorner = tc.domain.minCorner(1:3) + [(tc.domain.maxCorner(1:2) - tc.domain.minCorner(1:2)) .* rand(1, 2), 0]; % random spots on the ground
                        end
                        while any(candidateMaxCorner > tc.domain.maxCorner)
                            candidateMaxCorner = [candidateMinCorner(1:2), 0] + ((tc.domain.maxCorner(1:3) - tc.domain.minCorner(1:3)) .* rand(1, 3) ./ 2); % halved to keep from being excessively large
                        end
    
                        % once a domain-valid obstacle has been found, make
                        % sure it doesn't obstruct the sensing target
                        if all(candidateMinCorner(1:2) <= tc.objective.groundPos) && all(candidateMaxCorner(1:2) >= tc.objective.groundPos)
                            % reset to try again
                            candidateMinCorner = [-Inf(1, 2), 0];
                            candidateMaxCorner = Inf(1, 3);
                        else
                            obstructs = false;
                        end
                    end
                    if min(candidateMaxCorner - candidateMinCorner) >= tc.minObstacleDimension
                        tooSmall = false;
                    else
                        candidateMinCorner = [-Inf(1, 2), 0];
                        candidateMaxCorner = Inf(1, 3);
                    end
                end

                % Reduce infinite dimensions to the domain
                candidateMinCorner(isinf(candidateMinCorner)) = tc.domain.minCorner(isinf(candidateMinCorner));
                candidateMaxCorner(isinf(candidateMaxCorner)) = tc.domain.maxCorner(isinf(candidateMaxCorner));

                % Initialize obstacle geometry
                tc.obstacles{ii} = tc.obstacles{ii}.initialize([candidateMinCorner; candidateMaxCorner], REGION_TYPE.OBSTACLE, sprintf("Column obstacle %d", ii));
            end
            
            % Repeat this until a connected set of agent initial conditions
            % is found by random chance
            nIter = 0;
            connected = false;
            while ~connected
                % Randomly place agents in the domain
                for ii = 1:size(tc.agents, 1)
                    posInvalid = true;
                    while posInvalid
                        % Initialize the agent into a random spot in the
                        % domain (that is not too close to the sensing
                        % objective)
                        boringInit = true;
                        while boringInit
                            candidatePos = tc.domain.random();
                            if norm(candidatePos(1:2) - tc.objective.groundPos) >= norm(tc.domain.footprint(4, :) - tc.domain.footprint(1, :))/2
                                boringInit = false;
                            end
                        end
                        candidateGeometry = rectangularPrism;
                        tc.agents{ii} = tc.agents{ii}.initialize(candidatePos, zeros(1, 3), eye(3), candidateGeometry.initialize([candidatePos - tc.collisionRanges(ii) * ones(1, 3); candidatePos + tc.collisionRanges(ii) * ones(1, 3)], REGION_TYPE.COLLISION, sprintf("Agent %d collision volume", ii)), ii, sprintf("Agent %d", ii));
    
                        % Check obstacles to confirm that none are violated
                        for jj = 1:size(tc.obstacles, 1)
                            inside = false;
                            if tc.obstacles{jj, 1}.contains(tc.agents{ii, 1}.pos)
                                % Found a violation, stop checking
                                inside = true;
                                break;
                            end
                        end
    
                        % Agent is inside obstacle, try again
                        if inside
                            continue;
                        end
    
                        % Create a collision geometry for this agent
                        candidateGeometry = rectangularPrism;
                        candidateGeometry = candidateGeometry.initialize([tc.agents{ii}.pos - 0.1 * ones(1, 3); tc.agents{ii}.pos + 0.1 * ones(1, 3)], REGION_TYPE.COLLISION, sprintf("Agent %d collision volume", ii));
    
                        % Check previously placed agents for collisions
                        for jj = 1:(ii - 1)
                            % Check if previously defined agents collide with
                            % this one
                            colliding = false;
                            if candidateGeometry.contains(tc.agents{jj, 1}.pos)
                                % Found a violation, stop checking
                                colliding = true;
                                break;
                            end
                        end
    
                        % Agent is colliding with another, try again
                        if ii ~= 1 && colliding
                            continue;
                        end
    
                        % Allow to proceed since no obstacle/collision
                        % violations were found
                        posInvalid = false;
                    end
                end
    
                % Collect all agent positions
                posArray = arrayfun(@(x) x{1}.pos, tc.agents, 'UniformOutput', false);
                posArray = reshape([posArray{:}], size(tc.agents, 1), 3);  
    
                % Communications checks
                adjacency = false(size(tc.agents, 1), size(tc.agents, 1));
                for ii = 1:size(tc.agents, 1)
                    % Compute distance from each to all agents
                    for jj = 1:(size(tc.agents, 1))
                        if norm(posArray(ii, 1:3) - posArray(jj, 1:3)) <= tc.comRange
                            adjacency(ii, jj) = true;
                        end
                    end
                end
    
                % Check connectivity
                G = graph(adjacency);
                connected = all(conncomp(G) == 1);
                nIter = nIter + 1;
            end

            % Initialize the simulation
            tc.testClass = tc.testClass.initialize(tc.domain, tc.objective, tc.agents, tc.obstacles);

            % Plot domain
            f = tc.testClass.domain.plotWireframe;

            % Set plotting limits to focus on the domain
            xlim([tc.testClass.domain.minCorner(1) - 0.5, tc.testClass.domain.maxCorner(1) + 0.5]);
            ylim([tc.testClass.domain.minCorner(2) - 0.5, tc.testClass.domain.maxCorner(2) + 0.5]);
            zlim([tc.testClass.domain.minCorner(3) - 0.5, tc.testClass.domain.maxCorner(3) + 0.5]);

            % Plot obstacles
            for ii = 1:size(tc.testClass.obstacles, 1)
                tc.testClass.obstacles{ii, 1}.plotWireframe(f);
            end

            % Plot objective gradient
            f = tc.testClass.objective.plot(f);

            % Plot agents and their collision geometries
            for ii = 1:size(tc.testClass.agents, 1)
                f = tc.testClass.agents{ii, 1}.plot(f);
                f = tc.testClass.agents{ii, 1}.collisionGeometry.plotWireframe(f);
            end
        end
    end
end