function nextPos = basicGradientAscent(objectiveFunction, domain, pos, r)
    arguments (Input)
        objectiveFunction (1, 1) {mustBeA(objectiveFunction, 'function_handle')};
        domain (1, 1) {mustBeGeometry};
        pos (1, 3) double;
        r (1, 1) double;
    end
    arguments (Output)
        nextPos(1, 3) double;
    end

    % Evaluate objective at position offsets +/-[r, 0, 0] and +/-[0, r, 0]
    currentPos = pos(1:2);
    neighborPos = [currentPos(1) + r, currentPos(2); ... % (+x)
                   currentPos(1), currentPos(2) + r; ... % (+y)
                   currentPos(1) - r, currentPos(2); ... % (-x)
                   currentPos(1), currentPos(2) - r; ... % (-y)
                  ];

    % Check for neighbor positions that fall outside of the domain
    outOfBounds = false(size(neighborPos, 1), 1);
    for ii = 1:size(neighborPos, 1)
        if ~domain.contains([neighborPos(ii, :), 0])
            outOfBounds(ii) = true;
        end
    end

    % Replace out of bounds positions with inoffensive in-bounds positions
    neighborPos(outOfBounds, 1:3) = repmat(pos, sum(outOfBounds), 1);

    % Sense values at selected positions
    neighborValues = [objectiveFunction(neighborPos(1, 1), neighborPos(1, 2)), ... % (+x)
                      objectiveFunction(neighborPos(2, 1), neighborPos(2, 2)), ... % (+y)
                      objectiveFunction(neighborPos(3, 1), neighborPos(3, 2)), ... % (-x)
                      objectiveFunction(neighborPos(4, 1), neighborPos(4, 2)), ... % (-y)
                     ];

    % Prevent out of bounds locations from ever possibly being selected
    neighborValues(outOfBounds) = 0;
    
    % Select next position by maximum sensed value
    nextPos = neighborPos(neighborValues == max(neighborValues), :);
    nextPos = [nextPos(1, 1:2), pos(3)]; % just in case two get selected, simply pick one
end