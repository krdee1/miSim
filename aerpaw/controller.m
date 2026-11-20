function controller(numClients)
arguments (Input)
    numClients (1, 1) int32;
end

coder.extrinsic('disp', 'readScenarioCsv');

% Maximum clients supported (one initial position per UAV)
MAX_CLIENTS = 4;
% Three waypoints per UAV: stage (climb) → traverse (XY) → settle (descend)
MAX_TARGETS = MAX_CLIENTS * 3;

% Altitude-staggered flyout/flyback currently supports exactly 2 UAVs
if numClients ~= int32(2)
    error('Altitude-staggered flyout/flyback requires exactly 2 UAVs');
end

% Separation during scenario setup and teardown (flyout and return): one UAV
% moves at a time, and each move is planned (planTransit.m) to stay at least
% MIN_SEP + SEP_MARGIN from the other UAV whenever that UAV is airborne. A
% landed UAV imposes no constraint.
MIN_SEP     = 15.0; % m, required separation while both UAVs are airborne
SEP_MARGIN  = 5.0;  % m, planning margin (2 m arrival tolerance, tracking error)
TAKEOFF_ALT = 25.0; % m, takeoff and RTL altitude in uav_runner.py

% Allocate targets array (MAX_TARGETS x 3)
targets = zeros(MAX_TARGETS, 3);
totalLoaded  = int32(0);

% Experiment start positions from scenario CSV (N x 3)
scenarioPositions = zeros(MAX_CLIENTS, 3);

% Load experiment start positions from scenario CSV
if coder.target('MATLAB')
    disp('Loading initial positions from scenario.csv (simulation)...');
    sc = readScenarioCsv('aerpaw/config/scenario.csv');
    flatPos = double(sc.initialPositions);  % 1×(3*N) flat vector
    posMatrix = reshape(flatPos, 3, [])';   % N×3
    totalLoaded = int32(size(posMatrix, 1));
    scenarioPositions(1:totalLoaded, :) = posMatrix;
    disp(['Loaded ', num2str(double(totalLoaded)), ' initial positions']);
else
    coder.cinclude('controller_impl.h');
    filename = ['config/scenario.csv', char(0)];
    % Load into targets temporarily; copy to scenarioPositions below
    totalLoaded = coder.ceval('loadInitialPositions', coder.ref(filename), ...
                coder.ref(targets), int32(MAX_TARGETS));
    scenarioPositions(1:totalLoaded, :) = targets(1:totalLoaded, :);
end

% Load guidance scenario from CSV (parameters for guidance_step)
NUM_SCENARIO_PARAMS = 93; % must match NUM_SCENARIO_PARAMS in impl/controller_impl.h
MAX_OBSTACLES_CTRL  = int32(8);
scenarioParams = zeros(1, NUM_SCENARIO_PARAMS);
obstacleMin    = zeros(MAX_OBSTACLES_CTRL, 3);
obstacleMax    = zeros(MAX_OBSTACLES_CTRL, 3);
numObstacles   = int32(0);
if ~coder.target('MATLAB')
    coder.cinclude('controller_impl.h');
    scenarioFilename = ['config/scenario.csv', char(0)];
    scenarioOk = int32(0);
    scenarioOk = coder.ceval('loadScenario', coder.ref(scenarioFilename), coder.ref(scenarioParams));
    numObstacles = coder.ceval('loadObstacles', coder.ref(scenarioFilename), ...
                         coder.ref(obstacleMin), coder.ref(obstacleMax), ...
                         int32(MAX_OBSTACLES_CTRL));
    % Stop before any client connects or UAV takes off if the scenario did
    % not load, or if its sensor class is not the one this build runs
    % (scenarioParams(66) = sensorType; see buildSensorType.m). Standalone
    % generated code drops error(), so print and return instead.
    if scenarioOk == 0
        fprintf('controller: failed to load config/scenario.csv. Exiting.\n');
        return;
    end
    if scenarioParams(66) ~= buildSensorType()
        fprintf('controller: scenario.csv sensorType %g does not match this build (%g); see buildSensorType.m. Exiting.\n', ...
            scenarioParams(66), buildSensorType());
        return;
    end
end
% On MATLAB path, scenarioParams and obstacle arrays are left as zeros.
% guidance_step's MATLAB path loads parameters directly from scenario.csv
% via sim.initializeFromCsv and does not use these arrays.

% Initialize server
if coder.target('MATLAB')
    disp('Initializing server (simulation)...');
else
    coder.ceval('initServer');
end

% Accept clients
for i = 1:numClients
    if coder.target('MATLAB')
        disp(['Accepting client ', num2str(i)]);
    else
        coder.ceval('acceptClient', int32(i));
    end
end

% Query takeoff-pad GPS positions before sending any waypoints.
% These are also the flyback targets so each UAV lands where it took off.
initialPositions = zeros(MAX_CLIENTS, 3);
if ~coder.target('MATLAB')
    coder.ceval('sendRequestPositions', int32(numClients));
    coder.ceval('recvPositions', int32(numClients), coder.ref(initialPositions), int32(MAX_CLIENTS));
else
    % Simulation: treat scenario positions as the takeoff locations
    initialPositions(1:totalLoaded, :) = scenarioPositions(1:totalLoaded, :);
end

% ---- Sequential flyout ----------------------------------------------------
% One UAV at a time: TAKEOFF (vertical to TAKEOFF_ALT over its pad), climb or
% descend over the pad to its transit altitude, cross at that altitude to
% above its start position, descend or climb to the start, then hold there
% while the other UAV does the same. The first UAV flies while the other is
% still on the ground, so it needs no clearance check; the second UAV's
% whole path (takeoff column included) is planned to stay MIN_SEP +
% SEP_MARGIN from the first one's start position. Both orders are evaluated
% and the one that meets the clearance with the least vertical travel is
% flown. If no plan keeps MIN_SEP, the experiment stops here, before any
% takeoff.
if coder.target('MATLAB')
    transitAltMax = sc.domainMax(3);
else
    transitAltMax = scenarioParams(38); % domainMax(3), the scenario ceiling
end
padGround = [initialPositions(1:2, 1:2), zeros(2, 1)];
padTop    = [initialPositions(1:2, 1:2), TAKEOFF_ALT * ones(2, 1)];
starts    = scenarioPositions(1:2, :);

[flyOrder, flyAlt, flyClear] = chooseFlyout(padGround, padTop, starts, TAKEOFF_ALT, transitAltMax, MIN_SEP + SEP_MARGIN);
fprintf('Flyout plan: UAV %d first (transit %.1f m), then UAV %d (transit %.1f m); clearance %.1f m (required %.1f m, planned %.1f m)\n', ...
    int32(flyOrder(1)), flyAlt(1), int32(flyOrder(2)), flyAlt(2), flyClear, MIN_SEP, MIN_SEP + SEP_MARGIN);
if flyClear < MIN_SEP
    fprintf('Flyout: no sequence keeps %.1f m separation (best %.1f m). Check the start positions. Not taking off.\n', ...
        MIN_SEP, flyClear);
    for i = 1:numClients
        if ~coder.target('MATLAB')
            coder.ceval('sendMessageType', int32(i), int32(MESSAGE_TYPE.READY));
        end
    end
    if ~coder.target('MATLAB')
        coder.ceval('closeServer');
    end
    return;
elseif flyClear < MIN_SEP + SEP_MARGIN
    fprintf('Flyout: warning, planned clearance %.1f m is inside the %.1f m margin\n', flyClear, SEP_MARGIN);
end

for kk = 1:2
    u = flyOrder(kk);
    hU = flyAlt(kk);
    flyoutPath = [padTop(u, 1:2), hU; starts(u, 1:2), hU; starts(u, :)];
    if coder.target('MATLAB')
        disp(['Flyout UAV ', num2str(u), ': TAKEOFF, then ', mat2str(flyoutPath, 4)]);
    else
        sendCommand(int32(u), int32(MESSAGE_TYPE.TAKEOFF));
        for ww = 1:3
            sendLeg(int32(u), flyoutPath(ww, :));
        end
    end
end
% -------------------------------------------------------------------------------

% ---- Phase 2: miSim guidance loop ----------------------------------------
% Number of guidance steps comes from maxIter in scenario.csv (scenarioParams(2))
% so the controller and the agent step-decay schedule stay in sync.
if coder.target('MATLAB')
    MAX_GUIDANCE_STEPS = int32(sc.maxIter);
else
    MAX_GUIDANCE_STEPS = int32(scenarioParams(2));
end

% Enter guidance mode on all clients
if ~coder.target('MATLAB')
    coder.ceval('sendGuidanceToggle', int32(numClients));
end

% Request initial GPS positions and initialise guidance algorithm
positions = zeros(MAX_CLIENTS, 3);
if ~coder.target('MATLAB')
    coder.ceval('sendRequestPositions', int32(numClients));
    coder.ceval('recvPositions', int32(numClients), coder.ref(positions), int32(MAX_CLIENTS));
else
    % Simulation: seed positions from scenario positions so agents don't start at origin
    positions(1:totalLoaded, :) = scenarioPositions(1:totalLoaded, :);
end
guidance_step(positions(1:numClients, :), true, ...
              scenarioParams, obstacleMin, obstacleMax, numObstacles);

% Main guidance loop (event-triggered)
for step = 1:MAX_GUIDANCE_STEPS
    if ~coder.target('MATLAB')
        coder.ceval('setGuidanceStep', int32(step), int32(MAX_GUIDANCE_STEPS));
    end

    % Run one guidance step: feed current GPS positions in, get targets out
    nextPositions = guidance_step(positions(1:numClients, :), false, ...
                                  scenarioParams, obstacleMin, obstacleMax, numObstacles);

    % Send target to each client
    for i = 1:numClients
        target = nextPositions(i, :);
        if ~coder.target('MATLAB')
            coder.ceval('sendTarget', int32(i), coder.ref(target));
        else
            disp(['[step ', num2str(step), '] target UAV ', num2str(i), ': ', num2str(target)]);
        end
    end

    % Wait for ACK from all clients (each UAV ACKs when it arrives at its target)
    if ~coder.target('MATLAB')
        coder.ceval('waitForAllMessageType', int32(numClients), ...
                    int32(MESSAGE_TYPE.ACK));
    else
        disp(['[guidance] step ', num2str(step), ': all UAVs arrived']);
    end

    % Request current GPS positions from all clients
    if ~coder.target('MATLAB')
        coder.ceval('sendRequestPositions', int32(numClients));
        coder.ceval('recvPositions', int32(numClients), coder.ref(positions), int32(MAX_CLIENTS));
    else
        % Simulation: advance positions to guidance outputs for closed-loop feedback
        positions(1:numClients, :) = nextPositions(1:numClients, :);
    end
end

% Exit guidance mode on all clients (second toggle)
if ~coder.target('MATLAB')
    coder.ceval('sendGuidanceToggle', int32(numClients));
    % Wait for ACK from all clients: confirms each client has finished its
    % last guidance navigation and is back in sequential (ACK/READY) mode.
    coder.ceval('waitForAllMessageType', int32(numClients), ...
                int32(MESSAGE_TYPE.ACK));
    % Reset step counter so post-guidance logging carries no step prefix.
    coder.ceval('setGuidanceStep', int32(0), int32(MAX_GUIDANCE_STEPS));
end
% --------------------------------------------------------------------------

% ---- Sequential return -----------------------------------------------------
% One UAV at a time, while the other holds where guidance left it:
%   1. If the UAVs are closer than MIN_SEP + SEP_MARGIN, the first UAV moves
%      horizontally straight away from the other (the distance only grows
%      along that move) until they are that far apart.
%   2. If no transit altitude keeps the first UAV's return clear of the
%      holding UAV (e.g. the holder hovers above the first UAV's pad), the
%      holder first climbs straight up to the ceiling, itself checked
%      against the first UAV.
%   3. The first UAV climbs or descends to its transit altitude, crosses to
%      above its pad, RTLs to TAKEOFF_ALT over home and LANDs, all planned
%      to stay MIN_SEP + SEP_MARGIN from the holding UAV.
%   4. The second UAV then returns the same way; the first has landed, so
%      it needs no clearance check.
% Every combination of order and holder position is evaluated (see
% chooseReturn). The UAVs are airborne, so a plan below MIN_SEP is flown
% anyway (the best one found) with a warning.
finals = positions(1:2, :);
retStart = finals;
retMoved = false(1, 2);
for kk = 1:2
    [retStart(kk, :), retMoved(kk)] = separatePoint(finals(kk, :), finals(3 - kk, :), MIN_SEP + SEP_MARGIN);
end
[retOrder, retAlt, retClear, holdAt, holdMoved] = chooseReturn(retStart, finals, padTop, padGround, ...
    TAKEOFF_ALT, transitAltMax, MIN_SEP + SEP_MARGIN);
first = retOrder(1);
second = retOrder(2);
fprintf('Return plan: UAV %d first (transit %.1f m), then UAV %d (transit %.1f m); clearance %.1f m (required %.1f m, planned %.1f m); start separation %.1f m\n', ...
    int32(first), retAlt(1), int32(second), retAlt(2), retClear, MIN_SEP, MIN_SEP + SEP_MARGIN, norm(finals(1, :) - finals(2, :)));
if retMoved(first)
    fprintf('Return: UAV %d first moves away from UAV %d to (%.1f, %.1f, %.1f)\n', int32(first), int32(second), ...
        retStart(first, 1), retStart(first, 2), retStart(first, 3));
end
if holdMoved
    fprintf('Return: UAV %d climbs to (%.1f, %.1f, %.1f) and holds there while UAV %d returns\n', int32(second), ...
        holdAt(1), holdAt(2), holdAt(3), int32(first));
end
if retClear < MIN_SEP
    fprintf('Return: warning, best plan keeps only %.1f m separation (required %.1f m)\n', retClear, MIN_SEP);
elseif retClear < MIN_SEP + SEP_MARGIN
    fprintf('Return: warning, planned clearance %.1f m is inside the %.1f m margin\n', retClear, SEP_MARGIN);
end

firstPath  = [retStart(first, :); retStart(first, 1:2), retAlt(1); padTop(first, 1:2), retAlt(1)];
secondPath = [holdAt; holdAt(1:2), retAlt(2); padTop(second, 1:2), retAlt(2)];
if coder.target('MATLAB')
    disp(['Return UAV ', num2str(first), ': ', mat2str(firstPath, 4), ', then RTL and LAND']);
    disp(['Return UAV ', num2str(second), ': ', mat2str(secondPath, 4), ', then RTL and LAND']);
else
    if retMoved(first)
        sendLeg(int32(first), retStart(first, :));
    end
    if holdMoved
        sendLeg(int32(second), holdAt);
    end
    sendLeg(int32(first), firstPath(2, :));
    sendLeg(int32(first), firstPath(3, :));
    sendCommand(int32(first), int32(MESSAGE_TYPE.RTL));  % to TAKEOFF_ALT over home
    sendCommand(int32(first), int32(MESSAGE_TYPE.LAND)); % landed and disarmed on READY
    sendLeg(int32(second), secondPath(2, :));
    sendLeg(int32(second), secondPath(3, :));
    sendCommand(int32(second), int32(MESSAGE_TYPE.RTL));
    sendCommand(int32(second), int32(MESSAGE_TYPE.LAND));
end
% --------------------------------------------------------------------------

% Send READY to all clients to signal mission complete
for i = 1:numClients
    if coder.target('MATLAB')
        disp(['Sending READY to client ', num2str(i)]);
    else
        coder.ceval('sendMessageType', int32(i), int32(MESSAGE_TYPE.READY));
    end
end

% Close server
if ~coder.target('MATLAB')
    coder.ceval('closeServer');
end

disp('Experiment complete.');

end

% -------------------------------------------------------------------------
function [order, alt, clr] = chooseFlyout(padGround, padTop, starts, hMin, hMax, required)
% Order and transit altitudes for the sequential flyout. UAV u takes off
% from padGround(u) to padTop(u), then flies padTop(u) -> [pad xy, h] ->
% [start xy, h] -> starts(u). The first UAV flies while the other is on the
% ground (no constraint); the second must clear the first one's start. The
% order that meets `required` with the least vertical travel wins, else the
% one with the largest clearance.
order = [1, 2];
alt = [hMin, hMin];
clr = -Inf;
bestTravel = Inf;
for oo = 1:2
    a = oo;
    b = 3 - oo;
    [hA, ~, tA] = planTransit(padGround(a, :), padTop(a, :), starts(a, :), zeros(0, 3), NaN(1, 3), hMin, hMax, required);
    [hB, cB, tB] = planTransit(padGround(b, :), padTop(b, :), starts(b, :), zeros(0, 3), starts(a, :), hMin, hMax, required);
    if isBetter(cB, tA + tB, clr, bestTravel, required)
        order = [a, b];
        alt = [hA, hB];
        clr = cB;
        bestTravel = tA + tB;
    end
end
end

function [order, alt, clr, holdAt, holdMoved] = chooseReturn(retStart, finals, padTop, padGround, hMin, hMax, required)
% Order, transit altitudes and holding position for the sequential return.
% The first UAV a flies retStart(a) (its final position, or after moving
% away) -> [xy, h] -> [pad xy, h] -> padTop(a) (RTL) -> padGround(a) (LAND)
% while the other UAV b holds at holdAt. holdAt is finals(b), or finals(b)
% raised to hMax when that is needed to clear a's path (the climb itself must
% clear a, which waits at retStart(a)). b then returns from holdAt after a
% has landed (no constraint). Every order/holding combination is evaluated;
% the one meeting `required` with the least vertical travel wins, else the
% one with the largest clearance.
order = [1, 2];
alt = [hMin, hMin];
clr = -Inf;
holdAt = finals(2, :);
holdMoved = false;
bestTravel = Inf;
for oo = 1:2
    a = oo;
    b = 3 - oo;
    for hh = 1:2
        cand = finals(b, :);
        climb = 0;
        cClimb = Inf;
        if hh == 2
            if finals(b, 3) >= hMax
                continue;
            end
            cand(3) = hMax;
            climb = hMax - finals(b, 3);
            [~, cClimb, ~] = planTransit(zeros(0, 3), finals(b, :), cand, zeros(0, 3), retStart(a, :), cand(3), cand(3), required);
        end
        [hA, cA, tA] = planTransit(zeros(0, 3), retStart(a, :), padTop(a, :), padGround(a, :), cand, hMin, hMax, required);
        [hB, ~, tB] = planTransit(zeros(0, 3), cand, padTop(b, :), padGround(b, :), NaN(1, 3), hMin, hMax, required);
        c = min(cA, cClimb);
        t = tA + tB + climb;
        if isBetter(c, t, clr, bestTravel, required)
            order = [a, b];
            alt = [hA, hB];
            clr = c;
            holdAt = cand;
            holdMoved = hh == 2;
            bestTravel = t;
        end
    end
end
end

function tf = isBetter(c, t, cBest, tBest, required)
% A plan meeting `required` beats one that does not; among plans that meet
% it, less vertical travel wins; among plans that do not, more clearance.
okNew = c >= required;
okOld = cBest >= required;
tf = (okNew && (~okOld || t < tBest)) || (~okNew && ~okOld && c > cBest);
end

function [p, moved] = separatePoint(p, q, dist)
% Move p horizontally straight away from q until |p - q| = dist + 1 m (no
% change if already at least dist). The distance grows monotonically along
% the move; the extra metre keeps the result clear of dist after rounding.
moved = false;
dv = p(3) - q(3);
if norm(p - q) >= dist || abs(dv) >= dist
    return;
end
dist = dist + 1;
dxy = p(1:2) - q(1:2);
r = norm(dxy);
if r < 1e-6
    dxy = [1, 0];
    r = 1;
end
rNeed = sqrt(dist ^ 2 - dv ^ 2);
p(1:2) = q(1:2) + dxy / r * rNeed;
moved = true;
end

function sendLeg(clientId, target)
% Sequential-mode waypoint: TARGET, then ACK (received) and READY (arrived)
coder.cinclude('controller_impl.h');
t = target;
coder.ceval('sendTarget', clientId, coder.ref(t));
coder.ceval('waitForClientMessageType', clientId, int32(MESSAGE_TYPE.ACK));
coder.ceval('waitForClientMessageType', clientId, int32(MESSAGE_TYPE.READY));
end

function sendCommand(clientId, msgType)
% TAKEOFF / RTL / LAND to one client, then ACK and READY (completed)
coder.cinclude('controller_impl.h');
coder.ceval('sendMessageType', clientId, msgType);
coder.ceval('waitForClientMessageType', clientId, int32(MESSAGE_TYPE.ACK));
coder.ceval('waitForClientMessageType', clientId, int32(MESSAGE_TYPE.READY));
end
