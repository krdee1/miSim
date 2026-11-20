function [h, clearance, travel] = planTransit(pre, p0, p1, post, q, hMin, hMax, required)
% planTransit  Transit altitude for one UAV's climb-cross-descend move that
% keeps it clear of a stationary airborne UAV.
%
%   [h, clearance, travel] = planTransit(pre, p0, p1, post, q, hMin, hMax, required)
%
% The moving UAV flies the straight-line path
%   pre(1,:) -> ... -> p0 -> [p0(1:2), h] -> [p1(1:2), h] -> p1 -> post(1,:) -> ...
% (pre and post are fixed points flown before p0 and after p1, e.g. the
% takeoff column from the ground or the landing column to the pad; either may
% have zero rows). The other UAV holds at q; q = NaN(1,3) means it is on the
% ground and imposes no constraint.
%
% h is searched over [hMin, hMax] in 0.5 m steps. Among altitudes whose
% clearance (smallest distance from q to any segment of the path) is at
% least `required`, the one with the least vertical travel
% |p0(3) - h| + |h - p1(3)| is returned; among equal travel the highest one
% (cross high, then descend). If none qualifies, the altitude with the
% largest clearance is returned and the caller decides what to do.
%
% Used by controller.m to plan the sequential flyout and return.
arguments (Input)
    pre (:, 3) double;
    p0 (1, 3) double;
    p1 (1, 3) double;
    post (:, 3) double;
    q (1, 3) double;
    hMin (1, 1) double;
    hMax (1, 1) double;
    required (1, 1) double;
end
arguments (Output)
    h (1, 1) double;
    clearance (1, 1) double;
    travel (1, 1) double;
end

hStep = 0.5;
hTop = max(hMax, hMin);
nH = floor((hTop - hMin) / hStep) + 1;

bestFeasible = false;
h = hMin;
clearance = -Inf;
travel = Inf;
for kk = 1:nH
    hk = hMin + (kk - 1) * hStep;
    pts = [pre; p0; p0(1:2), hk; p1(1:2), hk; p1; post];
    c = pathClearance(pts, q);
    t = abs(p0(3) - hk) + abs(hk - p1(3));
    feasible = c >= required;
    if feasible
        better = ~bestFeasible || t <= travel + 1e-9; % h ascends, so ties go to the higher altitude
    else
        better = ~bestFeasible && c > clearance;
    end
    if better
        bestFeasible = feasible;
        h = hk;
        clearance = c;
        travel = t;
    end
end
end

function d = pathClearance(pts, q)
% Smallest distance from point q to the polyline through pts (Inf if q is NaN)
if any(isnan(q))
    d = Inf;
    return;
end
d = Inf;
for ii = 1:(size(pts, 1) - 1)
    d = min(d, pointSegmentDistance(q, pts(ii, :), pts(ii + 1, :)));
end
end

function d = pointSegmentDistance(q, a, b)
ab = b - a;
L2 = sum(ab .^ 2);
if L2 <= 0
    d = norm(q - a);
    return;
end
t = max(0, min(1, sum((q - a) .* ab) / L2));
d = norm(q - (a + t * ab));
end
