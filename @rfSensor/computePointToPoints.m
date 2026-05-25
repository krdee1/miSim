function [d, dx, dy, dz] = computePointToPoints(~, agentPos, targetPos)
    dx = targetPos(:,1) - agentPos(1);
    dy = targetPos(:,2) - agentPos(2);
    dz = targetPos(:,3) - agentPos(3);
    d  = sqrt(dx.^2 + dy.^2 + dz.^2);
end
