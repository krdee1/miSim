function mustBeDcm(dcm)
    % Assert 2D
    assert(numel(size(dcm)) == 2, "DCM is not 2D");
    % Assert square
    assert(size(unique(size(dcm)), 1) == 1, "DCM is not a square matrix");

    epsilon = 1e-9;
    % Assert inverse equivalent to transpose
    assert(all(abs(inv(dcm) - dcm') < epsilon, "all"), "DCM inverse is not equivalent to transpose");
    % Assert determinant is 1
    assert(det(dcm) > 1 - epsilon && det(dcm) < 1 + epsilon, "DCM has determinant not equal to 1");
end