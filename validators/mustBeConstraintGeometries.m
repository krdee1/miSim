function mustBeConstraintGeometries(constraintGeometry)
    validGeometries = ["rectangularPrismConstraint";];
    if isa(constraintGeometry, 'cell')
        for ii = 1:size(constraintGeometry, 1)
            assert(isa(constraintGeometry{ii}, validGeometries), "Constraint geometry in index %d is not a valid constraint geometry class", ii);
        end
    else
        assert(isa(constraintGeometry, validGeometries), "Constraint geometry is not a valid constraint geometry class");
    end
end