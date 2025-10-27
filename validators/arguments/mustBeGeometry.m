function mustBeGeometry(geometry)
    validGeometries = ["rectangularPrism";];
    if isa(geometry, 'cell')
        for ii = 1:size(geometry, 1)
            assert(isa(geometry{ii}, validGeometries), "Geometry in index %d is not a valid geometry class", ii);
        end
    else
        assert(isa(geometry, validGeometries), "Geometry is not a valid geometry class");
    end
end