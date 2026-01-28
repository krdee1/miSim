function mustBeGeometry(geometry)
    validGeometries = ["rectangularPrism"; "spherical"];
    if isa(geometry, "cell")
        for ii = 1:size(geometry, 1)
            assert(any(arrayfun(@(x) isa(geometry{ii}, x), validGeometries)), "Geometry in index %d is not a valid geometry class", ii);
        end
    else
        assert(any(arrayfun(@(x) isa(geometry, x), validGeometries)), "Geometry is not a valid geometry class");
    end
end