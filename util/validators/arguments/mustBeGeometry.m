function mustBeGeometry(geometry)
    if isa(geometry, 'cell')
        for ii = 1:size(geometry, 1)
            assert(isa(geometry{ii}, 'rectangularPrism') || isa(geometry{ii}, 'spherical'), ...
                   'Geometry in index %d is not a valid geometry class', ii);
        end
    else
        assert(isa(geometry, 'rectangularPrism') || isa(geometry, 'spherical'), ...
               'Geometry is not a valid geometry class');
    end
end