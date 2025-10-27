function mustBeAgents(agents)
    validGeometries = ["rectangularPrismConstraint";];
    if isa(agents, 'cell')
        for ii = 1:size(agents, 1)
            assert(isa(agents{ii}, "agent"), "Agent in index %d is not a valid agent class", ii);
        end
    else
        assert(isa(agents, validGeometries), "Agent is not a valid agent class");
    end
end