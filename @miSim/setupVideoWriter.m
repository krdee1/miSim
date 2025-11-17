function v = setupVideoWriter(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, 'miSim')};
    end
    arguments (Output)
        v (1, 1) {mustBeA(v, 'VideoWriter')};
    end

    if ispc || ismac
        v = VideoWriter(fullfile('sandbox', strcat(string(datetime('now'), 'yyyy_MM_dd_HH_mm_ss'), '_miSimHist')), 'MPEG-4');
    elseif isunix
        v = VideoWriter(fullfile('.', strcat(string(datetime('now'), 'yyyy_MM_dd_HH_mm_ss'), '_miSimHist')), 'Motion JPEG AVI');
    end
    
    v.FrameRate = 1 / obj.timestep;
    v.Quality = 90;
end