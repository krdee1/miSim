function v = setupVideoWriter(timestep)
    arguments (Input)
        timestep (1, 1) double;
    end
    arguments (Output)
        v (1, 1) {mustBeA(v, 'VideoWriter')};
    end

    if ispc || ismac
        v = VideoWriter(fullfile('sandbox', strcat(string(datetime('now'), 'yyyy_MM_dd_HH_mm_ss'), '_miSimHist')), 'MPEG-4');
    elseif isunix
        v = VideoWriter(fullfile('sandbox', strcat(string(datetime('now'), 'yyyy_MM_dd_HH_mm_ss'), '_miSimHist')), 'Motion JPEG AVI');
    end
    
    v.FrameRate = 1/timestep;
    v.Quality = 90;
end