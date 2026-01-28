function v = setupVideoWriter(obj)
    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
    end
    arguments (Output)
        v (1, 1) {mustBeA(v, "VideoWriter")};
    end

    if ispc || ismac
        v = VideoWriter(fullfile(matlab.project.rootProject().RootFolder, "sandbox", strcat(obj.artifactName, "_miSimHist")), "MPEG-4");
    elseif isunix
        v = VideoWriter(fullfile(matlab.project.rootProject().RootFolder, "sandbox", strcat(obj.artifactName, "_miSimHist")), "Motion JPEG AVI");
    end
    
    v.FrameRate = 1 / obj.timestep;
    v.Quality = 90;
end