function result = main_suture_demo(port)
% Run the embedded five-stitch, 3x demo from MATLAB (no Python required).
% Open CoppeliaSim Edu 4.10 first, stop its simulation, then run this function.
% Default ZMQ port: 23000. Pass 23002 for a simulator using that port.
    if nargin < 1, port = 23000; end
    root = fileparts(mfilename('fullpath'));
    addpath(fullfile(root, 'matlab'));
    javaaddpath(fullfile(root, 'matlab', 'jeromq-0.5.2.jar'));
    client = RemoteAPIClient('host','localhost','port',port,'timeoutMs',30000);
    cleanup = onCleanup(@()delete(client)); %#ok<NASGU>
    sim = client.require('sim');
    assert(sim.getSimulationState()==sim.simulation_stopped, ...
        'Save your current scene and stop its simulation before running this demo.');
    scene = fullfile(root,'scene.ttt');
    assert(isfile(scene),'scene.ttt is missing. Extract the complete demo package.');
    % Use an ASCII temporary filename for CoppeliaSim file-path compatibility.
    stage = fullfile(tempdir,'dual_arm_natural_3x_demo.ttt');
    copyfile(scene,stage);
    sim.loadScene(strrep(stage,'\','/'));
    driver = sim.getObject('/SoftSuturePlayback');
    sim.setStepping(true);
    sim.startSimulation();
    started = tic;
    try
        while true
            frameStarted = tic;
            client.step();
            progress = sim.callScriptFunction('getSoftProgress',driver);
            assert(isempty(progress.error),'Scene playback failed: %s',progress.error);
            hits = progress.collisions;
            assert(isempty(hits) || (isstruct(hits) && isempty(fieldnames(hits))), ...
                'Collision detected at playback frame %d.',progress.frame);
            if progress.done, break; end
            assert(toc(started)<600,'Demo timed out before completion.');
            pause(max(0,.05-toc(frameStarted)));
        end
        assert(progress.frame==1170 && progress.total==1170,'Unexpected demo length.');
        sim.pauseSimulation();sim.setStepping(false);
        result = struct('result','PASS','frames',progress.frame,'playback_speed',3, ...
            'matlab_version',version,'wall_seconds',toc(started),'collision_count',0);
        fid=fopen(fullfile(root,'matlab_validation.json'),'w');
        assert(fid>=0,'Cannot write the run report.');
        fileCleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
        fwrite(fid,jsonencode(result),'char');
        fprintf('PASS: %d checked frames; 3x demo complete.\n',progress.frame);
    catch err
        try, sim.pauseSimulation();sim.setStepping(false);catch, end
        rethrow(err);
    end
end
