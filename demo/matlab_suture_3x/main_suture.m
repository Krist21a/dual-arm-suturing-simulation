function result = main_suture(pythonExe, simPort, stiffness)
% MATLAB-controlled online suturing. Python only solves tissue/thread mechanics.
% Open CoppeliaSim, save/stop the current scene, then call main_suture.
% Example: main_suture('C:/Python313/python.exe',23000,35)
    if nargin<1 || isempty(pythonExe), pythonExe=getenv('SUTURE_PYTHON'); end
    if isempty(pythonExe), pythonExe='python'; end
    if nargin<2, simPort=23000; end
    if nargin<3, stiffness=35; end
    root=fileparts(mfilename('fullpath'));addpath(fullfile(root,'matlab'));
    client=RemoteAPIClient('host','localhost','port',simPort,'timeoutMs',30000);
    sim=client.require('sim');
    assert(sim.getSimulationState()==sim.simulation_stopped,'Save and stop the current simulation first.');
    stage=fullfile(tempdir,'matlab_online_suture.ttt');copyfile(fullfile(root,'scene.ttt'),stage);
    sim.loadScene(strrep(stage,'\','/'));
    commands=jsondecode(fileread(fullfile(root,'commands.json')));
    portProbe=java.net.ServerSocket(0);workerPort=portProbe.getLocalPort();portProbe.close();
    info=System.Diagnostics.ProcessStartInfo();info.FileName=pythonExe;
    info.Arguments=sprintf('"%s" --port %d --stiffness %.12g',fullfile(root,'physics','physics_worker.py'),workerPort,stiffness);
    info.UseShellExecute=false;info.CreateNoWindow=true;info.RedirectStandardError=true;
    worker=System.Diagnostics.Process.Start(info);
    guard=onCleanup(@()cleanupRuntime(sim,client,worker)); %#ok<NASGU>
    socket=[];
    for attempt=1:100
        if worker.HasExited, error('Physics process failed: %s',char(worker.StandardError.ReadToEnd())); end
        try, socket=tcpclient('127.0.0.1',double(workerPort),'Timeout',120);break;catch, pause(.1);end
    end
    assert(~isempty(socket),'Cannot connect to local physics solver. Install physics/requirements.txt.');
    configureTerminator(socket,'LF');
    controller=sim.getObject('/customizableTable/SutureTask/SutureController');
    geometry=sim.getObject('/OnlineSoftSuture');
    sim.setStepping(true);sim.startSimulation();client.step();
    started=tic;minimumJ=1;maxJointError=0;records=cell(numel(commands),1);
    checks={'arm_arm','left_pad','right_pad','left_table','right_table','left_self','right_self'};
    for k=1:numel(commands)
        a=commands(k);frameStarted=tic;
        % MATLAB issues both arm targets and the needle pose every step.
        sim.callScriptFunction('setSutureArm',controller,1,a.q(1,:));
        sim.callScriptFunction('setSutureArm',controller,2,a.q(2,:));
        sim.callScriptFunction('setSutureNeedle',controller,a.needle(:).',0);
        writeline(socket,jsonencode(a));response=jsondecode(char(readline(socket)));
        if ~response.ok,error('Physics solver error: %s',response.error);end
        sim.callScriptFunction('applySoftGeometry',geometry,response.geometry(:).');
        client.step();audit=sim.callScriptFunction('getSutureAudit',controller);
        for j=1:numel(checks), assert(audit.(checks{j}).collision==0,'Collision: %s, command %d',checks{j},k);end
        for arm=1:2
            if iscell(audit.joints),actual=audit.joints{arm};else,actual=audit.joints(arm,:);end
            if iscell(actual),actual=cellfun(@double,actual);end
            err=max(abs(double(actual(:))-a.q(arm,:).'));maxJointError=max(maxJointError,err);
        end
        assert(maxJointError<1e-6,'Actual joints differ from MATLAB commands.');
        minimumJ=min(minimumJ,response.minimum_volume_ratio);records{k}=response.metrics;
        if any(k==[1 145 numel(commands)])
            sensor=sim.getObject('/visionSensor_left');sim.handleVisionSensor(sensor);
            [rgb,res]=sim.getVisionSensorImg(sensor);if iscell(res),res=cellfun(@double,res);end
            img=flipud(permute(reshape(uint8(rgb),[3,double(res(1)),double(res(2))]),[3 2 1]));
            imwrite(img,fullfile(root,sprintf('online_%04d.png',k)));
        end
        if mod(k,100)==0,fprintf('%d / %d: %s, tension %.4f N\n',k,numel(commands),a.phase,response.metrics.tension_N);end
        pause(max(0,.05-toc(frameStarted)));
    end
    assert(response.completed_stitches==5 && response.metrics.punctures==5,'Incomplete suture.');
    result=struct('result','PASS','mode','MATLAB commands with online mechanics', ...
        'commands',numel(commands),'stiffness',stiffness,'minimum_volume_ratio',minimumJ, ...
        'maximum_joint_error',maxJointError,'collision_count',0,'final',response.metrics, ...
        'matlab_version',version,'wall_seconds',toc(started));
    writeJson(fullfile(root,'matlab_online_validation.json'),result);
    writeJson(fullfile(root,'online_metrics.json'),records);
    fprintf('PASS: MATLAB issued %d commands; five online-computed stitches complete.\n',numel(commands));
end
function writeJson(path,value)
    f=fopen(path,'w');assert(f>=0,'Cannot write %s',path);c=onCleanup(@()fclose(f)); %#ok<NASGU>
    fwrite(f,jsonencode(value),'char');
end
function cleanupRuntime(sim,client,worker)
    try, sim.pauseSimulation();sim.setStepping(false);catch, end
    try, if ~worker.HasExited,worker.Kill();end;catch, end
    try, delete(client);catch, end
end
