function RunSutureWristCycle(root)
% Execute the collision-reviewed small-arm / distal-wrist trajectory.
% Preserve the delivered joint10/joint11 geometry, axes and control commands.
    planPath=fullfile(root,'work','suture_review','wrist_plan.json');
    plan=jsondecode(fileread(planPath));
    assert(abs(plan.arcNormal(3))<1e-9 && abs(plan.needle_plane_tilt_deg)<1e-9, ...
        'Needle plane must be vertical to the pad. Inclined-needle plans are invalid.');
    review=jsondecode(fileread(fullfile(root,'output','reports','suture_plan_validation.json')));
    bytes=fileread(planPath);md=java.security.MessageDigest.getInstance('SHA-256');
    digest=typecast(md.digest(uint8(bytes)),'uint8');sha=lower(reshape(dec2hex(digest,2).',1,[]));
    assert(review.pass && strcmp(sha,review.plan_sha256),'Plan must pass live mesh review before execution.');
    [client,sim,port]=connectReviewedScene(plan);
    fprintf('Connected to reviewed suture scene on port %d.\n',port);
    assert(sim.getSimulationState()==sim.simulation_stopped,'Stop the previous simulation before running.');
    script=sim.getObject('/customizableTable/SutureTask/SutureController');
    sim.callScriptFunction('initSutureAudit',script);
    q={plan.initial.L(:).',plan.initial.R(:).'};q{1}(10:11)=q{1}(10:11)+.35;
    for a=1:2,sim.callScriptFunction('setSutureArm',script,a,q{a});end
    Tneedle=plan.calibration.R*GetFKSutureWrist(q{2},'R')*plan.relations.R;
    sim.callScriptFunction('setSutureNeedle',script,reshape(Tneedle(1:3,:).',1,[]),2);
    fixed0=sim.callScriptFunction('getSutureAudit',script);
    assert(norm(doubleVector(fixed0.fixed{1}(1:3))-[-.03 .345 .5755])<1e-8,'Pad moved.');
    frames=fullfile(tempdir,'codex_suture_video_final');if ~exist(frames,'dir'),mkdir(frames);end
    logPath=fullfile(root,'output','reports','suture_live_frames.jsonl');f=fopen(logPath,'w');
    transfers=fopen(fullfile(root,'output','reports','suture_handoff_validation.tsv'),'w');
    fprintf(transfers,'handoff\ttranslation_mm\trotation_deg\tresult\n');
    cleanup=onCleanup(@()cleanupFiles(f,transfers)); %#ok<NASGU>
    frame=0;theta=0;extracted=false;grip=[.35 0];
    sim.setStepping(true);sim.startSimulation();
    % CoppeliaSim's first few dynamics steps initialize joint state. Hold both
    % arms at the same prepared pose until measured values are stationary,
    % then begin the stitch and recording from that verified initial pose.
    for warmup=1:6
        for a=1:2,sim.callScriptFunction('setSutureArm',script,a,q{a});end
        client.step();
    end
    for a=1:2,sim.callScriptFunction('setSutureArm',script,a,q{a});end
    sim.callScriptFunction('setSutureNeedle',script,reshape(Tneedle(1:3,:).',1,[]),2);
    sim.callScriptFunction('startSutureRecording',script,strrep(frames,'\','/'));
    try
        capture('initial',0);
        for p=1:numel(plan.phases)
            phase=plan.phases(p);a=1+strcmp(phase.arm,'R');traj=phase.q;
            assert(all(traj(:,7)>=-1e-8 & traj(:,7)<=.16+1e-8),'joint7 limit');
            assert(all(abs(traj(:,9))<=deg2rad(85)+1e-8),'joint9 limit');
            ref=plan.dock.(phase.arm);ref=ref(:).';
            assert(all(all(abs(traj(:,1:6)-ref(1:6))<=deg2rad(plan.proximal_limit_deg)+1e-8)),'Proximal arm range');
            fprintf('%s: %d poses\n',phase.name,size(traj,1));
            for k=1:size(traj,1)
                q{a}=traj(k,:);q{a}(10:11)=q{a}(10:11)+grip(a);
                sim.callScriptFunction('setSutureArm',script,a,q{a});
                if ~isempty(phase.needleAngles),theta=phase.needleAngles(k);end
                if strcmp(phase.name,'left_handoff'),extracted=true;end
                capture(phase.name,k,squeeze(phase.targets(k,:,:)),a);
            end
            if strcmp(phase.name,'left_grasp')
                jaws(1,0,'left_close');transfer(1,'right_to_left');jaws(2,.35,'right_open');
            elseif strcmp(phase.name,'right_regrasp')
                jaws(2,0,'right_close');transfer(2,'left_to_right');jaws(1,.35,'left_open');
            end
        end
        for k=1:20,capture('complete',k);end
        sim.setInt32Signal('singleSutureDemoComplete',1);
        sim.callScriptFunction('finishSutureRecording',script);
        sim.pauseSimulation();sim.setStepping(false);
        fprintf('PASS: %d checked stereo frames; paused with right arm holding needle.\n',frame);
    catch err
        sim.callScriptFunction('abortSutureRecording',script);
        sim.pauseSimulation();sim.setStepping(false);rethrow(err);
    end
    function capture(name,step,target,activeArm)
        sim.callScriptFunction('updateSutureThread',script,theta,extracted);
        client.step();st=sim.callScriptFunction('getSutureAudit',script);
        needleMatrix=reshape(doubleVector(st.needle),4,3).';
        % The fitted needle lies in its local Y-Z plane. Its world normal
        % must therefore remain horizontal: the needle stays upright.
        assert(abs(needleMatrix(3,1))<sind(.5),'Needle plane tilted at %s',name);
        checks={'arm_arm','left_pad','right_pad','left_table','right_table','left_self','right_self'};
        for ci=1:numel(checks)
            assert(st.(checks{ci}).collision==0,'Collision at %s: %s',name,checks{ci});
        end
        for fi=1:3
            assert(norm(doubleVector(st.fixed{fi})-doubleVector(fixed0.fixed{fi}))<1e-9,'Pad/camera pose changed.');
        end
        if nargin>2
            live=reshape(doubleVector(st.tcps{activeArm}),4,3).';
            assert(norm(live(:,4)-target(1:3,4))<.0001,'TCP tracking error at %s step %d',name,step);
        end
        frame=frame+1;st.frame=frame;st.phase=name;st.step=step;
        fprintf(f,'%s\n',jsonencode(st));
        sim.callScriptFunction('captureSutureFrame',script);
    end
    function jaws(a,target,name)
        start=grip(a);
        for ki=1:12
            yaw=(q{a}(10)-q{a}(11))/2;
            grip(a)=start+(target-start)*ki/12;q{a}(10:11)=[yaw+grip(a),-yaw+grip(a)];
            sim.callScriptFunction('setSutureArm',script,a,q{a});capture(name,ki);
        end
    end
    function transfer(receiver,name)
        pair=sim.callScriptFunction('handoffSutureNeedle',script,receiver);
        before=reshape(doubleVector(pair{1}),4,3).';after=reshape(doubleVector(pair{2}),4,3).';
        dp=norm(before(:,4)-after(:,4));R=after(:,1:3)*before(:,1:3).';
        da=acos(max(-1,min(1,(trace(R)-1)/2)));
        assert(dp<1e-7 && da<1e-6,'Needle jumped during %s',name);
        fprintf(transfers,'%s\t%.9f\t%.9f\tPASS\n',name,dp*1000,rad2deg(da));
        capture(name,0);
    end
end
function [client,sim,port]=connectReviewedScene(plan)
    preferred=str2double(getenv('COPPELIA_ZMQ_PORT'));ports=23000:23009;
    if isfinite(preferred),ports=unique([preferred ports],'stable');end
    for port=ports
        try
            probe=RemoteAPIClient('port',port,'timeoutMs',1500);ps=probe.require('sim');
            if ~strcmp(char(ps.getStringParam(ps.stringparam_scene_name)),'dual_DVRK_ARM_suture.ttt'),continue;end
            ps.getObject('/customizableTable/SutureTask/SutureController');
            entry=ps.getObjectPosition(ps.getObject('/customizableTable/SutureTask/EntryPoint'),-1);
            if norm(doubleVector(entry)-plan.entryPoint(:).')>1e-8,continue;end
            clear probe;client=RemoteAPIClient('port',port,'timeoutMs',120000);sim=client.require('sim');return;
        catch
        end
    end
    error('Open the delivered reviewed dual_DVRK_ARM_suture.ttt scene.');
end
function x=doubleVector(x)
    if iscell(x),x=cellfun(@double,x);end;x=double(x(:)).';
end
function cleanupFiles(a,b)
    fclose(a);fclose(b);
end
