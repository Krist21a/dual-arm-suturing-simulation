from plan_wrist_motion import *
sys.path.insert(0,r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python\src')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
c=RemoteAPIClient(port=23001);s=c.require('sim');h=s.getScript(s.scripttype_addonscript,'record_suture_stereo')
p=json.loads((ROOT/'work/suture_review/wrist_plan.json').read_text())
for a,name in [('L','left_grasp'),('R','right_expose')]:
    q=next(x for x in p['phases'] if x['name']==name)['q'][-1]
    s.callScriptFunction('setSutureArm',h,1 if a=='L' else 2,q)
    shape=s.getObject(roots[a]+'/L0_respondable_TOOL2/L1_respondable_TOOL2')
    v=np.array(s.getShapeMesh(shape)[0]).reshape(-1,3);m=np.array(s.getObjectMatrix(shape,-1)).reshape(3,4)
    world=v@m[:,:3].T+m[:,3];mean=world.mean(axis=0);u,sv,vh=np.linalg.svd(world-mean,full_matrices=False);direction=vh[0];z=(world-mean)@direction
    ends=mean+np.array([z.min(),z.max()])[:,None]*direction
    print(a,'ends',ends.tolist(),'shaft axis',direction.tolist(),flush=True)
