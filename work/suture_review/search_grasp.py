"""Evaluate alternate grasp rolls with the existing calibrated FK/DLS."""
import sys, json, math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scene_inspection'))
from validate_suture_trajectory import *
ql = np.array([-.5235988,-.7853982,2.3561945,-2.268928,-1.5707963,1.3962634,.06,0,0,0,0])
qr = np.array([3.6651914,.7853982,-2.3561945,2.3561945,1.5707963,1.7453293,.1,1.3962634,-.8726646,0,0])
ll = pose4(-.0965305180783,.463791318812,.737393945071,-.442243035073,.825605445293,.0994321853465,.336020813566)
rr = pose4(.0717914064246,.501164659681,.747866412348,-.16093874618,.834979864671,.473866353381,.228818759514)
cal = {'L':ll@np.linalg.inv(fk(ql,'L')),'R':rr@np.linalg.inv(fk(qr,'R'))}
orig=pose4(-1.27738568e-5,.00383664369,.00160058175,.640856385205,-.640856378907,.298836231978,.298836245483)
gR = -orig[:3,:3].T@orig[:3,3]
centerLocal=np.array([0,.005854474479272,-.000125944725770]); tip=np.array([0,.006,-.0115])
rad=tip-centerLocal;radius=np.linalg.norm(rad);lr=rad/radius;lt=np.cross([1,0,0],lr)
center=np.array([-.03,.345,.5905+math.sqrt(radius**2-.008**2)])
wr=(np.array([-.022,.345,.5905])-center)/radius;wn=np.array([0,1,0]);wt=np.cross(wn,wr)
Rn=np.column_stack([wr,wt,wn])@np.column_stack([lr,lt,[1,0,0]]).T
entry=np.eye(4);entry[:3,:3]=Rn;entry[:3,3]=center-Rn@centerLocal
approach=entry.copy();approach[:3,3]+=[0,0,.035]
sweep=2*math.asin(.008/radius); exposure=math.radians(35)
end=entry.copy();end[:3,:3]=ry(sweep+exposure)@Rn;end[:3,3]=center-end[:3,:3]@centerLocal
graspAngle=math.atan2(rad[2],rad[1])-math.radians(15)
gL=centerLocal+radius*np.array([0,math.cos(graspAngle),math.sin(graspAngle)])
def relation(roll,point):
    t=np.eye(4);t[:3,:3]=rotz4(math.radians(roll))[:3,:3]@orig[:3,:3];t[:3,3]=-t[:3,:3]@point;return t
def segment(q,arm,poses):
    for p in poses:
        q,ok,it,ep,eo=solve_pose(np.linalg.inv(cal[arm])@p,q,arm)
        if not ok:return q,False,ep,eo
    return q,True,ep,eo
if __name__=='__main__':
    for arm,point in [('R',gR),('L',gL)]:
        for roll in [0,180,90,-90,45,-45,135,-135]:
            rel=relation(roll,point);q=qr.copy() if arm=='R' else ql.copy()
            if arm=='R':
                poses=list(interpolate(rr,approach@np.linalg.inv(rel),16))+list(interpolate(approach@np.linalg.inv(rel),entry@np.linalg.inv(rel),10))+arc_poses(entry,center,rel,sweep+exposure,np.zeros(3),40)[1]
            else:
                target=end@np.linalg.inv(rel);pre=target.copy();pre[:3,3]+=[-.025,0,.025]
                poses=list(interpolate(ll,pre,16))+list(interpolate(pre,target,10))+arc_poses(end,center,rel,math.radians(180),np.zeros(3),45)[1]
            q,ok,ep,eo=segment(q,arm,poses)
            print(arm,roll,ok,'ep',round(ep*1000,3),'eo',round(math.degrees(eo),3),'q7',round(q[6]*1000,2),'q9',round(math.degrees(q[8]),2),flush=True)
    print('grasp points',gR,gL)
