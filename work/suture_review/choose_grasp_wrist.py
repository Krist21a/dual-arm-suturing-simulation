from plan_wrist_motion import *
from suture_wrist_kinematics import fk

def choose_wrist(q,a,angle,positions=None):
 z=q.copy();z[7:]=0;R0=(cal[a]@fk(z,a))[:3,:3]
 axes=[]
 for j in [7,8,9]:
  t=z.copy();t[j]=1e-5
  if j==9:t[10]=-1e-5
  axes.append(Rotation.from_matrix((cal[a]@fk(t,a))[:3,:3]@R0.T).as_rotvec()/1e-5)
 C=np.array(axes).T;sign=np.linalg.det(C);C[:,2]*=sign
 assert np.linalg.norm(C.T@C-np.eye(3))<1e-6
 back=np.linalg.inv(pose4(-.003876566886900,-.000133752822876,.000012136995792,-.5,.5,.5,-.5)@pose4(-.000006699866697,.000120966265910,.014496663479400,0,0,0,1))
 choices=[]
 for delta in range(-180,181,15):
  for pitch,yaw0 in [(p,y) for p in [0,20,35,50,65,75,-20,-35,-50,-65,-75] for y in [-75,-60,-30,0,30,60,75]]:
   t=q.copy();t[7]+=np.radians(delta);t[8]=np.radians(pitch);t[9:]=[np.radians(yaw0),-np.radians(yaw0)];Rstart=(cal[a]@fk(t,a))[:3,:3]
   rotations=Rotation.from_rotvec(np.linspace(0,angle,41)[:,None]*np.array([0,1,0])).as_matrix()@Rstart
   euler=Rotation.from_matrix(C.T@rotations@R0.T@C).as_euler('XYZ');euler[:,2]*=sign
   euler=np.unwrap(euler,axis=0);pmax=np.degrees(abs(euler[:,1])).max();ymax=np.degrees(abs(euler[:,2])).max()
   if pmax<80 and ymax<78 and np.degrees(euler[:,0]).min()>-174 and np.degrees(euler[:,0]).max()<174:
    if positions is not None:
     heights=[]
     for k,wrist in enumerate(euler):
      qi=q.copy();qi[7:]=[wrist[0],wrist[1],wrist[2],-wrist[2]]
      T=cal[a]@fk(qi,a);joint9=cal[a]@fk_legacy(qi,a)@back
      heights.append(positions[k,2]+joint9[2,3]-T[2,3])
     if min(heights)<.593:continue
    roll=abs(np.degrees(euler[-1,0]-euler[0,0]));rollPenalty=.8*max(0,np.degrees(angle)*.8-roll)
    choices.append((pmax+ymax+rollPenalty+.03*abs(delta)+.05*abs(pitch-35)+.1*abs(yaw0),delta,pitch,yaw0,pmax,ymax,np.degrees(euler[-1,0]-euler[0,0])))
 assert choices,(a,'No wrist coverage')
 return sorted(choices)
if __name__=='__main__':
 samples={'L':[-.79856,-.9398,2.47175,-1.99711,-1.60449,1.38849,.06488,-3.00067,.61087,0,0],'R':[3.79231,.8885,-2.63442,2.41283,1.61383,1.97807,.07641,3.39914,.61087,0,0]}
 for a,q in samples.items():print(a,choose_wrist(np.array(q),a,np.radians(170 if a=='L' else 105.384))[:8])
