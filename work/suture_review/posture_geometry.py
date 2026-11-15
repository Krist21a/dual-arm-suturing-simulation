from plan_wrist_motion import *
from validate_suture_trajectory import rotz4

def chain(q,a):
 base=np.eye(4);base[:3,3]=[.6 if a=='R' else 0,0,.51465]
 ls=[np.array([[1,0,0,0],[0,1,0,0],[0,0,1,.0085],[0,0,0,1]]),np.array([[0,0,-1,-.0703],[0,1,0,0],[1,0,0,.06605],[0,0,0,1]]),np.array([[1,0,0,.4251],[0,1,0,0],[0,0,1,0],[0,0,0,1]]),np.array([[1,0,0,.39215],[0,1,0,0],[0,0,1,0],[0,0,0,1]]),np.array([[0,0,1,.04557],[0,1,0,0],[-1,0,0,.0397],[0,0,0,1]]),np.array([[0,0,-1,.01443],[0,1,0,0],[1,0,0,.04918],[0,0,0,1]])]
 out=[];T=base
 for i,l in enumerate(ls):
  T=T@l@rotz4(q[i]);out.append((cal[a]@T)[:3,3])
 return np.array(out)
if __name__=='__main__':
 p=json.loads((ROOT/'work/suture_review/upright_narrow_plan.json').read_text())
 for a in ['L','R']:
  print(a,'original',chain(q0[a],a).round(3));print('new',chain(np.array(p['initial'][a]),a).round(3))
