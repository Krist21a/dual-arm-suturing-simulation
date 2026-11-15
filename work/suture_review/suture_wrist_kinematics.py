"""Existing mirrored jaw axes: opening and common physical yaw superpose."""
from plan_wrist_motion import fk as fk_static, np, Rotation
TCP=np.array([-.000006699866697,.000120966265910,.014496663479400])
PIVOT=np.array([-.000019473955035,.000121751800179,.005111404694620])
def fk(q,arm):
    yaw=(q[9]-q[10])/2
    T=fk_static(q,arm);R=Rotation.from_rotvec([yaw,0,0]).as_matrix()
    U=np.eye(4);U[:3,:3]=R;U[:3,3]=PIVOT-TCP+R@(TCP-PIVOT)
    return T@U
