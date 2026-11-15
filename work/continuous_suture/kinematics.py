import math
import numpy as np
from scipy.spatial.transform import Rotation

def rotz4(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], float)

def pose4(x, y, z, qx, qy, qz, qw) -> np.ndarray:
    q = np.array([qx, qy, qz, qw], float)
    q /= np.linalg.norm(q)
    xq, yq, zq, wq = q
    r = np.array(
        [
            [1 - 2 * (yq * yq + zq * zq), 2 * (xq * yq - zq * wq), 2 * (xq * zq + yq * wq)],
            [2 * (xq * yq + zq * wq), 1 - 2 * (xq * xq + zq * zq), 2 * (yq * zq - xq * wq)],
            [2 * (xq * zq - yq * wq), 2 * (yq * zq + xq * wq), 1 - 2 * (xq * xq + yq * yq)],
        ]
    )
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = r, [x, y, z]
    return t

def fk(q: np.ndarray, arm: str) -> np.ndarray:
    base = np.eye(4)
    base[:3, 3] = ([0.6, 0, 0.51465] if arm == "R" else [0, 0, 0.51465])
    l010 = np.eye(4); l010[2, 3] = 0.0085
    l120 = np.array([[0, 0, -1, -0.0703], [0, 1, 0, 0], [1, 0, 0, 0.06605], [0, 0, 0, 1]], float)
    l230 = np.eye(4); l230[0, 3] = 0.4251
    l340 = np.eye(4); l340[0, 3] = 0.39215
    l450 = np.array([[0, 0, 1, 0.04557], [0, 1, 0, 0], [-1, 0, 0, 0.0397], [0, 0, 0, 1]], float)
    l560 = np.array([[0, 0, -1, 0.01443], [0, 1, 0, 0], [1, 0, 0, 0.04918], [0, 0, 0, 1]], float)
    l660 = np.eye(4); l660[2, 3] = 0.08938
    t6 = base @ l010 @ rotz4(q[0]) @ l120 @ rotz4(q[1]) @ l230 @ rotz4(q[2]) @ l340 @ rotz4(q[3]) @ l450 @ rotz4(q[4]) @ l560 @ rotz4(q[5]) @ l660
    if arm == "R":
        l670 = pose4(0.0442, -0.0682, 0.1175710689583, 0, 2**-0.5, 2**-0.5, 0)
        l7tool = pose4(0.0442, 0, 0, 0, 0, -2**-0.5, 2**-0.5)
    else:
        l670 = pose4(-0.0442, -0.0682, 0.1175710689583, -2**-0.5, 0, 0, 2**-0.5)
        l7tool = pose4(0.0442, 0, 0, 0, 0, 2**-0.5, 2**-0.5)
    m7 = np.eye(4); m7[2, 3] = q[6]
    ltool8 = pose4(0, 0, 0, 0, 0, -1, 0)
    l8link1 = pose4(0.000455975532532, -0.000129997730255, 0.226981401443, -2**-0.5, 2**-0.5, 0, 0)
    llink19 = pose4(0.000007484114705, 0.000230443676010, -0.225139383808, 2**-0.5, 0, 2**-0.5, 0)
    l9wrist = pose4(-0.003876566886900, -0.000133752822876, 0.000012136995792, -0.5, 0.5, 0.5, -0.5)
    lwristtip = pose4(-0.000006699866697, 0.000120966265910, 0.014496663479400, 0, 0, 0, 1)
    ttool = t6 @ l670 @ m7 @ l7tool
    return ttool @ ltool8 @ rotz4(q[7]) @ l8link1 @ llink19 @ rotz4(q[8]) @ l9wrist @ lwristtip

fk_static=fk
TCP=np.array([-.000006699866697,.000120966265910,.014496663479400])
PIVOT=np.array([-.000019473955035,.000121751800179,.005111404694620])
def fk(q,arm):
    yaw=(q[9]-q[10])/2
    T=fk_static(q,arm);R=Rotation.from_rotvec([yaw,0,0]).as_matrix()
    U=np.eye(4);U[:3,:3]=R;U[:3,3]=PIVOT-TCP+R@(TCP-PIVOT)
    return T@U
