"""Inspect the existing live scene; never relocate the pad or the cameras."""
import sys, json, re
from pathlib import Path
import numpy as np
sys.path[:0] = [r'D:\仿真环境\work\python_deps', r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python\src']
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
ROOT = Path(r'D:\仿真环境')
c = RemoteAPIClient(); s = c.require('sim')
assert s.getStringParam(s.stringparam_scene_name) == 'dual_DVRK_ARM_suture.ttt'
objects = []
for h in s.getObjectsInTree(s.handle_scene, s.handle_all, 0):
    alias = s.getObjectAlias(h, 1)
    item = dict(handle=h, alias=alias, type=s.getObjectType(h), parent=s.getObjectParent(h), pose=s.getObjectPose(h, -1))
    if item['type'] == s.sceneobject_joint:
        item['q'] = s.getJointPosition(h)
        item['interval'] = s.getJointInterval(h)
    if any(v in alias for v in ['Needle', 'needle', 'Suture', 'Tissue', 'PadBase', 'visionSensor', 'customizableTable', 'L3_', 'L2_']):
        item['matrix'] = s.getObjectMatrix(h,-1)
        if item['type'] == s.sceneobject_shape:
            item['mesh'] = s.getShapeMesh(h)[:2]
        if item['type'] == s.sceneobject_visionsensor:
            item['resolution'] = s.getVisionSensorRes(h)
            item['angle'] = s.getObjectFloatParam(h,s.visionfloatparam_perspective_angle)
    objects.append(item)
(ROOT/'work/suture_review/scene_live.json').write_text(json.dumps(objects),encoding='utf-8')
for o in objects:
    if any(x in o['alias'] for x in ['Suture','Needle','needle','Tissue','PadBase','visionSensor','joint7','joint9','L3_']):
        print(o['handle'],o['alias'],o.get('q',''),o['pose'][:3],o.get('interval',''))
print('ring6-11:',[o['alias'] for o in objects if re.search(r'/ring(?:[6-9]|1[01])(?:$|\[)',o['alias'])])
