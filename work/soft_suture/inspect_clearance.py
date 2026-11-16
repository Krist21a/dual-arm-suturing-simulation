import numpy as np
from PIL import Image
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from record_video import camera
s=RemoteAPIClient(port=23002).require('sim');d=s.getObject('/SoftSuturePlayback');s.setStepping(True);s.startSimulation()
f=np.fromfile('output/soft_suture_continuous/frames.bin',dtype='<f4').reshape(-1,1721)
s.callScriptFunction('applyLiveState',d,f[252].astype(float).tolist());s.step()
h=camera(s,'ClearanceReview',[-.12,.48,.68],[-.03,.345,.60],[1000,1000],45)
s.handleVisionSensor(h);raw,res=s.getVisionSensorImg(h);Image.frombytes('RGB',tuple(res),bytes(raw)).transpose(Image.Transpose.FLIP_TOP_BOTTOM).save('output/soft_suture_continuous/clearance_review.png')
s.pauseSimulation();s.setStepping(False)
