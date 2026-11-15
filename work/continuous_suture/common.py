"""Portable runtime paths; no developer machine or installed-package paths."""
import sys,json,os,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from kinematics import fk
STAGE=Path(os.getenv('SUTURE_STAGE_DIR',str(Path(tempfile.gettempdir())/'codex_continuous_suture')))
SCENE=STAGE/'dual_DVRK_ARM_continuous_suture.ttt'
PLAN=ROOT/'work/continuous_suture/continuous_plan.json'
FRAMES=Path(os.getenv('SUTURE_FRAMES_DIR',str(ROOT/'work/continuous_suture/raw_frames'))).resolve()
STEREO_FRAMES=Path(os.getenv('SUTURE_STEREO_FRAMES_DIR',str(ROOT/'work/continuous_suture/stereo_frames'))).resolve()
def connect():
    c=RemoteAPIClient(port=int(os.getenv('COPPELIA_ZMQ_PORT','23002')));c.initialTimeout=15
    return c,c.require('sim')
def initialize(s,p):
    h=s.getObject('/customizableTable/SutureTask/SutureController')
    def call(n,*a):return s.callScriptFunction(n,h,*a)
    call('initSutureAudit')
    for i,a in enumerate(['L','R'],1):
        q=np.array(p['initial'][a]);q[9:]+=.35 if a=='L' else 0;call('setSutureArm',i,q.tolist())
    tn=np.array(p['calibration']['R'])@fk(np.array(p['initial']['R']),'R')@np.array(p['relations']['R'])
    call('setSutureNeedle',tn[:3].ravel().tolist(),2)
    return h,call
