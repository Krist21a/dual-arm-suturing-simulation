"""Capture actual CoppeliaSim sensor renders of the computed deformation."""
import os,json,tempfile
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
ROOT=Path(__file__).resolve().parents[2]

def main():
    out=ROOT/'output/soft_suture';m=json.loads((out/'manifest.json').read_text())
    c=RemoteAPIClient(port=int(os.getenv('COPPELIA_ZMQ_PORT','23002')));s=c.require('sim')
    assert s.getSimulationState()==s.simulation_paused
    driver=s.getObject('/SoftSuturePlayback')
    eye=np.array([-.03,.395,.675]);target=np.array([-.03,.351,.589])
    z=target-eye;z/=np.linalg.norm(z);x=np.cross([0,0,1],z);x/=np.linalg.norm(x);y=np.cross(z,x)
    h=s.createVisionSensor(1+2+4+64+128,[1200,900,0,0],[.005,2,np.radians(50),.01,0,0,.13,.15,.19,0,0])
    s.setObjectAlias(h,'SoftSutureCloseup');s.setObjectMatrix(h,np.column_stack([x,y,z,eye]).ravel().tolist(),-1)
    s.setObjectInt32Param(h,s.visionintparam_render_mode,0)
    first=int(m['puncture_events'][0]['time']/.05)+1
    snapshots=[(1,'initial'),(first,'puncture'),(m['before_tightening']['frame'],'before_pull'),(m['frame_count'],'closed')]
    for frame,name in snapshots:
        s.callScriptFunction('previewFrame',driver,frame);s.handleVisionSensor(h)
        raw,res=s.getVisionSensorImg(h)
        im=Image.frombytes('RGB',tuple(res),bytes(raw)).transpose(Image.Transpose.FLIP_TOP_BOTTOM)
        im.save(out/(name+'.png'))
    font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',28)
    board=Image.new('RGB',(1600,1260),(245,247,250));draw=ImageDraw.Draw(board)
    labels=['初始：可变形组织网格','穿刺：针尖接触造成局部凹陷','拉紧前：五针连续缝线','拉紧后：张力带动切口靠拢']
    for k,(_,name) in enumerate(snapshots):
        x=(k%2)*800;y=(k//2)*630
        board.paste(Image.open(out/(name+'.png')).resize((800,600)),(x,y+30))
        draw.text((x+15,y),labels[k],font=font,fill=(20,35,55))
    board.save(out/'soft_suture_comparison.png')
    print(out/'soft_suture_comparison.png')

if __name__=='__main__':main()
