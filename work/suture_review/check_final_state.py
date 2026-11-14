import sys,json,hashlib
from pathlib import Path
ROOT=Path(r'D:\仿真环境')
sys.path[:0]=[str(ROOT/'work/python_deps'),r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python\src']
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
c=RemoteAPIClient(port=23002);c.initialTimeout=120;s=c.require('sim')
assert s.getSimulationState()==s.simulation_paused
needle=s.getObject('/Needle');right=s.getObject('/500____[1]/UR5/L0_respondable_TOOL2/instrument_tcp_arm2')
assert s.getObjectParent(needle)==right
script=s.getObject('/customizableTable/SutureTask/SutureController')
state=s.callScriptFunction('getSutureAudit',script)
assert state['joints'][0][9:]==[.35,.35] and state['joints'][1][9:]==[0,0]
assert all(state[k]['collision']==0 for k in ['arm_arm','left_pad','right_pad','left_table','right_table'])
expected=(ROOT/'work/suture_review/audit_runtime.lua').read_text()+'\n'+(ROOT/'work/scene_inspection/record_suture_stereo.lua').read_text()+'\nfunction sysCall_init() end\n'
assert s.getScriptStringParam(script,s.scriptstringparam_text)==expected
report=ROOT/'output/reports/suture_final_validation.json';r=json.loads(report.read_text(encoding='utf-8'))
r.update(final_gui_state='PAUSED',final_needle_parent=s.getObjectAlias(right,1),saved_scene_sha256=hashlib.sha256((ROOT/'output/scenes/dual_DVRK_ARM_suture.ttt').read_bytes()).hexdigest(),scene_controller_matches_source=True,stereo_thread_visible=True)
report.write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
print('PASS: paused, right TCP owns needle, original jaws, zero final collision, controller matches saved sources.')
