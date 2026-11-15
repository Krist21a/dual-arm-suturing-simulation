from common import *
c,s=connect();assert s.getSimulationState()==s.simulation_paused
report=json.loads((ROOT/'output/reports/continuous_suture_stereo_capture_validation.json').read_text());assert report['result']=='PASS'
for name in ['/SutureOverviewFront','/ContinuousRecorder','/ContinuousPlayback']:
    h=s.getObject(name,{'noError':True})
    if h>=0:s.removeObjects([h])
print('Temporary capture objects removed; final right-held needle state preserved.')
