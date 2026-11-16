from mechanics import SoftSuture,Parameters
import numpy as np,time,json
s=SoftSuture();start=time.time()
for k in range(100):s.step()
print('rest',s.gap_mm(),flush=True)
for k in range(100):
    t=np.array([-.022,.345,.593-k*.00005])
    r=s.step(tip=t,puncture_key=1)
print('puncture',s.puncture_events,flush=True)
tail=np.array([-.038,.357,.605])
for j in range(1,6):s.deploy(j,tail)
for k in range(500):
    end=tail+np.array([0,0,.05*min(k/300,1)])
    r=s.step(cable_end=end)
    if k%100==0:print(k,r,flush=True)
print('final',r,'elapsed',time.time()-start,flush=True)
