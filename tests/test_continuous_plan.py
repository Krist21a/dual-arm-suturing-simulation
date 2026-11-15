"""Check the distributable FK against every reviewed target, without a simulator."""
import hashlib,json,sys,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'work/continuous_suture'))
from kinematics import fk

class ReviewedContinuousPlan(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path=ROOT/'work/continuous_suture/continuous_plan.json'
        cls.p=json.loads(cls.path.read_text(encoding='utf-8'))
    def test_reviewed_bytes(self):
        review=json.loads((ROOT/'output/reports/continuous_suture_plan_validation.json').read_text())
        self.assertEqual(review['result'],'PASS')
        self.assertEqual(hashlib.sha256(self.path.read_bytes()).hexdigest(),review['plan_sha256'])
    def test_five_points_and_fixed_spacing(self):
        self.assertEqual(self.p['stitch_count'],5)
        entries=np.array([s['entryPoint'] for s in self.p['stitches']])
        np.testing.assert_allclose(np.diff(entries,axis=0),np.tile([0,.003,0],(4,1)),atol=1e-12)
    def test_all_joint_limits_and_fk_targets(self):
        for phase in self.p['phases']:
            a=phase['arm'];q=np.array(phase['q']);ref=np.array(self.p['initial'][a]);cal=np.array(self.p['calibration'][a])
            self.assertTrue(np.all((q[:,6]>=0)&(q[:,6]<=.16)))
            self.assertLessEqual(np.max(abs(q[:,7])),np.pi)
            self.assertLessEqual(np.max(abs(q[:,8])),np.radians(85))
            self.assertLessEqual(np.max(abs(q[:,:6]-ref[:6])),np.radians(5))
            for qq,target in zip(q,phase['targets']):
                actual=cal@fk(qq,a);target=np.array(target)
                self.assertLess(np.linalg.norm(actual[:3,3]-target[:3,3]),.0001)
                self.assertLess(np.linalg.norm(actual[:3,:3]-target[:3,:3]),.003)
    def test_upright_circular_insertion(self):
        tip=np.array([0,.006,-.0115,1])
        for ph in self.p['phases']:
            if ph['name']!='right_insert':continue
            center=np.array(self.p['stitches'][ph['stitch']-1]['arcCenter'])
            for qq in ph['q']:
                needle=np.array(self.p['calibration']['R'])@fk(np.array(qq),'R')@np.array(self.p['relations']['R'])
                self.assertLess(abs(needle[2,0]),np.sin(np.radians(.5)))
                self.assertLess(abs(np.linalg.norm((needle@tip)[:3]-center)-.011375),.00001)

if __name__=='__main__':unittest.main()
