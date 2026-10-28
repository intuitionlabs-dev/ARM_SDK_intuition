"""Cross-language check of the vendor float32 MIT encoder, including J2."""
import math
from pathlib import Path
import random
import shutil
import subprocess
import tempfile
import unittest

from metal_arm_control.can_protocol import encode_request
from metal_arm_control.config import MOTOR_PMAX, MOTOR_VMAX, MOTOR_TMAX
from metal_arm_control.types import MotorRequest, MotorAction


class VendorEncodingTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('c++'), 'C++ compiler required for vendor encoding oracle')
    def test_exact_bytes_against_vendor_cpp(self):
        source=Path(__file__).parent/'fixtures/metal_mit_reference.cpp'
        with tempfile.TemporaryDirectory() as tmp:
            binary=Path(tmp)/'reference'
            subprocess.run(['c++','-std=c++11','-O0',str(source),'-o',str(binary)],check=True,capture_output=True)
            rows=[]; expected=[]
            rng=random.Random(1438)
            for i in range(7):
                for _ in range(100):
                    p,v,t=[rng.uniform(-limit,limit) for limit in (MOTOR_PMAX[i],MOTOR_VMAX[i],MOTOR_TMAX[i])]
                    kp,kd=rng.uniform(0,500),rng.uniform(0,5)
                    rows.append(' '.join(map(str,(MOTOR_PMAX[i],MOTOR_VMAX[i],MOTOR_TMAX[i],p,v,t,kp,kd))))
                    expected.append(encode_request(MotorRequest(0,i,i+1,'dm',MotorAction.CONTROL,0,
                                     position=p,velocity=v,torque=t,kp=kp,kd=kd)).data.hex())
            actual=subprocess.run([str(binary)],input='\n'.join(rows)+'\n',text=True,capture_output=True,check=True)
            self.assertEqual(actual.stdout.splitlines(),expected)


if __name__=='__main__': unittest.main()
