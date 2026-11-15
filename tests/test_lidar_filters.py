import math
import unittest
import numpy as np
from coind6.protocol import Point
from lidar_filters import filter_scan
from core import Geometry, Odometry, single_encoder_pose


class FilterTests(unittest.TestCase):
    def test_body_returns_excluded_but_obstacle_behind_remains(self):
        points=[Point(180,150,100,False),Point(180,1000,100,False),Point(0,300,100,False)]
        result,n=filter_scan(points,(.115,0,0,.05,12))
        self.assertEqual(n,1)
        np.testing.assert_allclose(result,[[-.885,0],[.415,0]],atol=1e-10)

    def test_sector_wrap_and_mount_rotation(self):
        points=[Point(179,1000,100,False),Point(181,1000,100,False),Point(0,1000,100,False)]
        result,n=filter_scan(points,(.115,0,0,.05,12),sector_center=180,sector_width=20)
        self.assertEqual(n,2)
        np.testing.assert_allclose(result,[[1.115,0]])
        # The sensor's zero rotates with its mounting; sector uses robot directions.
        result,n=filter_scan([Point(0,1000,100,False)],(.115,0,90,.05,12),sector_center=90,sector_width=20)
        self.assertEqual(n,1);self.assertEqual(result.shape,(0,2))

    def test_full_mask_empty_invalid_and_margin(self):
        self.assertEqual(filter_scan([], (.115,0,0,.05,12))[0].shape,(0,2))
        p=[Point(0,0,0,False),Point(0,300,100,False)]
        self.assertEqual(len(filter_scan(p,(.115,0,0,.05,12),sector_width=360)[0]),0)
        p=[Point(0,100,100,False)]
        self.assertEqual(len(filter_scan(p,(0,0,0,.05,12),body_margin=0)[0]),0)


class SingleEncoderPoseTests(unittest.TestCase):
    def test_commanded_spin_ignores_false_encoder_and_coasting(self):
        g=Geometry(diameter=100/math.pi,tpr_l=100,tpr_r=100,imu_sign=1,motor_l=-1,swap_lr=1)
        o=Odometry()
        base=dict(boot_id=1,ms=100,tl=0,tr=0,imu=1,heading=0,imu_read_ok=True,imu_age_ms=0)
        o.update(base,g)
        pl,pr=g.motor_output(-80,80)
        o.update(dict(base,ms=200,tl=900,tr=1000,heading=10,pl=pl,pr=pr),g)
        np.testing.assert_allclose(o.single_left,(0,0,math.radians(10)))
        np.testing.assert_allclose(o.single_right,(0,0,math.radians(10)))
        o.update(dict(base,ms=300,tl=1200,tr=1400,heading=12,pl=0,pr=0),g)
        np.testing.assert_allclose(o.single_left,(0,0,math.radians(12)))
        pl,pr=g.motor_output(80,80)
        o.update(dict(base,ms=400,tl=1300,tr=1400,heading=12,pl=pl,pr=pr),g)
        self.assertAlmostEqual(math.hypot(*o.single_left[:2]),100)

    def test_straight_reverse_and_pure_spin_both_sides(self):
        for side,sign in [('left',-1),('right',1)]:
            np.testing.assert_allclose(single_encoder_pose((0,0,0),100,0,200,side),(100,0,0))
            np.testing.assert_allclose(single_encoder_pose((0,0,0),-100,0,200,side),(-100,0,0))
            a=math.pi/2
            np.testing.assert_allclose(single_encoder_pose((0,0,0),sign*100*a,a,200,side),(0,0,a),atol=1e-10)

    def test_quarter_circle_from_either_wheel(self):
        a=math.pi/2
        for side,distance in [('left',900*a),('right',1100*a)]:
            np.testing.assert_allclose(single_encoder_pose((0,0,0),distance,a,200,side),(1000,1000,a),atol=1e-10)

    def test_odometry_single_left_independent_of_right_encoder(self):
        g=Geometry(diameter=100/math.pi,tpr_l=100,tpr_r=100,imu_sign=1)
        o=Odometry()
        base=dict(boot_id=1,ms=100,tl=0,tr=0,imu=1,heading=0,imu_read_ok=True,imu_age_ms=0)
        o.update(base,g)
        o.update(dict(base,ms=200,tl=100,tr=0),g)
        np.testing.assert_allclose(o.single_left,(100,0,0))
        np.testing.assert_allclose(o.single_right,(0,0,0))
        # Loss of IMU does not invent a single-encoder orientation or distance.
        o.update(dict(base,ms=300,tl=200,imu_read_ok=False),g)
        np.testing.assert_allclose(o.single_left,(100,0,0))
        self.assertFalse(o.imu_valid)
        self.assertGreater(o.imu_segment,0)
