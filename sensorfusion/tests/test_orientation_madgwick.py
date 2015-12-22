import itertools
import math
import unittest

import sensorfusion

class TestOrientationMadgwick(unittest.TestCase):
    def setUp(self):
        mag_min_max = [(-7023, 3028), (-2962, 9657), (-6128, 5920)]
        self.MAG_BIAS = [(mi + ma)/2.0 for mi, ma in mag_min_max]
        self.MAG_SCALE = [(ma - mi)/2.0 for mi, ma in mag_min_max]

    # pitch
    def _compute_attitude_x(self, mag):
        acc = [sensorfusion.GRAVITY_EARTH, 0, 0]
        return self._compute_attitude(mag, acc)

    # roll
    def _compute_attitude_y(self, mag):
        acc = [0, sensorfusion.GRAVITY_EARTH, 0]
        return self._compute_attitude(mag, acc)

    # yaw
    def _compute_attitude_z(self, mag):
        acc = [0, 0, sensorfusion.GRAVITY_EARTH]
        return self._compute_attitude(mag, acc)

    def _compute_attitude(self, mag, acc):
        self.fusion = sensorfusion.MadgwickQuaternion()
        gyro = [0.0, 0.0, 0.0]
        dT = 0.010

        for _ in range(sensorfusion.MIN_GYRO_SAMPLES * 10):
            self.fusion.update(*itertools.chain(acc, gyro, mag, (dT,)))

        return (self.fusion.q0, self.fusion.q1, self.fusion.q2, self.fusion.q3)

    def test_yaw_angles(self):
        mags = [
           [-4473, 6554, 4485],  # BACK
           [-3629, 1532, 4938],  # LEFT
           [-793, 6677, 4569],   # RIGHT
           [-730, 2380, 5387],   # FRONT
        ]
        expected_angle_axis = [
            (133.0, -1.0),
            (137.0,  1.0),
            (65.0,  -1.0),
            (31.0,   1.0)
            ]

        for j, mag in enumerate(mags):
            mag = [(mag[i] - self.MAG_BIAS[i])/self.MAG_SCALE[i] for i in range(3)]
            q = self._compute_attitude_z(mag)
            angle_axis = sensorfusion.quaternion_to_axis_angle(q[1], q[2], q[3], q[0])
            print q, angle_axis, sensorfusion.roll_pitch_yaw(q)
        self.fail('end-yaw')

    def test_pitch_angles(self):
        mags = [
#            [-6978, 5034, -2936],  # DOOR WALL (0.6881979703903198, -0.162129208445549, -0.689984142780304, -0.1549822986125946) (93.02473730638378, -0.2234653060239575, -0.9510162856920528, -0.21361460479461566)
#             [-7456, 1348, -1137],  # MIRROR (0.6673268675804138, 0.22581857442855835, -0.6823622584342957, 0.19509662687778473) (96.27783020031163, 0.3032078377778739, -0.9162115449743273, 0.2619573103898254)
#             [-7231, 6709, 732],    # RIGHT WALL (0.5264194011688232, -0.46728214621543884, -0.5616878867149353, -0.4347834289073944) (116.47230823977264, -0.5495981954289173, -0.6606343756829183, -0.5113745300676028)
             [-7525, 3502, 2523]    # WINDOW (0.63744056224823, -0.03323207050561905, -0.7693279385566711, -0.026450788602232933) (100.7975352980979, -0.043130552457690007, -0.998479435865259, -0.03432940253189996)
        ]
        expected_angle_axis = [
            (104.3, -1.0),
            (131.6, -1.0),
            (24.4, 1.0),
            (121.3, 1.0)
            ]

        for j, mag in enumerate(mags):
            mag = [(mag[i] - self.MAG_BIAS[i])/self.MAG_SCALE[i] for i in range(3)]
            q = self._compute_attitude_x(mag)
            angle_axis = sensorfusion.quaternion_to_axis_angle(q[1], q[2], q[3], q[0])
            print q, angle_axis, sensorfusion.roll_pitch_yaw(q)
        self.fail('end-pitch')

    def test_roll_angles(self):
        mags = [
            [-4385, -1112, -1782],
            [-452, -1381, -793],
            [-5137, -870, 2033],
            [-1663, -974, 3062],
        ]
        expected_angle_axis = [
            (104.3, -1.0),
            (131.6, -1.0),
            (24.4, 1.0),
            (121.3, 1.0)
            ]

        for j, mag in enumerate(mags):
            mag = [(mag[i] - self.MAG_BIAS[i])/self.MAG_SCALE[i] for i in range(3)]
            q = self._compute_attitude_y(mag)
            angle_axis = sensorfusion.quaternion_to_axis_angle(q[1], q[2], q[3], q[0])
            print q, angle_axis, sensorfusion.roll_pitch_yaw(q)
        self.fail('end-roll')
        
