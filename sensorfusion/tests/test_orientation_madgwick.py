import itertools
import math
import unittest

import sensorfusion

class TestOrientationMadgwick(unittest.TestCase):
    def setUp(self):
        mag_min_max = [(-7023, 3028), (-2962, 9657), (-6128, 5920)]
        self.MAG_BIAS = [(mi + ma)/2.0 for mi, ma in mag_min_max]
        self.MAG_SCALE = [(ma - mi)/2.0 for mi, ma in mag_min_max]

    def correct_mag(self, mag):
        return [(mag[i] - self.MAG_BIAS[i])/self.MAG_SCALE[i] for i in range(3)]

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
        expected_angles = [
            (0, 0, -134.0),
            (0, 0, 138.0),
            (0, 0, -66.0),
            (0, 0, 31.0)
            ]

        for j, mag in enumerate(mags):
            mag = self.correct_mag(mag)
            q = self._compute_attitude_z(mag)
            roll, pitch, yaw = sensorfusion.roll_pitch_yaw(q)
            self.assertAlmostEqual(roll,  expected_angles[j][0], 0)
            self.assertAlmostEqual(pitch, expected_angles[j][1], 0)
            self.assertAlmostEqual(yaw,   expected_angles[j][2], 0)

    def test_yaw_angles_again(self):
        accs = [
            [-509, -175, 16452],
            [55, 766, 16242],
            [-24, -1059, 16365],
            [-670, -189, 16359],
            ]
        mags = [
            [-5892, 2311, 4848],
            [-553, 1624, 5065],
            [-791, 7577, 5341],
            [-5849, 6998, 4939],
        ]
        expected_angles = [
            (0, 2, 167.0),  # (13+39) = 52
            (3, 0, 48.0),   # 119
            (-4, 0, -72.0),  # 120
            (-1, 2, -141.0)  # 69
            ]

        for j, mag in enumerate(mags):
            mag = self.correct_mag(mag)
            acc = [a_i/32768.0 * sensorfusion.GRAVITY_EARTH for a_i in accs[j]]
            q = self._compute_attitude(mag, acc)
            roll, pitch, yaw = sensorfusion.roll_pitch_yaw(q)
            self.assertAlmostEqual(roll,  expected_angles[j][0], 0)
            self.assertAlmostEqual(pitch, expected_angles[j][1], 0)
            self.assertAlmostEqual(yaw,   expected_angles[j][2], 0)

    def _test_pitch_angles(self):
        mags = [
            [-6978, 5034, -2936],  # DOOR WALL (0.6881979703903198, -0.162129208445549, -0.689984142780304, -0.1549822986125946) (93.02473730638378, -0.2234653060239575, -0.9510162856920528, -0.21361460479461566)
             [-7456, 1348, -1137],  # MIRROR (0.6673268675804138, 0.22581857442855835, -0.6823622584342957, 0.19509662687778473) (96.27783020031163, 0.3032078377778739, -0.9162115449743273, 0.2619573103898254)
             [-7231, 6709, 732],    # RIGHT WALL (0.5264194011688232, -0.46728214621543884, -0.5616878867149353, -0.4347834289073944) (116.47230823977264, -0.5495981954289173, -0.6606343756829183, -0.5113745300676028)
             [-7525, 3502, 2523]    # WINDOW (0.63744056224823, -0.03323207050561905, -0.7693279385566711, -0.026450788602232933) (100.7975352980979, -0.043130552457690007, -0.998479435865259, -0.03432940253189996)
        ]
        expected_angles = [
            (0, -134.0, 0),
            (0, 138.0, 0),
            (0, -66.0, 0),
            (0, 31.0, 0)
            ]

        for j, mag in enumerate(mags):
            mag = self.correct_mag(mag)
            q = self._compute_attitude_x(mag)
            roll, pitch, yaw = sensorfusion.roll_pitch_yaw(q)
            print roll, pitch, yaw
            # self.assertAlmostEqual(roll,  expected_angles[j][0], 0)
#             self.assertAlmostEqual(pitch, expected_angles[j][1], 0)
#             self.assertAlmostEqual(yaw,   expected_angles[j][2], 0)
        self.fail('end-pitch')

    def _test_roll_angles(self):
        mags = [
            [-4385, -1112, -1782],
            [-452, -1381, -793],
            [-5137, -870, 2033],
            [-1663, -974, 3062],
        ]
        expected_angles = [
            (104.3, 0.0, 0.0),
            (131.6, 0.0, 0.0),
            (24.4, 0.0, 0.0),
            (121.3, 0.0, 0.0)
            ]

        for j, mag in enumerate(mags):
            mag = self.correct_mag(mag)
            q = self._compute_attitude_y(mag)
            roll, pitch, yaw = sensorfusion.roll_pitch_yaw(q)
            print roll, pitch, yaw
        self.fail('end-roll')
        
    def test_roll_angles_again(self):
        accs = [
            [514, 16009, -706],
            [726, 15930, -284],
            [221, 16006, -457],
            [1038, 16102, 69]
        ]
        mags = [
            [-5913, -125, -1103],
            [-720, -599, -1646],
            [-1961, -366, 2760],
            [-5058, -177, 2515],
        ]
        expected_angles = [
            (94, -2, -160), # 117
            (91, -3, -43),  # 125
            (92, -2, 82),   # 51
            (88, -5, 133)   # 67
            ]

        for j, mag in enumerate(mags):
            mag = self.correct_mag(mag)
            acc = accs[j]
            q = self._compute_attitude(mag, acc)
            roll, pitch, yaw = sensorfusion.roll_pitch_yaw(q)
            self.assertAlmostEqual(roll,  expected_angles[j][0], 0)
            self.assertAlmostEqual(pitch, expected_angles[j][1], 0)
            self.assertAlmostEqual(yaw,   expected_angles[j][2], 0)

    def test_roll_angles_thrice(self):
        accs = [
            [-17913, 103, -394],
            [-1007, -565, -16685],
            [16047, 844, -542],
            [-579, 805, 16121]
        ]
        mags = [
            [1477, 7194, 2185],
            [-294, 7193, -4427],
            [-7156, 6615, -2931],
            [-5256, 7179, 4075],
        ]
        expected_angles = [
            (166, 89, 107),
            (-178, 4, 58),
            (130, -86, -177),
            (3, 2, -137),
            ]

        for j, mag in enumerate(mags):
            mag = self.correct_mag(mag)
            acc = accs[j]
            q = self._compute_attitude(mag, acc)
            roll, pitch, yaw = sensorfusion.roll_pitch_yaw(q)
            self.assertAlmostEqual(roll,  expected_angles[j][0], 0)
            self.assertAlmostEqual(pitch, expected_angles[j][1], 0)
            self.assertAlmostEqual(yaw,   expected_angles[j][2], 0)

