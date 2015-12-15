import math
import unittest

import sensorfusion

class TestOrientation(unittest.TestCase):
    def setUp(self):
        mag_min_max = [(-7023, 3028), (-2962, 9657), (-6128, 5920)]
        tesla_value = 30.0
        self.MAG_BIAS = [(mi + ma)/2.0 for mi, ma in mag_min_max]
        self.MAG_SCALE = [(ma - mi)/2.0 / tesla_value for mi, ma in mag_min_max]

    def assertIsNan(self, value):
        self.assertTrue(math.isnan(value))

    # yaw
    def _compute_attitude_z(self, mag):
        acc = [0, 0, sensorfusion.GRAVITY_EARTH]
        return self._compute_attitude(mag, acc)

    # roll
    def _compute_attitude_y(self, mag):
        acc = [0, sensorfusion.GRAVITY_EARTH, 0]
        return self._compute_attitude(mag, acc)

    # pitch
    def _compute_attitude_x(self, mag):
        acc = [sensorfusion.GRAVITY_EARTH, 0, 0]
        return self._compute_attitude(mag, acc)

    def _compute_attitude(self, mag, acc):
        self.fusion = sensorfusion.Fusion()
        gyro = [1e-6, 1e-6, 1e-6]
        dT = 0.010

        for _ in range(sensorfusion.MIN_GYRO_SAMPLES * 10):
            self.fusion.handleAcc(acc)
            self.fusion.handleMag(mag)
            self.fusion.handleGyro(gyro, dT)

        self.assertTrue(self.fusion.hasEstimate())
        bias = self.fusion.getBias()
        self.assertAlmostEqual(bias[0], 0, 3)
        self.assertAlmostEqual(bias[1], 0, 3)
        self.assertAlmostEqual(bias[2], 0, 3)
        return self.fusion.getAttitude()

    def _convert_mag_units(mag):
        GAUSS_TO_MICROTESLA = 100
        return [(float(v) / 2**15) * 2 * GAUSS_TO_MICROTESLA for v in mag]

    def test_yaw_angles(self):
        mags = [
            [-4473, 6554, 4485],  # DOOR WALL
            [-3629, 1532, 4938],  # MIRROR
            [-793, 6677, 4569],   # RIGHT WALL
            [-730, 2380, 5387],   # WINDOW
        ]
        expected_angle_axis = [
            (44.1, -1.0),
            (131.6, -1.0),
            (24.4, 1.0),
            (121.3, 1.0)
            ]

        for j, mag in enumerate(mags):
            mag = [(mag[i] - self.MAG_BIAS[i])/self.MAG_SCALE[i] for i in range(3)]
            attitude = self._compute_attitude_z(mag)
            axis_angle = sensorfusion.quaternion_to_axis_angle(*attitude)
            self.assertAlmostEqual(axis_angle[0], expected_angle_axis[j][0], 1)
            self.assertAlmostEqual(axis_angle[1], 0, 3)
            self.assertAlmostEqual(axis_angle[2], 0, 3)
            self.assertAlmostEqual(axis_angle[3], expected_angle_axis[j][1], 3)

    def _test_pitch_angles(self):
        mags = [
            [-6978, 5034, -2936],  # DOOR WALL
            [-7456, 1348, -1137],  # MIRROR
            [-7231, 6709, 732],    # RIGHT WALL
            [-7525, 3502, 2523]    # WINDOW
        ]
        expected_angle_axis = [
            (104.3, -1.0),
            (131.6, -1.0),
            (24.4, 1.0),
            (121.3, 1.0)
            ]

        for j, mag in enumerate(mags):
            mag = [(mag[i] - self.MAG_BIAS[i])/self.MAG_SCALE[i] for i in range(3)]
            attitude = self._compute_attitude_x(mag)
            axis_angle = sensorfusion.quaternion_to_axis_angle(*attitude)
            print j, axis_angle
            self.assertAlmostEqual(axis_angle[0], expected_angle_axis[j][0], 1)
            self.assertAlmostEqual(axis_angle[1], expected_angle_axis[j][1], 3)
            self.assertAlmostEqual(axis_angle[2], 0)
            self.assertAlmostEqual(axis_angle[3], 0)

    def _test_roll_angles(self):
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
            attitude = self._compute_attitude_y(mag)
            axis_angle = sensorfusion.quaternion_to_axis_angle(*attitude)
            self.assertAlmostEqual(axis_angle[0], expected_angle_axis[j][0], 1)
            self.assertAlmostEqual(axis_angle[1], 0)
            self.assertAlmostEqual(axis_angle[2], expected_angle_axis[j][1], 3)
            self.assertAlmostEqual(axis_angle[3], 0)

    def test_yaw_0_360(self):
        # 0/360: x 0, y +
        mag = [0.0, 40.0, 40.0]
        attitude = self._compute_attitude_z(mag)
        self.assertAlmostEqual(attitude[0], 0.0, 3)
        self.assertAlmostEqual(attitude[1], 0.0, 3)
        self.assertAlmostEqual(attitude[2], 0.0, 3)
        self.assertAlmostEqual(attitude[3], 1.0, 3)
        axis_angle = sensorfusion.quaternion_to_axis_angle(*attitude)
        self.assertEqual(axis_angle, (0, 0, 0, 0))

    def test_yaw_90(self):
        # 90: x -, y 0
        mag = [-40.0, 0.0, 40.0]
        attitude = self._compute_attitude_z(mag)
        self.assertAlmostEqual(attitude[0], 0.0, 3)
        self.assertAlmostEqual(attitude[1], 0.0, 3)
        self.assertAlmostEqual(attitude[2], -0.707, 3)
        self.assertAlmostEqual(attitude[3], 0.707, 3)
        axis_angle = sensorfusion.quaternion_to_axis_angle(*attitude)
        self.assertAlmostEqual(axis_angle[0], 90, 3)
        self.assertAlmostEqual(axis_angle[1], 0, 3)
        self.assertAlmostEqual(axis_angle[2], 0, 3)
        self.assertAlmostEqual(axis_angle[3], -1, 3)

    def test_yaw_180(self):
        # 180: x 0, y -
        mag = [0.0, -40.0, -40.0]
        attitude = self._compute_attitude_z(mag)
        self.assertAlmostEqual(attitude[0], 0.0, 3)
        self.assertAlmostEqual(attitude[1], 0.0, 3)
        self.assertAlmostEqual(attitude[2], 1, 3)
        self.assertAlmostEqual(attitude[3], 0, 3)
        axis_angle = sensorfusion.quaternion_to_axis_angle(*attitude)
        self.assertAlmostEqual(axis_angle[0], 180.0, 1)
        self.assertAlmostEqual(axis_angle[1], 0, 3)
        self.assertAlmostEqual(axis_angle[2], 0, 3)
        self.assertAlmostEqual(axis_angle[3], 1.0, 3)

    def test_yaw_270(self):
        # 270: x +, y 0
        mag = [40.0, 0.0, 40.0]
        attitude = self._compute_attitude_z(mag)
        self.assertAlmostEqual(attitude[0], 0.0, 3)
        self.assertAlmostEqual(attitude[1], 0.0, 3)
        self.assertAlmostEqual(attitude[2], 0.707, 3)
        self.assertAlmostEqual(attitude[3], 0.707, 3)
        axis_angle = sensorfusion.quaternion_to_axis_angle(*attitude)
        self.assertAlmostEqual(axis_angle[0], 90, 3)
        self.assertAlmostEqual(axis_angle[1], 0, 3)
        self.assertAlmostEqual(axis_angle[2], 0, 3)
        self.assertAlmostEqual(axis_angle[3], 1, 3)

    def _test_mag_ranges(self):
        from collections import defaultdict
        angles = defaultdict(list)
        for x in range(-100, 100, 10):
            for y in range(-100, 100, 10):
                for z in range(-100, 100, 10):
                    self.fusion = sensorfusion.Fusion()

                    acc = [0, sensorfusion.GRAVITY_EARTH, 0]
                    mag = [float(x), float(y), float(z)]
                    gyro = [1e-6, 1e-6, 1e-6]
                    dT = 0.010

                    for _ in range(sensorfusion.MIN_GYRO_SAMPLES):
                        self.fusion.handleAcc(acc)
                        self.fusion.handleMag(mag)
                        self.fusion.handleGyro(gyro, dT)

                    if self.fusion.hasEstimate():
                        matrix = self.fusion.getRotationMatrix()
                        attitude = self.fusion.getAttitude()
                        axis_angle = sensorfusion.quaternion_to_axis_angle(*attitude)
                        angle = math.acos(matrix[0]) * 180. / math.pi
                        # angle = (360 - angle) if matrix[6] < 0 else angle
                        # angle = axis_angle[0]
                        is_nan = angle != angle
                        angles[angle if not is_nan else 'nan'].append(mag)
        print 'nan', angles.pop('nan')
        print min(angles.keys()), max(angles.keys())

        for angle in sorted(angles):
            xs = [x for x, y, z in angles[angle]]
            ys = [y for x, y, z in angles[angle]]
            zs = [z for x, y, z in angles[angle]]
            print angle, (min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs)), angles[angle]
        raise Exception('END')
