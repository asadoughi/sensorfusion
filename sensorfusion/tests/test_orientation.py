import math
import unittest

import sensorfusion

class TestOrientation(unittest.TestCase):
    def setUp(self):
        self.fusion = sensorfusion.Fusion()

    def assertIsNan(self, value):
        self.assertTrue(math.isnan(value))

    def _compute_attitude(self, mag):
        acc = [0, 0, sensorfusion.GRAVITY_EARTH]
        gyro = [0, 0, 0]
        dT = 0.010

        for _ in range(sensorfusion.MIN_GYRO_SAMPLES):
            self.fusion.handleAcc(acc)
            self.fusion.handleMag(mag)
            self.fusion.handleGyro(gyro, dT)

        self.assertTrue(self.fusion.hasEstimate())
        self.assertEqual(self.fusion.getBias(), [0, 0, 0])
        return self.fusion.getAttitude()

    def test_yaw_nan(self):
        # nan: x 0, y 0
        mag = [0.0, 0.0, 10.0]
        attitude = self._compute_attitude(mag)
        self.assertIsNan(attitude[0])
        self.assertIsNan(attitude[1])
        self.assertIsNan(attitude[2])
        self.assertIsNan(attitude[3])

    def test_yaw_0_360(self):
        # 0/360: x 0, y +
        mag = [0.0, 40.0, 40.0]
        attitude = self._compute_attitude(mag)
        self.assertAlmostEqual(attitude[0], 0.0, 3)
        self.assertAlmostEqual(attitude[1], 0.0, 3)
        self.assertAlmostEqual(attitude[2], 0.0, 3)
        self.assertAlmostEqual(attitude[3], 1.0, 3)
        axis_angle = sensorfusion.quaternion_to_axis_angle(*attitude)
        self.assertEqual(axis_angle, (0, 0, 0, 0))

    def test_yaw_90(self):
        # 90: x -, y 0
        mag = [-40.0, 0.0, 40.0]
        attitude = self._compute_attitude(mag)
        self.assertAlmostEqual(attitude[0], 0.0, 3)
        self.assertAlmostEqual(attitude[1], 0.0, 3)
        self.assertAlmostEqual(attitude[2], -0.707, 3)
        self.assertAlmostEqual(attitude[3], 0.707, 3)
        axis_angle = sensorfusion.quaternion_to_axis_angle(*attitude)
        self.assertAlmostEqual(axis_angle[0], 90, 3)
        self.assertAlmostEqual(axis_angle[1], 0)
        self.assertAlmostEqual(axis_angle[2], 0)
        self.assertAlmostEqual(axis_angle[3], -1)

    def test_yaw_180(self):
        # 180: x 0, y -
        mag = [0.0, -40.0, -40.0]
        attitude = self._compute_attitude(mag)
        self.assertAlmostEqual(attitude[0], 0.0, 3)
        self.assertAlmostEqual(attitude[1], 0.0, 3)
        self.assertAlmostEqual(attitude[2], 1, 3)
        self.assertAlmostEqual(attitude[3], 0, 3)
        axis_angle = sensorfusion.quaternion_to_axis_angle(*attitude)
        self.assertAlmostEqual(axis_angle[0], 180.0, 1)
        self.assertAlmostEqual(axis_angle[1], 0)
        self.assertAlmostEqual(axis_angle[2], 0)
        self.assertAlmostEqual(axis_angle[3], 1.0)

    def test_yaw_270(self):
        # 270: x +, y 0
        mag = [40.0, 0.0, 40.0]
        attitude = self._compute_attitude(mag)
        self.assertAlmostEqual(attitude[0], 0.0, 3)
        self.assertAlmostEqual(attitude[1], 0.0, 3)
        self.assertAlmostEqual(attitude[2], 0.707, 3)
        self.assertAlmostEqual(attitude[3], 0.707, 3)
        axis_angle = sensorfusion.quaternion_to_axis_angle(*attitude)
        self.assertAlmostEqual(axis_angle[0], 90, 3)
        self.assertAlmostEqual(axis_angle[1], 0)
        self.assertAlmostEqual(axis_angle[2], 0)
        self.assertAlmostEqual(axis_angle[3], 1)

    def _test_mag_ranges(self):
        from collections import defaultdict
        angles = defaultdict(list)
        for x in range(-100, 100, 10):
            for y in range(-100, 100, 10):
                for z in range(-100, 100, 10):
                    self.fusion = sensorfusion.Fusion()

                    acc = [0, 0, sensorfusion.GRAVITY_EARTH]
                    mag = [float(x), float(y), float(z)]
                    gyro = [0, 0, 0]
                    dT = 0.010

                    for _ in range(sensorfusion.MIN_GYRO_SAMPLES):
                        self.fusion.handleAcc(acc)
                        self.fusion.handleMag(mag)
                        self.fusion.handleGyro(gyro, dT)

                    if self.fusion.hasEstimate():
                        matrix = self.fusion.getRotationMatrix()
                        angle = math.acos(matrix[0]) * 180. / math.pi
                        angle = (360 - angle) if matrix[1] < 0 else angle
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
