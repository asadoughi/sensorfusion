import math

from sensorfusion.glue import *  # noqa

GRAVITY_EARTH = 9.8066
MIN_GYRO_SAMPLES = 64


def quaternion_to_axis_angle(qx, qy, qz, qw):
    angle = 2 * math.acos(qw)
    qw2 = math.sqrt(1 - qw * qw)
    if qw <= 1.001 and qw >= 0.999:
        return (0.0, 0.0, 0.0, 0.0)

    x = qx / qw2
    y = qy / qw2
    z = qz / qw2
    return (angle * 180./math.pi, x, y, z)


def roll_pitch_yaw(q):
    yaw   = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
    pitch = -math.asin(2.0 * (q[1] * q[3] - q[0] * q[2]))
    roll  = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
    yaw = yaw * 180.0 / math.pi
    pitch = pitch * 180.0 / math.pi
    roll = roll * 180.0 / math.pi
    return roll, pitch, yaw
