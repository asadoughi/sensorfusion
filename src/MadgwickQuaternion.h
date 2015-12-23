#ifndef _MadgwickQuaternion_h_
#define _MadgwickQuaternion_h_


class MadgwickQuaternion {
 public:
  float q[4];
  MadgwickQuaternion() {
    q[0] = 1.0f;
    q[1] = q[2] = q[3] = 0.0f;
  }
  void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
  float q0() const { return q[0]; }
  float q1() const { return q[1]; }
  float q2() const { return q[2]; }
  float q3() const { return q[3]; }
};

#endif
