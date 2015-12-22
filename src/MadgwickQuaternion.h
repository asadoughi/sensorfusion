#ifndef _MadgwickQuaternion_h_
#define _MadgwickQuaternion_h_

#ifdef __cplusplus
extern "C" {
#endif

extern volatile float q[4];

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);

#ifdef __cplusplus
}
#endif

#endif
