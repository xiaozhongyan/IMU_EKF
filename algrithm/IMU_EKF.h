#ifndef _IMU_EKF_H
#define _IMU_EKF_H


#include "FastMath.h"
#include "Matrix.h"
#include <stdbool.h>
extern float X[13];

bool Init_ekf(float MagInclination);
bool EKFupdata(/*float *q,*/float *acc,float *gyro,float *mag,float *v,float dt);
void QUAT_GetAngle(float* rpy);

#endif

