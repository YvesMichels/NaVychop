#ifndef NAVIGATION_H
#define NAVIGATION_H

struct Quaternion {
  float w, x, y, z;
};
void computeEulerAngles(float q0, float q1, float q2, float q3, float* roll, float* pitch, float* yaw);
void computeRotationMatrix(float w, float x, float y, float z, float R[3][3]);
void rotMult(float R_A_T[3][3], float R_A_M[3][3], float R_M_T[3][3]);
Quaternion qNormalize(const Quaternion &q);
Quaternion qConj(const Quaternion &q);
Quaternion qMul(const Quaternion &a, const Quaternion &b);
Quaternion qAdd(const Quaternion &a, const Quaternion &b);
Quaternion qScale(const Quaternion &q, float s);
Quaternion qProjectOnAxis(const Quaternion &q, float ax, float ay, float az);
void qToEulerXYZ(const Quaternion &q, float &rx, float &ry, float &rz);
float computePitchCommandSignal(float R[3][3], float pitch_cmd);
float computeRollCommandSignal(float R[3][3], float roll_cmd);
float computeYawCommandSignalImproved(float R[3][3], float yaw_cmd);


#endif
