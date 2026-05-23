#ifndef NAVIGATION_H
#define NAVIGATION_H
#include <Arduino.h>

struct Quaternion {
  float w, x, y, z;
};

struct AttitudeState{
    float altitude;        // altitude filtrée (m)
    float altitude_raw;    // altitude instantanée
    float p0;              // pression de référence (Pa)
    float alt_bias;        // filtre / état interne
    uint16_t cal_counter;  // compteur calibration
    uint16_t cal_samples;  // nb échantillons attendus
    uint8_t  cal_active;   // état calibration
};

struct AttitudeParameters{
    float alpha_alt;       // filtre altitude (0..1)
    float alpha_p0;        // filtre pression calibration
    float g0;              // gravité (option extension)
    uint16_t tps_cal;      // temps de la calibration en ms
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

void attitude_init(AttitudeState *st, AttitudeParameters *params);
void estimateAltitude(AttitudeState *st, AttitudeParameters *params, float pressure_pa, float temperature_c, uint8_t cal_en);


#endif
