#include <Arduino.h>
#include "navigation.h"

void printEulerFromQuat(float q0, float q1, float q2, float q3) {
  // q0 = real, q1 = i, q2 = j, q3 = k
  float roll  = atan2(2.0 * (q0*q1 + q2*q3), 1.0 - 2.0 * (q1*q1 + q2*q2));
  float pitch = asin(2.0 * (q0*q2 - q3*q1));
  float yaw   = atan2(2.0 * (q0*q3 + q1*q2), 1.0 - 2.0 * (q2*q2 + q3*q3));

  // Convertir en degrés
  roll  *= 180.0 / PI;
  pitch *= 180.0 / PI;
  yaw   *= 180.0 / PI;
}

void computeEulerAngles(float q0, float q1, float q2, float q3, float* roll, float* pitch, float* yaw) {
  // q0 = real, q1 = i, q2 = j, q3 = k
  *roll  = atan2(2.0 * (q0*q1 + q2*q3), 1.0 - 2.0 * (q1*q1 + q2*q2));
  *pitch = asin(2.0 * (q0*q2 - q3*q1));
  *yaw   = atan2(2.0 * (q0*q3 + q1*q2), 1.0 - 2.0 * (q2*q2 + q3*q3));
}

void computeRotationMatrix(float w, float x, float y, float z, float R[3][3]) {
  R[0][0] = 1.0f - 2.0f * (y * y + z * z);
  R[0][1] = 2.0f * (x * y - z * w);
  R[0][2] = 2.0f * (x * z + y * w);

  R[1][0] = 2.0f * (x * y + z * w);
  R[1][1] = 1.0f - 2.0f * (x * x + z * z);
  R[1][2] = 2.0f * (y * z - x * w);

  R[2][0] = 2.0f * (x * z - y * w);
  R[2][1] = 2.0f * (y * z + x * w);
  R[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

Quaternion qNormalize(const Quaternion &q) {
  float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (n <= 0.0f) return {1,0,0,0};
  return {q.w/n, q.x/n, q.y/n, q.z/n};
}

Quaternion qConj(const Quaternion &q) {
  return {q.w, -q.x, -q.y, -q.z};
}

Quaternion qMul(const Quaternion &a, const Quaternion &b) {
  return {
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
  };
}

Quaternion qAdd(const Quaternion &a, const Quaternion &b) {
  return {a.w+b.w, a.x+b.x, a.y+b.y, a.z+b.z};
}

Quaternion qScale(const Quaternion &q, float s) {
  return {q.w*s, q.x*s, q.y*s, q.z*s};
}

// Projection de q sur rotation autour d'un axe unitaire a=(ax,ay,az)
Quaternion qProjectOnAxis(const Quaternion &q, float ax, float ay, float az) {
  float dot = q.x*ax + q.y*ay + q.z*az; // projection v·a
  Quaternion qp = {q.w, dot*ax, dot*ay, dot*az};
  if (qp.w < 0) { // harmonisation du signe
    qp.w = -qp.w; qp.x = -qp.x; qp.y = -qp.y; qp.z = -qp.z;
  }
  return qNormalize(qp);
}

// Conversion quaternion -> angles d'Euler (XYZ) pour affichage
void qToEulerXYZ(const Quaternion &q, float &rx, float &ry, float &rz) {
  float ww = q.w*q.w, xx = q.x*q.x, yy = q.y*q.y, zz = q.z*q.z;
  float r00 = ww + xx - yy - zz;
  float r10 = 2.0f*(q.x*q.y + q.w*q.z);
  float r20 = 2.0f*(q.x*q.z - q.w*q.y);
  float r21 = 2.0f*(q.y*q.z + q.w*q.x);
  float r22 = ww - xx - yy + zz;

  ry = asinf(-r20);
  if (fabsf(r20) < 0.999999f) {
    rx = atan2f(r21, r22);
    rz = atan2f(r10, r00);
  } else {
    float r12 = 2.0f*(q.y*q.z - q.w*q.x);
    float r11 = ww - xx + yy - zz;
    rx = atan2f(-r12, r11);
    rz = 0.0f;
  }
}

// --- CRC8 standard (polynôme 0x07) ---
uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
  }
  return crc;
}