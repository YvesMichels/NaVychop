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

void rotMult(float R_A_T[3][3], float R_A_M[3][3], float R_M_T[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_A_T[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                R_A_T[i][j] += R_A_M[i][k] * R_M_T[k][j];
            }
        }
    }
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

float computePitchCommandSignal(float R[3][3], float pitch_cmd) {
    // projection de y_avion dans le plan horizontal
    float y_sol[2] = {R[0][1], R[1][1]};

    float norm = sqrtf(y_sol[0]*y_sol[0] + y_sol[1]*y_sol[1]);
    if (norm< 1e-6f) return 0.0f;

    // direction latérale horizontale
    y_sol[0] /= norm;
    y_sol[1] /= norm;

    // composante verticale
    float z = R[2][1];

    //  cos(roll_mesuré) ≈ norme projection horizontale
    float y = norm;

    // cible
    float y_target = cosf(pitch_cmd);
    float z_target = sinf(pitch_cmd);

    float cross = y * z_target - z * y_target;
    float dot   = y * y_target + z * z_target;

    if (dot < 0.0f) {
        return (cross > 0.0f) ? 1.0f : -1.0f;
    }

    return cross;
}

float computeRollCommandSignal(float R[3][3], float roll_cmd) {
  // Construction X_sol
    float x_sol[2] = {R[0][0], R[1][0]};
    float norm = sqrtf(x_sol[0]*x_sol[0] + x_sol[1]*x_sol[1]);

    if (norm < 1e-6f) return 0.0f;

    // direction avant horizontale
    x_sol[0] /= norm;
    x_sol[1] /= norm;

    // Projection axe avion sur Z
    float z = R[2][0];

    // cos(pitch_mesuré) ≈ norme projection horizontale
    float x = norm;

    // cible
    float x_target = cosf(roll_cmd);
    float z_target = sinf(roll_cmd);

    float cross = x * z_target - z * x_target;
    float dot   = x * x_target + z * z_target;

    if (dot < 0.0f) {
        return (cross > 0.0f) ? 1.0f : -1.0f;
    }

    return cross;
}


void computeTargetVector(float yaw_cmd, float x_target[2]) {
    // yaw_cmd en radians, 0 = Nord, pi/2 = Est

    x_target[0] = sinf(yaw_cmd); // composante X (Est)
    x_target[1] = cosf(yaw_cmd); // composante Y (Nord)
}

void getForwardHorizontal(float R[3][3], float xh[2]) {
    xh[0] = R[0][0]; // X terre
    xh[1] = R[1][0]; // Y terre

    // Normalisation (important si pitch non nul)
    float norm = sqrtf(xh[0]*xh[0] + xh[1]*xh[1]);
    if (norm > 1e-6f) {
        xh[0] /= norm;
        xh[1] /= norm;
    }
}

float computeYawCommandSignalImproved(float R[3][3], float yaw_cmd) {
    float x_target[2];
    computeTargetVector(yaw_cmd, x_target);

    float xh[2];
    getForwardHorizontal(R, xh);

    float dot   = xh[0]*x_target[0] + xh[1]*x_target[1];
    float cross = xh[0]*x_target[1] - xh[1]*x_target[0];

    // Si erreur > 90°, on force une commande forte
    if (dot < 0.0f) {
        return (cross > 0.0f) ? 1.0f : -1.0f;
    }

    return cross;
}
