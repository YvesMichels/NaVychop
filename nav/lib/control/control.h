#ifndef CONTROL_H
#define CONTROL_H
#include <Arduino.h>

struct ControlParameters {
    float Kp_yaw;
    float Ki_yaw;
    float Kd_yaw;

    float Kp_pitch;
    float Ki_pitch;
    float Kd_pitch;

    float Kp_roll;
    float Ki_roll;
    float Kd_roll;

    float ctrl_bias_yaw;    // biais de position de la gouverne sur le yaw : calibration manuelle
    float ctrl_bias_pitch;  // biais de position de la gouverne sur le pitch : calibration manuelle
    float ctrl_bias_roll;   // biais de position de la gouverne sur le roll : calibration manuelle

};

struct ControlState {
    // Integrateurs 
    float eta_yaw;
    float eta_pitch;
    float eta_roll;

    // Estimateur biais lent
    float bias_roll;

    // Valeurs précédentes
    float old_yaw;
    float old_pitch;
    float old_roll;

    // Controles moteurs
    float ctrl_yaw;
    float ctrl_pitch;
    float ctrl_roll;

    uint8_t init;
};

void control_init(ControlState &state);
void control_gain_init(ControlParameters
 &gains);
void control_update_inner(ControlState &state, ControlParameters
 &gains, float R_A_T[3][3], float cmd_yaw, float cmd_pitch, float cmd_roll);
 
#endif