#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <LittleFS.h>
#include "config.h"
#include "control.h"
#include "navigation.h"

#define binpt_yaw 8 // mise à l'échelle yaw (puissance de 2 car quantification en virgule fixe)
#define binpt_pitch 4 // mise à l'échelle pitch (puissance de 2 car quantification en virgule fixe)
#define binpt_roll 4 // mise à l'échelle roll (puissance de 2 car quantification en virgule fixe)
#define nb_compress 50

struct logState
{
    // Temps
    unsigned long t;

    // Accélération
    float ax;
    float ay;
    float az;

    // Quaternion rotation
    float qw;
    float qx;
    float qy;
    float qz;

    // Quaternion game rotation
    float gqw;
    float gqx;
    float gqy;
    float gqz;

    // Altitude / pression
    float altitude;
    float p0;

    // Intégrateurs
    float eta_yaw;
    float eta_pitch;
    float eta_roll;

    // Projections précédentes
    float old_yaw;
    float old_pitch;
    float old_roll;

    // Commandes
    float ctrl_yaw;
    float ctrl_pitch;
    float ctrl_roll;

    // ESC principal
    float esc;
};

void quantif_angle(int16_t* delta_angle, float angle, int16_t* last_angle, uint8_t nbits, uint8_t binpt);
void send_quaternions_and_logs(float *q_grv, float *q_rv, int16_t *logs);

bool lora_check_and_echo();

uint8_t crc8(const uint8_t* data, size_t len);

void motors_control(int8_t servo1, int8_t servo2, int8_t servo3, int8_t servo4, int8_t esc);

bool decode_lora_frame_to_cmd(uint8_t *rx_buffer, uint8_t *cmd);
bool lora_check_cmd(uint8_t *lora_cmd);
void lora_failsafe(uint8_t lora_cmd[4], uint32_t last_lora_ms, uint32_t now_ms, uint8_t old_value, uint16_t timeout_ms);

void addLog(
    logState &s, 
    uint16_t &logIndex, 
    unsigned long t,
    float ax,
    float ay,
    float az,
    const Quaternion& q_rot,
    const Quaternion& q_game,
    const AttitudeState& att,
    const ControlState& ctrl,
    int8_t esc);
void saveLogsToFlash(logState *buffer);
void loadLogsFromFlash(logState *buffer);
void sendLogsCSV(logState *buffer);
void checkUartAndTriggerDump(uint8_t sync_state, logState *buffer);

#endif