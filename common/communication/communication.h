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
    int16_t ax;         // 4 bits fractionnaire
    int16_t ay;         // 4 bits fractionnaire
    int16_t az;         // 4 bits fractionnaire

    // Quaternion rotation
    int16_t qw;         // 14 bits fractionnaire
    int16_t qx;         // 14 bits fractionnaire
    int16_t qy;         // 14 bits fractionnaire
    int16_t qz;         // 14 bits fractionnaire

    // Quaternion game rotation
    int16_t gqw;        // 14 bits fractionnaire
    int16_t gqx;        // 14 bits fractionnaire
    int16_t gqy;        // 14 bits fractionnaire
    int16_t gqz;        // 14 bits fractionnaire

    // Altitude / pression
    int16_t altitude;   // 6 bits fractionnaire
    float p0;

    // Intégrateurs
    int16_t eta_yaw;    // 6 bits fractionnaire
    int16_t eta_pitch;  // 6 bits fractionnaire
    int16_t eta_roll;   // 6 bits fractionnaire

    // Projections précédentes
    int16_t old_yaw;    // 14 bits fractionnaire
    int16_t old_pitch;  // 14 bits fractionnaire
    int16_t old_roll;   // 14 bits fractionnaire

    // Commandes
    int8_t ctrl_yaw;    // 0 bits fractionnaire
    int8_t ctrl_pitch;  // 0 bits fractionnaire
    int8_t ctrl_roll;   // 0 bits fractionnaire

    // ESC principal
    uint8_t esc;        // 0 bits fractionnaire
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