#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <LittleFS.h>
#include "config.h"
#include "control.h"
#include "navigation.h"

struct logState
{
    // Temps
    unsigned long t;

    // Accélération
    int16_t a_x;         // 4 bits fractionnaire
    int16_t a_y;         // 4 bits fractionnaire
    int16_t a_z;         // 4 bits fractionnaire

    // magneto
    int16_t mag_x;         // 14 bits fractionnaire
    int16_t mag_y;         // 14 bits fractionnaire
    int16_t mag_z;         // 14 bits fractionnaire

    // Quaternion game rotation
    int16_t gqw;        // 14 bits fractionnaire
    int16_t gqx;        // 14 bits fractionnaire
    int16_t gqy;        // 14 bits fractionnaire
    int16_t gqz;        // 14 bits fractionnaire

    // Altitude / pression
    int16_t altitude;   // 6 bits fractionnaire

    // Intégrateurs
    int16_t eta_yaw;    // 6 bits fractionnaire
    int16_t eta_pitch;  // 6 bits fractionnaire
    int16_t eta_roll;   // 6 bits fractionnaire

    // Projections précédentes
    int16_t proj_yaw;    // 14 bits fractionnaire
    int16_t proj_pitch;  // 14 bits fractionnaire
    int16_t proj_roll;   // 14 bits fractionnaire

    // Commandes sol
    int8_t cmd_yaw;    // 0 bits fractionnaire
    int8_t cmd_pitch;  // 0 bits fractionnaire
    int8_t cmd_roll;   // 0 bits fractionnaire

    // Commandes moteurs
    int8_t ctrl_yaw;    // 0 bits fractionnaire
    int8_t ctrl_pitch;  // 0 bits fractionnaire
    int8_t ctrl_roll;   // 0 bits fractionnaire

    // ESC principal
    uint8_t esc;        // 0 bits fractionnaire
};

enum LinkState : uint8_t {
    LINK_NO_FRAME = 0,   // démarrage : aucune trame pilote reçue
    LINK_LOST     = 1,   // timeout dépassé
    LINK_OK       = 2    // lien établi, commandes appliquées
};

struct CmdState {
    // Sorties de commande (valeurs 6 bits)
    uint8_t motor_6;
    uint8_t yaw_6;
    uint8_t pitch_6;
    uint8_t roll_6;

    // Demande de sauvegarde (événement : à consommer par main)
    uint8_t save_req;
    bool    save_pending;

    // Supervision du lien
    LinkState link;
    uint32_t last_cmd_ms;     // dernière trame pilote valide (lien OK) / dernière trame de reprise
    uint8_t  resume_count;    // trames consécutives de confirmation de reprise

    // Diagnostic
    uint32_t n_cmd;
    uint32_t n_save;
    uint16_t n_lost;          // nombre de passages en failsafe
};

void quantif_angle(int16_t* delta_angle, float angle, int16_t* last_angle, uint8_t nbits, uint8_t binpt);
void send_quaternions_and_logs(float *q_grv, float *q_rv, int16_t *logs);

bool lora_check_and_echo();

uint8_t crc8(const uint8_t* data, size_t len);

void motors_control(int8_t servo1, int8_t servo2, int8_t servo3, int8_t servo4, int8_t esc);

bool decode_lora_frame_to_cmd(uint8_t *rx_buffer, uint8_t *cmd);
bool lora_check_cmd(uint8_t *lora_cmd);
//void lora_failsafe(uint8_t lora_cmd[4], uint32_t last_lora_ms, uint32_t now_ms, uint8_t old_value, uint16_t timeout_ms);
void lora_cmd_init(CmdState &c);
// Traite UNE trame valide (types 0x01 / 0x02, les autres sont ignorés).
void lora_cmd_set(CmdState &c, const uint8_t lora_cmd[4], uint32_t now_ms);
// À appeler à chaque tour, trame reçue ou non : timeout et failsafe.
void lora_cmd_supervise(CmdState &c, uint32_t now_ms);


uint8_t addLog(
    logState &s, 
    uint16_t &logIndex, 
    unsigned long t,
    float ax, float ay, float az,
    float mag_x, float mag_y, float mag_z,
    const Quaternion& q_game,
    const AttitudeState& att,
    uint8_t cmd_yaw, uint8_t cmd_pitch, uint8_t cmd_roll, 
    const ControlState& ctrl,
    int8_t esc);
void saveLogsToFlash(logState *buffer, uint16_t packetIndex);
void sendLogsCSVFromFlash();
void sendLogsCSV(logState &s);
void checkUartAndTriggerDump(uint8_t sync_state, logState *buffer);

#endif