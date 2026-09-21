#include <Arduino.h>
#include <LoRa.h>
#include "config.h"
#include "communication.h"


#define SYNC_1 0xAA
#define SYNC_2 0x55
#define SYNC_3 0xCC
#define SYNC_4 0x33
#define TRANSFORM_FORMAT 0x42
#define QUATERNION_FORMAT 0x51
#define LORA_FRAME_SIZE 7 // 2 octest sync, 1 octet type, 3 octets data, 1 octet CRC

#define Q4_SCALE   16.0f
#define Q6_SCALE   64.0f
#define Q14_SCALE  16384.0f


void quantif_angle(int8_t* delta_angle, float angle, int16_t* last_angle, uint8_t nbits, uint8_t binpt) {
    int16_t buffer;
    buffer = (int16_t)(((int16_t)(angle * (float)binpt)) - *last_angle);
    if(buffer > ((2<<(nbits-1))-1)){
        *delta_angle = (int8_t)(2<<(nbits-1))-1;
    }else if (buffer < -2<<(nbits-1)){
        *delta_angle = (int8_t)-2<<(nbits-1);
    }
    else{
        *delta_angle = (int8_t)buffer;
    }
    *last_angle = *last_angle + (int16_t)(*delta_angle);
    *delta_angle = *delta_angle + (2<<(nbits-1));
    return;
}

static inline int16_t float_to_q16(float x) {
    if (x >  0.999984f) x =  0.999984f;
    if (x < -1.0f)      x = -1.0f;
    return (int16_t)(x * 32767.0f);
}


void send_quaternions_and_logs(float *q_grv, float *q_rv, int16_t *logs)
{
    uint8_t buffer[54];
    uint8_t checksum = 0;
    uint8_t *ptr = buffer;

    // --- Synchronisation ---
    *ptr++ = SYNC_1; checksum ^= SYNC_1;
    *ptr++ = SYNC_2; checksum ^= SYNC_2;
    *ptr++ = SYNC_3; checksum ^= SYNC_3;
    *ptr++ = SYNC_4; checksum ^= SYNC_4;

    // --- Format ---
    *ptr++ = QUATERNION_FORMAT; checksum ^= QUATERNION_FORMAT;

    // --- Quaternion GAME_ROTATION_VECTOR ---
    for (int i = 0; i < 4; i++) {
        int16_t v = float_to_q16(q_grv[i]);
        uint8_t b0 = v & 0xFF;
        uint8_t b1 = (v >> 8) & 0xFF;
        *ptr++ = b0; checksum ^= b0;
        *ptr++ = b1; checksum ^= b1;
    }

    // --- Quaternion ROTATION_VECTOR ---
    for (int i = 0; i < 4; i++) {
        int16_t v = float_to_q16(q_rv[i]);
        uint8_t b0 = v & 0xFF;
        uint8_t b1 = (v >> 8) & 0xFF;
        *ptr++ = b0; checksum ^= b0;
        *ptr++ = b1; checksum ^= b1;
    }

    // --- Logs (16 x 16 bits) ---
    for (int i = 0; i < 16; i++) {
        uint16_t v = (uint16_t)logs[i];
        for (int j = 0; j < 2; j++) {
            uint8_t b = (v >> (8*j)) & 0xFF;
            *ptr++ = b;
            checksum ^= b;
        }
    }

    // --- Checksum ---
    *ptr++ = checksum;

    // --- Envoi ---
    Serial.write(buffer, sizeof(buffer));

    //LoRa.beginPacket();
    //LoRa.write(buffer, sizeof(buffer));
    //LoRa.endPacket();
}

bool lora_check_and_echo()
{
    int packetSize = LoRa.parsePacket();
    if (packetSize == 2*sizeof(uint32_t)) {

        uint32_t pkt[2];
        LoRa.readBytes((uint8_t*)&pkt, sizeof(pkt));

        // Echo immédiat
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&pkt, sizeof(pkt));
        LoRa.endPacket(true);   // async

        return true;
    }
    return false;
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

void motors_control(int8_t servo1, int8_t servo2, int8_t servo3, int8_t servo4, int8_t esc)
{
    uint32_t servo_msg;
    servo_msg =  (servo1 & 0x3F)
             | ((servo2 & 0x3F) << 6)
             | ((servo3 & 0x3F) << 12)
             | ((servo4 & 0x3F) << 18)
             | ((esc    & 0x3F) << 24);
    uint8_t buffer_msg[6];
    buffer_msg[0] = 0xFF;                          // start byte
    memcpy(&buffer_msg[1], &servo_msg, 4);         // 4 octets de données
    buffer_msg[5] = crc8(&buffer_msg[1], 4);       // CRC calculé sur les 4 octets

    // Envoi de la trame moteur complète
    Serial2.write(buffer_msg, sizeof(buffer_msg));
}

bool decode_lora_frame_to_cmd(uint8_t *rx_buffer, uint8_t *cmd)
{
    // cmd[0] = TYPE
    // cmd[1..3] = DATA

    if (rx_buffer[0] != SYNC_1 || rx_buffer[1] != SYNC_2)
        return false;

    uint8_t computed_crc = crc8(&rx_buffer[2], 4);

    if (computed_crc != rx_buffer[6])
        return false;

    // Copie directe des 4 octets utiles
    cmd[0] = rx_buffer[2];
    cmd[1] = rx_buffer[3];
    cmd[2] = rx_buffer[4];
    cmd[3] = rx_buffer[5];

    return true;
}

bool lora_check_cmd(uint8_t lora_cmd[4]){
    // =============================
    // Réception LoRa non bloquante
    // =============================

    uint8_t lora_rx_buffer[LORA_FRAME_SIZE];
    int packetSize = LoRa.parsePacket();
    bool msg_ok = false;

    if (packetSize)
    {
        // Taille strictement attendue
        if (packetSize == LORA_FRAME_SIZE)
        {
            // Lecture FIFO complète
            for (int i = 0; i < LORA_FRAME_SIZE; i++)
            {
                lora_rx_buffer[i] = LoRa.read();
            }

            // Décodage protocole applicatif
            if (decode_lora_frame_to_cmd(lora_rx_buffer, lora_cmd))
            {
                // =============================
                // Exploitation commandes
                // =============================

                uint8_t type = lora_cmd[0];

                switch(type)
                {
                    // Commande manuelle
                    case 0x01:
                    {
                        msg_ok = true;
                        break;
                    }

                    // Paramètres / config
                    case 0x02:
                    {
                        msg_ok = true;
                        break;
                    }

                    default:
                        break;
                }
            }
        }

        // Vidage sécurité FIFO si taille incorrecte
        while (LoRa.available())
        {
            LoRa.read();
        }

        // Retour immédiat RX
        //LoRa.receive();
    }
    return msg_ok;
}

void lora_failsafe(uint8_t lora_cmd[4], uint32_t last_lora_ms, uint32_t now_ms, uint8_t old_value, uint16_t timeout_ms)
{
    if ((now_ms - last_lora_ms) > timeout_ms)
    {
        // FAILSAFE MODE

        lora_cmd[0] = 0;       // flag désactivé
        lora_cmd[1] = 1;       // motor=0, yaw=31
        lora_cmd[2] = 247;     // yaw=31, pitch=31
        lora_cmd[3] = 223;     // pitch=31, roll=31

        return;
    }
}

uint8_t addLog(
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
    int8_t esc)
{
    // Temps
    s.t = t;

    // Accélération (Q4)
    s.ax = (int16_t)roundf(ax * Q4_SCALE);
    s.ay = (int16_t)roundf(ay * Q4_SCALE);
    s.az = (int16_t)roundf(az * Q4_SCALE);

    // Quaternion rotation (Q14)
    s.qw = (int16_t)roundf(q_rot.w * Q14_SCALE);
    s.qx = (int16_t)roundf(q_rot.x * Q14_SCALE);
    s.qy = (int16_t)roundf(q_rot.y * Q14_SCALE);
    s.qz = (int16_t)roundf(q_rot.z * Q14_SCALE);

    // Quaternion game (Q14)
    s.gqw = (int16_t)roundf(q_game.w * Q14_SCALE);
    s.gqx = (int16_t)roundf(q_game.x * Q14_SCALE);
    s.gqy = (int16_t)roundf(q_game.y * Q14_SCALE);
    s.gqz = (int16_t)roundf(q_game.z * Q14_SCALE);

    // Altitude (Q6)
    s.altitude = (int16_t)roundf(att.altitude * Q6_SCALE);
    s.p0 = att.p0;

    // Intégrateurs (Q6)
    s.eta_yaw   = (int16_t)roundf(ctrl.eta_yaw   * Q6_SCALE);
    s.eta_pitch = (int16_t)roundf(ctrl.eta_pitch * Q6_SCALE);
    s.eta_roll  = (int16_t)roundf(ctrl.eta_roll  * Q6_SCALE);

    // Valeurs précédentes (Q14)
    s.old_yaw   = (int16_t)roundf(ctrl.old_yaw   * Q14_SCALE);
    s.old_pitch = (int16_t)roundf(ctrl.old_pitch * Q14_SCALE);
    s.old_roll  = (int16_t)roundf(ctrl.old_roll  * Q14_SCALE);

    // Commandes
    s.ctrl_yaw   = (int8_t)roundf(ctrl.ctrl_yaw);
    s.ctrl_pitch = (int8_t)roundf(ctrl.ctrl_pitch);
    s.ctrl_roll  = (int8_t)roundf(ctrl.ctrl_roll);

    // ESC
    s.esc = esc;

    // Incrément borné
    logIndex++;

    if (logIndex >= LOG_SIZE)
    {
        logIndex = LOG_SIZE - 1;
        return 1;
    }
    return 0;
    
}

void saveLogsToFlash(logState *buffer, uint16_t packetIndex)
{
    // Premier paquet : on supprime l'ancien fichier
    if (packetIndex == 0)
    {
        if (LittleFS.exists("/logs.bin"))
        {
            LittleFS.remove("/logs.bin");
        }
    }

    File file = LittleFS.open("/logs.bin", FILE_APPEND);

    if (!file)
    {
        Serial.println("Erreur ouverture fichier");
        return;
    }

    const size_t packetSize = LOG_SIZE * sizeof(logState);

    size_t written = file.write(
        (uint8_t*)buffer,
        packetSize);

    file.close();

    if (written != packetSize)
    {
        Serial.println("Erreur ecriture");
    }
}

void sendLogCSV(logState &s)
{
    Serial.print(s.t);

    // Accélérations Q4
    Serial.print(",");
    Serial.print((float)s.ax / Q4_SCALE, 4);
    Serial.print(",");
    Serial.print((float)s.ay / Q4_SCALE, 4);
    Serial.print(",");
    Serial.print((float)s.az / Q4_SCALE, 4);

    // Quaternion rotation
    Serial.print(",");
    Serial.print((float)s.qw / Q14_SCALE, 6);
    Serial.print(",");
    Serial.print((float)s.qx / Q14_SCALE, 6);
    Serial.print(",");
    Serial.print((float)s.qy / Q14_SCALE, 6);
    Serial.print(",");
    Serial.print((float)s.qz / Q14_SCALE, 6);

    // Quaternion game
    Serial.print(",");
    Serial.print((float)s.gqw / Q14_SCALE, 6);
    Serial.print(",");
    Serial.print((float)s.gqx / Q14_SCALE, 6);
    Serial.print(",");
    Serial.print((float)s.gqy / Q14_SCALE, 6);
    Serial.print(",");
    Serial.print((float)s.gqz / Q14_SCALE, 6);

    // Altitude
    Serial.print(",");
    Serial.print((float)s.altitude / Q6_SCALE, 3);

    Serial.print(",");
    Serial.print(s.p0, 3);

    // Intégrateurs
    Serial.print(",");
    Serial.print((float)s.eta_yaw / Q6_SCALE, 4);
    Serial.print(",");
    Serial.print((float)s.eta_pitch / Q6_SCALE, 4);
    Serial.print(",");
    Serial.print((float)s.eta_roll / Q6_SCALE, 4);

    // Anciennes projections
    Serial.print(",");
    Serial.print((float)s.old_yaw / Q14_SCALE, 6);
    Serial.print(",");
    Serial.print((float)s.old_pitch / Q14_SCALE, 6);
    Serial.print(",");
    Serial.print((float)s.old_roll / Q14_SCALE, 6);

    // Commandes
    Serial.print(",");
    Serial.print(s.ctrl_yaw);
    Serial.print(",");
    Serial.print(s.ctrl_pitch);
    Serial.print(",");
    Serial.print(s.ctrl_roll);

    // ESC
    Serial.print(",");
    Serial.println(s.esc);
}

void sendLogsCSVFromFlash()
{
    File file = LittleFS.open("/logs.bin", FILE_READ);

    if (!file)
    {
        Serial.println("Erreur lecture fichier");
        return;
    }

    Serial.println(
        "t,"
        "ax,ay,az,"
        "qw,qx,qy,qz,"
        "gqw,gqx,gqy,gqz,"
        "altitude,p0,"
        "eta_yaw,eta_pitch,eta_roll,"
        "old_yaw,old_pitch,old_roll,"
        "ctrl_yaw,ctrl_pitch,ctrl_roll,"
        "esc");

    logState log;

    while (file.read((uint8_t*)&log, sizeof(logState)) == sizeof(logState))
    {
        sendLogCSV(log);
    }

    file.close();
}

void checkUartAndTriggerDump(uint8_t sync_state, logState *buffer)
{
    while (Serial.available())
    {
        uint8_t b = Serial.read();

        switch (sync_state)
        {
            case 0:
                if (b == SYNC_1) sync_state = 1;
                else sync_state = 0;
                break;

            case 1:
                if (b == SYNC_2) sync_state = 2;
                else sync_state = (b == SYNC_1) ? 1 : 0;
                break;

            case 2:
                if (b == SYNC_3) sync_state = 3;
                else sync_state = 0;
                break;

            case 3:
                if (b == SYNC_4)
                {
                    sync_state = 0;

                    // =========================
                    // TRIGGER DUMP CSV
                    // =========================
                    sendLogsCSVFromFlash();
                }
                else
                {
                    sync_state = 0;
                }
                break;
        }
    }
}