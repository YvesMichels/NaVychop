#include <Arduino.h>
#include <LoRa.h>
#include "config.h"         // SYNC_1..4, TRANSFORM_FORMAT, TRANSFORM_FORMAT, LORA_* : source unique
#include "communication.h"
#include "logging.h"

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

bool lora_check_cmd(uint8_t *lora_cmd)
{
    const int packetSize = LoRa.parsePacket();
    if (packetSize == 0) return false;

    bool msg_ok = false;

    // Une trame = un paquet LoRa de exactement 7 octets
    if (packetSize == LORA_FRAME_SIZE)
    {
        uint8_t rx[LORA_FRAME_SIZE];
        for (int i = 0; i < LORA_FRAME_SIZE; i++) rx[i] = LoRa.read();
        // Synchro AA 55 + CRC ; lora_cmd = {TYPE, DAT0, DAT1, DAT2} si valide, quel que soit le TYPE
        msg_ok = decode_lora_frame_to_cmd(rx, lora_cmd);
    }

    // Vidage de sécurité de la FIFO
    while (LoRa.available()) LoRa.read();
    return msg_ok;
}

static void cmd_apply_safe(CmdState &c)
{
    c.motor_6 = CMD_MOTOR_SAFE;
    c.yaw_6   = CMD_AXIS_NEUTRAL;
    c.pitch_6 = CMD_AXIS_NEUTRAL;
    c.roll_6  = CMD_AXIS_NEUTRAL;
}

void lora_cmd_init(CmdState &c)
{
    cmd_apply_safe(c);
    c.save_req     = 0;
    c.save_pending = false;
    c.link         = LINK_NO_FRAME;
    c.last_cmd_ms  = 0;
    c.resume_count = 0;
    c.n_cmd = 0;
    c.n_save = 0;
    c.n_lost = 0;
}

void lora_cmd_set(CmdState &c, const uint8_t f[4], uint32_t now_ms)
{
    const uint8_t type = f[0];

    if (type == LORA_FRAME_CMD)
    {
        // Décodage complet dans des variables locales : aucune écriture partielle
        const uint8_t motor = (f[1] >> 2) & 0x3F;
        const uint8_t yaw   = ((f[1] & 0x03) << 4) | ((f[2] >> 4) & 0x0F);
        const uint8_t pitch = ((f[2] & 0x0F) << 2) | ((f[3] >> 6) & 0x03);
        const uint8_t roll  = f[3] & 0x3F;

        if (c.link != LINK_OK)
        {
            // Reprise (ou premier lien) : confirmation par trames rapprochées
            if (c.resume_count == 0 ||
                (uint32_t)(now_ms - c.last_cmd_ms) > LORA_RESUME_GAP_MS)
            {
                c.resume_count = 1;
            }
            else
            {
                c.resume_count++;
            }
            c.last_cmd_ms = now_ms;

            if (c.resume_count < LORA_RESUME_FRAMES)
                return;                       // pas encore appliqué : failsafe maintenu

            c.link = LINK_OK;
            c.resume_count = 0;
        }

        c.last_cmd_ms = now_ms;
        c.motor_6 = motor;
        c.yaw_6   = yaw;
        c.pitch_6 = pitch;
        c.roll_6  = roll;
        c.n_cmd++;
    }
    else if (type == LORA_FRAME_SAVE)
    {
        // Seules les valeurs 1 (start) et 2 (stop) sont acceptées
        if (f[1] == 1 || f[1] == 2)
        {
            c.save_req     = f[1];
            c.save_pending = true;
            c.n_save++;
            if (c.link == LINK_OK)
                c.last_cmd_ms = now_ms;       // le lien est vivant, mais ne rétablit pas le contrôle
        }
    }
    // autres types (paramètres 0x10+, inconnus) : ignorés ici
}

void lora_cmd_supervise(CmdState &c, uint32_t now_ms)
{
    if (c.link == LINK_OK &&
        (uint32_t)(now_ms - c.last_cmd_ms) > LORA_FAILSAFE_TIMEOUT_MS)
    {
        c.link = LINK_LOST;
        c.resume_count = 0;
        c.n_lost++;
    }

    // Réécrit les sorties neutres à chaque tour tant que le lien n'est pas OK
    if (c.link != LINK_OK)
        cmd_apply_safe(c);
}

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
    int8_t esc)
{
    // Temps
    s.t = t;

    // Accélération (Q4)
    s.a_x = (int16_t)roundf(ax * Q4_SCALE);
    s.a_y = (int16_t)roundf(ay * Q4_SCALE);
    s.a_z = (int16_t)roundf(az * Q4_SCALE);

    // Champ magnétique (Q4)
    s.mag_x = (int16_t)roundf(mag_x * Q4_SCALE);
    s.mag_y = (int16_t)roundf(mag_y * Q4_SCALE);
    s.mag_z = (int16_t)roundf(mag_z * Q4_SCALE);

    // Quaternion game (Q14)
    s.gqw = (int16_t)roundf(q_game.w * Q14_SCALE);
    s.gqx = (int16_t)roundf(q_game.x * Q14_SCALE);
    s.gqy = (int16_t)roundf(q_game.y * Q14_SCALE);
    s.gqz = (int16_t)roundf(q_game.z * Q14_SCALE);

    // Altitude (Q6)
    s.altitude = (int16_t)roundf(att.altitude * Q6_SCALE);
    
    // Commandes
    s.cmd_yaw   = cmd_yaw;
    s.cmd_pitch = cmd_pitch;
    s.cmd_roll  = cmd_roll;

    // Intégrateurs (Q6)
    s.eta_yaw   = (int16_t)roundf(ctrl.eta_yaw   * Q6_SCALE);
    s.eta_pitch = (int16_t)roundf(ctrl.eta_pitch * Q6_SCALE);
    s.eta_roll  = (int16_t)roundf(ctrl.eta_roll  * Q6_SCALE);

    // Valeurs précédentes (Q14)
    s.proj_yaw   = (int16_t)roundf(ctrl.old_yaw   * Q14_SCALE);
    s.proj_pitch = (int16_t)roundf(ctrl.old_pitch * Q14_SCALE);
    s.proj_roll  = (int16_t)roundf(ctrl.old_roll  * Q14_SCALE);

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

void saveLogsToFlash(logState *buffer, uint16_t packetIndex, uint16_t count)
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

    const size_t packetSize = count * sizeof(logState);

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
    Serial.print((float)s.a_x / Q4_SCALE, 4);
    Serial.print(",");
    Serial.print((float)s.a_y / Q4_SCALE, 4);
    Serial.print(",");
    Serial.print((float)s.a_z / Q4_SCALE, 4);

    // Magneto
    Serial.print(",");
    Serial.print((float)s.mag_x / Q4_SCALE, 6);
    Serial.print(",");
    Serial.print((float)s.mag_y / Q4_SCALE, 6);
    Serial.print(",");
    Serial.print((float)s.mag_z / Q4_SCALE, 6);

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

    // Commande sol
    Serial.print(",");
    Serial.print(s.cmd_yaw);
    Serial.print(",");
    Serial.print(s.cmd_pitch);
    Serial.print(",");
    Serial.print(s.cmd_roll);

    // Intégrateurs
    Serial.print(",");
    Serial.print((float)s.eta_yaw / Q6_SCALE, 4);
    Serial.print(",");
    Serial.print((float)s.eta_pitch / Q6_SCALE, 4);
    Serial.print(",");
    Serial.print((float)s.eta_roll / Q6_SCALE, 4);

    // Anciennes projections
    Serial.print(",");
    Serial.print((float)s.proj_yaw / Q14_SCALE, 6);
    Serial.print(",");
    Serial.print((float)s.proj_pitch / Q14_SCALE, 6);
    Serial.print(",");
    Serial.print((float)s.proj_roll / Q14_SCALE, 6);

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
        "a_x,a_y,a_z,"
        "mag_x,mag_y,mag_z,"
        "gqw,gqx,gqy,gqz,"
        "altitude,"
        "cmd_yaw,cmd_pitch,cmd_roll,"
        "eta_yaw,eta_pitch,eta_roll,"
        "proj_yaw,proj_pitch,proj_roll,"
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
                    //sendLogsCSVFromFlash();
                    log_dump_blocking();   // la boucle d'asservissement est suspendue : au sol uniquement
                }
                else
                {
                    sync_state = 0;
                }
                break;
        }
    }
}

void sendRotation(float R[3][3]){
    #if DEBUG_STREAM_R
        {
            char line[128];
            int n = snprintf(line, sizeof(line),
                "R,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                R[0][0], R[0][1], R[0][2],
                R[1][0], R[1][1], R[1][2],
                R[2][0], R[2][1], R[2][2]);
            Serial.write((uint8_t*)line, n);
        }
    #else
        (void)R;
    #endif
}