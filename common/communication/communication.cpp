#include <Arduino.h>
#include <LoRa.h>
#include "config.h"

#define SYNC_1 0xAA
#define SYNC_2 0x55
#define SYNC_3 0xCC
#define SYNC_4 0x33
#define TRANSFORM_FORMAT 0x42
#define QUATERNION_FORMAT 0x51

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