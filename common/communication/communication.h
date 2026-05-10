#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include "config.h"

#define binpt_yaw 8 // mise à l'échelle yaw (puissance de 2 car quantification en virgule fixe)
#define binpt_pitch 4 // mise à l'échelle pitch (puissance de 2 car quantification en virgule fixe)
#define binpt_roll 4 // mise à l'échelle roll (puissance de 2 car quantification en virgule fixe)
#define nb_compress 50
#define size_buffer_delta 4950 // taille du buffer de données à envoyer au sol

void quantif_angle(int16_t* delta_angle, float angle, int16_t* last_angle, uint8_t nbits, uint8_t binpt);
 
void send_quaternions_and_logs(float *q_grv, float *q_rv, int16_t *logs);

bool lora_check_and_echo();

uint8_t crc8(const uint8_t* data, size_t len);

void motors_control(int8_t servo1, int8_t servo2, int8_t servo3, int8_t servo4, int8_t esc);

bool decode_lora_frame_to_cmd(uint8_t *rx_buffer, uint8_t *cmd);

bool lora_check_cmd(uint8_t *lora_cmd);

#endif