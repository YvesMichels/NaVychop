#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <cstdint>
#include "config.h"
#define binpt_yaw 8 // mise à l'échelle yaw (puissance de 2 car quantification en virgule fixe)
#define binpt_pitch 4 // mise à l'échelle pitch (puissance de 2 car quantification en virgule fixe)
#define binpt_roll 4 // mise à l'échelle roll (puissance de 2 car quantification en virgule fixe)
#define nb_compress 50
#define size_buffer_delta 4950 // taille du buffer de données à envoyer au sol

void quantif_angle(int16_t* delta_angle, float angle, int16_t* last_angle, uint8_t nbits, uint8_t binpt);

void init_angles(float roll, float pitch, float yaw, int16_t* first_angle, int16_t* last_angle);

void write_fifo_d_angles(int8_t (*buffer_d_angles)[size_buffer_delta], float roll, float pitch, float yaw, int16_t* last_angle, uint16_t* ind_fifo_data);

void send_fifo_d_angles(int16_t first_angle[3], int8_t (*buffer_d_angles)[size_buffer_delta], uint16_t* ind_rd_fifo_data, uint16_t ind_pkt);
                         
#endif