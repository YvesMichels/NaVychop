#include <Arduino.h>
#include <LoRa.h>
#include "communication.h"

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

void init_angles(float roll, float pitch, float yaw, int16_t* first_angle, int16_t* last_angle) {
    last_angle[0] = 0;
    last_angle[1] = 0;
    last_angle[2] = 0;
    first_angle[0] = (int16_t)(yaw * 8);
    first_angle[1] = (int16_t)(pitch * 2);
    first_angle[2] = (int16_t)(roll * 2);
    return;
}
void write_fifo_d_angles(int8_t (*buffer_d_angles)[size_buffer_delta], float roll, float pitch, float yaw, int16_t* last_angle, uint16_t* ind_fifo_data) 
{
    quantif_angle(&buffer_d_angles[0][*ind_fifo_data], yaw, &last_angle[0], 6, binpt_yaw);
    quantif_angle(&buffer_d_angles[1][*ind_fifo_data], pitch, &last_angle[1], 5, binpt_pitch);
    quantif_angle(&buffer_d_angles[2][*ind_fifo_data], roll, &last_angle[2], 5, binpt_roll);

    // Incrémenter l’indice FIFO (si tu veux faire défiler la mémoire circulaire)
    (*ind_fifo_data)++;
    if (*ind_fifo_data >= size_buffer_delta) {
    *ind_fifo_data = 0;
    }
    return;
}

void send_fifo_d_angles(int16_t first_angle[3], int8_t (*buffer_d_angles)[size_buffer_delta], uint16_t* ind_rd_fifo_data, uint16_t ind_pkt)
{
    uint16_t word16;
    uint32_t word32; 
    int16_t yaw_q;
    int16_t roll_q;
    int16_t pitch_q;
    uint16_t roll_u;
    uint16_t pitch_u;
    uint16_t yaw_u;

    LoRa.beginPacket();
    // envoi de l'inndice du paquet
    LoRa.write((uint8_t)(ind_pkt >> 8));   // octet haut
    LoRa.write((uint8_t)(ind_pkt & 0xFF)); // octet bas

    // Construction du mot 16 bits : [yaw:12 | pitch:10 | roll:10]
    word32 = (first_angle[0] << 20) | (first_angle[1] << 10) | first_angle[2];
    
    LoRa.write((uint8_t)(word32 >> 24 & 0xFF));   // octet 3 (haut)
    LoRa.write((uint8_t)(word32 >> 16 & 0xFF)); // octet 2
    LoRa.write((uint8_t)(word32 >> 8 & 0xFF));   // octet 1
    LoRa.write((uint8_t)(word32 & 0xFF)); // octet 0 (bas)
    
    for (uint16_t i = 0; i < (nb_compress-1); i++) {
        // Récupère les valeurs dans la FIFO
        roll_q  = buffer_d_angles[2][*ind_rd_fifo_data];
        pitch_q = buffer_d_angles[1][*ind_rd_fifo_data];
        yaw_q   = buffer_d_angles[0][*ind_rd_fifo_data];

        // Mise à l’échelle pour rentrer dans les 6/5/5 bits
        // On suppose ici que les valeurs sont déjà quantifiées dans les bons intervalles.
        roll_u  = ((uint8_t)(roll_q  & 0x1F)) & 0x1F; // 5 bits
        pitch_u = ((uint8_t)(pitch_q & 0x1F)) & 0x1F; // 5 bits
        yaw_u   = ((uint8_t)(yaw_q   & 0x3F)) & 0x3F; // 6 bits

        // Construction du mot 16 bits : [roll:6 | pitch:5 | yaw:5]
        word16 = (yaw_u << 10) | (pitch_u << 5) | roll_u;

        // Envoi via LoRa.write (MSB en premier)
        LoRa.write((uint8_t)(word16 >> 8));   // octet haut
        LoRa.write((uint8_t)(word16 & 0xFF)); // octet bas

        // Avance l’indice FIFO circulaire
        (*ind_rd_fifo_data)++;
        if (*ind_rd_fifo_data >= size_buffer_delta)
            *ind_rd_fifo_data = 0;
    }
    LoRa.endPacket();
    return;
}