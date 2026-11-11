/**
 * \copyright Vychop
 * \brief fonctions de traitement d'images et communication debogue et sauvegarde. 
 * Fonctions Co-écrites par Chat-GPT
 */

#include <Arduino.h>
#include "image_process.h"
#include "transformation.h"

void verification_tranformation(uint32_t *points_ref, uint32_t *points_trans, uint16_t nb_points){
    for(int ii = 0; ii<20; ii++){
    delay(200);  // 
    transform_points(points_ref, points_trans, nb_points, 160*65536 , 120*65536, ii*150000-1500000, 0, 0, 0);
    send_point(points_trans, nb_points);
  }
  for(int ii = 0; ii<20; ii++){
    delay(200);  // 
    transform_points(points_ref, points_trans, nb_points, 160*65536 , 120*65536, 0, ii*150000-1500000, 0, 0);
    send_point(points_trans, nb_points);
  }
  for(int ii = 0; ii<20; ii++){
    delay(200);  // 
    transform_points(points_ref, points_trans, nb_points, 160*65536 , 120*65536, 0, 0, 0, ii*4000-20000);
    send_point(points_trans, nb_points);
  }
  for(int ii = 0; ii<100; ii++){
    delay(200);  // 
    transform_points(points_ref, points_trans, nb_points, 160*65536 , 120*65536, 0, 0, ii*4118, 0);
    send_point(points_trans, nb_points);
  }
}

void verification_q16_sin(uint32_t *points_trans, uint16_t nb_points){
    uint32_t word;
    uint8_t val = 50; // couleur sombre
    uint8_t grad = 100; // gradient élevé
    int32_t y_sin;
    int x;
    int y;
    for(int i = 0; i<nb_points; i++){

        x = i+1;
        y_sin = q16_sin(((int32_t) x)*1373); // multiplie par 65536*2*pi/300 (passage de [1,300] à [0+,2*pi] et format Q16.16)
        y = (y_sin/256) + 256; // multiplie par 100/65536 (passage de [-1,1] à [0,200] et format entier)
        if(y>511){
          y = 511;
        }
        word = ((y & 0x1FF) << 23)
            | ((x & 0x1FF) << 14)
            | ((val & 0xFF) << 6)
            | (grad & 0x3F);
        points_trans[i] = word;
    }

    send_point(points_trans, nb_points);
    delay(1000);
}

/* A METTRE DANS COMMUNICATION */
#define SYNC_1 0xAA
#define SYNC_2 0x55
#define SYNC_3 0xCC
#define SYNC_4 0x33

#define TRANSFORM_FORMAT 0x42
void send_transform_and_cost(
    transform_param_q16_t *p,
    uint32_t cost_q16
) {
    uint8_t checksum = 0;

    // --- Synchronisation 4 octets ---
    Serial.write(SYNC_1);
    Serial.write(SYNC_2);
    Serial.write(SYNC_3);
    Serial.write(SYNC_4);

    // --- Format ---
    Serial.write(TRANSFORM_FORMAT);

    // --- Coût ---
    uint32_t v = cost_q16;
    uint8_t b0 = (uint8_t)(v & 0xFF);
    uint8_t b1 = (uint8_t)((v >> 8) & 0xFF);
    uint8_t b2 = (uint8_t)((v >> 16) & 0xFF);
    uint8_t b3 = (uint8_t)((v >> 24) & 0xFF);

    Serial.write(b0); checksum ^= b0;
    Serial.write(b1); checksum ^= b1;
    Serial.write(b2); checksum ^= b2;
    Serial.write(b3); checksum ^= b3;

    // --- Paramètres transformation ---
    const int32_t *params = (const int32_t *)p;

    for (uint8_t i = 0; i < 6; i++) {
        v = (uint32_t)params[i];
        b0 = (uint8_t)(v & 0xFF);
        b1 = (uint8_t)((v >> 8) & 0xFF);
        b2 = (uint8_t)((v >> 16) & 0xFF);
        b3 = (uint8_t)((v >> 24) & 0xFF);

        Serial.write(b0); checksum ^= b0;
        Serial.write(b1); checksum ^= b1;
        Serial.write(b2); checksum ^= b2;
        Serial.write(b3); checksum ^= b3;
    }

    // --- Checksum final ---
    Serial.write(checksum);
}

void verification_interpolation(uint8_t* image, int idim, int jdim){

  // donne des valeurs sur les 3*3 premiers pixels
  image[0 * jdim + 0] = 10;
  image[0 * jdim + 1] = 20;
  image[0 * jdim + 2] = 30;
  image[1 * jdim + 0] = 100;
  image[1 * jdim + 1] = 200;
  image[1 * jdim + 2] = 250;
  image[2 * jdim + 0] = 100;
  image[2 * jdim + 1] = 100;
  image[2 * jdim + 2] = 100;

  transform_param_q16_t p_red;

  p_red.div   = 32768; // 0.5
  p_red.theta = 32768; // 0.5
  p_red.tx    = read_bilineaire(image, 32768, 32768, idim, jdim);
  p_red.ty    = 65536; // 1.0
  p_red.vp_x  = 65536; // 1.0
  p_red.vp_y  = read_bilineaire(image, 65536, 65536, idim, jdim);
  send_transform_and_cost(&p_red, 0);
  delay(1000);

  for(int i = 0; i < 256; i++){
    p_red.div   = 65536+i*256;
    p_red.theta = 65536+0;
    p_red.tx    = read_bilineaire(image, 65536+i*256, 65536+0, idim, jdim);
    p_red.ty    = 65536+0; 
    p_red.vp_x  = 65536+i*256;
    p_red.vp_y  = read_bilineaire(image, 65536+0, 65536+i*256, idim, jdim);
    send_transform_and_cost(&p_red, 0);
    delay(300);
  }
}