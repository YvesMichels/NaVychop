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