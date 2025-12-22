/**
 * \copyright Vychop
 * \brief fonctions de traitement d'images et communication debogue et sauvegarde. 
 * Fonctions Co-écrites par Chat-GPT
 */

#include <Arduino.h>
#include "transformation.h"

/*
bits 31 ............... 23 | 22 ............... 14 | 13 .............. 6 | 5 .............. 0
        y (9 bits)              x (9 bits)             valeur pixel         valeur HF
*/

#define Y_SHIFT 23
#define X_SHIFT 14
#define COORD_MASK 0x1FF      // 9 bits
#define LOW_MASK   0x3FFF     // 14 bits
#define Q16_SHIFT 16
#define Q16_ONE   (1 << Q16_SHIFT)
#define PI_Q16    205887   // π   * 2^16
#define TWO_PI_Q16 (2 * PI_Q16)
#define HALF_PI_Q16 (PI_Q16 / 2)

#define SIN_LUT_SIZE 128

// LUT 129 valeurs en Q1.15
const int16_t sin_lut[SIN_LUT_SIZE + 1] = {
      0,   402,   804,  1206,  1608,  2009,  2411,  2811,
   3212,  3612,  4011,  4410,  4808,  5205,  5601,  5997,
   6392,  6786,  7179,  7571,  7962,  8352,  8740,  9128,
   9514,  9899, 10283, 10665, 11046, 11425, 11803, 12180,
  12555, 12928, 13300, 13670, 14038, 14405, 14770, 15133,
  15495, 15855, 16213, 16569, 16924, 17276, 17627, 17976,
  18323, 18668, 19011, 19352, 19691, 20028, 20363, 20696,
  21027, 21355, 21682, 22006, 22328, 22648, 22966, 23281,
  23594, 23905, 24214, 24520, 24824, 25126, 25425, 25722,
  26016, 26308, 26598, 26885, 27170, 27452, 27732, 28009,
  28284, 28556, 28826, 29093, 29358, 29620, 29879, 30136,
  30390, 30642, 30891, 31138, 31382, 31623, 31862, 32098,
  32331, 32562, 32790, 32767
};

static inline int32_t q16_mul(int32_t a, int32_t b) {
    return (int32_t)(((int64_t)a * b) >> Q16_SHIFT);
}

int32_t q16_sin(int32_t theta) {
    // Normalisation
    while (theta < 0) theta += TWO_PI_Q16;
    while (theta >= TWO_PI_Q16) theta -= TWO_PI_Q16;

    int sign = 1;

    if (theta > PI_Q16) {
        theta -= PI_Q16;
        sign = -1;
    }

    if (theta > HALF_PI_Q16) {
        theta = PI_Q16 - theta;
    }

    // position fractionnaire sur la LUT (Q16.16)
    int32_t pos = (theta * SIN_LUT_SIZE << Q16_SHIFT) / HALF_PI_Q16;

    int32_t idx  = pos >> Q16_SHIFT;
    int32_t frac = pos & 0xFFFF;   // fraction Q0.16

    if (idx >= SIN_LUT_SIZE)
        return sign * ((int32_t)sin_lut[SIN_LUT_SIZE] << 1);

    // interpolation linéaire
    int32_t y0 = sin_lut[idx];
    int32_t y1 = sin_lut[idx + 1];

    int32_t y = y0 + ((y1 - y0) * frac >> Q16_SHIFT);

    // Q1.15 -> Q16.16
    return sign * (y << 1);
}


int32_t q16_cos(int32_t theta) {
    return q16_sin(HALF_PI_Q16 - theta);
}



uint32_t transform_point(
    uint32_t p,
    int32_t vp_x, int32_t vp_y,
    int32_t tx, int32_t ty,
    int32_t theta,
    int32_t div
) {
    // Extraction
    int32_t y = (p >> Y_SHIFT) & COORD_MASK;
    int32_t x = (p >> X_SHIFT) & COORD_MASK;
    uint32_t low = p & LOW_MASK;

    // Passage en Q16.16
    int32_t x_q = x << Q16_SHIFT;
    int32_t y_q = y << Q16_SHIFT;

    // Repère point de fuite
    int32_t dx = x_q - vp_x;
    int32_t dy = y_q - vp_y;

    // Divergence radiale (trapèze)
    // scale = 1 + div * dy
    int32_t scale = Q16_ONE + q16_mul(div, dy);
    dx = q16_mul(dx, scale);
    dy = q16_mul(dy, scale);

    // Rotation
    int32_t c = q16_cos(theta);
    int32_t s = q16_sin(theta);

    int32_t dx_r = q16_mul(dx, c) - q16_mul(dy, s);
    int32_t dy_r = q16_mul(dx, s) + q16_mul(dy, c);

    // Retour image + translation
    int32_t x_out = vp_x + dx_r + tx;
    int32_t y_out = vp_y + dy_r + ty;

    // Retour entier + saturation
    int32_t xi = x_out >> Q16_SHIFT;
    int32_t yi = y_out >> Q16_SHIFT;

    if (xi < 0) xi = 0;
    if (xi > 511) xi = 511;
    if (yi < 0) yi = 0;
    if (yi > 511) yi = 511;

    // Reconstruction uint32_t
    return ((uint32_t)yi << Y_SHIFT) |
           ((uint32_t)xi << X_SHIFT) |
           low;
}

void transform_points(
    const uint32_t *points_in,
    uint32_t *points_out,
    uint16_t nb_points,
    int32_t vp_x, int32_t vp_y,
    int32_t tx, int32_t ty,
    int32_t theta,
    int32_t div
) {
    for (uint16_t i = 0; i < nb_points; i++) {
        points_out[i] = transform_point(
            points_in[i],
            vp_x, vp_y,
            tx, ty,
            theta,
            div
        );
    }
}
