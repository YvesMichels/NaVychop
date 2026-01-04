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

#define Y_SHIFT     23
#define X_SHIFT     14
#define PIX_SHIFT   6
#define COORD_MASK  0x1FF   // 9 bits
#define LOW_MASK    0x3FFF  // 14 bits
#define PIX_MASK    0xFF    // 8 bits
#define Q16_SHIFT   16
#define Q16_ONE     (1 << Q16_SHIFT)
#define PI_Q16      205887   // π   * 2^16
#define TWO_PI_Q16  (2 * PI_Q16)
#define HALF_PI_Q16 (PI_Q16 / 2)

#define SIN_LUT_SIZE 128

// LUT 129 valeurs en Q1.15
static const int16_t sin_lut[SIN_LUT_SIZE + 1] = {
        0,    402,    804,   1206,   1608,   2009,   2410,   2811,
     3212,   3612,   4011,   4410,   4808,   5205,   5602,   5998,
     6393,   6786,   7179,   7571,   7962,   8351,   8739,   9126,
     9512,   9896,  10278,  10659,  11039,  11417,  11793,  12167,
    12539,  12910,  13279,  13645,  14010,  14372,  14732,  15090,
    15446,  15800,  16151,  16499,  16846,  17189,  17530,  17869,
    18204,  18537,  18868,  19195,  19519,  19841,  20159,  20475,
    20787,  21096,  21403,  21705,  22005,  22301,  22594,  22884,
    23170,  23452,  23731,  24007,  24279,  24547,  24811,  25072,
    25329,  25582,  25832,  26077,  26319,  26556,  26790,  27019,
    27245,  27466,  27683,  27896,  28105,  28310,  28510,  28706,
    28898,  29085,  29268,  29447,  29621,  29791,  29956,  30117,
    30273,  30424,  30571,  30714,  30852,  30985,  31113,  31237,
    31356,  31470,  31580,  31685,  31785,  31880,  31971,  32057,
    32137,  32213,  32285,  32351,  32412,  32469,  32521,  32567,
    32609,  32646,  32678,  32705,  32728,  32745,  32757,  32765,
    32767,
};

static inline int32_t q16_mul(int32_t a, int32_t b) {
    return (int32_t)(((int64_t)a * b) >> Q16_SHIFT);
}

int32_t q16_sin(int32_t theta)
{
    // normalisation [0, 2π[
    while (theta < 0)           theta += TWO_PI_Q16;
    while (theta >= TWO_PI_Q16) theta -= TWO_PI_Q16;

    int sign = 1;

    // quadrant 3 & 4
    if (theta >= PI_Q16) {
        theta -= PI_Q16;
        sign = -1;
    }

    // quadrant 2
    if (theta > HALF_PI_Q16) {
        theta = PI_Q16 - theta;
    }

    // maintenant : theta ∈ [0, π/2]

    int32_t pos = ((int64_t)theta * SIN_LUT_SIZE << 16) / HALF_PI_Q16;
    int32_t idx  = pos >> 16;
    int32_t frac = pos & 0xFFFF;

    if (idx >= SIN_LUT_SIZE)
        return sign * (32767 << 1);

    int32_t y0 = sin_lut[idx];
    int32_t y1 = sin_lut[idx + 1];

    int32_t y = y0 + (((y1 - y0) * frac) >> 16);

    return sign * (y << 1);   // Q16.16
}


int32_t q16_cos(int32_t theta) {
    return q16_sin(HALF_PI_Q16 - theta);
}



uint32_t transform_point(
    uint32_t p,
    int32_t vp_x, int32_t vp_y,
    int32_t tx, int32_t ty,
    int32_t sin_theta,
    int32_t cos_theta,
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
    // scale = 1 + div
    int32_t scale = Q16_ONE + div;
    dx = q16_mul(dx, scale);
    dy = q16_mul(dy, scale);

    // Rotation
    int32_t dx_r = q16_mul(dx, cos_theta) - q16_mul(dy, sin_theta);
    int32_t dy_r = q16_mul(dx, sin_theta) + q16_mul(dy, cos_theta);

    // Retour image + translation
    //int32_t x_out = vp_x + dx_r + tx;
    //int32_t y_out = vp_y + dy_r + ty;
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
    uint32_t *points_in,
    uint32_t *points_out,
    uint16_t nb_points,
    int32_t vp_x, int32_t vp_y,
    int32_t tx, int32_t ty,
    int32_t theta,
    int32_t div
) {
    // pre-calcul rotation
    int32_t cos_theta = q16_cos(theta);
    int32_t sin_theta = q16_sin(theta);
    for (uint16_t i = 0; i < nb_points; i++) {
        points_out[i] = transform_point(points_in[i], vp_x, vp_y, tx, ty, sin_theta, cos_theta, div);
    }
}

void ssd_huber_gradient_q16(
    const uint32_t *points_ref,
    int nb_points,
    const uint8_t *img_cur,
    int width, int height,

    const transform_param_q16_t *p,

    int32_t huber_delta_q16,
    transform_grad_q16_t *g_out
)
{
    int64_t g_div = 0;
    int64_t g_theta = 0;
    int64_t g_tx = 0;
    int64_t g_ty = 0;

    int32_t c = q16_cos(p->theta);
    int32_t s = q16_sin(p->theta);
    int32_t scale = Q16_ONE + p->div;

    for (int i = 0; i < nb_points; i++) {

        uint32_t w = points_ref[i];

        /* ---- extraction point référence ---- */
        int32_t y = (w >> Y_SHIFT) & COORD_MASK;
        int32_t x = (w >> X_SHIFT) & COORD_MASK;
        int32_t pix_ref = (w >> PIX_SHIFT) & PIX_MASK;
        /* HF_im disponible mais non utilisé */

        if (x <= 0 || x >= width - 1 ||
            y <= 0 || y >= height - 1)
            continue;

        /* ---- coordonnées Q16 ---- */
        int32_t x_q = x << Q16_SHIFT;
        int32_t y_q = y << Q16_SHIFT;

        int32_t dx = x_q - p->vp_x;
        int32_t dy = y_q - p->vp_y;

        int32_t dxp = q16_mul(dx, scale);
        int32_t dyp = q16_mul(dy, scale);

        int32_t dxr = q16_mul(dxp, c) - q16_mul(dyp, s);
        int32_t dyr = q16_mul(dxp, s) + q16_mul(dyp, c);

        int32_t xw = p->vp_x + dxr + p->tx;
        int32_t yw = p->vp_y + dyr + p->ty;

        int xi = xw >> Q16_SHIFT;
        int yi = yw >> Q16_SHIFT;

        if (xi <= 0 || xi >= width - 1 ||
            yi <= 0 || yi >= height - 1)
            continue;

        /* ---- erreur photométrique ---- */
        int32_t Ic = img_cur[yi * width + xi];
        int32_t e = Ic - pix_ref;          // Q0
        int32_t e_q = e << Q16_SHIFT;      // Q16.16

        /* ---- Huber : résidu tronqué ---- */
        int32_t abs_e = (e_q >= 0) ? e_q : -e_q;
        int32_t e_huber;

        if (abs_e <= huber_delta_q16) {
            e_huber = e_q;
        } else {
            e_huber = (e_q >= 0) ? huber_delta_q16 : -huber_delta_q16;
        }

        /* ---- gradient image (image courante) ---- */
        int32_t Ix =
            ((int32_t)img_cur[yi * width + xi + 1]
           - (int32_t)img_cur[yi * width + xi - 1]) << Q16_SHIFT;

        int32_t Iy =
            ((int32_t)img_cur[(yi + 1) * width + xi]
           - (int32_t)img_cur[(yi - 1) * width + xi]) << Q16_SHIFT;

        /* ---- dérivées géométriques ---- */
        int32_t dxd_div = q16_mul(dx, c) - q16_mul(dy, s);
        int32_t dyd_div = q16_mul(dx, s) + q16_mul(dy, c);

        int32_t dxd_th = -q16_mul(dxp, s) - q16_mul(dyp, c);
        int32_t dyd_th =  q16_mul(dxp, c) - q16_mul(dyp, s);

        /* ---- accumulation gradient ---- */
        g_div += (int64_t)e_huber *
                 ((int64_t)Ix * dxd_div +
                  (int64_t)Iy * dyd_div) >> Q16_SHIFT;

        g_theta += (int64_t)e_huber *
                   ((int64_t)Ix * dxd_th +
                    (int64_t)Iy * dyd_th) >> Q16_SHIFT;

        g_tx += (int64_t)e_huber * Ix;
        g_ty += (int64_t)e_huber * Iy;
    }

    g_out->div   = (int32_t)(g_div   >> Q16_SHIFT);
    g_out->theta = (int32_t)(g_theta >> Q16_SHIFT);
    g_out->tx    = (int32_t)(g_tx    >> Q16_SHIFT);
    g_out->ty    = (int32_t)(g_ty    >> Q16_SHIFT);
}
