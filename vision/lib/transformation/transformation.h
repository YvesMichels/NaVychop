#ifndef TRANSFORMATON_H
#define TRANSFORMATON_H

#include <stdint.h>

/* Paramètres de transformation (Q16.16) */
typedef struct {
    int32_t div;
    int32_t theta;
    int32_t tx;
    int32_t ty;
    int32_t vp_x;
    int32_t vp_y;
} transform_param_q16_t;
/* Gradient des paramètres de transformation (Q16.16) */
typedef struct {
    int32_t div;
    int32_t theta;
    int32_t tx;
    int32_t ty;
} transform_grad_q16_t;


int32_t q16_sin(int32_t theta);
int32_t q16_cos(int32_t theta);
uint32_t transform_point(uint32_t p, int32_t vp_x, int32_t vp_y, int32_t tx, int32_t ty, int32_t theta, int32_t div);
void transform_points(uint32_t *points_in, uint32_t *points_out, uint16_t nb_points, int32_t vp_x, int32_t vp_y, int32_t tx, int32_t ty, int32_t theta, int32_t div);
void ssd_huber_gradient_q16(const uint32_t *points_ref, int nb_points, const uint8_t *img_cur, int width, int height, const transform_param_q16_t *p, int32_t huber_delta_q16, transform_grad_q16_t *g_out);
#endif