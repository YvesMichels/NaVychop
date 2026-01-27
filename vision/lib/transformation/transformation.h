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
uint32_t huber_cost_q16(uint32_t *points_ref, uint16_t nb_points, uint8_t *img_cur, uint16_t width, uint16_t height, transform_param_q16_t *p, int32_t huber_delta_q16);
uint32_t exhaustive_init(uint32_t *points_ref, uint16_t nb_points, uint8_t *img_cur, transform_param_q16_t *p_init, int32_t huber_delta_q16);
void huber_gradient_q16(uint32_t *points_ref, int nb_points, uint8_t *img_cur, int jdim, int idim, transform_param_q16_t *p, int32_t huber_delta_q16, transform_grad_q16_t *g_out);
void optim_huber_grad_q16(uint8_t *img_cur, int idim, int jdim, uint32_t *points_ref, int nb_points, transform_param_q16_t *p, int nb_iter, int32_t step0_tx, int32_t step0_ty, int32_t step0_theta, int32_t step0_div, int32_t huber_delta_q16, int32_t *logs);
#endif