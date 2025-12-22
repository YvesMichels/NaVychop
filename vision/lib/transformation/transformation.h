#ifndef TRANSFORMATON_H
#define TRANSFORMATON_H

#include <stdint.h>

int32_t q16_sin(int32_t theta);
int32_t q16_cos(int32_t theta);
uint32_t transform_point(uint32_t p, int32_t vp_x, int32_t vp_y, int32_t tx, int32_t ty, int32_t theta, int32_t div);
void transform_points(
    const uint32_t *points_in,
    uint32_t *points_out,
    uint16_t nb_points,
    int32_t vp_x, int32_t vp_y,
    int32_t tx, int32_t ty,
    int32_t theta,
    int32_t div
);


#endif