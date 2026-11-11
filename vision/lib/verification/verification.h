#include <stdint.h>

void verification_q16_sin(uint32_t *points_trans, uint16_t nb_points);
void verification_tranformation(uint32_t *points_ref, uint32_t *points_trans, uint16_t nb_points);
void verification_interpolation(uint8_t* image, int idim, int jdim);