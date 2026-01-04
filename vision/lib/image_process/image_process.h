#ifndef IMAGE_PROCESS_H
#define IMAGE_PROCESS_H

#define JDIM   320
#define IDIM  240

#include <stdint.h>
// --- Calcul traitement d'images ---
void hist_cum(uint32_t *histo, uint8_t *img);
void gaussian_blur_9x9_stream_8bits(uint8_t *out, const uint8_t *in);
void gaussian_blur_15x15_stream_8bits(uint8_t *out, const uint8_t *in);
void gaussian_blur_9x9_stream_16bits(uint16_t *out, const uint8_t *in);
void norme_gradient_L1_8bits(uint8_t *out, const uint8_t *in);
uint16_t gradient_max_blocks_16x16(uint8_t *im_grad, uint8_t *im_gauss, uint32_t *points_ref);
void mark_points(uint8_t *im, uint32_t *points_ref, uint16_t nb_points);
void copy_image(uint8_t *out, const uint8_t *in, uint32_t n_pixels);

// --- I/O / debug ---
void send_image(uint8_t *img);
void send_im_16bits_8bits(uint16_t *img);
void image_ascii(uint8_t *img);
void send_point(const uint32_t *points, uint16_t max_points);
void send_motion_data(uint8_t *img, uint16_t img_width, uint16_t img_height, uint32_t *points, uint16_t max_points);

#endif