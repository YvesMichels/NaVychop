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
uint16_t mark_gradient_max_blocks_16x16(const uint8_t *im_grad, uint8_t *im_gauss, uint32_t *points_ref);


// --- I/O / debug ---
void send_image(uint8_t *img);
void send_im_16bits_8bits(uint16_t *img);
void image_ascii(uint8_t *img);

#endif