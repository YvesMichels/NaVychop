#ifndef IMAGE_PROCESS_H
#define IMAGE_PROCESS_H

#define JDIM    320
#define IDIM    240
#define JDIM_R  40
#define IDIM_R  30

#include <stdint.h>
// --- Calcul traitement d'images ---
void hist_cum(uint32_t *histo, uint8_t *img);
int32_t read_bilineaire(uint8_t* image, int32_t x_q, int32_t y_q, int idim, int jdim);
void gaussian_blur_15x15_stream_8bits(uint8_t *out, const uint8_t *in, int idim, int jdim);
void gaussian_blur_7x7_stream_8bits(uint8_t *out, const uint8_t *in, int idim, int jdim);
void norme_gradient_L1_8bits(uint8_t *out, const uint8_t *in, int idim, int jdim);
uint16_t extract_points(uint8_t *im_grad, uint8_t *im_gauss, uint32_t *points_ref);
uint16_t extract_points_red(uint8_t *im_gauss, uint32_t *points_ref);
void mark_points(uint8_t *im, uint32_t *points_ref, uint16_t nb_points);
void copy_image(uint8_t *out, const uint8_t *in, uint32_t n_pixels);
void red_image(uint8_t *out, const uint8_t *in);

// --- I/O / debug ---
void send_image(uint8_t *img);
void send_im_16bits_8bits(uint16_t *img);
void send_point(const uint32_t *points, uint16_t max_points);
void send_motion_data(uint8_t *img, uint16_t img_width, uint16_t img_height, 
                    uint8_t *img_r, uint16_t img_width_r, uint16_t img_height_r, 
                    uint32_t *points, uint16_t max_points, uint32_t *points_r, uint16_t max_points_r, uint32_t cam_param);

#endif