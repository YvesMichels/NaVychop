/**
 * \copyright Vychop
 * \brief fonctions de traitement d'images et communication debogue et sauvegarde. 
 * Fonctions Co-écrites par Chat-GPT
 */

#include <Arduino.h>
#include "image_process.h"

#define SYNC_1      0xAA
#define SYNC_2      0x55
#define IMG_FORMAT  0x01   // mono 8 bits

#define KERNEL_RADIUS_9 4
#define KERNEL_SIZE_9 9
#define GAUSS_SHIFT 8
#define KERNEL_SIZE   15
#define KERNEL_RADIUS 7

#define BLOCK_SIZE 16
#define DEBUG_PTS 1

// constantes pour l'affichage en ascii
const int OUT_W = 160;
const int OUT_H = 40;
const int BLOCK_W = 2;
const int BLOCK_H = 6;
const char* ASCII_RAMP = " .:-=+*#%@"; // ASCII ramp du plus sombre au plus clair
const int ASCII_LEN = 10;

// constante pour le filtrage gaussien (somme = 256 = 2^GAUSS_SHIFT)
static const uint16_t gauss9[9] = {
     1,  8, 28, 56, 70, 56, 28,  8,  1
};
static const uint8_t gauss15[15] = {
     0,  0,  1,  6, 16, 31, 47, 54,
    47, 31, 16,  6,  1,  0,  0
};
// row_buf: buffer intermédiaire 9 lignes (≈5.6 kB)
// Utilisé uniquement pour convolution séparable
//static uint16_t row_buf[KERNEL_SIZE_9][JDIM];
static uint16_t row_buf[KERNEL_SIZE][JDIM];

// --- Calcul traitement d'images ---
void hist_cum(uint32_t *histo, uint8_t *img){
    
  uint8_t max_lum;
      // initialise l'histogramme
  for(int i = 0; i < 256; i++){
    histo[i] = 0;
  }
  // calcul l'histogramme et le max
  for (int i = 0; i < IDIM; i++) {
    for (int j = 0; j < JDIM; j++) {
      histo[img[i * JDIM + j]] += 1;
    }
  }
  // cumule l'histogramme et cherche le quantile 0.99
  max_lum = 0;
  for(int i = 1; i < 256; i++){
    histo[i] = histo[i-1]+histo[i];
    if(histo[i]>0){
      max_lum = i;
    }
  }
}

static void gaussian9_horizontal_line_8bits(
    uint16_t *out,
    const uint8_t *in_line
)
{
    for (int j = 0; j < JDIM; j++) {
        uint32_t acc = 0;

        for (int k = 0; k < KERNEL_SIZE_9; k++) {
            int jj = j + k - KERNEL_RADIUS_9;

            if (jj < 0)          jj = 0;
            else if (jj >= JDIM) jj = JDIM - 1;

            acc += gauss9[k] * in_line[jj];
        }

        out[j] = (uint16_t)acc;   // Q8.8
    }
}

static void gaussian9_vertical_from_rows_8bits(
    uint8_t *out_line,
    uint16_t rows[KERNEL_SIZE][JDIM],
    int ind_ker
)
{
    for (int j = 0; j < JDIM; j++) {
        uint32_t acc = 0;

        for (int k = 0; k < KERNEL_SIZE_9; k++) {
            int idx = k - ind_ker;
            if (idx < 0) idx += KERNEL_SIZE_9;

            acc += gauss9[idx] * rows[k][j];
        }

        out_line[j] = (uint8_t)(acc >> 16); // Q16.16 → 8 bits
    }
}

void gaussian_blur_9x9_stream_8bits(
    uint8_t *out,
    const uint8_t *in
)
{
    /* === Pré-remplissage vertical correct (clamp haut) === */
    for (int k = 0; k < KERNEL_SIZE_9; k++) {

        int src_i = k - KERNEL_RADIUS_9;
        if (src_i < 0)        src_i = 0;
        if (src_i >= IDIM)    src_i = IDIM - 1;

        gaussian9_horizontal_line_8bits(
            row_buf[k],
            &in[src_i * JDIM]
        );
    }

    /* === Boucle principale streaming === */
    int ind_ker = KERNEL_RADIUS_9;

    for (int i = 0; i < IDIM; i++) {

        /* Convolution verticale centrée sur la ligne i */
        gaussian9_vertical_from_rows_8bits(
            &out[i * JDIM],
            row_buf,
            ind_ker
        );

        /* Préparation de la prochaine ligne horizontale */
        int next_i = i + KERNEL_RADIUS_9 + 1;
        if (next_i >= IDIM)
            next_i = IDIM - 1;

        ind_ker = (ind_ker + 1) % KERNEL_SIZE_9;

        gaussian9_horizontal_line_8bits(
            row_buf[ind_ker],
            &in[next_i * JDIM]
        );
    }
}


static void gaussian15_horizontal_line_8bits(
    uint16_t *out,
    const uint8_t *in_line
)
{
    for (int j = 0; j < JDIM; j++) {
        uint32_t acc = 0;

        for (int k = 0; k < KERNEL_SIZE; k++) {
            int jj = j + k - KERNEL_RADIUS;

            if (jj < 0)        jj = 0;
            else if (jj >= JDIM) jj = JDIM - 1;

            acc += gauss15[k] * in_line[jj];
        }

        out[j] = (uint16_t)acc;   // Q8.8
    }
}

static void gaussian15_vertical_from_rows_8bits(
    uint8_t *out_line,
    uint16_t rows[KERNEL_SIZE][JDIM],
    int ind_ker
)
{
    for (int j = 0; j < JDIM; j++) {
        uint32_t acc = 0;

        for (int k = 0; k < KERNEL_SIZE; k++) {
            int idx = k - ind_ker;
            if (idx < 0) idx += KERNEL_SIZE;

            acc += gauss15[idx] * rows[k][j];
        }

        out_line[j] = (uint8_t)(acc >> 16); // Q16.16 → 8 bits
    }
}

void gaussian_blur_15x15_stream_8bits(uint8_t *out, const uint8_t *in)
{
    // Remplissage initial : clamp vertical
for (int k = 0; k < KERNEL_SIZE; k++) {
    int src_i = k - KERNEL_RADIUS;
    if (src_i < 0) src_i = 0;
    if (src_i >= IDIM) src_i = IDIM - 1;

    gaussian15_horizontal_line_8bits(
        row_buf[k],
        &in[src_i * JDIM]
    );
}
    gaussian15_vertical_from_rows_8bits(
        &out[0],
        row_buf,
        0
    );

    /* Boucle principale */
    int ind_ker = KERNEL_RADIUS;

for (int i = 0; i < IDIM; i++) {

    gaussian15_vertical_from_rows_8bits(
        &out[i * JDIM],
        row_buf,
        ind_ker
    );

    // Préparer la prochaine ligne
    int next_i = i + KERNEL_RADIUS + 1;
    if (next_i >= IDIM) next_i = IDIM - 1;

    ind_ker = (ind_ker + 1) % KERNEL_SIZE;

    gaussian15_horizontal_line_8bits(
        row_buf[ind_ker],
        &in[next_i * JDIM]
    );
}

    /* Bords bas */
    for (int i = IDIM - KERNEL_RADIUS; i < IDIM; i++) {
        gaussian15_vertical_from_rows_8bits(
            &out[i * JDIM],
            row_buf,
            ind_ker
        );
    }
}


static void gaussian_horizontal_line_16bits(uint16_t *out, const uint8_t *in_line)
{
    for (int j = 0; j < JDIM; j++) {
        uint32_t acc = 0;

        for (int k = -KERNEL_RADIUS_9; k <= KERNEL_RADIUS_9; k++) {
            int jj = j + k;
            if (jj < 0) jj = 0;
            else if (jj >= JDIM) jj = JDIM - 1;

            acc += gauss9[k + KERNEL_RADIUS_9] * in_line[jj];
        }
        out[j] = (uint16_t)acc;
    }
}

static void gaussian_vertical_from_rows_16bits(uint16_t *out_line, uint16_t rows[KERNEL_SIZE_9][JDIM])
{
    for (int j = 0; j < JDIM; j++) {
        uint32_t acc = 0;

        for (int k = 0; k < KERNEL_SIZE_9; k++) {
            acc += gauss9[k] * rows[k][j];
        }
        // Normalisation totale : 256 (pour appliquer un facteur 256 sur les niveaux de gris de l'image convoluée par raport à l'image d'entrée)
        out_line[j] = acc >> GAUSS_SHIFT;
    }
}

void gaussian_blur_9x9_stream_16bits(uint16_t *out, const uint8_t *in)
{
    // Pré-remplissage (bords haut)
    for (int i = 0; i < KERNEL_RADIUS_9; i++) {
        gaussian_horizontal_line_16bits(row_buf[i], &in[0]);
    }

    int buf_idx = KERNEL_RADIUS_9;

    for (int i = 0; i < IDIM; i++) {

        int src_i = i + KERNEL_RADIUS_9;
        if (src_i >= IDIM) src_i = IDIM - 1;

        gaussian_horizontal_line_16bits(
            row_buf[buf_idx],
            &in[src_i * JDIM]
        );

        if (i >= KERNEL_RADIUS_9) {
            gaussian_vertical_from_rows_16bits(
                &out[(i - KERNEL_RADIUS_9) * JDIM],
                row_buf
            );
        }

        buf_idx = (buf_idx + 1) % KERNEL_SIZE_9;
    }

    // Bords bas
    for (int i = IDIM - KERNEL_RADIUS_9; i < IDIM; i++) {
        gaussian_vertical_from_rows_16bits(
            &out[i * JDIM],
            row_buf
        );
    }
}

static inline uint8_t abs8(int x)
{
    return (x < 0) ? -x : x;
}

void norme_gradient_L1_8bits(uint8_t *out, const uint8_t *in)
{
    for (int i = 0; i < IDIM; i++) {

        int im = (i > 0) ? i - 1 : 0;
        int ip = (i < IDIM - 1) ? i + 1 : IDIM - 1;

        for (int j = 0; j < JDIM; j++) {

            int jm = (j > 0) ? j - 1 : 0;
            int jp = (j < JDIM - 1) ? j + 1 : JDIM - 1;

            int dx = (int)in[i * JDIM + jp] - (int)in[i * JDIM + jm];
            int dy = (int)in[ip * JDIM + j] - (int)in[im * JDIM + j];

            uint16_t g = abs8(dx) + abs8(dy);

            if (g > 255) g = 255;

            out[i * JDIM + j] = (uint8_t)g;
        }
    }
}

static inline void draw_cross_3x3(
    uint8_t *img,
    int x,
    int y
)
{
    if (x <= 0 || x >= JDIM - 1 || y <= 0 || y >= IDIM - 1)
        return;

    img[y * JDIM + x] = 255;
    img[y * JDIM + (x - 1)] = 255;
    img[y * JDIM + (x + 1)] = 255;
    img[(y - 1) * JDIM + x] = 255;
    img[(y + 1) * JDIM + x] = 255;
}

uint16_t mark_gradient_max_blocks_16x16(
    const uint8_t *im_grad,
    uint8_t *im_gauss, 
    uint32_t *points_ref
)
{
    uint16_t n_points = 0;
    uint8_t pix_val;
    uint8_t grad_MSB;
    uint32_t word;

    for (int by = 0; by < IDIM; by += BLOCK_SIZE) {
        for (int bx = 0; bx < JDIM; bx += BLOCK_SIZE) {

            uint8_t max_val = 0;
            int max_x = bx;
            int max_y = by;

            /* Recherche du maximum dans le bloc */
            for (int y = 0; y < BLOCK_SIZE; y++) {
                int iy = by + y;
                if (iy >= IDIM) break;

                for (int x = 0; x < BLOCK_SIZE; x++) {
                    int ix = bx + x;
                    if (ix >= JDIM) break;

                    uint8_t v = im_grad[iy * JDIM + ix];
                    if (v > max_val) {
                        max_val = v;
                        max_x = ix;
                        max_y = iy;
                    }
                }
            }
            /* Encodage dans 32 bits : x:9 + y:9 + pixel:8 + gradMSB:6 */
            pix_val = im_gauss[max_y * JDIM + max_x];
            grad_MSB = max_val >> 2;  // 6 MSB

            word = ((max_y & 0x1FF) << 23)
                          | ((max_x & 0x1FF) << 14)
                          | ((pix_val & 0xFF) << 6)
                          | (grad_MSB & 0x3F);

            points_ref[n_points++] = word;

#if DEBUG_PTS
        if(max_val>20) {
            draw_cross_3x3(im_gauss, max_x, max_y);
        }
#endif
        }
    }
    return n_points;
}


// --- I/O / debug ---
void send_image(uint8_t *img)
{
    uint8_t checksum = 0;

    // Synchronisation
    Serial.write(SYNC_1);
    Serial.write(SYNC_2);

    // Dimensions
    Serial.write((uint8_t)(JDIM & 0xFF));
    Serial.write((uint8_t)(JDIM >> 8));
    Serial.write((uint8_t)(IDIM & 0xFF));
    Serial.write((uint8_t)(IDIM >> 8));

    // Format
    Serial.write(IMG_FORMAT);

    // Image
    for (uint32_t i = 0; i < JDIM * IDIM; i++) {
        uint8_t v = img[i];
        Serial.write(v);
        checksum ^= v;
    }

    // Checksum
    Serial.write(checksum);
}

void send_im_16bits_8bits(uint16_t *img)
{
    uint8_t checksum = 0;

    // Synchronisation
    Serial.write(SYNC_1);
    Serial.write(SYNC_2);

    // Dimensions
    Serial.write((uint8_t)(JDIM & 0xFF));
    Serial.write((uint8_t)(JDIM >> 8));
    Serial.write((uint8_t)(IDIM & 0xFF));
    Serial.write((uint8_t)(IDIM >> 8));

    // Format
    Serial.write(IMG_FORMAT);

    // Image
    for (uint32_t i = 0; i < JDIM * IDIM; i++) {
        uint8_t v = (uint8_t)(img[i] >> GAUSS_SHIFT);
        Serial.write(v);
        checksum ^= v;
    }

    // Checksum
    Serial.write(checksum);
}

void image_ascii(uint8_t *img)
{
  // Parcours des blocs 8×8
  for (int by = 0; by < OUT_H; by++) {
    for (int bx = 0; bx < OUT_W; bx++) {

      int sum = 0;

      // Moyenne du bloc 8×8
      for (int dy = 0; dy < BLOCK_H; dy++) {
        for (int dx = 0; dx < BLOCK_W; dx++) {
          int x = bx * BLOCK_W + dx;
          int y = by * BLOCK_H + dy;
          sum += img[y * 320 + x];
        }
      }

      int avg = sum / (BLOCK_W * BLOCK_H); // valeur 0-255

      // mapping vers caractères
      int idx = (avg * (ASCII_LEN - 1)) / 255;
      Serial.print(ASCII_RAMP[idx]);
    }
    Serial.println();
  }
}