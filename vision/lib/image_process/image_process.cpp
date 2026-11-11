/**
 * \copyright Vychop
 * \brief fonctions de traitement d'images et communication debogue et sauvegarde. 
 * Fonctions Co-écrites par Chat-GPT
 */

#include <Arduino.h>
#include "image_process.h"

#define SYNC_1      0xAA
#define SYNC_2      0x55
#define SYNC_3      0xCC
#define SYNC_4      0x33
#define IMG_FORMAT      0x01   // mono 8 bits
#define POINT_FORMAT    0x04   // mono 32 bits
#define META_FORMAT     0x04   // mono 32 bits

#define KERNEL_RADIUS_9 4
#define KERNEL_SIZE_9 9
#define GAUSS_SHIFT 8
#define KERNEL_SIZE   15
#define KERNEL_RADIUS 7

#define EDGE_SIZE 16
#define BLOCK_SIZE 16
#define EDGE_SIZE_R 8
#define BLOCK_SIZE_R 4

#define Q16_SHIFT   16
#define Q16_ONE     (1 << Q16_SHIFT)

#define FACT_R 8
#define R_SHIFT 6 // décalage de bits pour difisser par FACT_R^2

/* 
    Variable de test
*/
#define DEBUG_PTS 0
#define GAUSSIAN_OPTI 1

// constante pour le filtrage gaussien sigma = 4 pixels (somme = 256 = 2^GAUSS_SHIFT)
static const uint8_t gauss15[15] = {2, 4, 8, 16, 21, 28, 34, 36, 34, 28, 21, 13, 8, 4, 2};

// row_buf: buffer intermédiaire 9 lignes (≈5.6 kB)
// Utilisé uniquement pour convolution séparable
//static uint16_t row_buf[KERNEL_SIZE_9][JDIM];
static uint8_t row_buf[KERNEL_SIZE][JDIM];

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


int32_t read_bilineaire(
    uint8_t* image,
    int32_t x_q,
    int32_t y_q,
    int idim,
    int jdim
)
{
    // Partie entière
    int32_t x0 = x_q >> Q16_SHIFT;
    int32_t y0 = y_q >> Q16_SHIFT;

    if (x0 < 0 || x0 >= jdim - 1 ||
        y0 < 0 || y0 >= idim - 1)
    {
        return 0;
    }

    // Fractionnaires ramenés en Q0.8
    int32_t fx  = (x_q >> 8) & 0xFF;   // Q0.8
    int32_t fy  = (y_q >> 8) & 0xFF;   // Q0.8
    int32_t fx1 = 256 - fx; // Q0.8
    int32_t fy1 = 256 - fy; // Q0.8

    // Pixels (Q8.0)
    int32_t I00 = image[y0 * jdim + x0];
    int32_t I10 = image[y0 * jdim + (x0 + 1)];
    int32_t I01 = image[(y0 + 1) * jdim + x0];
    int32_t I11 = image[(y0 + 1) * jdim + (x0 + 1)];

    // Interpolation X (Q8.0 × Q0.8 → Q8.8)
    int32_t i0 = I00 * fx1 + I10 * fx; // Q8.8
    int32_t i1 = I01 * fx1 + I11 * fx; // Q8.8

    // Interpolation Y (Q8.8 × Q8.8 → Q16.16)
    int32_t acc = i0 * fy1 + i1 * fy;  // Q16.16

    // Normalisation finale → Q16.16
    return acc;
}



static void gaussian15_horizontal_line_8bits(uint16_t *out, const uint8_t *in_line, int jdim)
{
    /* Zone valide */
    for (int j = 0; j <= jdim - KERNEL_SIZE; j++) {
        uint32_t acc = 0;

        for (int k = 0; k < KERNEL_SIZE; k++) {
            acc += gauss15[k] * in_line[j + k];
        }
        out[j] = (uint16_t)acc;  // Q8.8
    }
}


static void gaussian15_vertical_from_rows_8bits(uint8_t *out_line, uint16_t rows[KERNEL_SIZE][JDIM], int ind_ker, int idim, int jdim)
{
    /* Zone valide */
    for (int j = 0; j <= jdim - KERNEL_SIZE; j++) {
        uint32_t acc = 0;

        for (int k = 0; k < KERNEL_SIZE; k++) {
            int idx = k - ind_ker;
            if (idx < 0) idx += KERNEL_SIZE;

            acc += gauss15[idx] * rows[k][j];
        }

        out_line[j] = (uint8_t)(acc >> 16); // Q16.16 → Q8.0
    }

    /* Partie droite forcée à 0 */
    for (int j = jdim - KERNEL_SIZE + 1; j < jdim; j++) {
        out_line[j] = 0;
    }
}

static void gaussian15_horizontal_line_8bits_opti(uint8_t *out, const uint8_t *in_line, int jdim)
{
    for (int j = 0; j <= jdim - KERNEL_SIZE; j++) {
        uint32_t acc = 0;
        acc +=  2 * in_line[j +  0];
        acc +=  4 * in_line[j +  1];
        acc +=  8 * in_line[j +  2];
        acc += 13 * in_line[j +  3];
        acc += 21 * in_line[j +  4];
        acc += 28 * in_line[j +  5];
        acc += 34 * in_line[j +  6];
        acc += 36 * in_line[j +  7];
        acc += 34 * in_line[j +  8];
        acc += 28 * in_line[j +  9];
        acc += 21 * in_line[j + 10];
        acc += 13 * in_line[j + 11];
        acc +=  8 * in_line[j + 12];
        acc +=  4 * in_line[j + 13];
        acc +=  2 * in_line[j + 14];

        out[j] = (uint8_t)(acc >> 8);
    }
}

static void gaussian15_vertical_from_rows_8bits_opti(uint8_t *out_line, uint8_t rows[KERNEL_SIZE][JDIM], int ind_ker, int idim, int jdim)
{
    for (int j = 0; j <= jdim - KERNEL_SIZE; j++) {
        uint32_t acc = 0;
        acc +=  2 * rows[( ind_ker + KERNEL_SIZE) % KERNEL_SIZE ][j];
        acc +=  4 * rows[( 1 + ind_ker + KERNEL_SIZE) % KERNEL_SIZE ][j];
        acc +=  8 * rows[( 2 + ind_ker + KERNEL_SIZE) % KERNEL_SIZE ][j];
        acc +=  13 * rows[( 3 + ind_ker + KERNEL_SIZE) % KERNEL_SIZE ][j];
        acc +=  21 * rows[( 4 + ind_ker + KERNEL_SIZE) % KERNEL_SIZE ][j];
        acc +=  28 * rows[( 5 + ind_ker + KERNEL_SIZE) % KERNEL_SIZE ][j];
        acc +=  34 * rows[( 6 + ind_ker + KERNEL_SIZE) % KERNEL_SIZE ][j];
        acc +=  36 * rows[( 7 + ind_ker + KERNEL_SIZE) % KERNEL_SIZE ][j];
        acc +=  34 * rows[( 8 + ind_ker + KERNEL_SIZE) % KERNEL_SIZE ][j];
        acc +=  28 * rows[( 9 + ind_ker + KERNEL_SIZE) % KERNEL_SIZE ][j];
        acc +=  21 * rows[( 10 + ind_ker + KERNEL_SIZE) % KERNEL_SIZE ][j];
        acc +=  13 * rows[( 11 + ind_ker + KERNEL_SIZE) % KERNEL_SIZE ][j];
        acc +=  8 * rows[( 12 + ind_ker + KERNEL_SIZE) % KERNEL_SIZE ][j];
        acc +=  4 * rows[( 13 + ind_ker + KERNEL_SIZE) % KERNEL_SIZE ][j];
        acc +=  2 * rows[( 14 + ind_ker + KERNEL_SIZE) % KERNEL_SIZE ][j];

        out_line[j] = (uint8_t)(acc >> 8);
    }

    for (int j = jdim - KERNEL_SIZE + 1; j < jdim; j++) {
        out_line[j] = 0;
    }
}

/* sigma = 4 */
void gaussian_blur_15x15_stream_8bits(uint8_t *out, const uint8_t *in, int idim, int jdim)
{
    /* Préchargement exact des KERNEL_SIZE premières lignes */
    for (int k = 0; k < KERNEL_SIZE; k++) {
        #if GAUSSIAN_OPTI
            gaussian15_horizontal_line_8bits_opti(row_buf[k], &in[k * jdim], jdim);
        #else
            gaussian15_horizontal_line_8bits(row_buf[k], &in[k * jdim], jdim);
        #endif
    }

    int ind_ker = 0;

    /* Boucle principale — uniquement zone valide */
    for (int i = 0; i <= idim - KERNEL_SIZE; i++) {
        #if GAUSSIAN_OPTI
            gaussian15_vertical_from_rows_8bits_opti(&out[i * jdim], row_buf, ind_ker, idim, jdim);
        #else
            gaussian15_vertical_from_rows_8bits(&out[i * jdim], row_buf, ind_ker, idim, jdim);
        #endif
        /* Charger la prochaine ligne exacte */
        if (i + KERNEL_SIZE < idim) {
            #if GAUSSIAN_OPTI
                gaussian15_horizontal_line_8bits_opti(row_buf[ind_ker],  &in[(i + KERNEL_SIZE) * jdim], jdim);
            #else
                gaussian15_horizontal_line_8bits(row_buf[ind_ker],  &in[(i + KERNEL_SIZE) * jdim], jdim);
            #endif
        }

        ind_ker = (ind_ker + 1) % KERNEL_SIZE;
    }
    for (int i = idim - KERNEL_SIZE + 1; i < idim; i++) {
        for (int j = 0; j < jdim; j++) {
            out[i * jdim + j] = 0;
        }
    }
}

/* sigma = 3 */
static void gaussian9_horizontal_line_8bits_opti(uint8_t *out, const uint8_t *in_line, int jdim)
{
    for (int j = 0; j <= jdim - 9; j++) {
        uint32_t acc = 0;
        acc +=  8 * in_line[j +  0];
        acc += 18 * in_line[j +  1];
        acc += 32 * in_line[j +  2];
        acc += 45 * in_line[j +  3];
        acc += 50 * in_line[j +  4];
        acc += 45 * in_line[j +  5];
        acc += 32 * in_line[j +  6];
        acc += 18 * in_line[j +  7];
        acc +=  8 * in_line[j +  8];

        out[j] = (uint16_t)(acc >> 8);
    }
}

/* sigma = 3 */
static void gaussian9_vertical_from_rows_8bits_opti(uint8_t *out_line, uint8_t rows[KERNEL_SIZE][JDIM], int ind_ker, int idim, int jdim)
{
    for (int j = 0; j <= jdim - 9; j++) {
        uint32_t acc = 0;
        acc +=  8 * rows[( ind_ker + 9) % 9 ][j];
        acc += 18 * rows[( 1 + ind_ker + 9) % 9 ][j];
        acc += 32 * rows[( 2 + ind_ker + 9) % 9 ][j];
        acc += 45 * rows[( 3 + ind_ker + 9) % 9 ][j];
        acc += 50 * rows[( 4 + ind_ker + 9) % 9 ][j];
        acc += 45 * rows[( 5 + ind_ker + 9) % 9 ][j];
        acc += 32 * rows[( 6 + ind_ker + 9) % 9 ][j];
        acc += 18 * rows[( 7 + ind_ker + 9) % 9 ][j];
        acc +=  8 * rows[( 8 + ind_ker + 9) % 9 ][j];

        out_line[j] = (uint8_t)(acc >> 8);
    }

    for (int j = jdim - 9 + 1; j < jdim; j++) {
        out_line[j] = 0;
    }
}

/* sigma = 3 */
void gaussian_blur_9x9_stream_8bits(uint8_t *out, const uint8_t *in, int idim, int jdim)
{
    /* Préchargement exact des 7 premières lignes */
    for (int k = 0; k < 9; k++) {
        gaussian9_horizontal_line_8bits_opti(row_buf[k], &in[k * jdim], jdim);
    }

    int ind_ker = 0;

    /* Boucle principale — uniquement zone valide */
    for (int i = 0; i <= idim - 9; i++) {
        gaussian9_vertical_from_rows_8bits_opti(&out[i * jdim], row_buf, ind_ker, idim, jdim);
        /* Charger la prochaine ligne exacte */
        if (i + 9 < idim) {
            gaussian9_horizontal_line_8bits_opti(row_buf[ind_ker],  &in[(i + 9) * jdim], jdim);
        }

        ind_ker = (ind_ker + 1) % 9;
    }
    for (int i = idim - 9 + 1; i < idim; i++) {
        for (int j = 0; j < jdim; j++) {
            out[i * jdim + j] = 0;
        }
    }
}

/* sigma = 2 */
static void gaussian7_horizontal_line_8bits_opti(uint8_t *out, const uint8_t *in_line, int jdim)
{
    for (int j = 0; j <= jdim - 7; j++) {
        uint32_t acc = 0;
        acc +=  8 * in_line[j +  0];
        acc += 27 * in_line[j +  1];
        acc += 57 * in_line[j +  2];
        acc += 72 * in_line[j +  3];
        acc += 57 * in_line[j +  4];
        acc += 27 * in_line[j +  5];
        acc +=  8 * in_line[j +  6];

        out[j] = (uint16_t)(acc >> 8);
    }
}

/* sigma = 2 */
static void gaussian7_vertical_from_rows_8bits_opti(uint8_t *out_line, uint8_t rows[KERNEL_SIZE][JDIM], int ind_ker, int idim, int jdim)
{
    for (int j = 0; j <= jdim - 7; j++) {
        uint32_t acc = 0;
        acc +=   8 * rows[( ind_ker + 7) % 7 ][j];
        acc += 27 * rows[( 1 + ind_ker + 7) % 7 ][j];
        acc += 57 * rows[( 2 + ind_ker + 7) % 7 ][j];
        acc += 72 * rows[( 3 + ind_ker + 7) % 7 ][j];
        acc += 57 * rows[( 4 + ind_ker + 7) % 7 ][j];
        acc += 27 * rows[( 5 + ind_ker + 7) % 7 ][j];
        acc +=  8 * rows[( 6 + ind_ker + 7) % 7 ][j];

        out_line[j] = (uint8_t)(acc >> 8);
    }

    for (int j = jdim - 7 + 1; j < jdim; j++) {
        out_line[j] = 0;
    }
}

/* sigma = 2 */
void gaussian_blur_7x7_stream_8bits(uint8_t *out, const uint8_t *in, int idim, int jdim)
{
    /* Préchargement exact des 7 premières lignes */
    for (int k = 0; k < 7; k++) {
        gaussian7_horizontal_line_8bits_opti(row_buf[k], &in[k * jdim], jdim);
    }

    int ind_ker = 0;

    /* Boucle principale — uniquement zone valide */
    for (int i = 0; i <= idim - 7; i++) {
        gaussian7_vertical_from_rows_8bits_opti(&out[i * jdim], row_buf, ind_ker, idim, jdim);
        /* Charger la prochaine ligne exacte */
        if (i + 7 < idim) {
            gaussian7_horizontal_line_8bits_opti(row_buf[ind_ker],  &in[(i + 7) * jdim], jdim);
        }

        ind_ker = (ind_ker + 1) % 7;
    }
    for (int i = idim - 7 + 1; i < idim; i++) {
        for (int j = 0; j < jdim; j++) {
            out[i * jdim + j] = 0;
        }
    }
}

static inline uint8_t abs8(int x)
{
    return (x < 0) ? -x : x;
}

void norme_gradient_L1_8bits(uint8_t *out, const uint8_t *in, int idim, int jdim)
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

uint16_t extract_points(uint8_t *im_grad, uint8_t *im_gauss, uint32_t *points_ref)
{
    uint16_t n_points = 0;
    uint8_t pix_val;
    uint8_t grad_MSB;
    uint32_t word;

    for (int by = 0; by < IDIM-EDGE_SIZE; by += BLOCK_SIZE) {
        for (int bx = 0; bx < JDIM-EDGE_SIZE; bx += BLOCK_SIZE) {

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
            grad_MSB = max_val; // >> 2;  // 6 LSB car gradient sur filtre 15*15 < 64
            if(grad_MSB>63)
            {
                grad_MSB = 63;
            }
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

uint16_t extract_points_red(uint8_t *im_gauss, uint32_t *points_ref)
{
    uint16_t n_points = 0;
    uint8_t pix_val;
    uint32_t word;

    for (int by = 0; by <= IDIM_R-EDGE_SIZE_R; by += 2) {
        for (int bx = 0; bx <= JDIM_R-EDGE_SIZE_R; bx += 2) {
            /* Encodage dans 32 bits : x:9 + y:9 + pixel:8 + gradMSB:6 */
            pix_val = im_gauss[by * JDIM_R + bx];

            word = ((by & 0x1FF) << 23)
                          | ((bx & 0x1FF) << 14)
                          | ((pix_val & 0xFF) << 6)
                          | (0 & 0x3F);
            points_ref[n_points++] = word;
        }
    }
    return n_points;
}

void mark_points(uint8_t *im, uint32_t *points_ref, uint16_t nb_points){
    int32_t y;
    int32_t x;
    uint8_t grad;
    for (uint16_t i = 0; i < nb_points; i++) {
        // Extraction
        y    = (points_ref[i] >> 23) & 0x1FF;  // 9 bits
        x    = (points_ref[i] >> 14) & 0x1FF;  // 9 bits
        //pix_val  = (points_ref[i] >> 6)  & 0xFF;   // 8 bits
        grad =  points_ref[i]        & 0x3F;   // 6 bits

        if(grad>5) {
            draw_cross_3x3(im, x, y);
        }
    }
}

void copy_image(uint8_t *out, const uint8_t *in, uint32_t n_pixels)
{
    for (uint32_t i = 0; i < n_pixels; i++)
    {
        out[i] = in[i];
    }
}

void red_image(uint8_t *out, const uint8_t *in)
{
  // Parcours des blocs 8×8
  for (int by = 0; by < IDIM_R; by++) {
    for (int bx = 0; bx < JDIM_R; bx++) {

      int sum = 0;

      // Moyenne du bloc 8×8
      for (int dy = 0; dy < FACT_R; dy++) {
        for (int dx = 0; dx < FACT_R; dx++) {
          int x = bx * FACT_R + dx;
          int y = by * FACT_R + dy;
          sum += in[y * JDIM + x];
        }
      }

      int avg = sum >> R_SHIFT; 
      out[by * JDIM_R + bx] = avg;
    }
  }
}


void gaussian_blur_15x15_stream_8bits_band(
    uint8_t *out, const uint8_t *in,
    int idim, int jdim,
    int i_start, int i_end,
    uint8_t row_buf[KERNEL_SIZE][JDIM])
{
    int preload_start = i_start;
    int preload_end   = i_start + KERNEL_SIZE - 1;

    // --- Préchargement exact ---
    for (int k = 0; k < KERNEL_SIZE; k++) {
        int src_i = preload_start + k;
        if (src_i < idim) {
            gaussian15_horizontal_line_8bits_opti(
                row_buf[k],
                &in[src_i * jdim],
                jdim
            );
        } else {
            memset(row_buf[k], 0, jdim * sizeof(uint16_t));
        }
    }

    int ind_ker = 0;

    // --- Boucle principale ---
    for (int i = i_start; i <= i_end; i++) {

        gaussian15_vertical_from_rows_8bits_opti(
            &out[i * jdim],
            row_buf,
            ind_ker,
            idim,
            jdim
        );

        int next_line = i + KERNEL_SIZE;
        if (next_line < idim) {
            gaussian15_horizontal_line_8bits_opti(
                row_buf[ind_ker],
                &in[next_line * jdim],
                jdim
            );
        } else {
            memset(row_buf[ind_ker], 0, jdim * sizeof(uint16_t));
        }

        ind_ker = (ind_ker + 1) % KERNEL_SIZE;
    }
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


void send_point(const uint32_t *points, uint16_t max_points)
{
    uint8_t checksum = 0;

    // Synchronisation
    Serial.write(SYNC_1);
    Serial.write(SYNC_2);

    // Nombre de points (16 bits, LSB first)
    Serial.write((uint8_t)(max_points & 0xFF));
    Serial.write((uint8_t)(max_points >> 8));

    // Format (optionnel mais recommandé)
    Serial.write(POINT_FORMAT);

    // Payload : MAX_POINTS mots de 32 bits
    for (uint32_t i = 0; i < max_points; i++) {
        uint32_t p = points[i];

        // Envoi LSB -> MSB
        uint8_t b0 = (uint8_t)( p        & 0xFF);
        uint8_t b1 = (uint8_t)((p >> 8 ) & 0xFF);
        uint8_t b2 = (uint8_t)((p >> 16) & 0xFF);
        uint8_t b3 = (uint8_t)((p >> 24) & 0xFF);

        Serial.write(b0); checksum ^= b0;
        Serial.write(b1); checksum ^= b1;
        Serial.write(b2); checksum ^= b2;
        Serial.write(b3); checksum ^= b3;
    }

    // Checksum final
    Serial.write(checksum);
}

void send_motion_data(uint8_t *img, uint16_t img_width, uint16_t img_height, 
                    uint8_t *img_r, uint16_t img_width_r, uint16_t img_height_r, 
                    uint32_t *points, uint16_t max_points, uint32_t *points_r, uint16_t max_points_r, uint32_t cam_param)
{
    uint8_t checksum = 0;
    uint32_t img_size = img_width * img_height;

    // --- Synchronisation 4 octets ---
    Serial.write(SYNC_1);
    Serial.write(SYNC_2);
    Serial.write(SYNC_3);
    Serial.write(SYNC_4);

    // --- Image ---
    Serial.write((uint8_t)(img_width & 0xFF));
    Serial.write((uint8_t)(img_width >> 8));
    Serial.write((uint8_t)(img_height & 0xFF));
    Serial.write((uint8_t)(img_height >> 8));
    Serial.write(IMG_FORMAT);

    for (uint32_t i = 0; i < img_size; i++) {
        uint8_t v = img[i];
        Serial.write(v);
        checksum ^= v;
    }
    // --- Image Réduite ---
    Serial.write((uint8_t)(img_width_r & 0xFF));
    Serial.write((uint8_t)(img_width_r >> 8));
    Serial.write((uint8_t)(img_height_r & 0xFF));
    Serial.write((uint8_t)(img_height_r >> 8));
    Serial.write(IMG_FORMAT);
    img_size = img_width_r * img_height_r;
    for (uint32_t i = 0; i < img_size; i++) {
        uint8_t v = img_r[i];
        Serial.write(v);
        checksum ^= v;
    }
    // --- Points ---
    Serial.write((uint8_t)(max_points & 0xFF));
    Serial.write((uint8_t)(max_points >> 8));
    Serial.write(POINT_FORMAT);

    for (uint16_t i = 0; i < max_points; i++) {
        uint32_t p = points[i];
        uint8_t b0 = (uint8_t)(p & 0xFF);
        uint8_t b1 = (uint8_t)((p >> 8) & 0xFF);
        uint8_t b2 = (uint8_t)((p >> 16) & 0xFF);
        uint8_t b3 = (uint8_t)((p >> 24) & 0xFF);

        Serial.write(b0); checksum ^= b0;
        Serial.write(b1); checksum ^= b1;
        Serial.write(b2); checksum ^= b2;
        Serial.write(b3); checksum ^= b3;
    }
    // --- Points Réduit ---
    Serial.write((uint8_t)(max_points_r & 0xFF));
    Serial.write((uint8_t)(max_points_r >> 8));
    Serial.write(POINT_FORMAT);

    for (uint16_t i = 0; i < max_points_r; i++) {
        uint32_t p = points_r[i];
        uint8_t b0 = (uint8_t)(p & 0xFF);
        uint8_t b1 = (uint8_t)((p >> 8) & 0xFF);
        uint8_t b2 = (uint8_t)((p >> 16) & 0xFF);
        uint8_t b3 = (uint8_t)((p >> 24) & 0xFF);

        Serial.write(b0); checksum ^= b0;
        Serial.write(b1); checksum ^= b1;
        Serial.write(b2); checksum ^= b2;
        Serial.write(b3); checksum ^= b3;
    }

    // --- Méta-données 256 octets ---
    Serial.write(META_FORMAT);
    uint8_t b0 = 0;
    uint8_t b1 = 0;
    uint8_t b2 = 0;
    uint8_t b3 = 0;
    b0 = (uint8_t)(cam_param & 0xFF);
    b1 = (uint8_t)((cam_param >> 8) & 0xFF);
    b2 = (uint8_t)((cam_param >> 16) & 0xFF);
    b3 = (uint8_t)((cam_param >> 24) & 0xFF);

    Serial.write(b0); checksum ^= b0;
    Serial.write(b1); checksum ^= b1;
    Serial.write(b2); checksum ^= b2;
    Serial.write(b3); checksum ^= b3;

    // complète les 63 mots restants avec des 0
    for (uint16_t i = 1; i < 64; i++) { // 64 mots de 32 bits = 256 octets
        b0 = 0;
        b1 = 0;
        b2 = 0;
        b3 = 0;

        Serial.write(b0); checksum ^= b0;
        Serial.write(b1); checksum ^= b1;
        Serial.write(b2); checksum ^= b2;
        Serial.write(b3); checksum ^= b3;
    }

    // --- Checksum final ---
    Serial.write(checksum);
}
