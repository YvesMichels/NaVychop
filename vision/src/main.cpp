#include "esp_camera.h"
#include <Arduino.h>

#include "image_process.h"
#include "transformation.h"
#include "verification.h"
#include "gaussian_dual_core.h"

#define LED_FLASH_PIN 4
#define MAX_POINTS 266
#define MAX_POINTS_R 204 // 104 pour un filtre 15, 204 pour un filtre 7
#define N_IMAGES 30

// Buffers temporaires pour PSRAM
static uint8_t (*psram_im_conv)[IDIM*JDIM] = nullptr;
static uint8_t (*psram_im_conv_red)[IDIM_R*JDIM_R] = nullptr;
static uint32_t (*psram_points_ref)[MAX_POINTS] = nullptr;
static uint32_t (*psram_points_ref_red)[MAX_POINTS_R] = nullptr;
// images en cache
static uint8_t im_conv[IDIM*JDIM];
// static uint8_t im_red[IDIM_R*JDIM_R]; <= suppression de im_red pour libérer de la mémoire (utilisation de im_conv à la place)
static uint8_t im_conv_red[IDIM_R*JDIM_R];
static uint32_t points_ref[MAX_POINTS];
static uint32_t points_ref_red[MAX_POINTS_R];
static int32_t logs[64]; // vecteur d'intrumentation du code pour les vérifications

// transformation
transform_param_q16_t p_red;

sensor_t *s;

// gestion du temps
unsigned long tps_0, tps;
int ind_tps = 0;
uint32_t ind_im = 0;

/* A METTRE DANS COMMUNICATION */
#define SYNC_1 0xAA
#define SYNC_2 0x55
#define SYNC_3 0xCC
#define SYNC_4 0x33
#define TRANSFORM_FORMAT 0x42

void send_transform_and_cost(const transform_param_q16_t *p, uint32_t cost_q16, int32_t *logs) {
    uint8_t buffer[4 + 1 + 2 + 2 + 2 + 16*4 + 1]; // taille totale : sync + format + params + logs + checksum
    uint8_t checksum = 0;
    uint8_t *ptr = buffer;

    // --- Synchronisation ---
    *ptr++ = SYNC_1; checksum ^= SYNC_1;
    *ptr++ = SYNC_2; checksum ^= SYNC_2;
    *ptr++ = SYNC_3; checksum ^= SYNC_3;
    *ptr++ = SYNC_4; checksum ^= SYNC_4;

    // --- Format ---
    *ptr++ = TRANSFORM_FORMAT; checksum ^= TRANSFORM_FORMAT;


    const int32_t *params = (const int32_t *)p;

    // --- theta (16 bits) ---
    for (int i = 0; i < 2; i++) {
        uint8_t b = (params[1] >> (8*i)) & 0xFF;
        *ptr++ = b;
        checksum ^= b;
    }

    // --- tx (16 bits intermédiaire, bits 8..23) ---
    for (int i = 1; i <= 2; i++) {
        uint8_t b = (params[2] >> (8*i)) & 0xFF;
        *ptr++ = b;
        checksum ^= b;
    }

    // --- ty (16 bits intermédiaire, bits 8..23) ---
    for (int i = 1; i <= 2; i++) {
        uint8_t b = (params[3] >> (8*i)) & 0xFF;
        *ptr++ = b;
        checksum ^= b;
    }

    // --- logs (16 x 32 bits) ---
    for (int i = 0; i < 16; i++) {
        uint32_t v = (uint32_t)logs[i];
        for (int j = 0; j < 4; j++) {
            uint8_t b = (v >> (8*j)) & 0xFF;
            *ptr++ = b;
            checksum ^= b;
        }
    }

    // --- Checksum final ---
    *ptr++ = checksum;

    // --- Envoi en un seul appel ---
    Serial.write(buffer, 76);
}


void setup() {
    pinMode(LED_FLASH_PIN, OUTPUT);
    digitalWrite(LED_FLASH_PIN, LOW);

    // Initialisation UART
    Serial.begin(115200);
    Serial.setDebugOutput(false);

    // Allouer les buffers PSRAM
    psram_im_conv = (uint8_t (*)[IDIM * JDIM])
      ps_malloc(N_IMAGES * IDIM * JDIM * sizeof(uint8_t));

    psram_im_conv_red = (uint8_t (*)[IDIM_R * JDIM_R])
        ps_malloc(N_IMAGES * IDIM_R * JDIM_R * sizeof(uint8_t));

    psram_points_ref = (uint32_t (*)[MAX_POINTS])
        ps_malloc(N_IMAGES * MAX_POINTS * sizeof(uint32_t));

    psram_points_ref_red = (uint32_t (*)[MAX_POINTS_R])
        ps_malloc(N_IMAGES * MAX_POINTS_R * sizeof(uint32_t));


    if (!psram_im_conv || !psram_im_conv_red || !psram_points_ref || !psram_points_ref_red) {
        // PSRAM non disponible ou allocation échouée
        while(1) {
            digitalWrite(LED_FLASH_PIN, HIGH);
            delay(500);
            digitalWrite(LED_FLASH_PIN, LOW);
            delay(500);
        }
    }

    // Initialisation caméra AI Thinker
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = 5;
    config.pin_d1       = 18;
    config.pin_d2       = 19;
    config.pin_d3       = 21;
    config.pin_d4       = 36;
    config.pin_d5       = 39;
    config.pin_d6       = 34;
    config.pin_d7       = 35;
    config.pin_xclk     = 0;
    config.pin_pclk     = 22;
    config.pin_vsync    = 25;
    config.pin_href     = 23;
    config.pin_sccb_sda = 26;
    config.pin_sccb_scl = 27;
    config.pin_pwdn     = 32;
    config.pin_reset    = -1;
    config.xclk_freq_hz = 24000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size   = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count     = 2;

    if (esp_camera_init(&config) != ESP_OK) {
        // Caméra non initialisée
        while(1) {
            digitalWrite(LED_FLASH_PIN, HIGH);
            delay(200);
            digitalWrite(LED_FLASH_PIN, LOW);
            delay(200);
        }
    }

    // Création de la tâche Core 1, priorité 1, pinning Core 1
    gaussian_dual_core_init();
 

    delay(1000);
    digitalWrite(LED_FLASH_PIN, HIGH);
    delay(5);
    digitalWrite(LED_FLASH_PIN, LOW);
    delay(100);
    digitalWrite(LED_FLASH_PIN, HIGH);
    delay(5);
    digitalWrite(LED_FLASH_PIN, LOW);
    delay(200);

    // initialisation du temps
    tps_0 = millis();
}

void loop() {
    // Capture et traitement N_IMAGES
    
    ind_tps = 0;
    logs[ind_tps] = (int32_t) millis();
    ind_tps = ind_tps + 1;
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
        
        tps = millis() - tps_0;
        s = esp_camera_sensor_get();
        uint16_t aec_value = s->status.aec_value;
        uint8_t agc_gain  = s->status.agc_gain;
        uint8_t ae_level  = (int8_t)s->status.ae_level;
        uint32_t cam_param = aec_value + (ae_level << 16) + (agc_gain << 24);
        uint32_t init_cost;
        uint8_t* img = fb->buf;
        logs[ind_tps] = (int32_t) millis();
        ind_tps = ind_tps + 1;

        // Initialisation transformation de référence (image réduite 40x30)
        p_red.div   = 0;
        p_red.theta = 0;
        p_red.tx    = 0;
        p_red.ty    = 0;
        p_red.vp_x  = (JDIM_R / 2 - 4) << 16;
        p_red.vp_y  = (IDIM_R / 2 - 4) << 16;

        /* ==== ==== Traitement image réduite ==== ==== */
        // Réduction
        red_image(im_conv, img); // remplacement de im_red par im_conv (OK car plus d'espace)
        logs[ind_tps] = (int32_t) millis();
        ind_tps = ind_tps + 1;

        // Traitement image réduite
        gaussian_blur_7x7_stream_8bits(im_conv_red, im_conv, IDIM_R, JDIM_R); // remplacement de im_red par im_conv (OK car plus d'espace) 

        // pré-estimation sur l'image réduite
        logs[ind_tps] = (int32_t) millis();
        ind_tps = ind_tps + 1;

        init_cost = exhaustive_init(points_ref_red, MAX_POINTS_R, im_conv_red, &p_red, 655360);
        
        logs[ind_tps] = (int32_t) millis();
        ind_tps = ind_tps + 1;

        optim_huber_grad_q16(im_conv_red, IDIM_R, JDIM_R, points_ref_red, MAX_POINTS_R, &p_red, 25, 10000, 10000, 1000, 500, 655360, logs);
        
        logs[ind_tps] = (int32_t) millis();
        ind_tps = ind_tps + 1;

        // extraction des points de l'image réduite pour l'image suivante
        extract_points_red(im_conv_red, points_ref_red);
        


        logs[ind_tps] = (int32_t) millis();
        ind_tps = ind_tps + 1;


        /* ==== ==== Traitement image complète ==== ==== */
        gaussian_blur_9x9_stream_8bits(im_conv, img, IDIM, JDIM);
        //gaussian_blur_15x15_dual_core(im_conv, img, IDIM, JDIM);
        logs[ind_tps] = (int32_t) millis();
        ind_tps = ind_tps + 1;

        norme_gradient_L1_8bits(img, im_conv, IDIM, JDIM);
        logs[ind_tps] = (int32_t) millis();
        ind_tps = ind_tps + 1;

        
        // passage à l'échelle supérieure
        p_red.tx    = p_red.tx<<3;
        p_red.ty    = p_red.ty<<3;
        p_red.vp_x  = (JDIM / 2 - 8) << 16;
        p_red.vp_y  = (IDIM / 2 - 8) << 16;

        // estimation sur l'image complète
        optim_huber_grad_q16(im_conv, IDIM, JDIM, points_ref, MAX_POINTS, &p_red, 25, 30000, 30000, 500, 200, 655360, logs);
        
        logs[ind_tps] = (int32_t) millis();
        ind_tps = ind_tps + 1;
        
        // Extraction des points d'intérets pour la trame suivante
        extract_points(img, im_conv, points_ref);
        
        logs[ind_tps] = (int32_t) millis();

        send_transform_and_cost(&p_red, init_cost, logs);
        ind_im = ind_im + 1;
    }

    
    
    esp_camera_fb_return(fb);

}
