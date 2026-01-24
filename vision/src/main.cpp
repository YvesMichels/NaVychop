#include "esp_camera.h"
#include <Arduino.h>

#include "image_process.h"
#include "transformation.h"
#include "verification.h"

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
static uint8_t im_red[IDIM_R*JDIM_R];
static uint8_t im_conv_red[IDIM_R*JDIM_R];
static uint32_t points_ref[MAX_POINTS];
static uint32_t points_ref_red[MAX_POINTS_R];

// transformation
transform_param_q16_t p_red;

sensor_t *s;

// gestion du temps
unsigned long tps_0, tps;

/* A METTRE DANS COMMUNICATION */
#define SYNC_1 0xAA
#define SYNC_2 0x55
#define SYNC_3 0xCC
#define SYNC_4 0x33

#define TRANSFORM_FORMAT 0x42
void send_transform_and_cost(
    const transform_param_q16_t *p,
    uint32_t cost_q16
) {
    uint8_t checksum = 0;

    // --- Synchronisation 4 octets ---
    Serial.write(SYNC_1);
    Serial.write(SYNC_2);
    Serial.write(SYNC_3);
    Serial.write(SYNC_4);

    // --- Format ---
    Serial.write(TRANSFORM_FORMAT);

    // --- Coût ---
    uint32_t v = cost_q16;
    uint8_t b0 = (uint8_t)(v & 0xFF);
    uint8_t b1 = (uint8_t)((v >> 8) & 0xFF);
    uint8_t b2 = (uint8_t)((v >> 16) & 0xFF);
    uint8_t b3 = (uint8_t)((v >> 24) & 0xFF);

    Serial.write(b0); checksum ^= b0;
    Serial.write(b1); checksum ^= b1;
    Serial.write(b2); checksum ^= b2;
    Serial.write(b3); checksum ^= b3;

    // --- Paramètres transformation ---
    const int32_t *params = (const int32_t *)p;

    for (uint8_t i = 0; i < 6; i++) {
        v = (uint32_t)params[i];
        b0 = (uint8_t)(v & 0xFF);
        b1 = (uint8_t)((v >> 8) & 0xFF);
        b2 = (uint8_t)((v >> 16) & 0xFF);
        b3 = (uint8_t)((v >> 24) & 0xFF);

        Serial.write(b0); checksum ^= b0;
        Serial.write(b1); checksum ^= b1;
        Serial.write(b2); checksum ^= b2;
        Serial.write(b3); checksum ^= b3;
    }

    // --- Checksum final ---
    Serial.write(checksum);
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
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size   = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count     = 1;

    if (esp_camera_init(&config) != ESP_OK) {
        // Caméra non initialisée
        while(1) {
            digitalWrite(LED_FLASH_PIN, HIGH);
            delay(200);
            digitalWrite(LED_FLASH_PIN, LOW);
            delay(200);
        }
    }

    delay(2000);
    digitalWrite(LED_FLASH_PIN, HIGH);
    delay(50);
    digitalWrite(LED_FLASH_PIN, LOW);
    delay(200);
    digitalWrite(LED_FLASH_PIN, HIGH);
    delay(50);
    digitalWrite(LED_FLASH_PIN, LOW);
    delay(200);

    // initialisation du temps
    tps_0 = millis();
}

void loop() {
    // Capture et traitement N_IMAGES
    for (uint8_t idx = 0; idx < N_IMAGES; idx++) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            idx--; // réessayer la capture
            delay(50);
            continue;
        }

        
        tps = millis() - tps_0;
        s = esp_camera_sensor_get();
        uint16_t aec_value = s->status.aec_value;
        uint8_t agc_gain  = s->status.agc_gain;
        uint8_t ae_level  = (int8_t)s->status.ae_level;
        uint32_t cam_param = aec_value + (ae_level << 16) + (agc_gain << 24);
        uint32_t init_cost;
        uint8_t* img = fb->buf;


        // Initialisation transformation de référence (image réduite 40x30)
        p_red.div   = 0;
        p_red.theta = 0;
        p_red.tx    = 0;
        p_red.ty    = 0;
        p_red.vp_x  = (JDIM_R / 2) << 16;
        p_red.vp_y  = (IDIM_R / 2) << 16;

        // Réduction
        red_image(im_red, img);

        // Traitement image complète
        //gaussian_blur_15x15_stream_8bits(im_conv, img, IDIM, JDIM);
        //norme_gradient_L1_8bits(img, im_conv, IDIM, JDIM);
        //extract_points(img, im_conv, points_ref);

        // Traitement image réduite
        gaussian_blur_7x7_stream_8bits(im_conv_red, im_red, IDIM_R, JDIM_R);
        //init_cost = exhaustive_init(points_ref_red, MAX_POINTS_R, im_conv_red, &p_red, (int32_t) 10*65536);
        
        init_cost = exhaustive_init(points_ref_red, MAX_POINTS_R, im_conv_red, &p_red, 10*65536);
        
        send_transform_and_cost(&p_red, init_cost);
   

        
        
        extract_points_red(im_conv_red, points_ref_red);
        esp_camera_fb_return(fb);

    }

/*
    // Après les 20 images, envoi UART
    for (uint8_t idx = 0; idx < N_IMAGES; idx++) {
        send_motion_data(
            psram_im_conv[idx], JDIM, IDIM,
            psram_im_conv_red[idx], JDIM_R, IDIM_R,
            psram_points_ref[idx], MAX_POINTS,
            psram_points_ref_red[idx], MAX_POINTS_R,
            0 // cam_param non utilisé ici ou stocker si besoin
        );

        // Indication visuelle : LED flash rapide pendant envoi
        digitalWrite(LED_FLASH_PIN, HIGH);
        delay(20);
        digitalWrite(LED_FLASH_PIN, LOW);
        delay(2000);
    }

    // Fin du programme, rien à faire après
    while(1) {
        // LED allumée fixe pour signaler fin
        digitalWrite(LED_FLASH_PIN, LOW);
        delay(1000);
    }
*/
}
