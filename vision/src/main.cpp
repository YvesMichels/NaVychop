#include "esp_camera.h"
#include <Arduino.h>
#include "image_process.h"

#define LED_FLASH_PIN 4
#define MAX_POINTS 300 // pour une image 320x240 avec des blocs de 16x16 pixels

// Résolution cible IDIM et JDIM définies dans image_process
// image convoluée en entiers 8 bits pour des limitations de mémoire [old : 16 bits conceptuellement Q8.8 (valeur réelle = val / 256)]
static uint8_t im_conv[IDIM*JDIM];
static uint32_t points_ref[MAX_POINTS];
static uint32_t points_trans[MAX_POINTS];


void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  // Configuration ESP32-CAM (AI Thinker)
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sccb_sda = 26;
  config.pin_sccb_scl = 27;
  config.pin_pwdn = 32;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;  // <-- pour simplifier la moyenne

  config.frame_size = FRAMESIZE_QVGA; // 320x240
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    while (1);
  }

  pinMode(LED_FLASH_PIN, OUTPUT);
  digitalWrite(LED_FLASH_PIN, LOW);
  Serial.println("Camera init OK");
}

void loop() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    //Serial.println("Failed to get frame");
    return;
  }

  // fb->buf contient une image GRAYSCALE 320x240
  uint8_t *img = fb->buf;
  uint16_t img_conv[IDIM*JDIM];

  // convolue l'image avec un noyeau gaussien de taille 9x9
  digitalWrite(LED_FLASH_PIN, HIGH);   // début mesure
  gaussian_blur_15x15_stream_8bits(im_conv, img);
  norme_gradient_L1_8bits(img, im_conv);
  mark_gradient_max_blocks_16x16(img, im_conv, points_ref);
  digitalWrite(LED_FLASH_PIN, LOW);    // fin mesure
  // envoie l'image
  send_image(im_conv);
  
  esp_camera_fb_return(fb);


  //delay(1000);  // 
}
