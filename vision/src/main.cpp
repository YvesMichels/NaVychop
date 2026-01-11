#include "esp_camera.h"
#include <Arduino.h>
#include "image_process.h"
#include "transformation.h"
#include "verification.h"

#define LED_FLASH_PIN 4
#define MAX_POINTS 266 // pour une image 320x240 avec des blocs de 16x16 pixels sans les bords bas et droite (convolution 15 pixels + gradient 1 pixel)
#define MAX_POINTS_R 104 // pour l'image 40x30 avec des bords de 15 pixels (26x16 pixels utiles) avec un padding de 2

// Résolution cible IDIM et JDIM définies dans image_process
// image convoluée en entiers 8 bits pour des limitations de mémoire [old : 16 bits conceptuellement Q8.8 (valeur réelle = val / 256)]
static uint8_t im_conv[IDIM*JDIM];
static uint8_t im_conv_red[IDIM_R*JDIM_R];
static uint8_t im_grad_red[IDIM_R*JDIM_R];
static uint32_t points_ref[MAX_POINTS];
static uint32_t points_trans[MAX_POINTS];
static uint32_t points_ref_red[MAX_POINTS_R];
static uint32_t points_trans_red[MAX_POINTS_R];

sensor_t *s;

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
  config.pixel_format = PIXFORMAT_GRAYSCALE;  // <-- format mono 8

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
  uint16_t aec_value;
  uint8_t agc_gain;
  uint8_t ae_level;
  uint32_t cam_param; // bits 31 ..agc_gain.. 24 | 23 ..ae_level.. 16 | 15 ..aec_value.. 0

  
  digitalWrite(LED_FLASH_PIN, HIGH);   // début mesure
  camera_fb_t* fb = esp_camera_fb_get();
  s = esp_camera_sensor_get();
  aec_value = s->status.aec_value;
  agc_gain  = s->status.agc_gain;
  ae_level  = (int8_t) s->status.ae_level;
  cam_param = aec_value + (ae_level<<16) + (agc_gain<<24);
  if (!fb) {
    //Serial.println("Failed to get frame");
    return;
  }

  // fb->buf contient une image GRAYSCALE 320x240
  uint8_t *img = fb->buf;
 
  red_image(im_grad_red, img);
  
  digitalWrite(LED_FLASH_PIN, LOW);    // fin mesure

  // traitement de l'image complète
  gaussian_blur_15x15_stream_8bits(im_conv, img, IDIM, JDIM);
  norme_gradient_L1_8bits(img, im_conv, IDIM, JDIM);
  extract_points(img, im_conv, points_ref);

  // traitement de l'image réduite
  gaussian_blur_15x15_stream_8bits(im_conv_red, im_grad_red, IDIM_R, JDIM_R);
  extract_points_red(im_conv_red, points_ref_red);


  // envoie 
  send_motion_data(im_conv, JDIM, IDIM, im_conv_red, JDIM_R, IDIM_R, points_ref, MAX_POINTS, points_ref_red, MAX_POINTS_R, cam_param);

  esp_camera_fb_return(fb);

  delay(2000);  // 
}
