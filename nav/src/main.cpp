#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <MS5611.h>
#include <TinyGPSPlus.h>
#include <LoRa.h>
#include "config.h"
#include "communication.h"
#include "navigation.h"



// Créer une instance du capteur
Adafruit_BNO08x bno08x = Adafruit_BNO08x(-1);

// GPS sur RX0 : lecture non bloquante
TinyGPSPlus gps;

// MS5611 barometer
MS5611 baro;

// LoRa SPI pins
#define LORA_CS   D4
#define LORA_RST  D3
#define LORA_DIO0 D0
#define LORA_FREQ 433E6  // fréquence de l'antenne 433 MHz

// Type de capteur à activer
sh2_SensorId_t rotType   = SH2_ROTATION_VECTOR;
sh2_SensorId_t accelType = SH2_ACCELEROMETER;
long reportIntervalUs = 50000;  // période de raffraichissement de l'UMI 50ms = 20Hz
const float SQ2_2 = 0.70710678118f;

// variables IMU
sh2_SensorValue_t sensorValue;
float Tm0[3][3];
float Tmg[3][3];
float yaw;
float pitch;
float roll;
float accel_x=0;
float accel_y=0;
float accel_z=0;
Quaternion q_M;
Quaternion q_A;
Quaternion q_dec_GPS;
const Quaternion q_M_A = { SQ2_2, 0.0f, 0.0f, -SQ2_2 };
const Quaternion q_delta_M_A = { 0.999734139768333f, 0.021377856659570f, 0.008637297823682f, -0.000184696018127f };

// variables com LoRa
float yaw0=0;
float pitch0=0;
float roll0=0;
float esc0=0;
int cpt_lora=0;

// variables altitude 
float baroPressure = 0;
float baroTemperature = 0;
float baroAltitude = 0;

// variables moteurs
int servo1 = 90;
int servo2 = 90;
int servo3 = 90;
int servo4 = 90;
int esc = 0;
uint32_t servo_msg;
uint8_t buffer_msg[size_msg];

// variables de temps
unsigned long t_target = 0;
unsigned long t_0; // temps au démarrage de la boucle principale (après initialisation)
unsigned long t_cur; // temps courant mesuré
int32_t delta_t_loop; // décallage entre le temps mesuré et le temps 

// mémoire pour des essais terrains, rejeux et simulations
int16_t buffer_angles[3][nb_paquet]; // fifo de données (angles), peut être plus grand que "nb_compress" pour temporiser les commnuications non temps réel
int8_t buffer_d_angles[3][size_buffer_delta]; // fifo de données (variation angles), peut être plus grand que "nb_compress" pour temporiser les commnuications non temps réel
int16_t buffer_accel[3][nb_paquet]; // fifo de données (accellération), peut être plus grand que "nb_compress" pour temporiser les commnuications non temps réel
int8_t buffer_d_accel[3][size_buffer_delta]; // fifo de données (variation accellération), peut être plus grand que "nb_compress" pour temporiser les commnuications non temps réel
int16_t buffer_pression[3][nb_paquet]; // fifo de données (pression athmosphérique), peut être plus grand que "nb_compress" pour temporiser les commnuications non temps réel
int8_t buffer_d_pression[3][size_buffer_delta]; // fifo de données (variation pression athmosphérique), peut être plus grand que "nb_compress" pour temporiser les commnuications non temps réel
int16_t buffer_gps[2][size_buffer_sparce]; // fifo de données (GPS), peut être plus grand que "nb_compress" pour temporiser les commnuications non temps réel
int8_t buffer_alt_gps[size_buffer_sparce]; // fifo de données (altitude gps), peut être plus grand que "nb_compress" pour temporiser les commnuications non temps réel
int8_t buffer_temp[size_buffer_sparce]; // fifo de données (température), peut être plus grand que "nb_compress" pour temporiser les commnuications non temps réel
int16_t first_angle[3] = {0, 0, 0}; // angle initial quantifié (pour supprimer les erreurs accumulées)
int16_t last_angle[3] = {0, 0, 0}; // angle reconstruit quantifié (pour supprimer les erreurs accumulées)
uint16_t ind_wr_fifo_data = 0; // indice d'écriture dans la fifo (compatible pour des envois différés)
uint16_t ind_rd_fifo_data = 0; // indice de lecture dans la fifo (compatible pour des envois différés)
int8_t ind_msg_data = 0; // nombre de messages écrits après le dernier envoi (pour des envois en temps réel)
uint16_t ind_pkt = 0; // indice du paquet de donné envoyé (pour la reconstruction des données au sol même en cas de perte)



void setup() {
  Serial.begin(115200);
  Wire.begin(D2, D1); // IMPORTANT : Initialise le bus I2C

  Serial.println("Initialisation du BNO08x...");

  if (!bno08x.begin_I2C(0x4B, &Wire)) {
    Serial.println("Échec de détection du BNO08x");
    while (1);
  }

  Serial.println("BNO08x détecté !");
  bno08x.enableReport(rotType, reportIntervalUs);
  bno08x.enableReport(accelType, reportIntervalUs);

  // Init LoRa
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
  } else {
    Serial.println("LoRa init OK");
    // définir le Spreading Factor (7..12)
    LoRa.setSpreadingFactor(12);   // SF7 rapide, SF12 lent mais robuste
    // définir la bande passante (Hz) — typiquement 125k, 250k, 500k
    LoRa.setSignalBandwidth(250E3);  // 250-500 kHz
    // définir le coding rate (4/5 .. 4/8)
    LoRa.setCodingRate4(5);  // 4/5 (valeur par défaut courante)
    // autres réglages utiles
    LoRa.setPreambleLength(8);       // longueur préambule (par défaut ~8)
    LoRa.setTxPower(10);             // puissance (dBm), selon module et réglementation
  }

  q_dec_GPS = {0.9999154660, 0, 0, 0.0130023365}; // quaternion de correction de la dérive magnétique (position par défaut Paris 09/2025)

  // Init Baro
  if (!baro.begin()) {
    Serial.println("MS5611 not found!");
  }
  Serial.println("ESP8266: Ready.");
  delay(5000);
  t_0 = millis();
}

void loop() {

  // Lecture GPS
  while (Serial.available()) {
    gps.encode(Serial.read());
  }

  // Lecture baro
  baro.read();  // lit température et pression brutes
  baroTemperature = baro.getTemperature(); // °C
  baroPressure = baro.getPressure();       // mbar
  // Calculer altitude (formule barométrique simplifiée)
  // 1013.25 mbar = niveau mer standard
  baroAltitude = 44330.0 * (1.0 - pow(baroPressure / 1013.25, 0.1903));

  // Lecture BNO
  while(bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == rotType) {
      q_M = {sensorValue.un.rotationVector.real,
            sensorValue.un.rotationVector.i,
            sensorValue.un.rotationVector.j,
            sensorValue.un.rotationVector.k};
      q_A = qNormalize( qMul(q_M, q_M_A) );
      q_M = qNormalize(qMul(q_dec_GPS, q_A));
      //q_M = qNormalize(qMul(q_delta_M_A, q_M));
      computeEulerAngles(q_M.w, q_M.x, q_M.y, q_M.z, &roll, &pitch, &yaw);  
    }
    if (sensorValue.sensorId == accelType) {
      accel_x = sensorValue.un.accelerometer.x;
      accel_y = sensorValue.un.accelerometer.y;
      accel_z = sensorValue.un.accelerometer.z;
    }
  }

  // met à jour les buffers de données
  if (ind_msg_data==0){
    first_angle[0] = (int16_t)(yaw * binpt_yaw * 57.2957805);
    first_angle[1] = (int16_t)(pitch * binpt_pitch * 57.2957805);
    first_angle[2] = (int16_t)(roll * binpt_roll * 57.2957805);
  }
  else {
    write_fifo_d_angles(buffer_d_angles, roll * 57.2957805, pitch * 57.2957805, yaw * 57.2957805, last_angle, &ind_wr_fifo_data);
  }
  // envoie les pacquets de données
  if(ind_msg_data == nb_compress){
    ind_msg_data = 0;
    //send_fifo_d_angles(first_angle, buffer_d_angles, &ind_rd_fifo_data, ind_pkt);
    Serial.print("envoi paquet : ");
    Serial.println(ind_pkt);
    Serial.print(((float)first_angle[0])/binpt_yaw);
    Serial.print(" - ");
    Serial.print(((float)first_angle[1])/binpt_pitch);
    Serial.print(" - ");
    Serial.println(((float)first_angle[2])/binpt_roll);

    ind_pkt ++;
  }
  else{
    ind_msg_data ++;
  }

  // lecture LoRa
  int packetSize = LoRa.parsePacket();
  Serial.print("paquet reçu : ");
    Serial.println(packetSize);
  if (packetSize == 3) { // On attend exactement 3 octets
    byte roll_byte  = LoRa.read();
    byte pitch_byte = LoRa.read();
    byte esc_byte   = LoRa.read();
    roll0 = (float)roll_byte -128;
    roll0 = roll0*0.01745329;
    pitch0 = (float)pitch_byte -128;
    pitch0 = pitch0*0.01745329;
    esc0 = (float)(esc_byte  & 0x1F);

    cpt_lora = max_cpt_lora;
    Serial.print("pitch reçu : ");
    Serial.println(pitch0);
    Serial.print("roll reçu : ");
    Serial.println(roll0);
    Serial.print("esc reçu : ");
    Serial.println(roll0);
    Serial.print("cmd reçu : ");
    Serial.println(esc_byte>>5);
    Serial.println(" --- ");
  }
  else
  {
    cpt_lora = cpt_lora-d_t;
    if(cpt_lora<0)
    {cpt_lora = 0;}
  }

  // gestion perte de communication LoRa
  if(cpt_lora==0)
  {
    roll0 = 0;
    pitch0 = 0;
    esc0 = 0;
  }
  servo1 = max(0, min(63, (int)((pitch-pitch0)*50 + 32)));;
  servo2 = max(0, min(63, (int)((roll-roll0)*100 + 32)));
  servo3 = max(0, min(63, (int)((roll-roll0)*100 + 32)));
  servo4 = 0;
  esc = esc0/8;

  servo_msg =  (servo1 & 0x3F)
             | ((servo2 & 0x3F) << 6)
             | ((servo3 & 0x3F) << 12)
             | ((servo4 & 0x3F) << 18)
             | ((esc    & 0x3F) << 24);

  // Préparer trame moteur avec start byte
  //uint8_t buffer_msg[6];
  buffer_msg[0] = 0xFF;                          // start byte
  memcpy(&buffer_msg[1], &servo_msg, 4);         // 4 octets de données
  buffer_msg[4] = crc8(&buffer_msg[1], 4);       // CRC calculé sur les 4 octets

  // Envoi de la trame moteur complète
  //Serial.write(buffer_msg, sizeof(buffer_msg));

  // gestion du temps
  t_target = t_target + d_t;
  t_cur = millis();
  delta_t_loop =  ((int32_t)t_target) - ((int32_t) (t_cur - t_0));
  if(delta_t_loop>0)
  {delay(delta_t_loop);}
  
}