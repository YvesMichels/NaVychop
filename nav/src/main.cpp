#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO08x.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <SPI.h>
#include <LoRa.h>

#include "navigation.h"
#include "communication.h"

typedef struct {
    int32_t dummy;   // index 0 non utilisé ici
    int32_t theta;   // index 1
    int32_t tx;      // index 2
    int32_t ty;      // index 3
} transform_param_q16_t;

typedef struct {
    int16_t theta;
    int16_t tx;
    int16_t ty;
    int32_t logs[16];
    uint8_t checksum;
} vision_frame_t;


#define SYNC_1 0xAA
#define SYNC_2 0x55
#define SYNC_3 0xCC
#define SYNC_4 0x33
#define TRANSFORM_FORMAT 0x42
#define VISION_FRAME_SIZE 76

#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23
#define LORA_SS    5
#define LORA_RST   27
#define LORA_DIO0  26
#define LORA_FREQ 433E6   // adapter si besoin (433E6 ou 868E6)

SPIClass spiLoRa(VSPI);

uint8_t navMac[] = {0x02, 0xAA, 0xBB, 0xA0, 0x00, 0x00}; // adresse centrale de navigation
uint8_t receiverMac[] = {0x02, 0xAA, 0xBB, 0xA0, 0x20, 0x20}; // adresse centrale sol
static int16_t logs[32]; // vecteur d'intrumentation du code pour les vérifications

Adafruit_BMP3XX bmp;
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// variables IMU
long reportIntervalUs = 50000;  // période de raffraichissement de l'UMI 50ms = 20Hz
Quaternion q_M_GRV;
Quaternion q_M_RV;
float R_M_RV[3][3]; // matrice de rotation de l'umu (M) par rapport à la terre hybridée magnéto
float R_M_GRV[3][3];// matrice de rotation de l'umu (M) par rapport à la terre sans magnéto
float R_A_M[3][3] = {
    {  0.0f,  1.0f,  0.0f },  // X_M
    { -1.0f,  0.0f,  0.0f },  // Y_M
    {  0.0f,  0.0f,  1.0f }   // Z_M
};// matrice de rotation de l'avion (A) par rapport à l'IMU (M) (fixe, dépend du montage)
float R_A_RV[3][3]; // matrice de rotation de l'avion (A) par rapport à la terre hybridée magnéto
float R_A_GRV[3][3];// matrice de rotation de l'avion (A) par rapport à la terre sans magnéto

// variables moteurs
int8_t servo1 = 90;
int8_t servo2 = 90;
int8_t servo3 = 90;
int8_t servo4 = 90;
int8_t esc = 0;
uint32_t servo_msg;

// gestion du temps
unsigned long t_0, t_cur; // temps courant mesuré
unsigned long t_target = 0;
int32_t delta_t_loop; // décallage entre le temps mesuré et le temps 


bool readVisionFrame(vision_frame_t &out)
{
    static uint8_t buf[VISION_FRAME_SIZE];
    static uint8_t idx = 0;

    while (Serial1.available()) {
        uint8_t b = Serial1.read();

        if (idx == 0 && b != SYNC_1) continue;
        if (idx == 1 && b != SYNC_2) { idx = 0; continue; }
        if (idx == 2 && b != SYNC_3) { idx = 0; continue; }
        if (idx == 3 && b != SYNC_4) { idx = 0; continue; }

        buf[idx++] = b;

        if (idx == VISION_FRAME_SIZE) {
            idx = 0;

            // checksum
            uint8_t cs = 0;
            for (int i = 0; i < VISION_FRAME_SIZE - 1; i++)
                cs ^= buf[i];

            if (cs != buf[VISION_FRAME_SIZE - 1])
                return false;

            if (buf[4] != TRANSFORM_FORMAT)
                return false;

            // --- décodage ---
            out.theta = buf[5] | (buf[6] << 8);
            out.tx    = buf[7] | (buf[8] << 8);
            out.ty    = buf[9] | (buf[10] << 8);

            int k = 11;
            for (int i = 0; i < 16; i++) {
                out.logs[i] =
                    (buf[k]) |
                    (buf[k+1] << 8) |
                    (buf[k+2] << 16) |
                    (buf[k+3] << 24);
                k += 4;
            }

            out.checksum = cs;
            return true;
        }
    }
    return false;
}

void setup()
{
    // UART instrumentation
    Serial.begin(115200);
    // UART vision
    Serial1.begin(115200, SERIAL_8N1, 25, -1);
    // UART commandes moteurs
    Serial2.begin(19200, SERIAL_8N1, -1, 17);
    delay(500);

    // I2C
    Wire.begin(21, 22);


    // BMP390
    if (!bmp.begin_I2C(0x76)) {
        Serial.println("BMP390 NOT FOUND");
        while (1);
    }
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);

    // BNO085
    if (!bno08x.begin_I2C(0x4B)) {
        Serial.println("BNO085 NOT FOUND");
        while (1);
    }

    bno08x.enableReport(SH2_ROTATION_VECTOR, reportIntervalUs);
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, reportIntervalUs); 

    Serial.println("INIT OK");

      // Changement MAC STA
    /*esp_err_t err = esp_wifi_set_mac(WIFI_IF_STA, navMac);
    if (err != ESP_OK) {
        Serial.println("Erreur changement MAC");
        return;
    }
    Serial.print("MAC projet : ");
    Serial.println(WiFi.macAddress());

    // Initialisation ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Erreur ESP-NOW");
        return;
    }
    */

    // Reset LoRa et initialisation SPI explicite
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(10);
    spiLoRa.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

    // Liaison SPI custom à LoRa
    LoRa.setSPI(spiLoRa);
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

    if (!LoRa.begin(LORA_FREQ)) {
        Serial.println("LoRa init failed");
        while (1);
    }

    // Paramétrage radio
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.enableCrc();

    Serial.println("LoRa ready");
    
    // synchronisation du module de vision et initialisation du temps
    delay(1000);
    pinMode(32, OUTPUT);
    digitalWrite(32, HIGH);
    delay(500);
    digitalWrite(32, LOW);
    delay(10);
    digitalWrite(32, HIGH);
    delayMicroseconds(5000);
    digitalWrite(32, LOW);
    t_0 = millis();
    t_cur = t_0;
}

void loop()
{
    float yaw;  // pour test à remplacer par produits scalaires
    float pitch;// pour test à remplacer par produits scalaires
    float roll; // pour test à remplacer par produits scalaires
    float pressure, temp;
    float qw, qx, qy, qz;
    // --- BMP390 ---
    if (bmp.performReading()) {
        pressure = bmp.pressure;
        temp = bmp.temperature;
    }

    // --- BNO085 ---
    while (bno08x.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
            q_M_GRV = {sensorValue.un.gameRotationVector.real,
                sensorValue.un.gameRotationVector.i,
                sensorValue.un.gameRotationVector.j,
                sensorValue.un.gameRotationVector.k};
        }

        if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
            q_M_RV = {sensorValue.un.rotationVector.real,
                sensorValue.un.rotationVector.i,
                sensorValue.un.rotationVector.j,
                sensorValue.un.rotationVector.k};
        }
    }
    computeRotationMatrix(q_M_RV.w, q_M_RV.x, q_M_RV.y, q_M_RV.z, R_M_RV);
    rotMult(R_A_RV, R_A_M, R_M_RV);
    computeRotationMatrix(q_M_GRV.w, q_M_GRV.x, q_M_GRV.y, q_M_GRV.z, R_M_GRV);
    rotMult(R_A_GRV, R_A_M, R_M_GRV);

    // test control moteurs
    computeEulerAngles(q_M_RV.w, q_M_RV.x, q_M_RV.y, q_M_RV.z, &roll, &pitch, &yaw); 
    servo1 = max(0, min(63, (int)((roll)*50 + 32)));
    servo2 = max(0, min(63, (int)((pitch)*100 + 32)));
    servo3 = max(0, min(63, (int)((pitch)*100 + 32)));
    servo4 = max(20, min(43, (int)((yaw)*0 + 32)));
    esc = 0;
    /*if(((t_cur-t_0)/250)<48){ 
        esc = (t_cur-t_0)/250;
    }
    else
    {
        esc = 48 ;
    }*/

    motors_control(servo1, servo2, servo3, servo4, esc);

    // --- Vision ---
    vision_frame_t vf;
    if (readVisionFrame(vf)) {
        // Exemple d’exploitation
        logs[0] = (int16_t) (256*R_M_RV[0][0]);
        logs[1] = (int16_t) (256*R_M_RV[0][1]);
        logs[2] = (int16_t) (256*R_M_RV[0][2]);
        logs[3] = (int16_t) (256*R_M_RV[1][0]);
        logs[4] = (int16_t) (256*R_M_RV[1][1]);
        logs[5] = (int16_t) (256*R_M_RV[1][2]);
        logs[6] = (int16_t) (256*R_M_RV[2][0]);
        logs[7] = (int16_t) (256*R_M_RV[2][1]);
        logs[8] = (int16_t) (256*R_M_RV[2][2]);
        logs[9] = (int16_t) (256*computePitchCommandSignal(R_A_GRV, 0));
        logs[10] = (int16_t) (256*computeRollCommandSignal(R_A_GRV, 0));
        logs[11] = (int16_t) (256*computeYawCommandSignalImproved(R_A_GRV, 0));
        logs[12] = (int16_t) (256*computePitchCommandSignal(R_A_RV, 0));
        logs[13] = (int16_t) (256*computeRollCommandSignal(R_A_RV, 0));
        logs[14] = (int16_t) (256*computeYawCommandSignalImproved(R_A_RV, 0));
        send_quaternions_and_logs((float *)(&q_M_GRV), (float *)(&q_M_RV), logs);
    }


    // gestion du temps
  t_target = t_target + d_t;
  t_cur = millis();
  delta_t_loop =  ((int32_t)t_target) - ((int32_t) (t_cur - t_0));
  if(delta_t_loop>0)
  {delay(delta_t_loop);}
}
