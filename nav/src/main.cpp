#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO08x.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <SPI.h>
#include <LoRa.h>

#include "config.h"
#include "navigation.h"
#include "communication.h"
#include "control.h"
#include "datalink.h"

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

// paramètres systeme
parameters system_param;

uint8_t lora_cmd[4] = {1, 0, 0, 127};
CmdState cmd_state;
static uint32_t last_lora_ms = 0;
static uint8_t old_yaw_cmd = 0;
bool lora_link_ok = false;

SPIClass spiLoRa(VSPI);

uint8_t navMac[] = {0x02, 0xAA, 0xBB, 0xA0, 0x00, 0x00}; // adresse centrale de navigation
uint8_t receiverMac[] = {0x02, 0xAA, 0xBB, 0xA0, 0x20, 0x20}; // adresse centrale sol
static int16_t logs[32]; // vecteur d'intrumentation du code pour les vérifications

// variables LoRa
Adafruit_BMP3XX bmp;
Adafruit_BNO08x bno08x;
static uint32_t cpt_rv = 0;
static uint32_t cpt_grv = 0;
static uint32_t cpt_acc = 0;
static uint32_t cpt_mag = 0;
uint8_t motor_6 = 0;   // commande LoRa moteur 6 bits (puissnace moteur par défaut à 0)
uint8_t yaw_6 = 31;     // commande LoRa yaw 6 bits (lacet neutre à 31)
uint8_t pitch_6 = 31;   // commande LoRa pitch 6 bits (tangage neutre à 31)
uint8_t roll_6 = 31;    // commande LoRa roll 6 bits (roulis neutre à 31)

// variables IMU
sh2_SensorValue_t sensorValue;
// commenté car directement calculé dans les appels : long reportIntervalUs = 1000*d_t;  // période de raffraichissement de l'UMI
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
float accel[3];     // vecteur d'accéleration
float magneto[3];   // vecteur de champs magnétiques

// variables altitude
AttitudeParameters att_param;
AttitudeState att_states;

// variables asservissement
ControlState control_states; 

// variables moteurs
uint8_t servo1 = 32;
uint8_t servo2 = 32;
uint8_t servo3 = 32;
uint8_t servo4 = 32;
uint8_t esc = 0;
uint32_t servo_msg;

// gestion du temps
unsigned long t_0, t_cur; // temps courant mesuré
unsigned long t_target = 0;
int32_t delta_t_loop; // décallage entre le temps mesuré et le temps 

// logs 
constexpr uint8_t LOG_IDLE      = 0;
constexpr uint8_t LOG_RECORDING = 1;
constexpr uint8_t LOG_STOPPING  = 2;
logState logBuffer[LOG_SIZE];
//logState logBuffer[LOG_SIZE];
uint16_t logIndex = 0;
uint16_t flash_logIndex = 0;
uint8_t save_en = LOG_IDLE;
uint8_t sync_state = 0;

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
    Serial1.begin(115200, SERIAL_8N1, SERIAL_VISION, -1);
    // UART commandes moteurs
    Serial2.begin(19200, SERIAL_8N1, -1, SERIAL_MOTORS);
    delay(50);

    // I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    // BMP390
    if (!bmp.begin_I2C(I2C_ID_BMP)) {
        Serial.println("BMP390 NOT FOUND");
        while (1);
    }
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);

    // BNO085
    if (!bno08x.begin_I2C(I2C_ID_BNO)) {
        Serial.println("BNO085 NOT FOUND");
        while (1);
    }

    // bno08x.enableReport(SH2_ROTATION_VECTOR, 24500*d_t); // remplacée par le magnétomètre seul
    bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 24500*d_t);
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 1000*d_t);
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 2865*d_t);

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
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setCodingRate4(LORA_CR4);
    LoRa.enableCrc();

    // Initialisation de la structure de communication
    lora_cmd_init(cmd_state);

    Serial.println("LoRa ready");
    
    delay(1000);
    // initialisation altitude
    attitude_init(&att_states, &att_param);
    if (bmp.performReading()) {
        estimateAltitude(&att_states, &att_param, bmp.pressure, bmp.temperature, 0); // purge la mémoire du bmp390
    }


    // initialisation asservissement
    control_gain_init(system_param.control_param);
    control_init(control_states);


    if (!LittleFS.begin(true)) {
        Serial.println("Erreur LittleFS");
        return;
    }

    Serial.println("LittleFS OK");


    // synchronisation du module de vision et initialisation du temps
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
    float proj_yaw;         // produit scalaire : projection yaw sur commande yaw
    float proj_pitch;       // produit scalaire : projection pitch sur commande pitch
    float proj_roll;        // produit scalaire : projection roll sur commande roll
    float yaw_ref = 0.0;    // yaw de référence (calibration au démmarage ou sur demande)
    float pitch_ref = 0.0;  // yaw de référence (calibration au démmarage ou sur demande)
    float roll_ref = 0.0;   // yaw de référence (calibration au démmarage ou sur demande)
    float cmd_yaw;          // commande angulaire yaw (lacet)
    float cmd_pitch;        // commande angulaire pitch (tangage)
    float cmd_roll;         // commande angulaire roll (roulis)

    float pressure, temp;
    float qw, qx, qy, qz;
    // --- BMP390 ---
    if (bmp.performReading()) {
        pressure = bmp.pressure;
        temp = bmp.temperature;
        estimateAltitude(&att_states, &att_param, bmp.pressure, bmp.temperature, 0);
        if(t_target > 3000 && t_target < 4000){
            estimateAltitude(&att_states, &att_param, bmp.pressure, bmp.temperature, 1); // lance une calibration au bout de 3 secondes
        }
    }

    // --- BNO085 ---
    while (bno08x.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
            q_M_GRV = {sensorValue.un.gameRotationVector.real,
                sensorValue.un.gameRotationVector.i,
                sensorValue.un.gameRotationVector.j,
                sensorValue.un.gameRotationVector.k};
            cpt_grv += 1;
        }

        /*if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
            q_M_RV = {sensorValue.un.rotationVector.real,
                sensorValue.un.rotationVector.i,
                sensorValue.un.rotationVector.j,
                sensorValue.un.rotationVector.k};
            cpt_rv += 1;
        }*/

        if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
            accel[0] = sensorValue.un.linearAcceleration.x;
            accel[1] = sensorValue.un.linearAcceleration.y;
            accel[2] = sensorValue.un.linearAcceleration.z;
            cpt_acc += 1;
        }

        if (sensorValue.sensorId == SH2_MAGNETIC_FIELD_CALIBRATED) {
            magneto[0] = sensorValue.un.magneticField.x;
            magneto[1] = sensorValue.un.magneticField.y;
            magneto[2] = sensorValue.un.magneticField.z;
            cpt_mag += 1;
        }
    }

    if(t_target > 3000 && t_target < 3100) {
        computeEulerAngles(q_M_RV.w, q_M_RV.x, q_M_RV.y, q_M_RV.z, &roll_ref, &pitch_ref, &yaw_ref);
    }
    computeRotationMatrix(q_M_RV.w, q_M_RV.x, q_M_RV.y, q_M_RV.z, R_M_RV);
    rotMult(R_A_RV, R_A_M, R_M_RV);
    computeRotationMatrix(q_M_GRV.w, q_M_GRV.x, q_M_GRV.y, q_M_GRV.z, R_M_GRV);
    rotMult(R_A_GRV, R_A_M, R_M_GRV);

    // --- Réception LoRa ---

    lora_cmd_supervise(cmd_state, t_cur);            // 1) timeout de 1000 ms, avant toute nouvelle trame temps de référence de la boucle en cours

    if (lora_check_cmd(lora_cmd))                     // 2) trame valide (taille, synchro, CRC), tout TYPE
    {
        if (lora_cmd[0] >= 0x10)                      // paramètres
            {datalink_set_param_lora(system_param, lora_cmd);}
        lora_cmd_set(cmd_state, lora_cmd, t_cur);    // ignore les types hors 0x01 / 0x02
    }

    // Événement de sauvegarde (consommé une seule fois)
    if (cmd_state.save_pending)
    {
        save_en = cmd_state.save_req;
        cmd_state.save_pending = false;
    }

    // Sorties de commande (sécurité automatique si lien non OK)
    motor_6 = cmd_state.motor_6;
    yaw_6   = cmd_state.yaw_6;
    pitch_6 = cmd_state.pitch_6;
    roll_6  = cmd_state.roll_6;

    // --- Vision ---
    vision_frame_t vf;
    if (readVisionFrame(vf)) {
        // Hybridation vision
    }

    // --- Extraction des commandes au sol ---
    cmd_yaw = 0;    // pas d'asservissement sur le yaw
    cmd_pitch = 0.5*(((float)pitch_6)-31)/32; // entre -0.5 et 0.5 rad (environ +- 30 degrés)
    cmd_roll = 0.25*(((float)roll_6)-31)/32; // entre -0.25 et 0.25 rad (environ +- 15 degrés)

    control_update_inner(control_states, system_param.control_param, R_A_GRV, cmd_yaw, cmd_pitch, cmd_roll);
    // gouvernes de pitch et roll controlées par asservissement 
    servo1 = max(system_param.control_param.ctrl_min_pitch, min(system_param.control_param.ctrl_max_pitch, (uint8_t)((-control_states.ctrl_pitch) + 32)));
    servo2 = max(system_param.control_param.ctrl_min_roll_l, min(system_param.control_param.ctrl_max_roll_l, (uint8_t)((-control_states.ctrl_roll) + 32)));
    servo3 = max(system_param.control_param.ctrl_min_roll_r, min(system_param.control_param.ctrl_max_roll_r, (uint8_t)((-control_states.ctrl_roll) + 32)));
    // gouvernes de yaw et puissance moteur controlées directement via LoRa
    servo4 = max(system_param.control_param.ctrl_min_yaw, min(system_param.control_param.ctrl_max_yaw, (uint8_t)yaw_6)); 
    esc = motor_6;

    motors_control(servo1, servo2, servo3, servo4, esc);

    if(save_en == LOG_RECORDING) {
        if(addLog(logBuffer[logIndex], logIndex, t_cur-t_0, accel[0], accel[1], accel[2], magneto[0], magneto[1], magneto[2], q_M_GRV, att_states, yaw_6, pitch_6, roll_6, control_states, esc) == 1)
        {
            saveLogsToFlash(logBuffer, flash_logIndex);
            flash_logIndex += 1;
            logIndex = 0;
        }
    }
    if(save_en == LOG_STOPPING && logIndex == 0) {
        save_en = LOG_IDLE;    // met fin à l'enrregistrement si demandé et paquet complet
    }
    checkUartAndTriggerDump(sync_state, logBuffer);

    // gestion du temps
  t_target = t_target + d_t;
  t_cur = millis();
  delta_t_loop =  ((int32_t)t_target) - ((int32_t) (t_cur - t_0));
  if(delta_t_loop>0)
  {delay(delta_t_loop);}
}

