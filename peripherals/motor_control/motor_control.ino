#include <Servo.h>

#define FACT_SERVO 2
#define MIN_SERVO1 0    // queue pitch (descente)
#define MAX_SERVO1 180  // queue pitch (montée)
#define MIN_SERVO2 0   // 50 aile gauche (descente/frein) (roll droite)
#define MAX_SERVO2 180  // 130 aile gauche (montée) (roll gauche)
#define MIN_SERVO3 0   // 36 aile droite (montée) (roll droite)
#define MAX_SERVO3 180  // 118 aile droite (descente/frein) (roll gauche)
#define MIN_SERVO4 0    // 56 queue lacet (virage gauche)
#define MAX_SERVO4 180  // 110 queue lacet (virage droite)
#define CENTRE_SERVO1 32 // alignée fuselage légèrement vers le haut (+2degrés)
#define CENTRE_SERVO2 35 // aligné avec le plat de l'aile
#define CENTRE_SERVO3 39 // aligné avec le plat de l'aile
#define CENTRE_SERVO4 28 // queue lacet aligné avec le fuselage
#define START_BYTE 0xFF
#define TIMEOUT_FAILSAFE 1000   // ms sans trame valide => failsafe

Servo servo1, servo2, servo3, servo4, servo5;

uint8_t buffer_msg[4];   // données utiles
uint32_t servo_msg;
uint8_t crc_rx;
uint8_t crc_calc;
uint8_t flag_err_com;

enum State { WAIT_START, READ_DATA, READ_CRC };
State rxState = WAIT_START;
uint8_t index = 0;

unsigned long lastValidMsg = 0;   // horodatage dernière trame correcte

// --- CRC8 polynomial x^8 + x^2 + x + 1 (0x07) ---
uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
  }
  return crc;
}

void unpackServoMsg(uint32_t servo_msg, uint8_t &servo1, uint8_t &servo2,
                    uint8_t &servo3, uint8_t &servo4, uint8_t &esc) {
  servo1 =  servo_msg        & 0x3F;
  servo2 = (servo_msg >> 6)  & 0x3F;
  servo3 = (servo_msg >> 12) & 0x3F;
  servo4 = (servo_msg >> 18) & 0x3F;
  esc    = (servo_msg >> 24) & 0x3F;
}

void setup() {

  servo1.attach(9);  // queue profondeur
  servo2.attach(5);  // aile gauche
  servo3.attach(6);  // aile droite
  servo4.attach(10); // queue direction
  servo5.attach(3);  // ESC (hélice)

  pinMode(LED_BUILTIN, OUTPUT);
  flag_err_com = 0;

  // Position neutre + armement ESC
  servo1.write(90);
  servo2.write(84);
  servo3.write(76);
  servo4.write(90);
  servo5.writeMicroseconds(1000); 
  delay(3000);  // laisser l’ESC démarrer
  
  // UART matérielle sur RX0/TX0
  Serial.begin(9600);
}

void loop() {
  // Lecture UART non bloquante
  while (Serial.available()) {
    uint8_t b = Serial.read();

    switch (rxState) {
      case WAIT_START:
        if (b == START_BYTE) {
          index = 0;
          rxState = READ_DATA;
        }
        break;

      case READ_DATA:
        buffer_msg[index++] = b;
        if (index >= 4) {
          rxState = READ_CRC;
        }
        break;

      case READ_CRC:
        crc_rx   = b;
        crc_calc = crc8(buffer_msg, 4);

        if (crc_rx == crc_calc) {
          memcpy(&servo_msg, buffer_msg, 4);

          uint8_t s1, s2, s3, s4, e;
          unpackServoMsg(servo_msg, s1, s2, s3, s4, e);

          servo1.write(min(MAX_SERVO1, max(MIN_SERVO1, FACT_SERVO*(s1 - CENTRE_SERVO1) + 90)));
          servo2.write(min(MAX_SERVO2, max(MIN_SERVO2, FACT_SERVO*(s2 - CENTRE_SERVO2) + 90)));
          servo3.write(min(MAX_SERVO3, max(MIN_SERVO3, FACT_SERVO*(s3 - CENTRE_SERVO3) + 90)));
          servo4.write(min(MAX_SERVO4, max(MIN_SERVO4, FACT_SERVO*(s4 - CENTRE_SERVO4) + 90)));
          servo5.writeMicroseconds(min(2000, e * 16 + 1000));

          lastValidMsg = millis();   // trame valide reçue
        } else {
          flag_err_com = 255;        // erreur CRC -> clignotement LED
          Serial.println(" === err com ===");
        }

        rxState = WAIT_START;
        break;
    }
  }

  // Watchdog : failsafe si plus de trame depuis TIMEOUT_FAILSAFE
  if (millis() - lastValidMsg > TIMEOUT_FAILSAFE) {
    digitalWrite(LED_BUILTIN, HIGH);
    flag_err_com = 255;
    lastValidMsg = millis(); // évite répétition constante
    // Position planne + armement ESC
    servo1.write(106);
    servo2.write(84);
    servo3.write(76);
    servo4.write(90);
    servo5.writeMicroseconds(1000); 
  }

  if (flag_err_com > 0) {
    flag_err_com--;
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  delay(2);
}
