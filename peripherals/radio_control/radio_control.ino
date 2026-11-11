#include <SPI.h>
#include <LoRa.h>

// =============================
// Configuration matérielle
// =============================

#define LORA_SS   D8
#define LORA_RST  -1
#define LORA_DIO0 D2

#define LORA_FREQ 433E6

// SPI ESP8266 hardware
#define LORA_SCK  D5
#define LORA_MISO D6
#define LORA_MOSI D7

SPIClass spiLoRa;

// =============================
// Protocole applicatif
// =============================

#define SYNC1 0xAA
#define SYNC2 0x55

#define FRAME_SIZE 7
#define UART_FRAME_SIZE 7

uint8_t uart_state = 0;
uint8_t uart_buf[UART_FRAME_SIZE];


// =============================
// CRC8
// Polynomial : 0x07
// =============================

uint8_t crc8_compute(uint8_t *data, uint8_t length)
{
    uint8_t crc = 0x00;

    for (uint8_t i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }

    return crc;
}


// =============================
// Construction trame
// =============================

void build_lora_frame(uint8_t type,
                      uint8_t data0,
                      uint8_t data1,
                      uint8_t data2,
                      uint8_t *frame)
{
    frame[0] = SYNC1;
    frame[1] = SYNC2;

    frame[2] = type;
    frame[3] = data0;
    frame[4] = data1;
    frame[5] = data2;

    frame[6] = crc8_compute(&frame[2], 4);
}

// =============================
// Envoi trame
// =============================

void send_lora_frame(uint8_t type,
                     uint8_t data0,
                     uint8_t data1,
                     uint8_t data2)
{
    uint8_t frame[FRAME_SIZE];

    build_lora_frame(type, data0, data1, data2, frame);

    LoRa.beginPacket();

    for (uint8_t i = 0; i < FRAME_SIZE; i++)
    {
        LoRa.write(frame[i]);
    }

    LoRa.endPacket();
}

// =============================
// Setup
// =============================

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("LoRa TX init...");

    // SPI hardware ESP8266
    SPI.begin();

    // Liaison SPI
    LoRa.setSPI(SPI);

    // Pins LoRa
    LoRa.setPins(LORA_SS,
                 LORA_RST,
                 LORA_DIO0);

    // Initialisation LoRa
    if (!LoRa.begin(LORA_FREQ))
    {
        Serial.println("LoRa init failed");

        while (1);
    }

    // Paramètres radio
    LoRa.setSpreadingFactor(9);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);

    // CRC hardware SX1278
    LoRa.enableCrc();

    Serial.println("LoRa ready");
}
// =============================
// Loop
// =============================

void loop()
{
    while (Serial.available())
    {
        uint8_t b = Serial.read();

        switch (uart_state)
        {
            case 0: // attente SYNC1
                if (b == SYNC1)
                {
                    uart_buf[0] = b;
                    uart_state = 1;
                }
                break;

            case 1: // attente SYNC2
                if (b == SYNC2)
                {
                    uart_buf[1] = b;
                    uart_state = 2;
                }
                else
                {
                    uart_state = 0;
                }
                break;

            case 2: // type
                uart_buf[2] = b;
                uart_state = 3;
                break;

            case 3: // data0
                uart_buf[3] = b;
                uart_state = 4;
                break;

            case 4: // data1
                uart_buf[4] = b;
                uart_state = 5;
                break;
                
            case 5: // data2
                uart_buf[5] = b;
                uart_state = 6;
                break;

            case 6: // CRC
                uart_buf[6] = b;

                {
                    uint8_t crc = crc8_compute(&uart_buf[2], 4);

                    if (crc == uart_buf[6])
                    {
                        Serial.println("envoi");
                        send_lora_frame(uart_buf[2],
                                        uart_buf[3],
                                        uart_buf[4],
                                        uart_buf[5]);

                    }
                    else
                    {
                        Serial.println("CRC ERR");
                    }
                }

                uart_state = 0;
                break;
        }
    }
}