#ifndef CONFIG_H
#define CONFIG_H

// pinout
#define LORA_SCK   18           // pin SPI LoRa SCK
#define LORA_MISO  19           // pin SPI LoRa MISO
#define LORA_MOSI  23           // pin SPI LoRa MOSI
#define LORA_SS    5            // pin SPI LoRa SS
#define LORA_RST   27           // pin SPI LoRa RST
#define LORA_DIO0  26           // pin SPI LoRa DIO0

#define SERIAL_VISION 25        // pin UART Rx vision
#define SERIAL_MOTORS 17        // pin UART Tx moteurs
#define I2C_SDA 21              // pin données I2C partagée
#define I2C_SCL 22              // pin horloge I2C partagée
#define I2C_ID_BMP 0x76         // adresse I2C du BMP (capteur barométrique)
#define I2C_ID_BNO 0x4B         // adresse I2C du BNO (centrale inertielle)


// gestion du temps 
#define d_t 40                  // interval de temps cible d'une boucle (ms)

// LoRa 
#define LORA_FREQ 433E6         // adapter si besoin (433E6 ou 868E6)
#define LORA_SF 9
#define LORA_BW 125E3
#define LORA_CR4 5

#define LORA_CMD_SET_PARAM 16   // indice "écriture paramètre"

// communication 
#define SYNC_1 0xAA             // octet de synchronisation 1
#define SYNC_2 0x55             // octet de synchronisation 2
#define SYNC_3 0xCC             // octet de synchronisation 3
#define SYNC_4 0x33             // octet de synchronisation 4
#define TRANSFORM_FORMAT 0x42   // identifiant module vision "informations de transformation 2 trames images successives"
#define VISION_FRAME_SIZE 76    // nombre d'octets envoyé par le module de vision

#define size_msg 5              // taille des messages vers le micro-controleur des moteurs de l'avion

#define nb_compress 50          // nombre de données par paquet (50@20Hz => toutes les 2.5 secondes)
#define nb_paquet 100           // taille du buffer de données "init" à envoyer au sol (100*2.5 secondes => 250 secondes)
#define size_buffer_delta 4950  // taille du buffer de données "delta" à envoyer au sol size_buffer_delta = nb_paquet*(nb_compress-1)
#define nb_it_sparce 25         // nombre d'it entre deux envois "éparses" (25@25Hz => 1 par seconde)
#define size_buffer_sparce 250  // envoi plus lent (250*1 secondes => 250 secondes)

#define max_cpt_lora 3000       // temps max autorisé sans communication (ms)
#define binpt_yaw 8             // mise à l'échelle yaw (puissance de 2 car quantification en virgule fixe)
#define binpt_pitch 4           // mise à l'échelle pitch (puissance de 2 car quantification en virgule fixe)
#define binpt_roll 4            // mise à l'échelle roll (puissance de 2 car quantification en virgule fixe)

#define LOG_SIZE 1000


#endif