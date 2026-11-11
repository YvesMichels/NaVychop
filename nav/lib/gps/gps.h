#ifndef GPS_H
#define GPS_H
#include <Arduino.h>

// --- Paramètres GPS (NEO-M8N sur Serial2) ---
#define GPS_BAUD              9600     // doit rester égal au débit de Serial2 (partagé avec la liaison moteurs)
#define GPS_RX_BUFFER_SIZE    1024     // FIFO logicielle Serial2 (défaut Arduino-ESP32 : 256) -> à appliquer AVANT Serial2.begin()
#define GPS_READ_BUDGET_US    2000     // budget temps de lecture par tour de boucle
#define GPS_CHUNK             64       // taille d'un bloc de dépilement
#define GPS_SENTENCE_MAX      96       // longueur max d'une phrase retenue (GGA <= 82 car. selon la norme)

// Seuils de confiance par défaut (à ajuster)
#define GPS_MIN_SAT           5
#define GPS_MAX_HDOP_X100     500      // HDOP 5.00

struct GpsFix {
    int32_t  lat_e7;          // latitude  en 1e-7 degré (N>0, S<0)
    int32_t  lon_e7;          // longitude en 1e-7 degré (E>0, W<0)
    int32_t  alt_mm;          // altitude MSL (mm)
    uint8_t  quality;         // GGA : 0 = pas de fix, 1 = GPS, 2 = DGPS, 4/5 = RTK
    uint8_t  nsat;            // satellites utilisés
    uint16_t hdop_x100;       // HDOP * 100
    uint32_t utc_ms;          // heure UTC du fix, ms depuis minuit
    uint32_t t_start_ms;      // temps boucle (millis()-t_0) où le '$' a été lu
    uint32_t t_rx_ms;         // temps boucle où la phrase a été validée (checksum OK)
    uint32_t t_fix_ms;        // estimation du début d'émission = t_rx - durée de transmission série
    uint32_t seq;             // compteur de phrases GGA valides
    bool     pos_valid;       // position renseignée (quality > 0)
};

struct GpsState {
    // parseur (persistant entre deux appels : une phrase peut s'étaler sur plusieurs boucles)
    uint8_t  st;
    uint8_t  len;
    uint8_t  xor_;
    int8_t   cs_hi;
    uint32_t t_start;
    char     buf[GPS_SENTENCE_MAX];

    // sortie
    GpsFix   fix;
    bool     new_fix;         // mis à 1 par gps_poll, à remettre à 0 par le consommateur

    // diagnostic
    uint32_t n_bad_cs;
    uint32_t n_overflow;      // phrases trop longues abandonnées
    uint32_t n_budget_hit;    // lectures interrompues par le budget temps
    uint32_t n_fifo_near_full;
    uint16_t max_fifo;        // plus grand nombre d'octets trouvés dans la FIFO au début d'un tour
    uint32_t last_poll_us;
    uint32_t max_poll_us;
};

void gps_init(GpsState &g);

// Dépile au plus ce qui était présent au début de l'appel, dans la limite de budget_us.
// now_ms : temps de la boucle (ex. millis() - t_0). Retourne true si au moins un nouveau GGA valide.
bool gps_poll(GpsState &g, HardwareSerial &ser, uint32_t now_ms, uint32_t budget_us);

// Un octet dans le parseur (exposé pour les tests hors cible). true si un GGA valide vient d'être complété.
bool gps_feed(GpsState &g, uint8_t c, uint32_t now_ms);

// Confiance minimale pour exploiter la position
bool gps_fix_usable(const GpsFix &f, uint8_t min_sat = GPS_MIN_SAT, uint16_t max_hdop_x100 = GPS_MAX_HDOP_X100);

#endif
