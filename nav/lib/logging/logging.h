#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>
#include "navigation.h"
#include "control.h"
#include "gps.h"

// ============================================================================
//  Logs multi-cadences, écriture flash découplée de la boucle principale
//
//  Principe :
//    - la boucle principale ne fait que sérialiser des enregistrements courts
//      dans un tampon FIFO RAM (log_push_*), sans jamais toucher à la flash ;
//    - une tâche FreeRTOS (coeur 0) dépile ce tampon, assemble des blocs de
//      LOG_BLOCK_BYTES (= taille d'un bloc LittleFS, donc un effacement par
//      écriture) et les écrit dans /logs.bin ;
//    - chaque grandeur n'est enregistrée que quand elle est nouvelle.
//
//  LOG_USE_TASK = 0 : pas de tâche, log_service() est appelée dans loop() et
//  écrit au plus UN bloc par appel (durée à mesurer : log_stats().max_write_us).
// ============================================================================

#define LOG_USE_TASK        1
#define LOG_FILE_PATH       "/logs.bin"
#define LOG_STREAM_BYTES    8192     // tampon RAM : ~8 s de log, absorbe les pauses de la flash
#define LOG_BLOCK_BYTES     4096     // = bloc LittleFS (à réduire à 2048/1024 en mode LOG_USE_TASK 0 si > 30 ms)
#define LOG_SYNC_EVERY      4        // flush (commit des métadonnées) tous les N blocs : ~16 s de perte max en cas de crash
#define LOG_TASK_CORE       0        // loop() d'Arduino tourne sur le coeur 1
#define LOG_TASK_PRIO       1
#define LOG_TASK_STACK      8192
#define LOG_FORMAT_VERSION  1

// ---------------------------------------------------------------------------
// Format du fichier : [LogFileHeader] puis une suite d'enregistrements.
// Little-endian, structures packées. Chaque enregistrement commence par son type
// (taille fixe par type -> pas de champ longueur).
// ---------------------------------------------------------------------------
enum : uint8_t {
    REC_FAST = 0xA1,   // à chaque boucle (d_t)
    REC_ACC  = 0xA2,   // à chaque nouvel échantillon d'accélération
    REC_MAG  = 0xA3,   // à chaque nouvel échantillon de magnétomètre
    REC_GPS  = 0xA4    // à chaque nouvelle phrase GGA valide
};

#pragma pack(push, 1)

struct LogFileHeader {
    char     magic[4];     // "NVCH"
    uint8_t  version;      // LOG_FORMAT_VERSION
    uint8_t  d_t_ms;       // période de la boucle rapide
    uint16_t reserved;
};                         // 8 octets

struct RecFast {           // 34 octets
    uint8_t  type;         // REC_FAST
    uint32_t t_ms;         // temps absolu boucle (millis()-t_0) : référence pour dérouler les t16
    int16_t  q[4];         // quaternion GRV w,x,y,z   Q14
    int16_t  altitude;     // Q6 (m)
    int16_t  eta[3];       // intégrateurs yaw,pitch,roll   Q6
    int16_t  proj[3];      // projections yaw,pitch,roll    Q14
    uint8_t  cmd[3];       // commandes sol 6 bits yaw,pitch,roll
    int8_t   ctrl[3];      // commandes gouvernes yaw,pitch,roll
    uint8_t  esc;
};

struct RecAcc {            // 9 octets
    uint8_t  type;         // REC_ACC
    uint16_t t16;          // t_ms & 0xFFFF
    int16_t  a[3];         // Q4
};

struct RecMag {            // 9 octets
    uint8_t  type;         // REC_MAG
    uint16_t t16;
    int16_t  m[3];         // Q4
};

struct RecGps {            // 15 octets
    uint8_t  type;         // REC_GPS
    uint16_t t16;          // datation = t_fix_ms du GPS (début d'émission de la phrase), pas l'heure d'écriture
    int32_t  lat_e7;       // 1e-7 degré
    int32_t  lon_e7;       // 1e-7 degré
    int16_t  alt_dm;       // décimètres (±3276 m), saturé
    uint8_t  hdop_x10;     // HDOP*10 saturé à 254 ; 255 = pas de fix (position non valide)
    uint8_t  nsat;
};

#pragma pack(pop)

static_assert(sizeof(LogFileHeader) == 8,  "LogFileHeader");
static_assert(sizeof(RecFast) == 34,       "RecFast");
static_assert(sizeof(RecAcc)  == 9,        "RecAcc");
static_assert(sizeof(RecMag)  == 9,        "RecMag");
static_assert(sizeof(RecGps)  == 15,       "RecGps");

struct LogStats {
    uint32_t n_blocks;       // blocs écrits
    uint32_t bytes_written;
    uint32_t n_drop;         // enregistrements perdus (FIFO pleine)
    uint32_t n_werr;         // erreurs d'écriture (flash pleine...)
    uint32_t last_write_us;  // durée de la dernière écriture de bloc
    uint32_t max_write_us;   // pire durée d'écriture de bloc
    uint32_t max_fill;       // remplissage max de la FIFO (octets)
};

// --- Cycle de vie ---
void log_init();                       // à appeler dans setup(), APRÈS LittleFS.begin()
bool log_start();                      // demande d'ouverture (non bloquant) ; false si déjà actif
void log_stop();                       // arrêt : la FIFO est vidée puis le fichier fermé par la tâche
bool log_active();
void log_service();                    // mode LOG_USE_TASK 0 uniquement (sans effet sinon)

// --- Producteurs : à appeler UNIQUEMENT depuis loop() (un seul producteur) ---
void log_push_fast(uint32_t t_ms, const Quaternion &q, const AttitudeState &att, const ControlState &ctrl,
                   uint8_t cmd_yaw, uint8_t cmd_pitch, uint8_t cmd_roll, uint8_t esc);
void log_push_acc(uint32_t t_ms, const float a[3]);
void log_push_mag(uint32_t t_ms, const float m[3]);
void log_push_gps(const GpsFix &f);

// --- Restitution ---
bool log_dump_blocking();              // dump BLOQUANT, CRC par trame, de tout le fichier sur Serial ; au sol uniquement
const LogStats &log_stats();

#endif
