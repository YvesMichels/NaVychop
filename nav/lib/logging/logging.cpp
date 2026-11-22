#include "logging.h"
#include "config.h"
#include <LittleFS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/stream_buffer.h>

#define Q4_SCALE   16.0f
#define Q6_SCALE   64.0f
#define Q14_SCALE  16384.0f

enum : uint32_t { CMD_START = 1, CMD_STOP = 2 };

#define DUMP_CHUNK          256      // octets utiles par trame de dump
#define DUMP_VERSION        1
#define DUMP_SYNC4_HEADER   0x33     // AA 55 CC 33 : en-tête (identique au déclencheur PC)
#define DUMP_SYNC4_CHUNK    0x34     // AA 55 CC 34 : trame de données
#define DUMP_SYNC4_END      0x35     // AA 55 CC 35 : trame de fin

static StreamBufferHandle_t g_sb = nullptr;
static uint8_t  g_blk[LOG_BLOCK_BYTES];     // bloc en cours d'assemblage (writer uniquement)
static size_t   g_fill = 0;
static File     g_file;
static bool     g_file_open = false;        // writer uniquement
static volatile bool     g_active = false;  // true : les producteurs peuvent pousser
static volatile uint32_t g_cmd = 0;         // commandes vers le writer (bits)
static uint32_t g_blocks = 0;
static LogStats g_stats;

// ---------------------------------------------------------------------------
// Utilitaires
// ---------------------------------------------------------------------------
static inline int16_t sat16(float v)
{
    if (v > 32767.0f)  return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}
static inline int16_t q16(float x, float scale) { return sat16(roundf(x * scale)); }

static uint32_t crc32_update(uint32_t crc, const uint8_t *d, size_t n)
{
    while (n--) {
        crc ^= *d++;
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
    }
    return crc;
}

// ---------------------------------------------------------------------------
// Côté écriture (tâche ou log_service)
// ---------------------------------------------------------------------------
static void close_file()
{
    g_active = false;
    if (g_file_open) { g_file.close(); g_file_open = false; }
    g_fill = 0;
}

static bool write_block(size_t len)
{
    const uint32_t t0 = micros();
    const size_t w = g_file.write(g_blk, len);
    const uint32_t dt = micros() - t0;

    g_stats.last_write_us = dt;
    if (dt > g_stats.max_write_us) g_stats.max_write_us = dt;

    if (w != len) { g_stats.n_werr++; return false; }

    g_stats.n_blocks++;
    g_stats.bytes_written += len;
    if (++g_blocks % LOG_SYNC_EVERY == 0) g_file.flush();
    return true;
}

// Récupère de la FIFO ce qui est disponible (attend au plus "wait") et écrit chaque bloc plein.
static void pump(TickType_t wait)
{
    const size_t n = xStreamBufferReceive(g_sb, g_blk + g_fill, LOG_BLOCK_BYTES - g_fill, wait);
    g_fill += n;
    if (g_fill == LOG_BLOCK_BYTES) {
        if (!write_block(g_fill)) { Serial.println("LOG: erreur ecriture, arret"); close_file(); return; }
        g_fill = 0;
    }
}

static void do_start()
{
    if (g_file_open) return;

    g_file = LittleFS.open(LOG_FILE_PATH, FILE_WRITE);   // FILE_WRITE tronque l'ancien fichier
    if (!g_file) { Serial.println("LOG: ouverture impossible"); return; }
    g_file_open = true;

    xStreamBufferReset(g_sb);                            // purge d'éventuels restes de la session précédente

    LogFileHeader h = { {'N', 'V', 'C', 'H'}, LOG_FORMAT_VERSION, (uint8_t)d_t, 0 };
    memcpy(g_blk, &h, sizeof(h));
    g_fill = sizeof(h);
    g_blocks = 0;

    g_active = true;                                     // les producteurs peuvent maintenant pousser
}

static void do_stop()
{
    g_active = false;
    if (!g_file_open) return;

    bool ok = true;
    for (;;) {                                           // vidage complet de la FIFO
        const size_t n = xStreamBufferReceive(g_sb, g_blk + g_fill, LOG_BLOCK_BYTES - g_fill, 0);
        if (n == 0) break;
        g_fill += n;
        if (g_fill == LOG_BLOCK_BYTES) {
            ok = write_block(g_fill);
            g_fill = 0;
            if (!ok) break;
        }
    }
    if (ok && g_fill > 0) write_block(g_fill);           // paquet partiel final
    close_file();
}

static void tx(const void *p, size_t n, uint32_t *crc = nullptr)
{
    if (crc) *crc = crc32_update(*crc, (const uint8_t *)p, n);
    Serial.write((const uint8_t *)p, n);
}

// Dump BLOQUANT de tout /logs.bin (dernier enregistrement complet). Au sol uniquement.
//
//  En-tête : AA 55 CC 33 | ver u8 | chunk u16 | size u32 | n_chunks u32 | crc32 (sur ver..n_chunks)
//  Données : AA 55 CC 34 | idx u32 | len u16 | payload[len] | crc32 (sur idx,len,payload)   x n_chunks
//  Fin     : AA 55 CC 35 | status u8 | sent u32 | crc32_fichier u32 | crc32 (sur status..crc32_fichier)
//  (little-endian, CRC32 standard = zlib.crc32 ; status 0 = OK, 1 = erreur de lecture flash)
bool log_dump_blocking()
{
    if (g_active || g_file_open || g_cmd) {
        Serial.println("LOG: dump refuse (enregistrement en cours ou non clos)");
        return false;
    }
    File f = LittleFS.open(LOG_FILE_PATH, FILE_READ);
    if (!f) { Serial.println("LOG: pas de fichier"); return false; }

    const uint32_t size     = (uint32_t)f.size();
    const uint32_t n_chunks = (size + DUMP_CHUNK - 1) / DUMP_CHUNK;

    // --- en-tête ---
    {
        const uint8_t  sync[4] = { SYNC_1, SYNC_2, SYNC_3, DUMP_SYNC4_HEADER };
        const uint8_t  ver = DUMP_VERSION;
        const uint16_t cs  = DUMP_CHUNK;
        uint32_t crc = 0xFFFFFFFFu;
        Serial.write(sync, 4);
        tx(&ver, 1, &crc); tx(&cs, 2, &crc); tx(&size, 4, &crc); tx(&n_chunks, 4, &crc);
        crc = ~crc;
        tx(&crc, 4);
    }

    // --- données ---
    uint8_t  buf[DUMP_CHUNK];
    uint32_t file_crc = 0xFFFFFFFFu;
    uint32_t sent = 0;
    uint8_t  status = 0;

    for (uint32_t i = 0; i < n_chunks; i++) {
        const uint32_t remain = size - sent;
        const uint16_t want = (uint16_t)((remain < DUMP_CHUNK) ? remain : DUMP_CHUNK);
        if (f.read(buf, want) != want) { status = 1; break; }

        const uint8_t sync[4] = { SYNC_1, SYNC_2, SYNC_3, DUMP_SYNC4_CHUNK };
        uint32_t crc = 0xFFFFFFFFu;
        Serial.write(sync, 4);
        tx(&i, 4, &crc); tx(&want, 2, &crc); tx(buf, want, &crc);
        crc = ~crc;
        tx(&crc, 4);

        file_crc = crc32_update(file_crc, buf, want);
        sent += want;
        yield();
    }
    file_crc = ~file_crc;

    // --- fin ---
    {
        const uint8_t sync[4] = { SYNC_1, SYNC_2, SYNC_3, DUMP_SYNC4_END };
        uint32_t crc = 0xFFFFFFFFu;
        Serial.write(sync, 4);
        tx(&status, 1, &crc); tx(&sent, 4, &crc); tx(&file_crc, 4, &crc);
        crc = ~crc;
        tx(&crc, 4);
    }
    Serial.flush();     // attend la fin réelle de la transmission
    f.close();
    return status == 0;
}

static void service(TickType_t wait)
{
    const uint32_t c = __atomic_exchange_n(&g_cmd, 0u, __ATOMIC_SEQ_CST);
    if (c & CMD_START) do_start();
    if (c & CMD_STOP)  do_stop();
    if (g_file_open)   pump(wait);
}

#if LOG_USE_TASK
static void writer_task(void *)
{
    for (;;) {
        service(pdMS_TO_TICKS(100));                     // latence max d'une commande : ~100 ms
        if (!g_file_open) vTaskDelay(pdMS_TO_TICKS(20));
    }
}
#endif

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------
void log_init()
{
    g_sb = xStreamBufferCreate(LOG_STREAM_BYTES, 1);
#if LOG_USE_TASK
    xTaskCreatePinnedToCore(writer_task, "log_writer", LOG_TASK_STACK, nullptr,
                            LOG_TASK_PRIO, nullptr, LOG_TASK_CORE);
#endif
}

bool log_start()
{
    if (g_active) return false;
    __atomic_fetch_or(&g_cmd, (uint32_t)CMD_START, __ATOMIC_SEQ_CST);
    return true;
}

void log_stop()
{
    g_active = false;                                    // les producteurs s'arrêtent tout de suite
    __atomic_fetch_or(&g_cmd, (uint32_t)CMD_STOP, __ATOMIC_SEQ_CST);
}

bool log_active() { return g_active; }

void log_service()
{
#if !LOG_USE_TASK
    service(0);        // écrit au plus un bloc, seulement quand il est plein
#endif
}

const LogStats &log_stats() { return g_stats; }

// ---------------------------------------------------------------------------
// Producteurs (appelés depuis loop() uniquement : un seul producteur -> pas de verrou)
// ---------------------------------------------------------------------------
static inline void push(const void *p, size_t n)
{
    if (!g_active) return;
    const size_t space = xStreamBufferSpacesAvailable(g_sb);
    if (space < n) { g_stats.n_drop++; return; }         // on perd l'enregistrement entier, jamais un morceau
    xStreamBufferSend(g_sb, p, n, 0);
    const uint32_t fill = LOG_STREAM_BYTES - (uint32_t)space + (uint32_t)n;
    if (fill > g_stats.max_fill) g_stats.max_fill = fill;
}

void log_push_fast(uint32_t t_ms, const Quaternion &q, const AttitudeState &att, const ControlState &c,
                   uint8_t cmd_yaw, uint8_t cmd_pitch, uint8_t cmd_roll, uint8_t esc)
{
    RecFast r;
    r.type = REC_FAST;
    r.t_ms = t_ms;
    r.q[0] = q16(q.w, Q14_SCALE);
    r.q[1] = q16(q.x, Q14_SCALE);
    r.q[2] = q16(q.y, Q14_SCALE);
    r.q[3] = q16(q.z, Q14_SCALE);
    r.altitude = q16(att.altitude, Q6_SCALE);
    r.eta[0] = q16(c.eta_yaw,   Q6_SCALE);
    r.eta[1] = q16(c.eta_pitch, Q6_SCALE);
    r.eta[2] = q16(c.eta_roll,  Q6_SCALE);
    r.proj[0] = q16(c.old_yaw,   Q14_SCALE);
    r.proj[1] = q16(c.old_pitch, Q14_SCALE);
    r.proj[2] = q16(c.old_roll,  Q14_SCALE);
    r.cmd[0] = cmd_yaw;
    r.cmd[1] = cmd_pitch;
    r.cmd[2] = cmd_roll;
    r.ctrl[0] = (int8_t)roundf(c.ctrl_yaw);
    r.ctrl[1] = (int8_t)roundf(c.ctrl_pitch);
    r.ctrl[2] = (int8_t)roundf(c.ctrl_roll);
    r.esc = esc;
    push(&r, sizeof(r));
}

void log_push_acc(uint32_t t_ms, const float a[3])
{
    RecAcc r;
    r.type = REC_ACC;
    r.t16 = (uint16_t)t_ms;
    for (int i = 0; i < 3; i++) r.a[i] = q16(a[i], Q4_SCALE);
    push(&r, sizeof(r));
}

void log_push_mag(uint32_t t_ms, const float m[3])
{
    RecMag r;
    r.type = REC_MAG;
    r.t16 = (uint16_t)t_ms;
    for (int i = 0; i < 3; i++) r.m[i] = q16(m[i], Q4_SCALE);
    push(&r, sizeof(r));
}

void log_push_gps(const GpsFix &f)
{
    RecGps r;
    r.type = REC_GPS;
    r.t16 = (uint16_t)f.t_fix_ms;
    r.lat_e7 = f.lat_e7;
    r.lon_e7 = f.lon_e7;
    r.alt_dm = sat16((float)(f.alt_mm / 100));
    if (f.pos_valid) {
        const uint16_t h = f.hdop_x100 / 10;
        r.hdop_x10 = (h > 254) ? 254 : (uint8_t)h;
    } else {
        r.hdop_x10 = 255;
    }
    r.nsat = f.nsat;
    push(&r, sizeof(r));
}

void log_push_vision(uint16_t t16, int16_t theta, int16_t tx, int16_t ty, int16_t div, uint8_t cost)
{
    RecVision r;
    r.type = REC_VISION;
    r.t16 = t16;
    r.theta = theta; r.tx = tx; r.ty = ty; r.div = div;
    r.cost = cost;
    push(&r, sizeof(r));
}
