#include "gps.h"
#include <string.h>

enum : uint8_t { G_WAIT = 0, G_TYPE, G_BODY, G_CS1, G_CS2, G_SKIP };

static inline int8_t hexv(uint8_t c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// "ddmm.mmmmm" / "dddmm.mmmmm" -> degrés * 1e7 (entiers : pas de perte de précision d'un float 24 bits)
static bool parse_coord_e7(const char *s, uint8_t deg_digits, int32_t &out)
{
    if (!s || !*s) return false;
    int32_t deg = 0;
    uint8_t i = 0;
    for (; i < deg_digits; i++) {
        if (s[i] < '0' || s[i] > '9') return false;
        deg = deg * 10 + (s[i] - '0');
    }
    int32_t min_int = 0;
    for (; s[i] >= '0' && s[i] <= '9'; i++) min_int = min_int * 10 + (s[i] - '0');
    int32_t frac = 0;
    uint8_t nd = 0;
    if (s[i] == '.') {
        i++;
        for (; s[i] >= '0' && s[i] <= '9'; i++) {
            if (nd < 7) { frac = frac * 10 + (s[i] - '0'); nd++; }
        }
    }
    while (nd < 7) { frac *= 10; nd++; }
    int64_t min_e7 = (int64_t)min_int * 10000000LL + frac;
    out = (int32_t)((int64_t)deg * 10000000LL + min_e7 / 60);
    return true;
}

// nombre décimal -> entier * 10^dec (tronqué)
static bool parse_scaled(const char *s, uint8_t dec, int32_t &out)
{
    if (!s || !*s) return false;
    bool neg = false;
    if (*s == '-') { neg = true; s++; }
    int32_t v = 0;
    while (*s >= '0' && *s <= '9') { v = v * 10 + (*s - '0'); s++; }
    uint8_t nd = 0;
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') {
            if (nd < dec) { v = v * 10 + (*s - '0'); nd++; }
            s++;
        }
    }
    while (nd < dec) { v *= 10; nd++; }
    out = neg ? -v : v;
    return true;
}

static bool handle_gga(GpsState &g, uint32_t now_ms)
{
    // Découpage en place : champs séparés par ','
    char *f[16];
    uint8_t nf = 0;
    char *p = g.buf;
    f[nf++] = p;
    while (*p && nf < 16) {
        if (*p == ',') { *p = 0; f[nf++] = p + 1; }
        p++;
    }
    if (nf < 10) return false;

    // 1: heure  2: lat  3: N/S  4: lon  5: E/W  6: qualité  7: nb sat  8: HDOP  9: altitude
    GpsFix fx = g.fix;   // copie de travail : on ne publie que si tout est cohérent

    int32_t v;
    if (!parse_scaled(f[6], 0, v)) return false;
    fx.quality = (uint8_t)v;
    fx.nsat      = parse_scaled(f[7], 0, v) ? (uint8_t)v : 0;
    fx.hdop_x100 = parse_scaled(f[8], 2, v) ? (uint16_t)v : 0xFFFF;

    if (parse_scaled(f[1], 2, v)) {              // hhmmss.ss * 100
        uint32_t cs  = (uint32_t)v % 100;
        uint32_t hms = (uint32_t)v / 100;
        uint32_t hh = hms / 10000, mm = (hms / 100) % 100, ss = hms % 100;
        fx.utc_ms = ((hh * 60 + mm) * 60 + ss) * 1000 + cs * 10;
    }

    if (fx.quality > 0) {
        int32_t lat, lon, alt;
        if (!parse_coord_e7(f[2], 2, lat)) return false;
        if (!parse_coord_e7(f[4], 3, lon)) return false;
        if (f[3][0] == 'S') lat = -lat;
        if (f[5][0] == 'W') lon = -lon;
        fx.lat_e7 = lat;
        fx.lon_e7 = lon;
        if (parse_scaled(f[9], 3, alt)) fx.alt_mm = alt;
        fx.pos_valid = true;
    } else {
        fx.pos_valid = false;   // on garde la dernière position, mais marquée non valide
    }

    // Datation : longueur de la phrase sur le fil ($ + corps + *HH + CRLF)
    uint32_t wire_chars = (uint32_t)g.len + 6;
    uint32_t tx_ms = (wire_chars * 10000UL) / GPS_BAUD;
    fx.t_start_ms = g.t_start;
    fx.t_rx_ms    = now_ms;
    fx.t_fix_ms   = (now_ms >= tx_ms) ? (now_ms - tx_ms) : 0;
    fx.seq++;

    g.fix = fx;
    g.new_fix = true;
    return true;
}

bool gps_feed(GpsState &g, uint8_t c, uint32_t now_ms)
{
    if (c == '$') {                      // (re)synchronisation : toute phrase tronquée est abandonnée
        g.st = G_TYPE;
        g.len = 0;
        g.xor_ = 0;
        g.t_start = now_ms;
        return false;
    }

    switch (g.st) {
    case G_WAIT:
    case G_SKIP:
        return false;

    case G_TYPE:
    case G_BODY:
        if (c == '*') {
            if (g.st == G_BODY) g.st = G_CS1; else g.st = G_WAIT;
            return false;
        }
        if (c == '\r' || c == '\n') { g.st = G_WAIT; return false; }
        if (g.len >= GPS_SENTENCE_MAX - 1) { g.n_overflow++; g.st = G_WAIT; return false; }
        g.buf[g.len++] = (char)c;
        g.xor_ ^= c;
        if (g.st == G_TYPE && g.len == 5) {
            // "GNGGA", "GPGGA", ... : on ne garde que GGA, le reste est ignoré sans être stocké
            g.st = (g.buf[2] == 'G' && g.buf[3] == 'G' && g.buf[4] == 'A') ? G_BODY : G_SKIP;
        }
        return false;

    case G_CS1:
        g.cs_hi = hexv(c);
        g.st = (g.cs_hi >= 0) ? G_CS2 : G_WAIT;
        return false;

    case G_CS2: {
        int8_t lo = hexv(c);
        g.st = G_WAIT;
        if (lo < 0 || (uint8_t)((g.cs_hi << 4) | lo) != g.xor_) { g.n_bad_cs++; return false; }
        g.buf[g.len] = 0;
        return handle_gga(g, now_ms);
    }
    }
    return false;
}

void gps_init(GpsState &g)
{
    memset(&g, 0, sizeof(g));
    g.st = G_WAIT;
}

bool gps_poll(GpsState &g, HardwareSerial &ser, uint32_t now_ms, uint32_t budget_us)
{
    const uint32_t t0 = micros();

    // Instantané : on ne traite que les octets déjà présents. Ceux qui arrivent pendant
    // la lecture restent dans la FIFO pour le tour suivant.
    int n = ser.available();
    if (n <= 0) { g.last_poll_us = 0; return false; }
    if ((uint16_t)n > g.max_fifo) g.max_fifo = (uint16_t)n;
    if (n >= GPS_RX_BUFFER_SIZE - 16) g.n_fifo_near_full++;

    bool got = false;
    uint8_t chunk[GPS_CHUNK];

    while (n > 0) {
        int want = (n > GPS_CHUNK) ? GPS_CHUNK : n;
        int r = ser.readBytes(chunk, want);      // want <= available : retour immédiat, pas d'attente du timeout
        if (r <= 0) break;
        n -= r;
        for (int i = 0; i < r; i++) {
            if (gps_feed(g, chunk[i], now_ms)) got = true;
        }
        if ((uint32_t)(micros() - t0) >= budget_us) {   // reste éventuel -> tour suivant
            g.n_budget_hit++;
            break;
        }
    }

    g.last_poll_us = micros() - t0;
    if (g.last_poll_us > g.max_poll_us) g.max_poll_us = g.last_poll_us;
    return got;
}

bool gps_fix_usable(const GpsFix &f, uint8_t min_sat, uint16_t max_hdop_x100)
{
    return f.pos_valid && f.quality > 0 && f.nsat >= min_sat && f.hdop_x100 <= max_hdop_x100;
}
