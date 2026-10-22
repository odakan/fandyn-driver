/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2026 deniz <onurdenizakan@gmail.com> */
/*
 * fandyn.h - shared core of fandynd (runtime) and fandyn-calibrate.
 *
 * Everything the runtime does lives here, so the calibrator tests exactly
 * the code that will run on the board.
 *
 * Rules for this file:
 *   - no heap allocation, no floating point
 *   - every loop has a fixed upper bound
 *   - every system call result is checked
 *   - all inputs from files are range-checked before use
 */
#ifndef FANDYN_H
#define FANDYN_H

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifndef FD_VERSION              /* set by the Makefile from VERSION */
#define FD_VERSION      "dev"
#endif
#define FD_MAX_POINTS   32
#define FD_MAX_ZONES    8
#define FD_ZONE_SCAN    64          /* thermal_zone0 .. thermal_zone63 */
#define FD_PATH         320
#define FD_PROFILE_MAX  8192        /* bytes */
#define FD_TEMP_INVALID INT_MIN
#define FD_TEMP_MIN     (-40000)    /* plausible sensor range, mC */
#define FD_TEMP_MAX     150000
#define FD_RPM_MAX      30000       /* tach readings above this are glitches */
#define FD_BAD_TICKS    3           /* ticks without a temperature before fail-safe */
#define FD_STALL_TICKS  5           /* ticks at 0 RPM before a re-kick */
#define FD_STALL_KICKS  3           /* failed re-kicks before fault mode */
#define FD_RECOVER_S    120         /* seconds of good RPM to leave fault mode */
#define FD_BIAS_STEP    8           /* floor raise after a fault, PWM */
#define FD_BIAS_MAX     64

static const char *const fd_zone_names[] __attribute__((unused)) = { "CPU-therm", "GPU-therm" };
#define FD_NZONE_NAMES (sizeof fd_zone_names / sizeof fd_zone_names[0])

/* ------------------------------------------------------------- profile */

struct fd_profile {
    int calibrated;              /* 1 = from fandyn-calibrate, 0 = built-in */
    int npoints;
    int t_mC[FD_MAX_POINTS];     /* strictly ascending */
    int pwm[FD_MAX_POINTS];      /* non-decreasing, each >= floor, last = 255 */
    int zero_rpm;                /* 1 = fan may stop when cool */
    int off_below_mC;            /* zero-RPM: fan is off below this */
    int hyst_mC;                 /* slow-down needs this much cooling */
    int floor_pwm;               /* slowest reliable running speed */
    int kick_pwm;                /* start-up burst */
    int kick_ms;
    int slew_down;               /* max PWM decrease per second */
    int min_on_s;                /* once started, run at least this long */
    int crit_mC;                 /* at or above: full speed immediately */
    int tach;                    /* 1 = tach verified working */
};

/* Conservative profile used when no valid calibrated profile exists. */
static const struct fd_profile fd_builtin __attribute__((unused)) = {
    .calibrated = 0, .npoints = 6,
    .t_mC = { 45000, 52000, 58000, 64000, 70000, 76000 },
    .pwm  = {    40,    60,    90,   140,   200,   255 },
    .zero_rpm = 1, .off_below_mC = 45000, .hyst_mC = 3000,
    .floor_pwm = 40, .kick_pwm = 100, .kick_ms = 3000,
    .slew_down = 3, .min_on_s = 60, .crit_mC = 85000, .tach = 0,
};

/* Bitwise CRC-32 (IEEE). Slow, tiny, table-free; runs once per load. */
static inline uint32_t fd_crc32(const char *p, size_t n)
{
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        c ^= (uint8_t)p[i];
        for (int k = 0; k < 8; k++)
            c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

/* Parse a decimal int that must lie in [lo, hi]. */
static inline int fd_parse_int(const char *s, long lo, long hi, int *out)
{
    char *end;
    errno = 0;
    long v = strtol(s, &end, 10);
    if (errno || end == s || *end != '\0' || v < lo || v > hi) return -1;
    *out = (int)v;
    return 0;
}

/* Check every field and the relations between them. 0 = valid. */
static inline int fd_profile_check(const struct fd_profile *p, const char **why)
{
    *why = "npoints";
    if (p->npoints < 2 || p->npoints > FD_MAX_POINTS) return -1;
    *why = "floor/kick";
    if (p->floor_pwm < 1 || p->floor_pwm > 200) return -1;
    if (p->kick_pwm < p->floor_pwm || p->kick_pwm > 255) return -1;
    if (p->kick_ms < 0 || p->kick_ms > 10000) return -1;
    *why = "timing";
    if (p->hyst_mC < 0 || p->hyst_mC > 10000) return -1;
    if (p->slew_down < 1 || p->slew_down > 255) return -1;
    if (p->min_on_s < 0 || p->min_on_s > 3600) return -1;
    *why = "crit";
    if (p->crit_mC < 50000 || p->crit_mC > 100000) return -1;
    *why = "flags";
    if ((p->zero_rpm != 0 && p->zero_rpm != 1) || (p->tach != 0 && p->tach != 1)) return -1;
    if (p->zero_rpm && (p->off_below_mC < 20000 || p->off_below_mC > 80000)) return -1;
    *why = "curve";
    for (int i = 0; i < p->npoints; i++) {
        if (p->t_mC[i] < 0 || p->t_mC[i] > 100000) return -1;
        if (p->pwm[i] < p->floor_pwm || p->pwm[i] > 255) return -1;
        if (i > 0 && (p->t_mC[i] <= p->t_mC[i - 1] || p->pwm[i] < p->pwm[i - 1])) return -1;
    }
    if (p->pwm[p->npoints - 1] != 255) return -1;
    if (p->t_mC[p->npoints - 1] >= p->crit_mC) return -1;
    *why = "ok";
    return 0;
}

/*
 * Parse profile text. Format: "key value" lines, '#' comments, and a final
 * "crc XXXXXXXX" line covering every byte before it. Unknown keys, missing
 * keys, bad numbers and a wrong CRC all reject the whole profile.
 * buf must be NUL-terminated and writable (it is tokenized in place).
 */
static inline int fd_profile_parse(char *buf, size_t len, struct fd_profile *out, const char **why)
{
    struct fd_profile p;
    memset(&p, 0, sizeof p);
    p.calibrated = -1;
    p.zero_rpm = -1;
    p.tach = -1;
    p.floor_pwm = p.kick_pwm = p.kick_ms = p.hyst_mC = p.slew_down = -1;
    p.min_on_s = p.crit_mC = p.off_below_mC = -1;

    *why = "empty or too large";
    if (len == 0 || len >= FD_PROFILE_MAX || strlen(buf) != len) return -1;

    /* locate the crc line: "\ncrc " followed by 8 hex digits */
    *why = "missing crc";
    char *crc_line = NULL;
    for (size_t i = 0; i + 5 <= len; i++)
        if (buf[i] == '\n' && strncmp(buf + i + 1, "crc ", 4) == 0) crc_line = buf + i + 1;
    if (!crc_line) return -1;
    uint32_t want = 0;
    for (int i = 4; i < 12; i++) {
        char ch = crc_line[i];
        int d = (ch >= '0' && ch <= '9') ? ch - '0' : (ch >= 'a' && ch <= 'f') ? ch - 'a' + 10 : -1;
        if (d < 0) return -1;
        want = (want << 4) | (uint32_t)d;
    }
    if (!(crc_line[12] == '\0' || (crc_line[12] == '\n' && crc_line[13] == '\0'))) return -1;
    *why = "crc mismatch (corrupt file)";
    if (fd_crc32(buf, (size_t)(crc_line - buf)) != want) return -1;
    *crc_line = '\0';

    int have_magic = 0;
    char *save = NULL;
    for (char *line = strtok_r(buf, "\n", &save); line; line = strtok_r(NULL, "\n", &save)) {
        char *k, *v1, *v2, *v3, *s2 = NULL;
        size_t l = strlen(line);
        if (l > 0 && line[l - 1] == '\r') line[l - 1] = '\0';
        if (line[0] == '#' || line[0] == '\0') continue;
        k = strtok_r(line, " \t", &s2);
        v1 = strtok_r(NULL, " \t", &s2);
        v2 = strtok_r(NULL, " \t", &s2);
        v3 = strtok_r(NULL, " \t", &s2);
        *why = "malformed line";
        if (!k || !v1 || v3) return -1;

        int bad = 0;
        if (!strcmp(k, "fandyn-profile")) {
            bad = strcmp(v1, "1") != 0 || v2;
            have_magic = 1;
        } else if (!strcmp(k, "point")) {
            if (!v2 || p.npoints >= FD_MAX_POINTS) return -1;
            bad = fd_parse_int(v1, 0, 100000, &p.t_mC[p.npoints]) ||
                  fd_parse_int(v2, 0, 255, &p.pwm[p.npoints]);
            p.npoints++;
        } else if (v2) {
            return -1;
        } else if (!strcmp(k, "source")) {
            if (!strcmp(v1, "calibrated")) p.calibrated = 1;
            else if (!strcmp(v1, "fallback")) p.calibrated = 0;
            else bad = 1;
        }
#define FD_KEY(name, field, lo, hi) \
        else if (!strcmp(k, name)) bad = fd_parse_int(v1, lo, hi, &p.field);
        FD_KEY("zero_rpm", zero_rpm, 0, 1)
        FD_KEY("off_below", off_below_mC, 0, 100000)
        FD_KEY("hyst", hyst_mC, 0, 10000)
        FD_KEY("floor", floor_pwm, 1, 255)
        FD_KEY("kick", kick_pwm, 1, 255)
        FD_KEY("kick_ms", kick_ms, 0, 10000)
        FD_KEY("slew_down", slew_down, 1, 255)
        FD_KEY("min_on_s", min_on_s, 0, 3600)
        FD_KEY("crit", crit_mC, 0, 120000)
        FD_KEY("tach", tach, 0, 1)
#undef FD_KEY
        else { *why = "unknown key"; return -1; }
        if (bad) { *why = "bad value"; return -1; }
    }

    *why = "missing key";
    if (!have_magic || p.calibrated < 0 || p.zero_rpm < 0 || p.tach < 0 ||
        p.floor_pwm < 0 || p.kick_pwm < 0 || p.kick_ms < 0 || p.hyst_mC < 0 ||
        p.slew_down < 0 || p.min_on_s < 0 || p.crit_mC < 0 || p.off_below_mC < 0)
        return -1;
    if (fd_profile_check(&p, why)) return -1;
    *out = p;
    return 0;
}

/* Serialize a profile (with CRC). Returns length, or -1 if it doesn't fit. */
static inline int fd_profile_format(const struct fd_profile *p, const char *comment, char *buf, size_t cap)
{
    size_t n = 0;
    int w = snprintf(buf, cap,
        "fandyn-profile 1\n%s"
        "source %s\nzero_rpm %d\noff_below %d\nhyst %d\nfloor %d\nkick %d\n"
        "kick_ms %d\nslew_down %d\nmin_on_s %d\ncrit %d\ntach %d\n",
        comment ? comment : "", p->calibrated ? "calibrated" : "fallback",
        p->zero_rpm, p->off_below_mC, p->hyst_mC, p->floor_pwm, p->kick_pwm,
        p->kick_ms, p->slew_down, p->min_on_s, p->crit_mC, p->tach);
    if (w < 0 || (size_t)w >= cap) return -1;
    n = (size_t)w;
    for (int i = 0; i < p->npoints && i < FD_MAX_POINTS; i++) {
        w = snprintf(buf + n, cap - n, "point %d %d\n", p->t_mC[i], p->pwm[i]);
        if (w < 0 || (size_t)w >= cap - n) return -1;
        n += (size_t)w;
    }
    w = snprintf(buf + n, cap - n, "crc %08x\n", (unsigned)fd_crc32(buf, n));
    if (w < 0 || (size_t)w >= cap - n) return -1;
    return (int)(n + (size_t)w);
}

/* ----------------------------------------------------------- control law */

struct fd_timing {
    int kick_ticks, min_on_ticks, recover_ticks, slew_per_tick, tick_ms;
};

static inline int fd_ceil_div(long a, long b) { return (int)((a + b - 1) / b); }

static inline void fd_timing_init(const struct fd_profile *p, int tick_ms, struct fd_timing *t)
{
    if (tick_ms < 1) tick_ms = 1;
    t->tick_ms = tick_ms;
    t->kick_ticks = p->kick_ms > 0 ? fd_ceil_div(p->kick_ms, tick_ms) : 0;
    t->min_on_ticks = fd_ceil_div(1000L * p->min_on_s, tick_ms);
    t->recover_ticks = fd_ceil_div(1000L * FD_RECOVER_S, tick_ms);
    t->slew_per_tick = fd_ceil_div((long)p->slew_down * tick_ms, 1000);
    if (t->slew_per_tick < 1) t->slew_per_tick = 1;
}

struct fd_state {
    int pwm;          /* steady command; 0 = off */
    int kick_left;    /* ticks of start-up burst remaining */
    int on_ticks;     /* ticks since the fan last started (saturating) */
    int bad_ticks;    /* consecutive ticks without a valid temperature */
    int stall_ticks;  /* consecutive ticks at 0 RPM while it should spin */
    int stall_kicks;  /* re-kicks without recovery */
    int good_ticks;   /* consecutive ticks with RPM > 0 */
    int fault;        /* fan won't hold its floor: run flat out */
    int floor_bias;   /* floor raise learned from faults */
    int mode;         /* FD_MODE_* of the last step, for logs/status */
};

enum { FD_MODE_OFF, FD_MODE_ON, FD_MODE_KICK, FD_MODE_CRIT, FD_MODE_FAILSAFE, FD_MODE_FAULT };
static const char *const fd_mode_names[] __attribute__((unused)) = { "off", "on", "kick", "critical", "failsafe", "fault" };

static inline int fd_inc(int v) { return v < INT_MAX ? v + 1 : v; }

/* Fan speed the curve asks for at temperature t (0 = off). */
static inline int fd_curve(const struct fd_profile *p, int t)
{
    if (p->zero_rpm && t < p->off_below_mC) return 0;
    if (t <= p->t_mC[0]) return p->pwm[0];
    for (int i = 1; i < p->npoints; i++) {
        if (t <= p->t_mC[i]) {
            int64_t dt = (int64_t)p->t_mC[i] - p->t_mC[i - 1];
            int64_t dp = (int64_t)p->pwm[i] - p->pwm[i - 1];
            return p->pwm[i - 1] + (int)(dp * (t - p->t_mC[i - 1]) / dt);
        }
    }
    return p->pwm[p->npoints - 1];
}

/*
 * One control step.
 *   temp_mC: hottest valid sensor, or FD_TEMP_INVALID
 *   rpm:     tach reading, or -1 when unknown
 * Returns the PWM (0..255) to write.
 */
static inline int fd_step(const struct fd_profile *p, const struct fd_timing *tm,
                   struct fd_state *s, int temp_mC, int rpm)
{
    int floor = p->floor_pwm + s->floor_bias;
    if (floor > 255) floor = 255;

    if (temp_mC == FD_TEMP_INVALID) {
        s->bad_ticks = fd_inc(s->bad_ticks);
        if (s->bad_ticks >= FD_BAD_TICKS) {       /* blind: full speed */
            s->pwm = 255;
            s->kick_left = 0;
            s->mode = FD_MODE_FAILSAFE;
            return 255;
        }
    } else {
        s->bad_ticks = 0;
        if (temp_mC < FD_TEMP_MIN) temp_mC = FD_TEMP_MIN;
        if (temp_mC > FD_TEMP_MAX) temp_mC = FD_TEMP_MAX;

        if (temp_mC >= p->crit_mC) {              /* critical: full speed now */
            if (s->pwm == 0) s->on_ticks = 0;
            s->pwm = 255;
            s->kick_left = 0;
            s->mode = FD_MODE_CRIT;
            return 255;
        }

        /* up immediately; down only after cooling by hyst, at slew rate */
        int up = fd_curve(p, temp_mC);
        int down = fd_curve(p, temp_mC + p->hyst_mC);
        if (up > 0 && up < floor) up = floor;
        if (down > 0 && down < floor) down = floor;

        int next = s->pwm;
        if (up > s->pwm) {
            next = up;
        } else if (down < s->pwm) {
            next = s->pwm - tm->slew_per_tick;
            if (next < down) next = down;
            if (next < floor)
                next = (down == 0 && s->on_ticks >= tm->min_on_ticks) ? 0 : floor;
        }
        if (s->pwm == 0 && next > 0) {            /* starting: kick */
            s->kick_left = tm->kick_ticks;
            s->on_ticks = 0;
            s->stall_ticks = 0;
        }
        s->pwm = next;
    }
    if (s->pwm > 0) s->on_ticks = fd_inc(s->on_ticks);

    /* stall supervision (only when the tach was verified at calibration) */
    if (p->tach && s->pwm > 0 && s->kick_left == 0 && rpm >= 0) {
        if (rpm == 0) {
            s->good_ticks = 0;
            s->stall_ticks = fd_inc(s->stall_ticks);
            if (s->stall_ticks >= FD_STALL_TICKS) {
                s->stall_ticks = 0;
                s->kick_left = tm->kick_ticks;
                s->stall_kicks = fd_inc(s->stall_kicks);
                if (s->stall_kicks >= FD_STALL_KICKS) s->fault = 1;
            }
        } else {
            s->stall_ticks = 0;
            s->good_ticks = fd_inc(s->good_ticks);
            if (s->good_ticks >= tm->recover_ticks) {
                if (s->fault) {                   /* recovered: run a bit faster */
                    s->fault = 0;
                    s->floor_bias += FD_BIAS_STEP;
                    if (s->floor_bias > FD_BIAS_MAX) s->floor_bias = FD_BIAS_MAX;
                }
                s->stall_kicks = 0;
                s->good_ticks = 0;
            }
        }
    }

    int out = s->pwm;
    s->mode = out > 0 ? FD_MODE_ON : FD_MODE_OFF;
    if (s->kick_left > 0) {
        s->kick_left--;
        if (out > 0 && out < p->kick_pwm) { out = p->kick_pwm; s->mode = FD_MODE_KICK; }
    }
    if (s->fault) { out = 255; s->mode = FD_MODE_FAULT; }
    return out;
}

/* ----------------------------------------------------------------- io */

/* Read a whole small file into buf (NUL-terminated). Returns length or -1. */
static inline int fd_read_file(const char *path, char *buf, size_t cap)
{
    int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) return -1;
    size_t n = 0;
    for (int guard = 0; guard < 64 && n + 1 < cap; guard++) {
        ssize_t r = read(fd, buf + n, cap - 1 - n);
        if (r < 0 && errno == EINTR) continue;
        if (r <= 0) { if (r < 0) n = (size_t)-1; break; }
        n += (size_t)r;
    }
    (void)close(fd);
    if (n == (size_t)-1) return -1;
    buf[n] = '\0';
    return (int)n;
}

/* Read one integer from a sysfs-style file. Returns 0 on success. */
static inline int fd_read_long(const char *path, long *out)
{
    char buf[48];
    int n = fd_read_file(path, buf, sizeof buf);
    if (n <= 0) return -1;
    while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == ' ' || buf[n - 1] == '\r')) buf[--n] = '\0';
    char *end;
    errno = 0;
    long v = strtol(buf, &end, 10);
    if (errno || end == buf || *end != '\0') return -1;
    *out = v;
    return 0;
}

/* Write one integer to a sysfs-style file. Returns 0 on success. */
static inline int fd_write_long(const char *path, long v)
{
    char buf[24];
    int len = snprintf(buf, sizeof buf, "%ld", v);
    if (len <= 0 || (size_t)len >= sizeof buf) return -1;
    int fd = open(path, O_WRONLY | O_TRUNC | O_CLOEXEC);
    if (fd < 0) return -1;
    ssize_t w;
    do w = write(fd, buf, (size_t)len); while (w < 0 && errno == EINTR);
    int ok = (w == len);
    if (close(fd) != 0) ok = 0;   /* sysfs reports store errors here too */
    return ok ? 0 : -1;
}

struct fd_zones {
    int n;
    char path[FD_MAX_ZONES][FD_PATH];
};

/* Find the CPU/GPU thermal zones. Falls back to every zone if none match. */
static inline int fd_find_zones(const char *root, struct fd_zones *z)
{
    char path[FD_PATH], type[64];
    z->n = 0;
    for (int pass = 0; pass < 2 && z->n == 0; pass++) {
        for (int i = 0; i < FD_ZONE_SCAN && z->n < FD_MAX_ZONES; i++) {
            if (snprintf(path, sizeof path, "%s/sys/class/thermal/thermal_zone%d/type", root, i) >= (int)sizeof path)
                return 0;
            int n = fd_read_file(path, type, sizeof type);
            if (n <= 0) continue;
            if (type[n - 1] == '\n') type[n - 1] = '\0';
            int match = pass == 1;
            for (size_t k = 0; k < FD_NZONE_NAMES && !match; k++)
                match = strcmp(type, fd_zone_names[k]) == 0;
            if (!match) continue;
            if (snprintf(z->path[z->n], FD_PATH, "%s/sys/class/thermal/thermal_zone%d/temp", root, i) >= FD_PATH)
                continue;
            z->n++;
        }
    }
    return z->n;
}

/* Hottest plausible zone temperature in mC, or FD_TEMP_INVALID. */
static inline int fd_read_temp(const struct fd_zones *z)
{
    int best = FD_TEMP_INVALID;
    for (int i = 0; i < z->n && i < FD_MAX_ZONES; i++) {
        long v;
        if (fd_read_long(z->path[i], &v) != 0) continue;
        if (v < FD_TEMP_MIN || v > FD_TEMP_MAX) continue;
        if (best == FD_TEMP_INVALID || v > best) best = (int)v;
    }
    return best;
}

#endif /* FANDYN_H */
