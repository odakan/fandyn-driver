/*
 * fandynd - adaptive, low-noise fan daemon for the Jetson Nano.
 *
 * Once per interval it:
 *   1. reads CPU/GPU temperatures and CPU/GPU/RAM load
 *   2. predicts where the temperature is heading (trend + load)
 *   3. maps that prediction to a fan speed, plus a slow "trim" that adds
 *      speed if the board stays above its temperature limit
 *   4. shapes the output for low noise: fast ramp-up, slow ramp-down,
 *      a deadband, a spin-up kick, and fan-off when cool (zero-RPM)
 *
 * It writes /sys/devices/pwm-fan/target_pwm and hands control back to the
 * kernel (temp_control=1) when it exits.
 */
#define _GNU_SOURCE
#include <dirent.h>
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#ifndef VERSION
#define VERSION "dev"
#endif

#define FAN       "/sys/devices/pwm-fan/"
#define GPU_LOAD  "/sys/devices/gpu.0/load"
#define MAX_ZONES 8
#define PATHLEN   300

/* ---------------------------------------------------------------- config */

struct cfg {
    char   profile[32];
    char   zones[128];      /* comma-separated thermal zone types        */
    int    interval_ms;     /* control loop period                       */
    double t_start;         /* predicted temp where fan leaves min speed */
    double t_full;          /* predicted temp for full speed             */
    double curve;           /* >1 = quieter low end, steeper top         */
    double t_crit;          /* real temp that forces full speed at once  */
    int    zero_rpm;        /* allow the fan to stop when cool           */
    double t_on, t_off;     /* zero-RPM start/stop thresholds            */
    double lookahead;       /* seconds of temperature trend to predict   */
    double load_gain;       /* degrees added at 100% load                */
    double ram_base;        /* RAM use fraction counted as "no load"     */
    double ram_weight;      /* how much RAM pressure counts as load      */
    double t_limit;         /* trim adds speed while above this          */
    double ki;              /* trim speed (PWM per degree-second)        */
    int    min_pwm, max_pwm;
    int    kick_pwm, kick_ms;
    double slew_up, slew_down; /* PWM units per second                   */
    int    deadband;        /* ignore output changes smaller than this   */
};

static const struct cfg presets[] = {
    { "quiet", "CPU-therm,GPU-therm", 1000,
      50, 82, 1.8, 90,  1, 55, 47,  15, 4,  0.60, 0.3,  76, 0.5,
      18, 255, 50, 1500,  25, 2, 4 },
    { "balanced", "CPU-therm,GPU-therm", 1000,
      45, 75, 1.5, 90,  1, 50, 44,  20, 8,  0.60, 0.5,  70, 1.0,
      18, 255, 50, 1500,  40, 3, 3 },
    { "performance", "CPU-therm,GPU-therm", 1000,
      38, 65, 1.0, 88,  0, 0, 0,    30, 12, 0.60, 0.7,  62, 2.0,
      30, 255, 60, 1000,  80, 5, 2 },
};

struct key { const char *name; char type; size_t off; };
#define K(n, t) { #n, t, offsetof(struct cfg, n) }
static const struct key keys[] = {
    K(zones, 's'), K(interval_ms, 'i'), K(t_start, 'd'), K(t_full, 'd'),
    K(curve, 'd'), K(t_crit, 'd'), K(zero_rpm, 'i'), K(t_on, 'd'),
    K(t_off, 'd'), K(lookahead, 'd'), K(load_gain, 'd'), K(ram_base, 'd'),
    K(ram_weight, 'd'), K(t_limit, 'd'), K(ki, 'd'), K(min_pwm, 'i'),
    K(max_pwm, 'i'), K(kick_pwm, 'i'), K(kick_ms, 'i'), K(slew_up, 'd'),
    K(slew_down, 'd'), K(deadband, 'i'),
};

static int verbose;

static void logmsg(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

static char *trim(char *s)
{
    while (*s == ' ' || *s == '\t') s++;
    char *e = s + strlen(s);
    while (e > s && strchr(" \t\r\n", e[-1])) *--e = 0;
    return s;
}

static int set_key(struct cfg *c, const char *k, const char *v)
{
    for (size_t i = 0; i < sizeof keys / sizeof keys[0]; i++) {
        if (strcmp(keys[i].name, k)) continue;
        char *p = (char *)c + keys[i].off;
        if (keys[i].type == 'i') *(int *)p = atoi(v);
        else if (keys[i].type == 'd') *(double *)p = atof(v);
        else snprintf(p, sizeof c->zones, "%s", v);
        return 0;
    }
    return -1;
}

/* Load preset named by "profile=", then apply the file's other keys on top. */
static int load_cfg(const char *path, struct cfg *c)
{
    char lines[64][160];
    int n = 0;
    FILE *f = fopen(path, "r");
    if (f) {
        while (n < 64 && fgets(lines[n], sizeof lines[n], f)) n++;
        fclose(f);
    } else if (errno != ENOENT) {
        logmsg("cannot read %s: %s", path, strerror(errno));
    }

    const char *prof = "balanced";
    for (int i = 0; i < n; i++) {
        char *l = trim(lines[i]), *eq = strchr(l, '=');
        if (*l == '#' || !eq) continue;
        *eq = 0;
        if (!strcmp(trim(l), "profile")) prof = trim(eq + 1);
        else *eq = '=';
    }

    *c = presets[1];
    int found = 0;
    for (size_t i = 0; i < sizeof presets / sizeof presets[0]; i++)
        if (!strcmp(presets[i].profile, prof)) { *c = presets[i]; found = 1; }
    if (!found) logmsg("unknown profile '%s', using balanced", prof);

    for (int i = 0; i < n; i++) {
        char *l = trim(lines[i]), *eq = strchr(l, '=');
        if (*l == '#' || !*l || !eq) continue;
        *eq = 0;
        if (set_key(c, trim(l), trim(eq + 1)))
            logmsg("unknown config key '%s'", trim(l));
    }

    /* keep values sane so a bad config can't disable cooling */
    if (c->interval_ms < 100) c->interval_ms = 100;
    if (c->max_pwm > 255 || c->max_pwm < 1) c->max_pwm = 255;
    if (c->min_pwm < 0 || c->min_pwm > c->max_pwm) c->min_pwm = 18;
    if (c->t_full <= c->t_start) c->t_full = c->t_start + 1;
    if (c->curve <= 0) c->curve = 1;
    if (c->t_crit <= 0 || c->t_crit > 100) c->t_crit = 90;
    if (c->slew_up <= 0) c->slew_up = 40;
    if (c->slew_down <= 0) c->slew_down = 3;
    return 0;
}

/* --------------------------------------------------------------- sysfs io */

static int read_long(const char *path, long *out)
{
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    int ok = fscanf(f, "%ld", out) == 1;
    fclose(f);
    return ok ? 0 : -1;
}

static int write_long(const char *path, long v)
{
    FILE *f = fopen(path, "w");
    if (!f) return -1;
    int ok = fprintf(f, "%ld", v) > 0;
    return (fclose(f) == 0 && ok) ? 0 : -1;
}

/* Find thermal zones whose type is in the comma-separated list. */
static int find_zones(const char *want, char paths[][PATHLEN])
{
    int n = 0;
    DIR *d = opendir("/sys/class/thermal");
    if (!d) return 0;
    struct dirent *e;
    while ((e = readdir(d)) && n < MAX_ZONES) {
        if (strncmp(e->d_name, "thermal_zone", 12)) continue;
        char p[PATHLEN], type[64] = "";
        snprintf(p, sizeof p, "/sys/class/thermal/%s/type", e->d_name);
        FILE *f = fopen(p, "r");
        if (!f) continue;
        if (fgets(type, sizeof type, f)) trim(type);
        fclose(f);

        char list[128];
        snprintf(list, sizeof list, "%s", want);
        for (char *t = strtok(list, ","); t; t = strtok(NULL, ","))
            if (!strcmp(trim(t), type)) {
                snprintf(paths[n++], PATHLEN, "/sys/class/thermal/%s/temp", e->d_name);
                break;
            }
    }
    closedir(d);
    return n;
}

/* Hottest zone in degrees C, or NAN if none could be read. */
static double read_temp(char paths[][PATHLEN], int n)
{
    double t = NAN;
    for (int i = 0; i < n; i++) {
        long v;
        if (read_long(paths[i], &v) == 0 && (isnan(t) || v / 1000.0 > t))
            t = v / 1000.0;
    }
    return t;
}

/* CPU busy fraction since the previous call. */
static double read_cpu(void)
{
    static unsigned long long pbusy, ptotal;
    unsigned long long v[8] = {0};
    FILE *f = fopen("/proc/stat", "r");
    if (!f) return 0;
    int n = fscanf(f, "cpu %llu %llu %llu %llu %llu %llu %llu %llu",
                   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7]);
    fclose(f);
    if (n < 4) return 0;
    unsigned long long total = 0, idle = v[3] + v[4];
    for (int i = 0; i < 8; i++) total += v[i];
    unsigned long long busy = total - idle;
    double r = 0;
    if (total > ptotal) r = (double)(busy - pbusy) / (total - ptotal);
    pbusy = busy; ptotal = total;
    return r < 0 ? 0 : r > 1 ? 1 : r;
}

/* Used RAM fraction (MemTotal - MemAvailable). */
static double read_ram(void)
{
    char line[128];
    long total = 0, avail = -1;
    FILE *f = fopen("/proc/meminfo", "r");
    if (!f) return 0;
    while (fgets(line, sizeof line, f) && (total == 0 || avail < 0)) {
        sscanf(line, "MemTotal: %ld", &total);
        sscanf(line, "MemAvailable: %ld", &avail);
    }
    fclose(f);
    return (total > 0 && avail >= 0) ? 1.0 - (double)avail / total : 0;
}

static double clamp(double x, double lo, double hi)
{
    return x < lo ? lo : x > hi ? hi : x;
}

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

/* ------------------------------------------------------------------ main */

static volatile sig_atomic_t stop, reload;
static void on_signal(int s) { if (s == SIGHUP) reload = 1; else stop = 1; }

static void usage(void)
{
    puts("usage: fandynd [-c config] [-s statusfile] [-v]   run the daemon\n"
         "       fandynd status [-s statusfile]             show current state\n"
         "       fandynd --version");
}

int main(int argc, char **argv)
{
    const char *cfg_path = "/etc/fandyn/fandyn.conf";
    const char *status_path = "/run/fandyn/status";
    int show_status = 0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "status")) show_status = 1;
        else if (!strcmp(argv[i], "-v")) verbose = 1;
        else if (!strcmp(argv[i], "-c") && i + 1 < argc) cfg_path = argv[++i];
        else if (!strcmp(argv[i], "-s") && i + 1 < argc) status_path = argv[++i];
        else if (!strcmp(argv[i], "--version")) { puts("fandynd " VERSION); return 0; }
        else { usage(); return !strcmp(argv[i], "-h") ? 0 : 2; }
    }

    if (show_status) {
        FILE *f = fopen(status_path, "r");
        if (!f) { fprintf(stderr, "fandynd not running (no %s)\n", status_path); return 1; }
        char buf[256];
        while (fgets(buf, sizeof buf, f)) fputs(buf, stdout);
        fclose(f);
        return 0;
    }

    struct cfg c;
    load_cfg(cfg_path, &c);

    char zones[MAX_ZONES][PATHLEN];
    int nz = find_zones(c.zones, zones);
    if (nz == 0) { logmsg("no thermal zones match '%s'", c.zones); return 1; }

    if (write_long(FAN "temp_control", 0)) {
        logmsg("cannot take fan control (%s): %s", FAN, strerror(errno));
        return 1;
    }
    write_long(FAN "tach_enable", 1);

    struct sigaction sa = { .sa_handler = on_signal };
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL);

    logmsg("fandynd %s: profile=%s, %d zones, interval %dms",
           VERSION, c.profile, nz, c.interval_ms);

    double tf = NAN, slope = 0, trimv = 0, cur = 0;
    double last_t = now_s(), kick_until = 0, last_kick = -100, on_since = 0;
    int written = -1, fan_on = !c.zero_rpm, stalls = 0, bad_reads = 0;
    long rpm = 0;
    read_cpu();

    while (!stop) {
        struct timespec ts = { c.interval_ms / 1000, (c.interval_ms % 1000) * 1000000L };
        nanosleep(&ts, NULL);
        if (stop) break;

        if (reload) {
            reload = 0;
            load_cfg(cfg_path, &c);
            nz = find_zones(c.zones, zones);
            logmsg("config reloaded: profile=%s", c.profile);
        }

        double t = now_s(), dt = clamp(t - last_t, 0.05, 5);
        last_t = t;

        /* 1. sensors */
        double temp = read_temp(zones, nz);
        double cpu = read_cpu();
        long g = 0;
        double gpu = read_long(GPU_LOAD, &g) == 0 ? clamp(g / 1000.0, 0, 1) : 0;
        double ram = clamp((read_ram() - c.ram_base) / (1 - c.ram_base), 0, 1);
        double load = fmax(fmax(cpu, gpu), c.ram_weight * ram);

        long r;
        if (read_long(FAN "rpm_measured", &r) == 0 && r < 15000) rpm = r; /* drop tach glitches */

        /* 2. filter temperature and its trend, then predict */
        if (isnan(temp)) {
            if (++bad_reads >= 3) temp = c.t_crit; /* sensors gone: fail safe */
            else continue;
        } else {
            bad_reads = 0;
        }
        if (isnan(tf)) tf = temp;
        double prev = tf;
        tf += 0.4 * (temp - tf);
        slope += 0.2 * ((tf - prev) / dt - slope);
        double tp = tf + fmax(0, slope) * c.lookahead + c.load_gain * load;

        /* 3. curve + adaptive trim */
        double x = clamp((tp - c.t_start) / (c.t_full - c.t_start), 0, 1);
        double want = c.min_pwm + pow(x, c.curve) * (c.max_pwm - c.min_pwm);

        double err = tf - c.t_limit;
        trimv += c.ki * err * dt * (err > 0 ? 1 : 0.25); /* bleed off slowly */
        trimv = clamp(trimv, 0, c.max_pwm - c.min_pwm);
        want = clamp(want + trimv, c.min_pwm, c.max_pwm);

        /* zero-RPM: stop when cool, restart on heat or load (hysteresis + 30s min run) */
        if (c.zero_rpm) {
            if (!fan_on && tp >= c.t_on) { fan_on = 1; on_since = t; }
            else if (fan_on && tp < c.t_off && trimv == 0 && t - on_since > 30) fan_on = 0;
        } else {
            fan_on = 1;
        }
        if (!fan_on) want = 0;

        /* 4. shape for noise: fast up, slow down */
        int crit = temp >= c.t_crit;
        if (crit) cur = c.max_pwm;
        else if (want > cur) cur = fmin(want, cur + c.slew_up * dt);
        else cur = fmax(want, cur - c.slew_down * dt);

        int out = (int)lround(cur);
        if (out > 0 && out < c.min_pwm) out = fan_on ? c.min_pwm : 0;

        /* stall detection: fan should spin but tach says 0 */
        stalls = (written >= c.min_pwm && rpm == 0 && t > kick_until + 3) ? stalls + 1 : 0;

        if ((written <= 0 && out > 0) || (stalls >= 3 && t - last_kick > 10)) {
            kick_until = t + c.kick_ms / 1000.0;
            last_kick = t;
            stalls = 0;
            if (rpm == 0 && written > 0) logmsg("fan stall detected, kicking");
        }
        if (t < kick_until && out > 0) out = (int)fmax(out, c.kick_pwm);

        int changed = written < 0 || abs(out - written) >= c.deadband || out == 0 ||
                      out == c.max_pwm || t < kick_until || written == c.kick_pwm;
        if (changed && out != written) {
            if (write_long(FAN "target_pwm", out) == 0) written = out;
            else logmsg("write target_pwm failed: %s", strerror(errno));
        }

        /* status + debug */
        char tmp[300];
        snprintf(tmp, sizeof tmp, "%s.tmp", status_path);
        FILE *f = fopen(tmp, "w");
        if (f) {
            fprintf(f, "profile=%s\ntemp=%.1f\npredicted=%.1f\ntrend=%.2f C/s\n"
                       "cpu=%.0f%%\ngpu=%.0f%%\nram_pressure=%.0f%%\ntrim=%.1f\n"
                       "pwm=%d\nrpm=%ld\nstate=%s\n",
                    c.profile, tf, tp, slope, cpu * 100, gpu * 100, ram * 100,
                    trimv, written, rpm,
                    crit ? "critical" : t < kick_until ? "kick" : fan_on ? "on" : "off");
            fclose(f);
            rename(tmp, status_path);
        }
        if (verbose)
            logmsg("T=%.1f pred=%.1f d=%+.2f cpu=%3.0f gpu=%3.0f ram=%3.0f trim=%4.1f want=%5.1f pwm=%3d rpm=%ld",
                   tf, tp, slope, cpu * 100, gpu * 100, ram * 100, trimv, want, written, rpm);
    }

    write_long(FAN "temp_control", 1);
    unlink(status_path);
    logmsg("stopped, fan control returned to kernel");
    return 0;
}
