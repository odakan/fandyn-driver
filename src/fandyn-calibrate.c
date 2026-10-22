/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2026 deniz <onurdenizakan@gmail.com> */
/*
 * fandyn-calibrate - one-time board calibration for fandyn (run at install).
 *
 * Loud on purpose: it drives the fan through its whole range and loads every
 * CPU core for about 15 minutes. From the measurements it:
 *
 *   1. characterizes the fan: RPM curve, stall point, reliable floor,
 *      start-up kick (verified 3 times from a standstill)
 *   2. fits a physical thermal model of the board (lumped RC network):
 *        C dTs/dt = P - G(u) (Ts - Ta)      heatsink node
 *        Td = Ts + Rjc P                    die sensor
 *        G(u) = G0 + G1 u^n                 natural + forced convection
 *      where u = fan RPM / max RPM and P is the measured input power
 *      (INA3221 sensor) or CPU load when that sensor is unavailable.
 *      Output-error least squares (Nelder-Mead, multi-start), with residual
 *      statistics, Durbin-Watson, and parameter covariance from the Jacobian.
 *   3. designs the fixed profile: the slowest fan speed that still holds each
 *      temperature, with capacity growing linearly from idle to full speed
 *   4. tests the profile on the real board (closed loop, real fd_step code)
 *      and in simulation (design power, hot room, bursty load)
 *
 * Writes into the output directory (default /var/lib/fandyn):
 *   profile          fixed profile used by fandynd (atomic, CRC-protected)
 *   calibration.csv  every raw sample (the measurement database)
 *   report.txt       model, statistics and test results
 *   history.csv      one line per calibration, to track dust and aging
 */
#define _GNU_SOURCE
#include "fandyn.h"

#include <dirent.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>

#define FAN        "/sys/devices/pwm-fan/"
#define MAXS       4000
#define MAXP       8
#define MAXCPU     16
#define NLEVELS    20

/* ------------------------------------------------------------------ state */

static volatile sig_atomic_t g_abort;
static int g_load, g_quit;                  /* shared with burn threads (atomics) */
static int g_ncpu;
static pthread_t g_thr[MAXCPU];

static struct fd_zones g_z;
static char g_pwr_path[FD_PATH];
static int g_have_power;
static FILE *g_csv, *g_rep;
static double g_t0;
static int g_cmd = -1;
static double g_guard = 80.0, g_trip_until;
static int g_trips, g_phase;
static int g_quick;
static const char *g_phase_name[] = { "fan", "thermal", "validate" };

struct sample { double t, temp, power, busy, u; int cmd, cur, rpm, load, phase; };
static struct sample S[MAXS];
static int NS;

struct fan {
    int tach, rpm_max, floor, kick, kick_ms, kick_ok, start_pwm, lowest_run;
    double ramp_rate;                          /* kernel ramp, PWM per second */
    int n, pwm[NLEVELS], rpm[NLEVELS];         /* measured curve, ascending pwm */
};
static struct fan F;
static double fan_u(int pwm);    /* fan curve: PWM -> relative airflow (RPM / max) */

/* ---------------------------------------------------------------- helpers */

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static void say(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    fflush(stdout);
}

static void rep(const char *fmt, ...)
{
    if (!g_rep) return;
    va_list ap;
    va_start(ap, fmt);
    vfprintf(g_rep, fmt, ap);
    va_end(ap);
}

static double clampd(double x, double lo, double hi) { return x < lo ? lo : x > hi ? hi : x; }
static int clampi(int x, int lo, int hi) { return x < lo ? lo : x > hi ? hi : x; }

static void sleep_ms(int ms)
{
    struct timespec ts = { ms / 1000, (long)(ms % 1000) * 1000000L };
    while (!g_abort && nanosleep(&ts, &ts) != 0 && errno == EINTR) { }
}

static void on_signal(int sig) { (void)sig; g_abort = 1; }

/* -------------------------------------------------------------- sensors */

static double cpu_busy(void)
{
    static unsigned long long pb, pt;
    unsigned long long v[8] = { 0 };
    FILE *f = fopen("/proc/stat", "r");
    if (!f) return 0;
    int n = fscanf(f, "cpu %llu %llu %llu %llu %llu %llu %llu %llu",
                   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7]);
    fclose(f);
    if (n < 5) return 0;
    unsigned long long tot = 0;
    for (int i = 0; i < 8; i++) tot += v[i];
    unsigned long long busy = tot - v[3] - v[4];
    static double last;
    /* FIX [Robustness]: the aggregate counters can step backwards (CPU hotplug on
     * Tegra); the unsigned difference then wrapped to ~2^64 and read as 100% busy. */
    if (tot <= pt || busy < pb) { pb = busy; pt = tot; return last; }
    double r = (double)(busy - pb) / (double)(tot - pt);
    pb = busy;
    pt = tot;
    last = clampd(r, 0, 1);
    return last;
}

/* Find the board input power rail of the INA3221 monitor (root only). */
static void find_power(void)
{
    const char *base = "/sys/bus/i2c/drivers/ina3221x";
    DIR *d = opendir(base);
    if (!d) return;
    struct dirent *e;
    while (!g_have_power && (e = readdir(d))) {
        if (e->d_name[0] == '.') continue;
        for (int dev = 0; dev < 4 && !g_have_power; dev++)
            for (int ch = 0; ch < 3 && !g_have_power; ch++) {
                char p[FD_PATH], name[64];
                snprintf(p, sizeof p, "%s/%s/iio:device%d/rail_name_%d", base, e->d_name, dev, ch);
                int n = fd_read_file(p, name, sizeof name);
                if (n <= 0) continue;
                if (name[n - 1] == '\n') name[n - 1] = 0;
                if (strcmp(name, "POM_5V_IN") && strcmp(name, "VDD_IN")) continue;
                snprintf(g_pwr_path, sizeof g_pwr_path, "%s/%s/iio:device%d/in_power%d_input",
                         base, e->d_name, dev, ch);
                long mw;
                g_have_power = fd_read_long(g_pwr_path, &mw) == 0 && mw > 0;
            }
    }
    closedir(d);
}

static double read_power(void)
{
    long mw;
    if (!g_have_power || fd_read_long(g_pwr_path, &mw) != 0 || mw <= 0) return NAN;
    return (double)mw / 1000.0;
}

static int read_cur(void) { long v; return fd_read_long(FAN "cur_pwm", &v) == 0 ? (int)v : -1; }

static int read_rpm(void)
{
    long v;
    if (fd_read_long(FAN "rpm_measured", &v) != 0 || v < 0 || v > FD_RPM_MAX) return -1;
    return (int)v;
}

static double read_temp_c(void)
{
    int t = fd_read_temp(&g_z);
    return t == FD_TEMP_INVALID ? NAN : t / 1000.0;
}

/* ------------------------------------------------------- fan + load control */

static void set_load(int on)
{
    if (on && now_s() < g_trip_until) on = 0;
    __atomic_store_n(&g_load, on, __ATOMIC_RELAXED);
}

static void set_fan(int pwm)
{
    if (now_s() < g_trip_until) pwm = 255;   /* safety override active */
    g_cmd = pwm;
    if (fd_write_long(FAN "target_pwm", pwm) != 0) say("  warning: cannot write target_pwm\n");
}

/* Heat source: 8 independent NEON multiply-add chains kept in registers.
 * FIX [Domain]: the old loop updated one volatile double through memory, so
 * the core mostly stalled on store-to-load forwarding and added little heat
 * (die step ~1.8 C), a weak excitation for the thermal identification. */
typedef double v2d __attribute__((vector_size(16)));
static volatile double g_sink;

static void *burn(void *arg)
{
    (void)arg;
    v2d a[8], m = { 0.9999999, 0.9999998 }, c = { 1e-7, 2e-7 };
    for (int i = 0; i < 8; i++) a[i] = (v2d){ 1.0 + i, 2.0 + i };
    while (!__atomic_load_n(&g_quit, __ATOMIC_RELAXED)) {
        if (__atomic_load_n(&g_load, __ATOMIC_RELAXED)) {
            for (int it = 0; it < 100000; it++)
                for (int i = 0; i < 8; i++) a[i] = a[i] * m + c;
            g_sink = a[0][0] + a[7][1];            /* keep the work observable */
        } else {
            struct timespec ts = { 0, 20000000L };
            nanosleep(&ts, NULL);
        }
    }
    return NULL;
}

/* Safety: too hot -> load off, fan full, for at least 60 s. */
static void guard(double temp)
{
    /* FIX [Robustness]: NaN (no readable sensor) used to pass this test, so the
     * CPU load kept running blind. Unreadable now trips the safety like overheating. */
    if (!(isnan(temp) || temp >= g_guard)) return;
    if (now_s() >= g_trip_until) {
        g_trips++;
        if (isnan(temp)) say("  ! temperature unreadable: load off, fan full\n");
        else say("  ! %.1f C reached the %.0f C safety limit: load off, fan full\n", temp, g_guard);
    }
    g_trip_until = now_s() + 60;
    __atomic_store_n(&g_load, 0, __ATOMIC_RELAXED);
    g_cmd = 255;
    (void)fd_write_long(FAN "target_pwm", 255);
}

static int median_int(int *v, int n)
{
    for (int i = 1; i < n; i++)
        for (int j = i; j > 0 && v[j - 1] > v[j]; j--) { int t = v[j]; v[j] = v[j - 1]; v[j - 1] = t; }
    return n ? v[n / 2] : -1;
}

static int rpm_median(int n, int gap_ms)
{
    int v[16], k = 0;
    for (int i = 0; i < n && i < 16 && !g_abort; i++) {
        int r = read_rpm();
        if (r >= 0) v[k++] = r;
        if (i + 1 < n) sleep_ms(gap_ms);
    }
    return k ? median_int(v, k) : -1;
}

static int rpm_median(int n, int gap_ms);

/* Robust RPM at the current setting: median of 9 reads that lie in [0, hi]
 * (a physical bound); re-measures up to 3 times. -1 if never plausible.
 * The tach on this fan produces sporadic and occasional burst glitches. */
static int rpm_robust(int hi)
{
    for (int attempt = 0; attempt < 3 && !g_abort; attempt++) {
        int v[9], k = 0;
        for (int i = 0; i < 9 && !g_abort; i++) {
            int r = read_rpm();
            if (r >= 0 && r <= hi) v[k++] = r;
            sleep_ms(300);
        }
        if (k >= 5) return median_int(v, k);
        sleep_ms(1000);
    }
    return -1;
}

/* Wait until the kernel's ramp reaches the target. Returns seconds taken. */
static double wait_settle(int pwm, int timeout_s)
{
    double t0 = now_s();
    for (int i = 0; i < timeout_s * 10 && !g_abort; i++) {
        if (read_cur() == pwm) break;
        if (i % 10 == 9) guard(read_temp_c());
        sleep_ms(100);
    }
    return now_s() - t0;
}

/* Record one sample into the database. */
static struct sample *take_sample(void)
{
    if (NS >= MAXS) return NULL;
    struct sample *s = &S[NS++];
    s->t = now_s() - g_t0;
    s->temp = read_temp_c();
    s->power = read_power();
    s->busy = cpu_busy();
    s->cmd = g_cmd;
    s->cur = read_cur();
    s->rpm = F.tach ? rpm_median(3, 30) : -1;              /* median: tach glitches */
    s->load = __atomic_load_n(&g_load, __ATOMIC_RELAXED);
    s->phase = g_phase;
    /* FIX [Robustness]: airflow comes from the drive level through the validated
     * fan curve; the raw tach (1-2% glitches up to 2x) only goes into the database. */
    s->u = fan_u(s->cur > 0 ? s->cur : 0);
    if (g_csv)
        fprintf(g_csv, "%s,%.2f,%.2f,%.3f,%.3f,%d,%d,%d,%d,%.4f\n", g_phase_name[s->phase], s->t,
                s->temp, s->power, s->busy, s->load, s->cmd, s->cur, s->rpm, s->u);
    guard(s->temp);
    return s;
}

/* ------------------------------------------------------ phase 1: the fan */

static double fan_u(int pwm)
{
    if (pwm <= 0) return 0;
    if (!F.tach || F.n < 2 || F.rpm_max <= 0) return pwm / 255.0;
    if (pwm < F.lowest_run) return 0;
    for (int i = 1; i < F.n; i++)
        if (pwm <= F.pwm[i]) {
            double a = (double)(pwm - F.pwm[i - 1]) / (F.pwm[i] - F.pwm[i - 1]);
            return (F.rpm[i - 1] + a * (F.rpm[i] - F.rpm[i - 1])) / F.rpm_max;
        }
    return (double)F.rpm[F.n - 1] / F.rpm_max;
}

static void stop_fan(void)
{
    set_fan(0);
    wait_settle(0, 30);
    for (int i = 0, zero = 0; i < 60 && zero < 3 && !g_abort; i++) {
        zero = read_rpm() == 0 ? zero + 1 : 0;
        sleep_ms(300);
    }
    sleep_ms(1000);
}

/* From standstill, does `pwm` start the fan within hold_ms? */
static int try_start(int pwm, int hold_ms, int *spin_ms)
{
    stop_fan();
    set_fan(pwm);
    double t0 = now_s();
    *spin_ms = -1;
    for (int i = 0; i < hold_ms / 100 && !g_abort; i++) {
        if (*spin_ms < 0 && read_rpm() > 0) *spin_ms = (int)((now_s() - t0) * 1000);
        sleep_ms(100);
    }
    return rpm_median(3, 200) > 0;
}

/* Exactly what fandynd does on start: kick for kick_ticks seconds, then floor. */
static int kick_works(int kick, int kick_ms, int floor)
{
    stop_fan();
    set_fan(kick);
    sleep_ms(((kick_ms + 999) / 1000) * 1000);
    set_fan(floor);
    sleep_ms(4000);
    return rpm_median(3, 300) > 0;
}

static void phase_fan(void)
{
    static const int levels[NLEVELS] = { 255, 200, 160, 128, 100, 80, 64, 52, 44, 36,
                                         30, 26, 22, 19, 16, 14, 12, 10, 8, 6 };
    int meas_p[NLEVELS], meas_r[NLEVELS], nm = 0;
    say("[1/4] fan: speed curve, stall point, start-up kick (about 3 min)\n");

    set_fan(255);
    wait_settle(255, 40);
    sleep_ms(3000);
    int m0 = rpm_median(9, 300);
    F.rpm_max = m0 > 0 ? rpm_robust(m0 + m0 / 8) : m0;    /* second pass rejects glitches */
    F.tach = F.rpm_max > 300;
    if (!F.tach) {
        say("  no tach signal: using conservative fixed settings (floor 40, kick 100, no zero-RPM)\n");
        F.floor = 40; F.kick = 100; F.kick_ms = 3000; F.kick_ok = 0; F.lowest_run = 1;
        F.ramp_rate = 13;
        return;
    }
    say("  full speed: %d RPM\n", F.rpm_max);

    int prev = F.rpm_max;
    for (int i = 0; i < NLEVELS && !g_abort; i++) {
        set_fan(levels[i]);
        wait_settle(levels[i], 30);
        sleep_ms(2000);
        /* FIX [Robustness]: a speed can't exceed the next-higher PWM's speed; readings
         * above that bound are tach glitches. Old code took a plain median of 5, and a
         * burst at pwm 128 (9463 RPM vs 8143 max) went into the curve. */
        int r = rpm_robust(prev + prev / 20 + 50);
        say("  pwm %3d -> %5d RPM%s\n", levels[i], r, r < 0 ? "  (no plausible reading, skipped)" : "");
        if (r < 0) continue;
        meas_p[nm] = levels[i];
        meas_r[nm] = r;
        nm++;
        prev = r;
        if (r == 0) break;
    }
    /* store ascending; FIX [Logical]: repair monotonicity by capping each speed at
     * the next-higher PWM's speed. The old code raised every higher PWM to a high
     * outlier, which makes the fan look stronger than it is (anti-conservative);
     * capping can only make it look weaker, which designs a louder, safer curve. */
    F.n = 0;
    for (int i = nm - 1; i >= 0; i--) { F.pwm[F.n] = meas_p[i]; F.rpm[F.n] = meas_r[i]; F.n++; }
    for (int i = F.n - 2; i >= 0; i--) if (F.rpm[i] > F.rpm[i + 1]) F.rpm[i] = F.rpm[i + 1];
    if (F.n > 0) F.rpm_max = F.rpm[F.n - 1] > 0 ? F.rpm[F.n - 1] : F.rpm_max;
    F.lowest_run = 255;
    for (int i = 0; i < F.n; i++) if (F.rpm[i] > 0) { F.lowest_run = F.pwm[i]; break; }

    /* reliable floor: lowest running speed plus margin, must hold 10 s */
    F.floor = F.lowest_run + (F.lowest_run / 4 > 3 ? F.lowest_run / 4 : 3);
    for (int tries = 0; tries < 6 && !g_abort; tries++) {
        set_fan(clampi(F.floor + 30, 0, 255));
        wait_settle(clampi(F.floor + 30, 0, 255), 20);
        set_fan(F.floor);
        wait_settle(F.floor, 20);
        int ok = 1;
        for (int k = 0; k < 10 && ok && !g_abort; k++) { sleep_ms(1000); ok = read_rpm() > 0; }
        if (ok) break;
        F.floor += 4;
    }
    say("  lowest running speed %d, reliable floor %d\n", F.lowest_run, F.floor);

    /* kernel ramp rate: floor -> 255 */
    set_fan(255);
    double ramp_s = wait_settle(255, 40);
    F.ramp_rate = ramp_s > 0.2 ? (255 - F.floor) / ramp_s : 255;
    say("  kernel ramp: %.0f PWM/s\n", F.ramp_rate);

    /* lowest PWM that starts the fan from standstill */
    F.start_pwm = -1;
    int spin = -1;
    for (int c = F.floor; c <= 120 && !g_abort; c += 6) {
        int sp;
        if (try_start(c, 4000, &sp)) { F.start_pwm = c; spin = sp; break; }
    }
    if (F.start_pwm < 0) {
        F.kick = 160; F.kick_ms = 4000;
    } else {
        /* FIX [Domain]: starting from standstill is random (rotor position): the
         * same fan started at 15 in one run and needed 27 in others. A 1.5x margin
         * was too thin; use 2x the observed threshold, at least +16 and >= 40. */
        int k = F.start_pwm * 2;
        if (k < F.start_pwm + 16) k = F.start_pwm + 16;
        if (k < 40) k = 40;
        F.kick = clampi(k, F.floor, 255);
        F.kick_ms = clampi(((spin < 0 ? 1000 : spin) * 2 + 500 + 499) / 500 * 500, 1000, 5000);
    }
    say("  starts from standstill at pwm %d; kick %d for %d ms\n", F.start_pwm, F.kick, F.kick_ms);

    /* verify the kick exactly as the runtime performs it.
     * FIX [Numerical]: 3/3 successes only bound the start probability >= 37%
     * (one-sided 95% Clopper-Pearson: 0.05^(1/n)); 5/5 gives >= 55%, and the
     * runtime's stall supervision re-kicks the rare failure. */
    F.kick_ok = 0;
    for (int round = 0; round < 3 && !F.kick_ok && !g_abort; round++) {
        int good = 0;
        for (int k = 0; k < 5 && !g_abort; k++) good += kick_works(F.kick, F.kick_ms, F.floor);
        say("  kick check: %d/5 starts\n", good);
        if (good == 5) F.kick_ok = 1;
        else { F.kick = clampi(F.kick + 24, 0, 255); F.kick_ms = clampi(F.kick_ms + 500, 0, 5000); }
    }
    if (!F.kick_ok) say("  kick unreliable: zero-RPM will be disabled\n");
}

/* --------------------------------------------- phase 2: thermal response */

struct seg { int fan, load, secs; };

static void run_segments(const struct seg *sg, int n)
{
    int total = 0, done = 0, prev = g_cmd;
    for (int i = 0; i < n; i++) total += sg[i].secs;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    for (int i = 0; i < n && !g_abort; i++) {
        int kick_left = (prev == 0 && sg[i].fan > 0) ? (F.kick_ms + 999) / 1000 : 0;
        set_fan(kick_left ? F.kick : sg[i].fan);
        set_load(sg[i].load);
        for (int k = 0; k < sg[i].secs && !g_abort; k++, done++) {
            next.tv_sec += 1;
            while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL) == EINTR && !g_abort) { }
            if (kick_left && --kick_left == 0) set_fan(sg[i].fan);
            if (now_s() >= g_trip_until) {          /* re-apply after a safety trip */
                if (g_cmd != (kick_left ? F.kick : sg[i].fan)) set_fan(kick_left ? F.kick : sg[i].fan);
                set_load(sg[i].load);
            }
            struct sample *s = take_sample();
            if (s && done % 15 == 0) {
                say("  [%3d/%3d s] %5.1f C  fan %3d  %4d RPM  load %s", done, total, s->temp,
                    s->cmd, s->rpm, s->load ? "on " : "off");
                if (!isnan(s->power)) say("  %.2f W", s->power);
                say("\n");
            }
        }
        prev = sg[i].fan;
    }
    set_load(0);
}

static void phase_thermal(void)
{
    int mid = 255;
    for (int p = F.floor; p <= 255; p++) if (fan_u(p) >= 0.45) { mid = p; break; }
    int off = F.kick_ok ? 0 : F.floor;
    struct seg sg[] = {
        { 255, 0, 60 }, { 255, 1, 100 }, { mid, 1, 80 }, { F.floor, 1, 100 }, { off, 1, 50 },
        { off, 0, 70 }, { F.floor, 0, 50 }, { mid, 0, 40 }, { 255, 0, 50 },
    };
    int n = (int)(sizeof sg / sizeof sg[0]);
    if (g_quick) for (int i = 0; i < n; i++) sg[i].secs = sg[i].secs * 2 / 5;
    int total = 0;
    for (int i = 0; i < n; i++) total += sg[i].secs;
    say("[2/4] thermal: heating and cooling at 4 fan levels (%d min %d s)\n", total / 60, total % 60);
    g_phase = 1;
    run_segments(sg, n);
}

/* ----------------------------------------------------- model + fitting */

static double ft[MAXS], fT[MAXS], fu[MAXS], fPraw[MAXS];
static double fPd[MAXS];   /* heat at sample k (die term) */
static double fPi[MAXS];   /* heat during the step k -> k+1 */
static double fui[MAXS];   /* fan airflow during the step k -> k+1 */
/* Weak physical priors (Gaussian in parameter space) make the fit a MAP
 * estimate: room temperature ~ 25 +- 12 C; without a power sensor, idle heat
 * ~ 0.5x the CPU-load heat within a factor ~2 (Jetson Nano: ~1.5 W idle,
 * ~3 W added by 4 busy cores). Data overrides them when it is informative. */
/* Noise variance weighting the priors. Starts at 0.09 C^2, then is set to the
 * fitted residual variance and the fit is repeated (see fit_model). */
static double g_sn2 = 0.09;
struct prior { int idx; double mu, s; };
static struct prior g_pri[2];
static int g_npri;
static int fL[MAXS];
static int fn, g_proxy, g_dim;

struct model { double C, G0, G1, n, Rjc, Ta, off, P0; };

static struct model unpack(const double *th)
{
    struct model m;
    m.C = exp(th[0]); m.G0 = exp(th[1]); m.G1 = exp(th[2]);
    m.n = 0.3 + 1.2 / (1 + exp(-th[3]));
    m.Rjc = exp(th[4]); m.Ta = th[5]; m.off = th[6];
    m.P0 = g_proxy ? exp(th[7]) : 0;
    return m;
}

static double G_of(const struct model *m, double u) { return m->G0 + m->G1 * pow(u > 0 ? u : 0, m->n); }
static double R_of(const struct model *m, double u) { return m->Rjc + 1 / G_of(m, u); }

/* Simulate the die temperature over the fit data; returns SSE. */
static double simulate(const double *th, double *pred)
{
    struct model m = unpack(th);
    double Ts = fT[0] - m.Rjc * (fPd[0] + m.P0) + m.off, sse = 0;
    for (int k = 0; k < fn; k++) {
        double y = Ts + m.Rjc * (fPd[k] + m.P0), e = fT[k] - y;
        if (pred) pred[k] = y;
        sse += e * e;
        if (k + 1 < fn) {
            /* FIX [Logical]: the step k -> k+1 now uses the heat and airflow of that
             * step (fPi, fui); it used the value logged at k, one interval early. */
            double P = fPi[k] + m.P0, G = G_of(&m, fui[k]), eq = m.Ta + P / G;
            /* NOTE: exact zero-order-hold solution of the linear ODE over the step,
             * unconditionally stable for any dt (no Euler step-size limit). */
            Ts = eq + (Ts - eq) * exp(-G * (ft[k + 1] - ft[k]) / m.C);
        }
    }
    return isfinite(sse) ? sse : 1e30;
}

static double objective(const double *th)
{
    double f = simulate(th, NULL);
    for (int i = 0; i < g_npri; i++) {
        double z = (th[g_pri[i].idx] - g_pri[i].mu) / g_pri[i].s;
        f += g_sn2 * z * z;
    }
    return f;
}

/* Nelder-Mead simplex minimizer. */
static double nelder_mead(double *x, const double *step, int n, int maxev)
{
    double s[MAXP + 1][MAXP], f[MAXP + 1], c[MAXP], xr[MAXP], xe[MAXP], xc[MAXP];
    for (int i = 0; i <= n; i++) {
        for (int j = 0; j < n; j++) s[i][j] = x[j] + (i == j + 1 ? step[j] : 0);
        f[i] = objective(s[i]);
    }
    int ev = n + 1;
    while (ev < maxev) {
        int b = 0, w = 0, sw = 0;
        for (int i = 1; i <= n; i++) { if (f[i] < f[b]) b = i; if (f[i] > f[w]) w = i; }
        sw = b;
        for (int i = 0; i <= n; i++) if (i != w && f[i] > f[sw]) sw = i;
        if (f[w] - f[b] <= 1e-10 * (fabs(f[b]) + 1e-12)) break;
        for (int j = 0; j < n; j++) {
            c[j] = 0;
            for (int i = 0; i <= n; i++) if (i != w) c[j] += s[i][j];
            c[j] /= n;
            xr[j] = c[j] + (c[j] - s[w][j]);
        }
        double fr = objective(xr); ev++;
        if (fr < f[b]) {
            for (int j = 0; j < n; j++) xe[j] = c[j] + 2 * (c[j] - s[w][j]);
            double fe = objective(xe); ev++;
            if (fe < fr) { memcpy(s[w], xe, sizeof xe); f[w] = fe; }
            else { memcpy(s[w], xr, sizeof xr); f[w] = fr; }
        } else if (fr < f[sw]) {
            memcpy(s[w], xr, sizeof xr); f[w] = fr;
        } else {
            int outside = fr < f[w];
            for (int j = 0; j < n; j++) xc[j] = outside ? c[j] + 0.5 * (xr[j] - c[j]) : c[j] + 0.5 * (s[w][j] - c[j]);
            double fc = objective(xc); ev++;
            if (fc < (outside ? fr : f[w])) { memcpy(s[w], xc, sizeof xc); f[w] = fc; }
            else {
                for (int i = 0; i <= n; i++) if (i != b) {
                    for (int j = 0; j < n; j++) s[i][j] = s[b][j] + 0.5 * (s[i][j] - s[b][j]);
                    f[i] = objective(s[i]); ev++;
                }
            }
        }
    }
    int b = 0;
    for (int i = 1; i <= n; i++) if (f[i] < f[b]) b = i;
    memcpy(x, s[b], sizeof(double) * (size_t)n);
    return f[b];
}

/* Invert a small symmetric matrix (Gauss-Jordan, partial pivoting). */
static int invert(double a[MAXP][MAXP], double inv[MAXP][MAXP], int n)
{
    double m[MAXP][2 * MAXP];
    double maxdiag = 0;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) { m[i][j] = a[i][j]; m[i][n + j] = i == j; }
        if (fabs(a[i][i]) > maxdiag) maxdiag = fabs(a[i][i]);
    }
    for (int col = 0; col < n; col++) {
        int piv = col;
        for (int r = col + 1; r < n; r++) if (fabs(m[r][col]) > fabs(m[piv][col])) piv = r;
        if (fabs(m[piv][col]) < 1e-14 * (maxdiag + 1e-300)) return -1;
        for (int j = 0; j < 2 * n; j++) { double t = m[col][j]; m[col][j] = m[piv][j]; m[piv][j] = t; }
        double d = m[col][col];
        for (int j = 0; j < 2 * n; j++) m[col][j] /= d;
        for (int r = 0; r < n; r++) if (r != col) {
            double k = m[r][col];
            for (int j = 0; j < 2 * n; j++) m[r][j] -= k * m[col][j];
        }
    }
    for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) inv[i][j] = m[i][n + j];
    return 0;
}

struct fit {
    double th[MAXP], cov[MAXP][MAXP];
    int have_cov, ok;
    const char *why;
    double sse, sigma, r2, maxerr, dw, rho, p_idle, p_load;
    struct model m;
};
static struct fit FIT;
static double g_pred[MAXS], g_pp[MAXS], g_pm[MAXS], g_J[MAXP][MAXS];

/* Standard error of a derived quantity q(theta) via the delta method. */
static double delta_se(double (*q)(const struct model *, double), double arg)
{
    if (!FIT.have_cov) return NAN;
    double g[MAXP], th[MAXP];
    for (int j = 0; j < g_dim; j++) {
        memcpy(th, FIT.th, sizeof th);
        double h = 1e-5 * (fabs(th[j]) + 1);
        th[j] += h;
        struct model a = unpack(th);
        th[j] -= 2 * h;
        struct model b = unpack(th);
        g[j] = (q(&a, arg) - q(&b, arg)) / (2 * h);
    }
    double v = 0;
    for (int i = 0; i < g_dim; i++) for (int j = 0; j < g_dim; j++) v += g[i] * FIT.cov[i][j] * g[j];
    return v > 0 ? sqrt(v) : 0;
}

static double q_R(const struct model *m, double u) { return R_of(m, u); }
static double q_tau(const struct model *m, double u) { return m->C / G_of(m, u); }
static double q_Ta(const struct model *m, double u) { (void)u; return m->Ta; }
static double q_dR(const struct model *m, double u) { (void)u; return R_of(m, 0) - R_of(m, 1); }
static double g_pidle_meas;     /* measured mean idle heat (without the fitted P0) */
static double q_Tidle(const struct model *m, double u) { (void)u; return m->Ta + (g_pidle_meas + m->P0) * R_of(m, 0); }

static void fit_model(void)
{
    say("[3/4] fitting thermal model\n");
    fn = 0;
    double pl = 0, pi = 0; int nl = 0, ni = 0;
    for (int i = 0; i < NS && fn < MAXS; i++) {
        if (S[i].phase != 1 || isnan(S[i].temp)) continue;
        ft[fn] = S[i].t; fT[fn] = S[i].temp; fu[fn] = S[i].u; fL[fn] = S[i].load;
        fPraw[fn] = g_proxy ? S[i].busy : S[i].power;
        if (isnan(fPraw[fn])) fPraw[fn] = fn ? fPraw[fn - 1] : 0;
        fn++;
    }
    /* FIX [Logical]: align heat with the interval it describes. cpu_busy() at
     * sample k is the load over (t[k-1], t[k]], so the heat during the step
     * t[k] -> t[k+1] is busy[k+1]; the old code used busy[k], one step early,
     * which smears every load step and biases Rjc and C. INA3221 readings are
     * near-instantaneous, so for them the step uses the trapezoid mean.
     * FIX [Numerical]: this replaces a centred 3-point smoothing that leaked the
     * next sample's heat into the current die prediction. */
    for (int k = 0; k < fn; k++) {
        int k1 = k + 1 < fn ? k + 1 : k;
        fPd[k] = fPraw[k];
        fPi[k] = g_proxy ? fPraw[k1] : 0.5 * (fPraw[k] + fPraw[k1]);
        fui[k] = 0.5 * (fu[k] + fu[k1]);        /* the fan ramps during the step */
    }
    for (int k = 0; k < fn; k++) {
        if (fL[k]) { pl += fPd[k]; nl++; } else { pi += fPd[k]; ni++; }
    }
    FIT.ok = 0;
    if (fn < 60) { say("  not enough samples (%d)\n", fn); return; }

    double tmin = fT[0];
    for (int k = 1; k < fn; k++) if (fT[k] < tmin) tmin = fT[k];
    g_dim = g_proxy ? 8 : 7;
    g_npri = 0;
    g_pri[g_npri++] = (struct prior){ 5, 25.0, 12.0 };
    if (g_proxy) g_pri[g_npri++] = (struct prior){ 7, log(0.5), 0.7 };
    double best = 1e300, bth[MAXP] = { 0 };
    static const double cscale[] = { 0.5, 1.0, 3.0 };
    for (int ms = 0; ms < 3; ms++) {
        double th[MAXP], st[MAXP];
        if (g_proxy) {
            th[0] = log(50 * cscale[ms]); th[1] = log(0.04); th[2] = log(0.12); th[4] = log(3.0); th[7] = log(0.4);
        } else {
            th[0] = log(200 * cscale[ms]); th[1] = log(0.15); th[2] = log(0.5); th[4] = log(0.8); th[7] = 0;
        }
        th[3] = -0.336; th[5] = tmin - 5; th[6] = 0;
        for (int j = 0; j < MAXP; j++) st[j] = 0.5;
        st[5] = 3; st[6] = 1;
        double f = 0;
        for (int r = 0; r < 4; r++) f = nelder_mead(th, st, g_dim, 4000);
        if (f < best) { best = f; memcpy(bth, th, sizeof th); }
    }
    /* FIX [Numerical]: the priors were weighted with a fixed 0.09 C^2 noise variance
     * while the posterior covariance below uses the fitted one, so estimate and
     * uncertainty came from different objectives. One empirical-Bayes iteration:
     * estimate the residual variance, then refit with it. */
    g_sn2 = 0.09;
    double sse0 = simulate(bth, NULL);
    g_sn2 = fmax(sse0 / (fn - g_dim), 0.25 * 0.25 / 12);  /* floor: 0.5 C quantization noise */
    for (int r = 0; r < 2; r++) best = nelder_mead(bth, (const double[MAXP]){ 0.2, 0.2, 0.2, 0.2, 0.2, 1, 0.5, 0.2 }, g_dim, 4000);
    memcpy(FIT.th, bth, sizeof bth);
    FIT.m = unpack(FIT.th);
    FIT.sse = simulate(FIT.th, g_pred);

    double mean = 0, sst = 0, dwn = 0;
    for (int k = 0; k < fn; k++) mean += fT[k];
    mean /= fn;
    FIT.maxerr = 0;
    for (int k = 0; k < fn; k++) {
        double e = fT[k] - g_pred[k];
        sst += (fT[k] - mean) * (fT[k] - mean);
        if (fabs(e) > FIT.maxerr) FIT.maxerr = fabs(e);
        if (k) { double d = e - (fT[k - 1] - g_pred[k - 1]); dwn += d * d; }
    }
    FIT.sigma = sqrt(FIT.sse / (fn - g_dim));
    FIT.r2 = sst > 0 ? 1 - FIT.sse / sst : 0;
    FIT.dw = FIT.sse > 0 ? dwn / FIT.sse : 2;
    FIT.rho = clampd(1 - FIT.dw / 2, 0, 0.95);
    g_pidle_meas = ni ? pi / ni : 0;
    FIT.p_idle = g_pidle_meas + FIT.m.P0;
    FIT.p_load = (nl ? pl / nl : 0) + FIT.m.P0;

    /* parameter covariance: sigma^2 (J'J)^-1, inflated for AR(1) residuals */
    double jtj[MAXP][MAXP] = { { 0 } }, inv[MAXP][MAXP];
    for (int j = 0; j < g_dim; j++) {
        double th[MAXP];
        memcpy(th, FIT.th, sizeof th);
        double h = 1e-4 * (fabs(th[j]) + 1);
        th[j] += h; simulate(th, g_pp);
        th[j] -= 2 * h; simulate(th, g_pm);
        for (int k = 0; k < fn; k++) g_J[j][k] = (g_pp[k] - g_pm[k]) / (2 * h);
    }
    double infl = (1 + FIT.rho) / (1 - FIT.rho), s2 = FIT.sigma * FIT.sigma * infl;
    for (int a = 0; a < g_dim; a++)
        for (int b = 0; b < g_dim; b++) {
            double sum = 0;
            for (int k = 0; k < fn; k++) sum += g_J[a][k] * g_J[b][k];
            jtj[a][b] = sum / s2;
        }
    for (int i = 0; i < g_npri; i++) jtj[g_pri[i].idx][g_pri[i].idx] += 1 / (g_pri[i].s * g_pri[i].s);
    FIT.have_cov = invert(jtj, inv, g_dim) == 0;
    for (int a = 0; a < g_dim; a++)
        for (int b = 0; b < g_dim; b++) FIT.cov[a][b] = FIT.have_cov ? inv[a][b] : 0;

    struct model *m = &FIT.m;
    double R0 = R_of(m, 0), R1 = R_of(m, 1), tau1 = m->C / G_of(m, 1);
    double se_R1 = delta_se(q_R, 1), se_dR = delta_se(q_dR, 0), se_Ta = delta_se(q_Ta, 0);
    const char *why = NULL;
    if (!(FIT.r2 >= 0.80 && FIT.sigma <= 1.5)) why = "poor fit";
    /* FIX [Domain]: with heat always flowing out, ambient lies below every measured
     * die temperature. A fit with Ta >= min(T) is unphysical and would make
     * (t - Ta) <= 0 in the profile design; it was accepted before (Ta < 60 only). */
    else if (!(isfinite(R0) && isfinite(R1) && R1 > 0 && tau1 > 10 && tau1 < 5000 && m->Ta > -20 && m->Ta < tmin &&
               (g_proxy || (R1 > 0.1 && R1 < 30)))) why = "physically implausible parameters";
    else if (!FIT.have_cov) why = "parameters not identifiable";
    else if (!(R0 - R1 > 1.96 * se_dR)) why = "fan effect not statistically significant";
    else if (!(1.96 * se_R1 < 0.5 * R1)) why = "thermal resistance too uncertain";
    else if (!(1.96 * se_Ta < 10)) why = "ambient temperature too uncertain";
    FIT.ok = why == NULL;
    FIT.why = why ? why : "all checks passed";
    say("  %d samples, residual %.2f C, R^2 %.3f, fan effect %.2f +- %.2f -> %s%s%s\n", fn, FIT.sigma, FIT.r2,
        R0 - R1, 1.96 * se_dR, FIT.ok ? "model accepted" : "model REJECTED (", FIT.ok ? "" : why,
        FIT.ok ? "" : "), using safe fallback design");
}

/* ------------------------------------------------------- profile design */

static double g_Rd[256];     /* design thermal resistance per PWM (pessimistic) */
static int g_tmax = 70;
static double g_hyst_extra;     /* added after a hunting test failure, C */

static double Rdesign(int pwm) { return g_Rd[clampi(pwm, 0, 255)]; }
static double g_idle_pred = NAN, g_idle_hi = NAN;    /* fan-off idle temperature, C */
#define FAN_ON_MIN_C 50                               /* zero-RPM threshold never below this */

static void design_profile(struct fd_profile *p, int t_full)
{
    memset(p, 0, sizeof *p);
    p->tach = F.tach;
    p->floor_pwm = F.floor;
    p->kick_pwm = clampi(F.kick, F.floor, 255);
    p->kick_ms = F.kick_ms;
    p->crit_mC = clampi((t_full + 15) * 1000, 60000, 90000);

    if (!FIT.ok) {                          /* fallback: built-in curve, measured fan */
        struct fd_profile b = fd_builtin;
        b.tach = F.tach;
        b.floor_pwm = F.floor;
        b.kick_pwm = clampi(F.kick, F.floor, 255);
        b.kick_ms = F.kick_ms;
        b.zero_rpm = F.kick_ok;
        for (int i = 0; i < b.npoints; i++) if (b.pwm[i] < b.floor_pwm) b.pwm[i] = b.floor_pwm;
        *p = b;
        p->calibrated = 0;
        return;
    }
    p->calibrated = 1;
    struct model *m = &FIT.m;
    for (int q = 0; q < 256; q++) {
        double u = fan_u(q);
        double se = delta_se(q_R, u);
        g_Rd[q] = R_of(m, u) + (isnan(se) ? 0.1 * R_of(m, u) : 1.64 * se);
    }
    double Ta = m->Ta;
    /* FIX [Numerical]: the fan-on threshold used the pessimistic R(0) bound AND a
     * +4 C margin (two stacked margins); with an uncertain R(0) that pushed it past
     * the span limit and silently disabled zero-RPM. Now: one-sided 95% upper bound
     * of the predicted fan-off idle temperature (delta method) + 1 C. */
    double se_idle = delta_se(q_Tidle, 0);
    g_idle_pred = q_Tidle(m, 0);
    g_idle_hi = g_idle_pred + 1.645 * (isnan(se_idle) ? 3.0 : se_idle);
    int t_on = (int)ceil(g_idle_hi + 1);
    /* FIX [Domain]: "idle" during calibration is the quietest state the board has
     * (1.07 W here), but everyday use runs higher (~3 W with a desktop: ~44 C fan
     * off). A threshold just above true idle made the fan hover and cycle in
     * normal use (sim: 19 starts/h, 112 reversals/h). Policy floor: 50 C, still
     * 20 C below full speed and 35 C below critical. */
    if (t_on < FAN_ON_MIN_C) t_on = FAN_ON_MIN_C;
    p->zero_rpm = F.kick_ok && t_on <= t_full - 10;
    if (t_on > t_full - 10) t_on = t_full - 10;

    p->hyst_mC = (int)(clampd(round((2.5 * FIT.sigma + 1.0) * 2) / 2 + g_hyst_extra, 1.5, 6.0) * 1000);
    p->off_below_mC = t_on * 1000;
    double tau_full = m->C / G_of(m, 1), tau_floor = m->C / G_of(m, fan_u(F.floor));
    p->slew_down = clampi((int)lround(255 / (0.5 * tau_full)), 1, 6);
    p->min_on_s = clampi((int)lround(tau_floor / 3), 30, 300);

    double cap_on = (t_on - Ta) / Rdesign(F.floor), cap_full = (t_full - Ta) / Rdesign(255);
    int span = t_full - t_on, step = (span + 23) / 24;
    if (step < 1) step = 1;
    p->npoints = 0;
    for (int t = t_on; t < t_full && p->npoints < FD_MAX_POINTS - 1; t += step) {
        int pwm = 255;
        if (cap_full > 1.1 * cap_on) {
            double s = (double)(t - t_on) / span, cap = cap_on + s * (cap_full - cap_on);
            double rreq = (t - Ta) / cap;
            for (int q = F.floor; q <= 255; q++) if (Rdesign(q) <= rreq) { pwm = q; break; }
        } else {
            pwm = F.floor + (255 - F.floor) * (t - t_on) / span;   /* fan barely matters: linear */
        }
        if (p->npoints && pwm < p->pwm[p->npoints - 1]) pwm = p->pwm[p->npoints - 1];
        p->t_mC[p->npoints] = t * 1000;
        p->pwm[p->npoints] = clampi(pwm, F.floor, 255);
        p->npoints++;
    }
    p->t_mC[p->npoints] = t_full * 1000;
    p->pwm[p->npoints] = 255;
    p->npoints++;
}

/* --------------------------------------------- closed-loop simulation */

struct simres { double t_max, t_end; int pwm_end, reversals, starts; double mean_pwm; };

/* Run the real control law against the fitted model. power(t) gives heat. */
static struct simres sim_closed(const struct fd_profile *p, double Ta, double (*power)(int), int secs)
{
    struct model m = FIT.m;
    m.Ta = Ta;
    struct fd_timing tm;
    struct fd_state st;
    memset(&st, 0, sizeof st);
    fd_timing_init(p, 1000, &tm);
    double P = FIT.p_idle, G = G_of(&m, p->zero_rpm ? 0 : fan_u(p->floor_pwm));
    double Ts = Ta + P / G, cur = 0, sum = 0;
    struct simres r = { -1e9, 0, 0, 0, 0, 0 };
    int last = 0, dir = 0;
    for (int k = 0; k < secs; k++) {
        P = power(k);
        double td = Ts + m.Rjc * P;
        int q = (int)lround(td * 2) * 500;                  /* sensor: 0.5 C steps */
        int out = fd_step(p, &tm, &st, q, -1);
        if (st.pwm > 0 && last == 0) r.starts++;
        if (st.pwm != last) { int d = st.pwm > last ? 1 : -1; if (dir && d != dir) r.reversals++; dir = d; }
        last = st.pwm;
        double rate = F.ramp_rate > 1 ? F.ramp_rate : 13;   /* kernel ramp */
        cur = out > cur ? fmin(out, cur + rate) : fmax(out, cur - rate);
        G = G_of(&m, fan_u((int)lround(cur)));
        double eq = Ta + P / G;
        Ts = eq + (Ts - eq) * exp(-G / m.C);
        if (td > r.t_max) r.t_max = td;
        sum += out;
        r.t_end = td;
        r.pwm_end = out;
    }
    r.mean_pwm = sum / secs;
    return r;
}

static double g_sim_p;
static double pw_const(int k) { (void)k; return g_sim_p; }
static double pw_idle(int k) { (void)k; return FIT.p_idle; }
static double pw_burst(int k) { return (k / 30) % 2 ? FIT.p_load : FIT.p_idle; }

/* ------------------------------------------------ phase 4: real-board test */

struct valres { double t_max, sim_rms; int reversals, faults, failsafe, starts, secs, stalls, sweep_rpm_lo, sweep_rpm_hi; };

static struct valres validate(const struct fd_profile *p)
{
    struct valres v = { -1e9, NAN, 0, 0, 0, 0, 0, 0, -1, 0 };
    struct fd_timing tm;
    struct fd_state st, ss;
    memset(&st, 0, sizeof st);
    memset(&ss, 0, sizeof ss);
    fd_timing_init(p, 1000, &tm);
    int on = g_quick ? 60 : 150, offs = g_quick ? 45 : 90;
    v.secs = on + offs;
    say("[4/4] testing the profile on the board: %d s load, %d s idle, then a fan sweep\n", on, offs);
    g_phase = 2;

    struct model m = FIT.m;
    double Ts = NAN, cur_sim = 0, err2 = 0, prev_pd = 0;
    int nerr = 0, written = -1, since = 0, last = -1, dir = 0, prev_kicks = 0;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    for (int k = 0; k < v.secs && !g_abort; k++) {
        set_load(k < on);
        next.tv_sec += 1;
        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL) == EINTR && !g_abort) { }
        int temp = fd_read_temp(&g_z);
        int rpm = F.tach ? read_rpm() : -1;
        int out = fd_step(p, &tm, &st, temp, rpm);
        if (out != written || ++since >= 30) { set_fan(out); written = out; since = 0; }
        if (st.mode == FD_MODE_FAULT) v.faults++;
        if (st.mode == FD_MODE_FAILSAFE) v.failsafe++;
        if (st.stall_kicks > prev_kicks) v.stalls++;
        prev_kicks = st.stall_kicks;
        if (st.pwm > 0 && last == 0) v.starts++;
        if (last >= 0 && st.pwm != last) {
            int d = st.pwm > last ? 1 : -1;
            if (dir && d != dir) v.reversals++;
            dir = d;
        }
        last = st.pwm;
        struct sample *s = take_sample();
        if (!s) continue;
        if (s->temp > v.t_max) v.t_max = s->temp;

        if (FIT.ok) {                     /* same controller, simulated board */
            double pd = g_proxy ? s->busy : s->power;
            pd = (isnan(pd) ? FIT.p_idle - m.P0 : pd) + m.P0;
            if (isnan(Ts)) {
                Ts = s->temp - m.Rjc * pd;
            } else {
                /* FIX [Logical]: advance over the second that just ended, with the heat
                 * measured for it and the simulated fan that ran during it. Before, the
                 * previous second's heat was applied to the next second (one-step lag). */
                double pi_ = g_proxy ? pd : 0.5 * (prev_pd + pd);
                double G = G_of(&m, fan_u((int)lround(cur_sim))), eq = m.Ta + pi_ / G;
                Ts = eq + (Ts - eq) * exp(-G / m.C);
            }
            prev_pd = pd;
            double td = Ts + m.Rjc * pd;
            int so = fd_step(p, &tm, &ss, (int)lround(td * 2) * 500, -1);
            double rate = F.ramp_rate > 1 ? F.ramp_rate : 13;
            cur_sim = so > cur_sim ? fmin(so, cur_sim + rate) : fmax(so, cur_sim - rate);
            err2 += (s->temp - td) * (s->temp - td);
            nerr++;
        }
        if (k % 15 == 0)
            say("  [%3d/%3d s] %5.1f C  fan %3d (%s)  %4d RPM  load %s\n", k, v.secs, s->temp, out,
                fd_mode_names[st.mode], rpm, k < on ? "on " : "off");
    }
    set_load(0);
    if (nerr) v.sim_rms = sqrt(err2 / nerr);

    /* Actuator sweep: CPU load alone cannot heat this heatsink into the
     * curve's range, so feed the real controller a virtual temperature ramp
     * across the whole curve and check the real fan follows without stalls. */
    int lo = p->zero_rpm ? p->off_below_mC - 3000 : p->t_mC[0] - 3000;
    int hi = p->t_mC[p->npoints - 1] + 2000, half = g_quick ? 25 : 45;
    /* FIX [Robustness]: first dwell 15 s at the curve's start so the floor speed is
     * measured after the kick; the ramp alone passed the floor during the kick. */
    int dwell = 15, t_start = p->zero_rpm ? p->off_below_mC : p->t_mC[0];
    say("  actuator sweep: %d s at %d C, then virtual %d -> %d -> %d C over %d s (real fan, real tach)\n",
        dwell, t_start / 1000, lo / 1000, hi / 1000, lo / 1000, 2 * half);
    memset(&st, 0, sizeof st);
    written = -1;
    prev_kicks = 0;
    int lo_v[256], hi_v[256], nlo = 0, nhi = 0;
    for (int k = 0; k <= dwell + 2 * half && !g_abort; k++) {
        next.tv_sec += 1;
        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL) == EINTR && !g_abort) { }
        int j = k - dwell, tri = j <= half ? j : 2 * half - j;
        int vt = k < dwell ? t_start : lo + (int)((long)(hi - lo) * tri / half);
        int rpm = F.tach ? rpm_median(3, 30) : -1;
        int out = fd_step(p, &tm, &st, vt, rpm);
        if (out != written) { set_fan(out); written = out; }
        if (st.mode == FD_MODE_FAULT) v.faults++;
        if (st.stall_kicks > prev_kicks) v.stalls++;
        prev_kicks = st.stall_kicks;
        /* FIX [Robustness]: collect readings and compare medians; the old min/max
         * picked up a 28846 RPM glitch as the top speed */
        if (F.tach && rpm >= 0 && st.pwm == p->floor_pwm && st.mode == FD_MODE_ON && nlo < 256) lo_v[nlo++] = rpm;
        if (F.tach && rpm >= 0 && out == 255 && nhi < 256) hi_v[nhi++] = rpm;
        (void)take_sample();
        if (k % 15 == 0) say("  [%3d/%3d s] virtual %5.1f C  fan %3d (%s)  %4d RPM\n", k, dwell + 2 * half,
                             vt / 1000.0, out, fd_mode_names[st.mode], rpm);
    }
    v.sweep_rpm_lo = nlo ? median_int(lo_v, nlo) : -1;
    v.sweep_rpm_hi = nhi ? median_int(hi_v, nhi) : 0;
    return v;
}

/* ------------------------------------------------------------- outputs */

static int write_atomic(const char *dir, const char *name, const char *data, size_t len)
{
    char tmp[FD_PATH], dst[FD_PATH];
    if (snprintf(tmp, sizeof tmp, "%s/.%s.tmp", dir, name) >= (int)sizeof tmp ||
        snprintf(dst, sizeof dst, "%s/%s", dir, name) >= (int)sizeof dst) return -1;
    int fd = open(tmp, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (fd < 0) return -1;
    int ok = write(fd, data, len) == (ssize_t)len && fsync(fd) == 0;
    if (close(fd) != 0) ok = 0;
    if (!ok || rename(tmp, dst) != 0) { unlink(tmp); return -1; }
    int dfd = open(dir, O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (dfd >= 0) { (void)fsync(dfd); close(dfd); }
    return 0;
}

static int save_profile(const char *dir, const struct fd_profile *p, const char *note)
{
    static char buf[FD_PROFILE_MAX], copy[FD_PROFILE_MAX];
    char comment[512];
    time_t t = time(NULL);
    struct tm tmv;
    localtime_r(&t, &tmv);
    char date[32];
    strftime(date, sizeof date, "%Y-%m-%d %H:%M", &tmv);
    snprintf(comment, sizeof comment, "# generated by fandyn-calibrate %s on %s\n# %s\n", FD_VERSION, date, note);
    int n = fd_profile_format(p, comment, buf, sizeof buf);
    if (n < 0) return -1;
    /* prove the runtime will accept it before writing */
    memcpy(copy, buf, (size_t)n + 1);
    struct fd_profile back;
    const char *why;
    if (fd_profile_parse(copy, (size_t)n, &back, &why) != 0) {
        say("  internal error: generated profile rejected (%s)\n", why);
        return -1;
    }
    return write_atomic(dir, "profile", buf, (size_t)n);
}

static void file_line(const char *path, char *out, size_t cap)
{
    out[0] = 0;
    int n = fd_read_file(path, out, cap);
    if (n <= 0) { snprintf(out, cap, "unknown"); return; }
    for (int i = 0; i < n; i++) if (out[i] == '\n' || out[i] == '\0') { out[i] = 0; break; }
}

/* ------------------------------------------------- offline re-analysis */

/* Refit the thermal model from a saved calibration.csv (no measurement, no fan). */
static int refit(const char *csv)
{
    FILE *f = fopen(csv, "r");
    if (!f) { fprintf(stderr, "fandyn-calibrate: cannot read %s\n", csv); return 1; }
    char line[256], ph[16];
    int have_power = 0;
    NS = 0;
    if (!fgets(line, sizeof line, f)) { fclose(f); return 1; }       /* header */
    while (NS < MAXS && fgets(line, sizeof line, f)) {
        struct sample *x = &S[NS];
        if (sscanf(line, "%15[^,],%lf,%lf,%lf,%lf,%d,%d,%d,%d,%lf", ph, &x->t, &x->temp, &x->power,
                   &x->busy, &x->load, &x->cmd, &x->cur, &x->rpm, &x->u) != 10) continue;
        x->phase = !strcmp(ph, "thermal") ? 1 : !strcmp(ph, "validate") ? 2 : 0;
        if (!isnan(x->power)) have_power = 1;
        NS++;
    }
    fclose(f);
    g_proxy = !have_power;
    printf("%d samples from %s (heat input: %s)\n", NS, csv, g_proxy ? "CPU load" : "power monitor");
    fit_model();
    if (fn < 60) return 2;
    struct model *m = &FIT.m;
    printf("  Ta %.2f +- %.2f C, R(fan off) %.3f +- %.3f, R(full) %.3f +- %.3f, tau(full) %.0f s\n",
           m->Ta, 1.96 * delta_se(q_Ta, 0), R_of(m, 0), 1.96 * delta_se(q_R, 0),
           R_of(m, 1), 1.96 * delta_se(q_R, 1), q_tau(m, 1));
    printf("  sigma %.3f C, max err %.2f C, R^2 %.4f, Durbin-Watson %.2f, Rjc %.3f, n %.2f%s\n",
           FIT.sigma, FIT.maxerr, FIT.r2, FIT.dw, m->Rjc, m->n, g_proxy ? "" : "");
    if (g_proxy) printf("  idle heat P0 %.3f load-units\n", m->P0);
    printf("  model %s: %s\n", FIT.ok ? "accepted" : "REJECTED", FIT.why);
    return FIT.ok ? 0 : 2;
}

/* ------------------------------------------------------------ guardian */

/*
 * A tiny child process that outlives us: if the calibrator dies for any
 * reason (crash, kill -9), the child hands the fan back to the kernel and
 * restarts the runtime.
 *
 * FIX [Robustness]: the first version used PR_SET_PDEATHSIG + getppid(). That
 * signal fires when the parent *thread* dies; with load threads alive the child
 * is reparented to a sibling thread of the same process, getppid() is unchanged,
 * and the guardian concluded the parent lived (found by a kill -9 test). Now a
 * pipe: we hold the write end, the kernel closes it only when the whole process
 * is gone. EOF = we died; one byte = clean goodbye.
 */
static int g_gpipe = -1;

static pid_t start_guardian(int restart_service)
{
    int fds[2];
    if (pipe2(fds, O_CLOEXEC) != 0) return -1;
    pid_t pid = fork();
    if (pid < 0) { close(fds[0]); close(fds[1]); return -1; }
    if (pid > 0) { close(fds[0]); g_gpipe = fds[1]; return pid; }

    close(fds[1]);
    signal(SIGINT, SIG_IGN);       /* Ctrl+C and service stop are the parent's to handle; */
    signal(SIGHUP, SIG_IGN);       /* the guardian must survive them in case the parent  */
    signal(SIGTERM, SIG_IGN);      /* dies uncleanly while cleaning up                   */
    char c;
    ssize_t r;
    do r = read(fds[0], &c, 1); while (r < 0 && errno == EINTR);
    if (r == 0) {                                     /* EOF: parent gone without goodbye */
        (void)fd_write_long(FAN "temp_control", 1);
        if (restart_service) execl("/bin/systemctl", "systemctl", "start", "fandyn.service", (char *)NULL);
    }
    _exit(0);                                         /* never flush the parent's stdio buffers */
}

static void stop_guardian(pid_t pid)
{
    if (pid <= 0) return;
    if (g_gpipe >= 0) {
        if (write(g_gpipe, "x", 1) != 1) { /* child then sees EOF and restores: harmless */ }
        close(g_gpipe);
        g_gpipe = -1;
    }
    while (waitpid(pid, NULL, 0) < 0 && errno == EINTR) { }
}

/* ---------------------------------------------------------------- main */

static void cleanup(void)
{
    __atomic_store_n(&g_load, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&g_quit, 1, __ATOMIC_RELAXED);
    for (int i = 0; i < g_ncpu; i++) pthread_join(g_thr[i], NULL);
    g_ncpu = 0;
    (void)fd_write_long(FAN "temp_control", 1);
}

static void usage(void)
{
    puts("usage: fandyn-calibrate [--force] [--max-temp C] [--quick] [-o DIR]\n"
         "       fandyn-calibrate --background | --next-boot   (recalibrate unattended)\n"
         "  --force       recalibrate now, replacing the existing profile\n"
         "  --background  recalibrate now as a service; follow: journalctl -fu fandyn-calibrate\n"
         "  --next-boot   recalibrate unattended 3 minutes after the next boot\n"
         "  --max-temp C  temperature where the fan reaches full speed (55-80, default 70)\n"
         "  --quick       shorter measurement (about 7 min, less accurate)\n"
         "  -o DIR        output directory (default /var/lib/fandyn)\n"
         "  --refit FILE  refit the thermal model from a saved calibration.csv (no fan)\n"
         "Calibration runs the fan at full speed and loads all CPU cores for ~16 minutes.");
}

/* Request flag: tells the calibration service to run even though a profile exists. */
static int request_flag(const char *dir, char *path, size_t cap)
{
    return snprintf(path, cap, "%s/recalibrate", dir) < (int)cap ? 0 : -1;
}

int main(int argc, char **argv)
{
    const char *out = "/var/lib/fandyn";
    int force = 0, background = 0, next_boot = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-o") && i + 1 < argc) out = argv[++i];
        else if (!strcmp(argv[i], "--quick")) g_quick = 1;
        else if (!strcmp(argv[i], "--force")) force = 1;
        else if (!strcmp(argv[i], "--background")) background = 1;
        else if (!strcmp(argv[i], "--next-boot")) next_boot = 1;
        else if (!strcmp(argv[i], "--refit") && i + 1 < argc) return refit(argv[i + 1]);
        else if (!strcmp(argv[i], "--max-temp") && i + 1 < argc &&
                 fd_parse_int(argv[i + 1], 55, 80, &g_tmax) == 0) i++;
        else if (!strcmp(argv[i], "--version")) { puts("fandyn-calibrate " FD_VERSION); return 0; }
        else { usage(); return strcmp(argv[i], "-h") && strcmp(argv[i], "--help") ? 2 : 0; }
    }
    long v;
    if (fd_read_long(FAN "temp_control", &v) != 0) {
        fprintf(stderr, "fandyn-calibrate: no pwm-fan device (%s); nothing to calibrate\n", FAN);
        return 1;
    }
    if (access(FAN "target_pwm", W_OK) != 0) {
        fprintf(stderr, "fandyn-calibrate: no write access to %s (run as root)\n", FAN);
        return 1;
    }
    if (fd_find_zones("", &g_z) == 0) {
        fprintf(stderr, "fandyn-calibrate: no thermal sensors found\n");
        return 1;
    }
    if (mkdir(out, 0755) != 0 && errno != EEXIST) {
        fprintf(stderr, "fandyn-calibrate: cannot create %s: %s\n", out, strerror(errno));
        return 1;
    }

    /* user-requested recalibration: unattended variants just leave a request */
    char flag[FD_PATH], prof_path[FD_PATH];
    if (request_flag(out, flag, sizeof flag) != 0 ||
        snprintf(prof_path, sizeof prof_path, "%s/profile", out) >= (int)sizeof prof_path) return 1;
    if (background || next_boot) {
        int fd = open(flag, O_WRONLY | O_CREAT | O_CLOEXEC, 0644);
        if (fd < 0 || close(fd) != 0) {
            fprintf(stderr, "fandyn-calibrate: cannot write %s: %s\n", flag, strerror(errno));
            return 1;
        }
        if (next_boot) {
            printf("Recalibration scheduled: it runs about 3 minutes after the next boot (fan loud ~16 min).\n"
                   "Cancel with: sudo rm %s\n", flag);
            return 0;
        }
        if (system("systemctl start --no-block fandyn-calibrate.service") != 0) {
            fprintf(stderr, "fandyn-calibrate: could not start fandyn-calibrate.service; it will run after the next boot\n");
            return 1;
        }
        printf("Recalibration started in the background (fan loud ~16 min).\n"
               "Follow it with: journalctl -fu fandyn-calibrate\n");
        return 0;
    }
    int requested = access(flag, F_OK) == 0;
    if (access(prof_path, F_OK) == 0 && !force && !requested) {
        fprintf(stderr, "fandyn-calibrate: this board is already calibrated (%s).\n"
                        "  recalibrate now:          sudo fandyn-calibrate --force\n"
                        "  in the background:        sudo fandyn-calibrate --background\n"
                        "  after the next boot:      sudo fandyn-calibrate --next-boot\n", prof_path);
        return 4;
    }
    g_guard = g_tmax + 10 > 85 ? 85 : g_tmax + 10;

    /* stop the runtime while we own the fan */
    int was_running = 0;
    if (geteuid() == 0 && access("/run/systemd/system", F_OK) == 0 &&
        system("systemctl is-active --quiet fandyn.service") == 0) {
        was_running = 1;
        if (system("systemctl stop fandyn.service") != 0) say("warning: could not stop fandyn.service\n");
    }

    fflush(stdout);
    pid_t guardian = start_guardian(was_running);

    char path[FD_PATH];
    snprintf(path, sizeof path, "%s/calibration.csv", out);
    g_csv = fopen(path, "w");
    if (g_csv) fprintf(g_csv, "phase,t_s,temp_c,power_w,cpu_busy,load,cmd_pwm,cur_pwm,rpm,u\n");

    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL);

    find_power();
    g_proxy = !g_have_power;
    long nproc = sysconf(_SC_NPROCESSORS_ONLN);
    int want = nproc < 1 ? 1 : nproc > MAXCPU ? MAXCPU : (int)nproc;
    for (int i = 0; i < want; i++)
        if (pthread_create(&g_thr[g_ncpu], NULL, burn, NULL) == 0) g_ncpu++;

    say("fandyn-calibrate %s: learning this board. The fan will be LOUD for ~%d minutes.\n",
        FD_VERSION, g_quick ? 7 : 16);
    say("  sensors: %d zone(s); heat input: %s; %d load threads; safety limit %.0f C\n", g_z.n,
        g_have_power ? "INA3221 power monitor (W)" : "CPU load (power monitor needs root)",
        g_ncpu, g_guard);

    g_t0 = now_s();
    if (fd_write_long(FAN "temp_control", 0) != 0) say("warning: cannot disable kernel fan control\n");
    (void)fd_write_long(FAN "tach_enable", 1);
    cpu_busy();

    phase_fan();
    if (!g_abort) phase_thermal();
    if (!g_abort) fit_model();

    struct fd_profile prof;
    struct valres val = { 0, NAN, 0, 0, 0, 0, 0, 0, -1, 0 };
    int t_full = g_tmax, attempts = 0, passed = 0;
    if (!g_abort) {
        design_profile(&prof, t_full);
        for (attempts = 1; attempts <= 3 && !g_abort; attempts++) {
            val = validate(&prof);
            int stall = val.faults || val.stalls;
            int hot = val.t_max > t_full + 2;
            int hunt = val.reversals > 3 + val.secs / 60;
            int track = F.tach && !(val.sweep_rpm_hi > 2 * (val.sweep_rpm_lo > 0 ? val.sweep_rpm_lo : 1));
            passed = !stall && !hot && !hunt && !val.failsafe && !track;
            say("  result: max %.1f C, %d reversals, %d stall kicks, %d fault ticks",
                val.t_max, val.reversals, val.stalls, val.faults);
            if (!isnan(val.sim_rms)) say(", model vs board %.2f C", val.sim_rms);
            if (F.tach) say(", sweep %d..%d RPM", val.sweep_rpm_lo, val.sweep_rpm_hi);
            say(" -> %s\n", passed ? "PASS" : "retry with safer settings");
            if (passed || attempts == 3) break;
            if (stall) { F.floor = clampi(F.floor + 6, 1, 200); F.kick = clampi(F.kick + 20, F.floor, 255); }
            if (hot) t_full -= 5;
            if (hunt) g_hyst_extra += 1.0;
            design_profile(&prof, t_full);
        }
    }
    cleanup();

    if (g_abort) {
        say("\naborted. ");
        snprintf(path, sizeof path, "%s/profile", out);
        if (access(path, F_OK) != 0) {
            prof = fd_builtin;
            if (F.floor > 0) { prof.floor_pwm = F.floor; prof.kick_pwm = clampi(F.kick, F.floor, 255); prof.kick_ms = F.kick_ms; prof.tach = F.tach; prof.zero_rpm = F.kick_ok; }
            for (int i = 0; i < prof.npoints; i++) if (prof.pwm[i] < prof.floor_pwm) prof.pwm[i] = prof.floor_pwm;
            if (save_profile(out, &prof, "calibration aborted: safe fallback") == 0)
                say("Wrote a safe fallback profile; run 'sudo fandyn-calibrate' to calibrate.\n");
        } else {
            say("Kept the existing profile.\n");
        }
        if (g_csv) fclose(g_csv);
        stop_guardian(guardian);
        if (was_running && system("systemctl start fandyn.service") != 0) { }
        return 3;
    }

    /* simulated tests of the final profile */
    struct simres hot = { 0 }, room = { 0 }, idle = { 0 }, burst = { 0 };
    double p_design = 0;
    if (FIT.ok) {
        p_design = g_proxy ? FIT.p_load * 2.5 : fmax(10.0, 1.2 * FIT.p_load);
        g_sim_p = p_design;
        room = sim_closed(&prof, FIT.m.Ta, pw_const, 3600);
        hot = sim_closed(&prof, FIT.m.Ta + 10, pw_const, 3600);
        idle = sim_closed(&prof, FIT.m.Ta, pw_idle, 3600);
        burst = sim_closed(&prof, FIT.m.Ta, pw_burst, 3600);
    }

    const char *verdict = !FIT.ok ? "FALLBACK (model rejected; safe built-in curve with measured fan limits)"
                         : passed ? "PASS" : "PASS WITH WARNINGS (validation not clean; see above)";
    char note[160];
    snprintf(note, sizeof note, "verdict: %s", verdict);
    int saved = save_profile(out, &prof, note) == 0;
    if (saved) (void)unlink(flag);                       /* request fulfilled */

    /* report */
    snprintf(path, sizeof path, "%s/report.txt", out);
    g_rep = fopen(path, "w");
    char model[128], rel[160];
    file_line("/proc/device-tree/model", model, sizeof model);
    file_line("/etc/nv_tegra_release", rel, sizeof rel);
    time_t now = time(NULL);
    char stamp[32];
    struct tm tmn;
    localtime_r(&now, &tmn);
    strftime(stamp, sizeof stamp, "%Y-%m-%d %H:%M", &tmn);
    rep("fandyn calibration report  (%s)\n", stamp);
    rep("board:   %s\nrelease: %s\nverdict: %s\n\n", model, rel, verdict);
    rep("FAN\n  tach: %s, full speed %d RPM, kernel ramp %.0f PWM/s\n", F.tach ? "yes" : "no", F.rpm_max, F.ramp_rate);
    for (int i = 0; i < F.n; i++) {
        double db = F.rpm[i] > 0 && F.rpm_max > 0 ? 50 * log10((double)F.rpm[i] / F.rpm_max) : -99;
        if (db > -40) rep("  pwm %3d: %5d RPM  (%+5.1f dB vs full speed, fan-law estimate)\n", F.pwm[i], F.rpm[i], db);
        else rep("  pwm %3d: %5d RPM  (below -40 dB: near silent)\n", F.pwm[i], F.rpm[i]);
    }
    rep("  lowest running %d, floor %d, starts at %d, kick %d for %d ms (%s)\n\n", F.lowest_run, F.floor,
        F.start_pwm, F.kick, F.kick_ms, F.kick_ok ? "verified 5/5 starts from standstill" : "NOT reliable");

    const char *pu = g_proxy ? "load-units" : "W";
    rep("THERMAL MODEL  C dTs/dt = P - (G0 + G1 u^n)(Ts - Ta),  Td = Ts + Rjc P\n");
    rep("  heat input: %s\n", g_proxy ? "CPU load (1 = all cores busy); power monitor not readable" : "INA3221 board input power");
    if (fn >= 60) {
        struct model *m = &FIT.m;
        rep("  samples %d, residual sigma %.3f C, max error %.2f C, R^2 %.4f\n", fn, FIT.sigma, FIT.maxerr, FIT.r2);
        rep("  Durbin-Watson %.2f (residual autocorrelation %.2f, errors inflated x%.2f)\n", FIT.dw, FIT.rho,
            sqrt((1 + FIT.rho) / (1 - FIT.rho)));
        rep("  ambient Ta        %6.2f C   +- %.2f\n", m->Ta, 1.96 * delta_se(q_Ta, 0));
        /* FIX [Domain]: in proxy mode C is in load-unit*s/K, not J/K */
        rep("  heat capacity C   %6.1f %s\n", m->C, g_proxy ? "load-unit*s/K" : "J/K");
        rep("  conductance G0    %6.4f %s/K (fan off)   G1 %6.4f   exponent n %.2f\n", m->G0, pu, m->G1, m->n);
        rep("  die-to-sink Rjc   %6.3f K/%s  (die jumps %.1f C within ~1 s of full CPU load)\n", m->Rjc, pu,
            m->Rjc * (FIT.p_load - FIT.p_idle));
        if (g_proxy) rep("  idle heat P0      %6.3f load-units\n", m->P0);
        rep("  power: idle %.2f %s, CPU load %.2f %s\n", FIT.p_idle, pu, FIT.p_load, pu);
        rep("  thermal resistance and time constant by fan speed (95%% CI):\n");
        static const int show[] = { 0, -1, 32, 64, 96, 128, 192, 255 };
        for (int i = 0; i < 8; i++) {
            int q = show[i] < 0 ? F.floor : show[i];
            double u = fan_u(q);
            rep("    pwm %3d  u %.2f  R %6.3f +- %.3f K/%s   tau %6.0f s\n", q, u, R_of(m, u),
                1.96 * delta_se(q_R, u), pu, q_tau(m, u));
        }
        rep("  priors: ambient 25 +- 12 C%s\n", g_proxy ? ", idle heat 0.5 load-units (x/2 to x2)" : "");
        rep("  model %s: %s\n\n", FIT.ok ? "accepted" : "REJECTED", FIT.why);
    }
    if (!isnan(g_idle_pred))
        rep("  predicted idle temperature with the fan off: %.1f C (95%% upper bound %.1f C);\n"
            "  fan-on threshold = max(that + 1 C, %d C policy floor)\n", g_idle_pred, g_idle_hi, FAN_ON_MIN_C);
    if (g_proxy)
        rep("  NOTE: without the power monitor, ambient and idle heat are only separable through\n"
            "  the priors; the fan-dependent part (the curve shape) is what the data determines.\n"
            "  The package calibrates as root, where the INA3221 power monitor is used.\n");
    rep("\nPROFILE  (fan full speed at %d C)\n", t_full);
    rep("  zero-RPM %s, off below %.1f C, hysteresis %.1f C\n", prof.zero_rpm ? "on" : "off",
        prof.off_below_mC / 1000.0, prof.hyst_mC / 1000.0);
    rep("  floor %d, kick %d for %d ms, slew down %d PWM/s, min on %d s, critical %d C\n",
        prof.floor_pwm, prof.kick_pwm, prof.kick_ms, prof.slew_down, prof.min_on_s, prof.crit_mC / 1000);
    for (int i = 0; i < prof.npoints; i++) rep("  %5.1f C -> pwm %3d\n", prof.t_mC[i] / 1000.0, prof.pwm[i]);
    rep("\nTEST ON THE BOARD  (%d attempt(s))\n", attempts);
    rep("  %d s, max %.1f C, %d fan reversals, %d fan starts, %d stall kicks, %d fault ticks, %d failsafe ticks\n",
        val.secs, val.t_max, val.reversals, val.starts, val.stalls, val.faults, val.failsafe);
    if (!isnan(val.sim_rms)) rep("  model vs board in closed loop: RMS %.2f C\n", val.sim_rms);
    if (F.tach) rep("  actuator sweep across the whole curve: median %d RPM at floor .. %d RPM at top, no stall: %s\n",
                    val.sweep_rpm_lo, val.sweep_rpm_hi, val.stalls ? "NO" : "yes");
    rep("  %s\n\n", passed ? "PASS" : "NOT CLEAN");
    if (FIT.ok) {
        rep("SIMULATED TESTS  (real control code, fitted model, 1 hour each)\n");
        rep("  design load %.2f %s%s:\n", p_design, pu, g_proxy ? " (assumed 2.5x CPU-only heat)" : "");
        rep("    room %.1f C:    settles at %.1f C, fan %d\n", FIT.m.Ta, room.t_end, room.pwm_end);
        rep("    room %.1f C:    settles at %.1f C, fan %d%s\n", FIT.m.Ta + 10, hot.t_end, hot.pwm_end,
            hot.t_end >= prof.crit_mC / 1000.0 - 3 ? "  WARNING: near critical; kernel throttling will protect" : "");
        rep("  idle:            settles at %.1f C, fan %d (%s)\n", idle.t_end, idle.pwm_end, idle.pwm_end ? "on" : "silent");
        rep("  30 s on/off load: max %.1f C, %d fan starts/h, %d reversals/h, mean pwm %.0f\n",
            burst.t_max, burst.starts, burst.reversals, burst.mean_pwm);
    }
    if (g_rep) fclose(g_rep);

    snprintf(path, sizeof path, "%s/history.csv", out);
    int newh = access(path, F_OK) != 0;
    FILE *h = fopen(path, "a");
    if (h) {
        char d[32];
        struct tm tmv;
        localtime_r(&now, &tmv);
        strftime(d, sizeof d, "%Y-%m-%d %H:%M", &tmv);
        if (newh) fprintf(h, "date,verdict,ambient_c,r_fan_off,r_fan_full,tau_full_s,sigma_c,floor,kick,rpm_max,units\n");
        /* FIX [Robustness]: without a fit the model is all zeros and R = 1/0;
         * write NA instead of inf/nan into the database */
        if (fn >= 60)
            fprintf(h, "%s,%s,%.2f,%.3f,%.3f,%.0f,%.3f,%d,%d,%d,%s\n", d, FIT.ok ? (passed ? "pass" : "warn") : "fallback",
                    FIT.m.Ta, R_of(&FIT.m, 0), R_of(&FIT.m, 1), FIT.m.C / G_of(&FIT.m, 1), FIT.sigma,
                    F.floor, F.kick, F.rpm_max, pu);
        else
            fprintf(h, "%s,fallback,NA,NA,NA,NA,NA,%d,%d,%d,%s\n", d, F.floor, F.kick, F.rpm_max, pu);
        fclose(h);
    }
    if (g_csv) fclose(g_csv);

    say("\n%s. Profile %s %s/profile; report in %s/report.txt\n", verdict,
        saved ? "written to" : "COULD NOT be written to", out, out);
    stop_guardian(guardian);
    if (was_running && system("systemctl start fandyn.service") != 0) say("warning: could not restart fandyn.service\n");
    return !saved ? 1 : FIT.ok && passed ? 0 : 2;
}
