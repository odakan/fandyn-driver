/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2026 deniz <onurdenizakan@gmail.com> */
/*
 * fandynd - fan control runtime for the Jetson Nano.
 *
 * Loads the fixed profile made by fandyn-calibrate, then once per second:
 * read temperature -> fd_step() -> write target_pwm. Nothing else.
 *
 * Design rules: no heap, no floating point, no threads, bounded loops,
 * every error handled locally. It never exits on hardware errors: it keeps
 * retrying quietly, and runs the fan flat out when it cannot see a sensor.
 * On SIGTERM/SIGINT it hands the fan back to the kernel.
 */
#define _GNU_SOURCE
#include "fandyn.h"

#include <signal.h>
#include <stdarg.h>
#include <stddef.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>

#define DEFAULT_PROFILE "/var/lib/fandyn/profile"
#define DEFAULT_RUNDIR  "/run/fandyn"
#define FAN_DIR         "/sys/devices/pwm-fan"
#define REFRESH_TICKS   30     /* rewrite target_pwm even when unchanged */
#define ASSERT_TICKS    10     /* re-check temp_control / tach_enable */
#define RESCAN_TICKS    10     /* re-discover sensors while they are failing */
#define LOG_QUIET_MS    600000 /* repeat one kind of warning at most every 10 min */

static volatile sig_atomic_t g_stop, g_reload;
static void on_signal(int sig) { if (sig == SIGHUP) g_reload = 1; else g_stop = 1; }

static struct {
    char target[FD_PATH], control[FD_PATH], tach_en[FD_PATH], rpm[FD_PATH];
    char status[FD_PATH], status_tmp[FD_PATH];
} path;

static char g_pbuf[FD_PROFILE_MAX];
static long g_tick;
static int  g_tick_ms = 1000;

/* ---------------------------------------------------------------- logging */

enum { EV_PROFILE, EV_ZONES, EV_TEMP, EV_WRITE, EV_CONTROL, EV_MODE, EV_COUNT };
static long g_last_log[EV_COUNT];
static int  g_logged[EV_COUNT];

static void vlog(int prio, const char *fmt, va_list ap)
{
    char buf[300];
    int n = snprintf(buf, sizeof buf, "<%d>", prio);
    int m = vsnprintf(buf + n, sizeof buf - (size_t)n - 1, fmt, ap);
    if (m < 0) return;
    size_t len = (size_t)n + (size_t)m;
    if (len > sizeof buf - 2) len = sizeof buf - 2;
    buf[len++] = '\n';
    if (write(STDERR_FILENO, buf, len) < 0) { /* nowhere else to report */ }
}

static void logmsg(int prio, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vlog(prio, fmt, ap);
    va_end(ap);
}

/* Rate-limited: each event kind logs at most once per LOG_QUIET_MS. */
static void logev(int ev, int prio, const char *fmt, ...)
{
    long quiet = LOG_QUIET_MS / g_tick_ms;
    if (g_logged[ev] && g_tick - g_last_log[ev] < quiet) return;
    g_logged[ev] = 1;
    g_last_log[ev] = g_tick;
    va_list ap;
    va_start(ap, fmt);
    vlog(prio, fmt, ap);
    va_end(ap);
}

/* --------------------------------------------------- systemd notification */

static int g_nfd = -1;
static struct sockaddr_un g_naddr;
static socklen_t g_nlen;

static void notify_init(void)
{
    const char *s = getenv("NOTIFY_SOCKET");
    if (!s || (s[0] != '/' && s[0] != '@')) return;
    size_t l = strlen(s);
    if (l >= sizeof g_naddr.sun_path) return;
    memset(&g_naddr, 0, sizeof g_naddr);
    g_naddr.sun_family = AF_UNIX;
    memcpy(g_naddr.sun_path, s, l);
    if (s[0] == '@') g_naddr.sun_path[0] = '\0';     /* abstract socket */
    g_nlen = (socklen_t)(offsetof(struct sockaddr_un, sun_path) + l);
    g_nfd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
}

static void notify(const char *msg)
{
    if (g_nfd < 0) return;
    (void)sendto(g_nfd, msg, strlen(msg), MSG_DONTWAIT | MSG_NOSIGNAL,
                 (const struct sockaddr *)&g_naddr, g_nlen);
}

/* ------------------------------------------------------------------ setup */

static int load_profile(const char *file, struct fd_profile *out, const char **why)
{
    int n = fd_read_file(file, g_pbuf, sizeof g_pbuf);
    if (n < 0) { *why = errno == ENOENT ? "not found" : "unreadable"; return -1; }
    if (n >= (int)sizeof g_pbuf - 1) { *why = "too large"; return -1; }
    return fd_profile_parse(g_pbuf, (size_t)n, out, why);
}

/* Load the profile, or fall back to the built-in one. Logs the outcome. */
static void use_profile(const char *file, struct fd_profile *p, int keep_current)
{
    struct fd_profile np;
    const char *why = "";
    if (load_profile(file, &np, &why) == 0) {
        *p = np;
        logmsg(6, "profile %s loaded (%s, %d points, floor %d)", file,
               np.calibrated ? "calibrated" : "fallback", np.npoints, np.floor_pwm);
    } else if (keep_current) {
        logmsg(4, "profile %s rejected (%s); keeping the current one", file, why);
    } else {
        *p = fd_builtin;
        logev(EV_PROFILE, 4, "profile %s rejected (%s); using built-in safe profile", file, why);
    }
}

static int mkpath(char *dst, const char *a, const char *b)
{
    return snprintf(dst, FD_PATH, "%s%s", a, b) >= FD_PATH ? -1 : 0;
}

/* "45.5 C" from millidegrees, sign-safe. */
static void fmt_temp(char *b, size_t cap, int t)
{
    if (t == FD_TEMP_INVALID) { snprintf(b, cap, "unknown"); return; }
    unsigned a = t < 0 ? 0u - (unsigned)t : (unsigned)t;
    snprintf(b, cap, "%s%u.%u C", t < 0 ? "-" : "", a / 1000u, a % 1000u / 100u);
}

static int write_status(const struct fd_profile *p, const struct fd_state *s, int temp, int rpm, int out)
{
    char buf[400];
    char t[24];
    fmt_temp(t, sizeof t, temp);
    int n = snprintf(buf, sizeof buf,
        "state=%s\ntemp=%s\npwm=%d\nrpm=%d\nprofile=%s\nfloor_bias=%d\nuptime_s=%ld\n",
        fd_mode_names[s->mode], t, out, rpm, p->calibrated ? "calibrated" : "fallback",
        s->floor_bias, g_tick * g_tick_ms / 1000);
    if (n <= 0 || (size_t)n >= sizeof buf) return -1;
    int fd = open(path.status_tmp, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (fd < 0) return -1;
    ssize_t w = write(fd, buf, (size_t)n);
    int ok = (w == n);
    if (close(fd) != 0) ok = 0;
    if (!ok || rename(path.status_tmp, path.status) != 0) { (void)unlink(path.status_tmp); return -1; }
    return 0;
}

/* --------------------------------------------------------------- commands */

static int cmd_status(const char *rundir)
{
    char file[FD_PATH], buf[512];
    if (mkpath(file, rundir, "/status")) return 2;
    int n = fd_read_file(file, buf, sizeof buf);
    if (n <= 0) { fprintf(stderr, "fandynd is not running (no %s)\n", file); return 1; }
    fputs(buf, stdout);
    return 0;
}

static int cmd_check(const char *root, const char *profile)
{
    struct fd_profile p;
    const char *why = "";
    int rc = 0;
    if (load_profile(profile, &p, &why) == 0) {
        printf("profile:  %s (%s)\n", profile, p.calibrated ? "calibrated" : "fallback");
    } else {
        printf("profile:  %s INVALID: %s -> built-in safe profile\n", profile, why);
        p = fd_builtin;
        rc = 2;
    }
    struct fd_zones z;
    int nz = fd_find_zones(root, &z);
    int t = fd_read_temp(&z);
    char tb[24];
    fmt_temp(tb, sizeof tb, t);
    printf("sensors:  %d zone(s), hottest %s\n", nz, tb);
    long v;
    int fan = fd_read_long(path.control, &v) == 0;
    printf("fan:      %s", fan ? "found" : "NOT FOUND\n");
    if (fan) printf(" (temp_control=%ld)\n", v);
    if (!fan && rc == 0) rc = 1;
    printf("zero-rpm: %s", p.zero_rpm ? "" : "off\n");
    if (p.zero_rpm) printf("fan stops below %d C (after cooling %d.%d C)\n", p.off_below_mC / 1000,
                           p.hyst_mC / 1000, p.hyst_mC % 1000 / 100);
    printf("floor %d, kick %d for %d ms, slew down %d/s, min on %d s, critical %d C, tach %s\n",
           p.floor_pwm, p.kick_pwm, p.kick_ms, p.slew_down, p.min_on_s, p.crit_mC / 1000,
           p.tach ? "on" : "off");
    printf("curve:   ");
    for (int c = 30; c <= 90; c += 5) printf(" %dC:%d", c, fd_curve(&p, c * 1000));
    printf("\n");
    return rc;
}

static void usage(void)
{
    puts("usage: fandynd                 run (normally started by systemd)\n"
         "       fandynd --check         show profile, sensors and fan\n"
         "       fandynd status          show live state\n"
         "       fandynd --version\n"
         "test options: --profile FILE  --root DIR  --run-dir DIR  --tick-ms N");
}

/* ------------------------------------------------------------------- main */

int main(int argc, char **argv)
{
    const char *profile = DEFAULT_PROFILE, *root = "", *rundir = DEFAULT_RUNDIR;
    int check = 0, status = 0;

    for (int i = 1; i < argc; i++) {
        const char *a = argv[i], *v = i + 1 < argc ? argv[i + 1] : NULL;
        if (!strcmp(a, "--check")) check = 1;
        else if (!strcmp(a, "status")) status = 1;
        else if (!strcmp(a, "--version")) { puts("fandynd " FD_VERSION); return 0; }
        else if (!strcmp(a, "--profile") && v) { profile = v; i++; }
        else if (!strcmp(a, "--root") && v) { root = v; i++; }
        else if (!strcmp(a, "--run-dir") && v) { rundir = v; i++; }
        else if (!strcmp(a, "--tick-ms") && v && fd_parse_int(v, 1, 10000, &g_tick_ms) == 0) i++;
        else { usage(); return !strcmp(a, "-h") || !strcmp(a, "--help") ? 0 : 2; }
    }
    if (status) return cmd_status(rundir);

    char fan[FD_PATH];
    if (snprintf(fan, sizeof fan, "%s%s", root, FAN_DIR) >= (int)sizeof fan ||
        mkpath(path.target, fan, "/target_pwm") || mkpath(path.control, fan, "/temp_control") ||
        mkpath(path.tach_en, fan, "/tach_enable") || mkpath(path.rpm, fan, "/rpm_measured") ||
        mkpath(path.status, rundir, "/status") || mkpath(path.status_tmp, rundir, "/status.tmp")) {
        fputs("fandynd: path too long\n", stderr);
        return 2;
    }
    if (check) return cmd_check(root, profile);

    (void)mlockall(MCL_CURRENT | MCL_FUTURE);     /* never swap out the fan loop */
    if (!isatty(STDERR_FILENO)) {                 /* a stuck journal must never block the loop */
        int fl = fcntl(STDERR_FILENO, F_GETFL);
        if (fl >= 0) (void)fcntl(STDERR_FILENO, F_SETFL, fl | O_NONBLOCK);
    }
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    (void)sigemptyset(&sa.sa_mask);
    (void)sigaction(SIGTERM, &sa, NULL);
    (void)sigaction(SIGINT, &sa, NULL);
    (void)sigaction(SIGHUP, &sa, NULL);
    sa.sa_handler = SIG_IGN;
    (void)sigaction(SIGPIPE, &sa, NULL);

    struct fd_profile prof;
    struct fd_timing tm;
    struct fd_state st;
    struct fd_zones zones;
    memset(&st, 0, sizeof st);
    use_profile(profile, &prof, 0);
    fd_timing_init(&prof, g_tick_ms, &tm);
    if (fd_find_zones(root, &zones) == 0) logev(EV_ZONES, 4, "no thermal zones found yet; will keep looking");
    notify_init();
    notify("READY=1");
    logmsg(6, "fandynd %s running, %d sensor(s)", FD_VERSION, zones.n);

    int written = -1, since_write = 0, bad_temp = 0, last_mode = -1, fan_ok = 1;
    struct timespec next;
    (void)clock_gettime(CLOCK_MONOTONIC, &next);

    while (!g_stop) {
        /* fixed-rate tick; resync instead of bursting if we fell behind */
        next.tv_nsec += (long)(g_tick_ms % 1000) * 1000000L;
        next.tv_sec += g_tick_ms / 1000 + next.tv_nsec / 1000000000L;
        next.tv_nsec %= 1000000000L;
        int sr = 0;
        while (!g_stop && (sr = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL)) == EINTR)
            if (g_reload) break;
        if (g_stop) break;
        if (sr != 0 && sr != EINTR) {           /* never spin: fall back to a plain sleep */
            struct timespec d = { g_tick_ms / 1000, (long)(g_tick_ms % 1000) * 1000000L };
            (void)nanosleep(&d, NULL);
            (void)clock_gettime(CLOCK_MONOTONIC, &next);
        }
        struct timespec now;
        (void)clock_gettime(CLOCK_MONOTONIC, &now);
        if (now.tv_sec - next.tv_sec > 2) next = now;
        g_tick++;

        if (g_reload) {
            g_reload = 0;
            use_profile(profile, &prof, 1);
            fd_timing_init(&prof, g_tick_ms, &tm);
        }

        /* keep the kernel's own fan logic switched off (it may come back after
           a driver reset, nvpmodel change or resume) */
        if (g_tick % ASSERT_TICKS == 1 || !fan_ok) {
            long ctl;
            if (fd_read_long(path.control, &ctl) != 0) {
                if (fan_ok) logev(EV_CONTROL, 4, "fan device not found at %s; retrying", fan);
                fan_ok = 0;
                written = -1;
            } else {
                if (!fan_ok) logmsg(6, "fan device found");
                fan_ok = 1;
                if (ctl != 0) {
                    if (fd_write_long(path.control, 0) == 0) {
                        if (g_tick > 1) logev(EV_CONTROL, 4, "kernel fan control was re-enabled; took it back");
                        written = -1;
                    } else {
                        logev(EV_CONTROL, 3, "cannot disable kernel fan control: %s", strerror(errno));
                    }
                }
                long te;
                if (prof.tach && (fd_read_long(path.tach_en, &te) != 0 || te != 1))
                    (void)fd_write_long(path.tach_en, 1);
            }
        }

        int temp = fd_read_temp(&zones);
        if (temp == FD_TEMP_INVALID) {
            bad_temp = fd_inc(bad_temp);
            if (bad_temp == FD_BAD_TICKS) logev(EV_TEMP, 3, "no readable temperature sensor; fan at full speed");
            if (bad_temp % RESCAN_TICKS == 0) (void)fd_find_zones(root, &zones);
        } else if (bad_temp) {
            if (bad_temp >= FD_BAD_TICKS) logmsg(6, "temperature sensors back");
            bad_temp = 0;
        }

        int rpm = -1;
        long r;
        if (prof.tach && fd_read_long(path.rpm, &r) == 0 && r >= 0 && r <= FD_RPM_MAX) rpm = (int)r;

        int out = fd_step(&prof, &tm, &st, temp, rpm);

        if (st.mode != last_mode) {
            if (st.mode == FD_MODE_CRIT || st.mode == FD_MODE_FAULT || st.mode == FD_MODE_FAILSAFE)
                logev(EV_MODE, 4, "mode %s (temp %d mC, rpm %d)", fd_mode_names[st.mode], temp, rpm);
            else if (last_mode == FD_MODE_CRIT || last_mode == FD_MODE_FAULT || last_mode == FD_MODE_FAILSAFE)
                logmsg(6, "back to normal (floor bias %d)", st.floor_bias);
            last_mode = st.mode;
        }

        since_write = fd_inc(since_write);
        if (fan_ok && (out != written || since_write >= REFRESH_TICKS)) {
            if (fd_write_long(path.target, out) == 0) {
                written = out;
                since_write = 0;
            } else {
                written = -1;
                logev(EV_WRITE, 3, "cannot write %s: %s", path.target, strerror(errno));
            }
        }

        (void)write_status(&prof, &st, temp, rpm, out);
        notify("WATCHDOG=1");
    }

    notify("STOPPING=1");
    int back = fd_write_long(path.control, 1);
    (void)unlink(path.status);
    logmsg(6, "stopped; fan control %s", back == 0 ? "returned to kernel" : "could not be returned to kernel");
    return 0;
}
