/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2026 deniz <onurdenizakan@gmail.com> */
/*
 * Unit tests for the shared core (fandyn.h): control law, profile parser,
 * and a fuzzer for the parser. Deterministic; no hardware needed.
 *   ./test_core              run all tests
 *   ./test_core --emit FILE  write a valid test profile (used by test_runtime.sh)
 *   ./test_core --emit-tach FILE   same, with tach supervision enabled
 */
#define _GNU_SOURCE
#include "../src/fandyn.h"

static int fails, checks;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { fails++; \
    printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static struct fd_profile test_profile(int tach)
{
    struct fd_profile p = {
        .calibrated = 1, .npoints = 4,
        .t_mC = { 45000, 55000, 65000, 70000 },
        .pwm  = {    20,    60,   150,   255 },
        .zero_rpm = 1, .off_below_mC = 45000, .hyst_mC = 2000,
        .floor_pwm = 20, .kick_pwm = 50, .kick_ms = 2000,
        .slew_down = 3, .min_on_s = 30, .crit_mC = 85000, .tach = tach,
    };
    return p;
}

static int fmt(const struct fd_profile *p, char *buf, size_t cap)
{
    return fd_profile_format(p, "# test\n", buf, cap);
}

static unsigned long long rng = 88172645463325252ull;
static unsigned rnd(void) { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return (unsigned)rng; }

/* Re-sign a mutated body with a correct CRC line so fuzzing reaches the parser proper. */
static int resign(char *buf, size_t cap)
{
    char *c = strstr(buf, "\ncrc ");
    size_t n = c ? (size_t)(c - buf) + 1 : strlen(buf);
    int w = snprintf(buf + n, cap - n, "crc %08x\n", (unsigned)fd_crc32(buf, n));
    return (w < 0 || (size_t)w >= cap - n) ? -1 : (int)(n + (size_t)w);
}

static void test_parser(void)
{
    char buf[FD_PROFILE_MAX], copy[FD_PROFILE_MAX];
    struct fd_profile p = test_profile(1), q;
    const char *why;
    int n = fmt(&p, buf, sizeof buf);
    CHECK(n > 0, "format");
    memcpy(copy, buf, sizeof buf);
    CHECK(fd_profile_parse(copy, (size_t)n, &q, &why) == 0, "roundtrip parse: %s", why);
    CHECK(memcmp(&p, &q, sizeof p) == 0, "roundtrip equal");
    CHECK(fd_profile_check(&fd_builtin, &why) == 0, "builtin valid: %s", why);

    /* each corruption must be rejected */
    struct { const char *from, *to; const char *what; } bad[] = {
        { "floor 20", "floor 300", "range" },
        { "floor 20", "floor abc", "number" },
        { "floor 20", "flor 20", "unknown key" },
        { "tach 1\n", "", "missing key" },
        { "point 55000 60", "point 55000 10", "below floor" },
        { "point 55000 60", "point 44000 60", "not ascending" },
        { "point 65000 150", "point 65000 10", "decreasing" },
        { "point 70000 255", "point 70000 250", "last not 255" },
        { "crit 85000", "crit 60000", "crit below curve" },
        { "fandyn-profile 1", "fandyn-profile 2", "version" },
        { "kick 50", "kick 10", "kick < floor" },
        { "point 45000 20", "point 45000 20 9", "extra field" },
    };
    for (size_t i = 0; i < sizeof bad / sizeof bad[0]; i++) {
        memcpy(copy, buf, sizeof buf);
        char *at = strstr(copy, bad[i].from);
        CHECK(at != NULL, "pattern %s", bad[i].from);
        if (!at) continue;
        size_t fl = strlen(bad[i].from), tl = strlen(bad[i].to);
        memmove(at + tl, at + fl, strlen(at + fl) + 1);
        memcpy(at, bad[i].to, tl);
        int m = resign(copy, sizeof copy);
        CHECK(m > 0 && fd_profile_parse(copy, (size_t)m, &q, &why) != 0, "accepted bad profile (%s)", bad[i].what);
    }
    /* wrong crc, truncated, trailing data, NUL byte, empty */
    memcpy(copy, buf, sizeof buf);
    copy[20] ^= 1;
    CHECK(fd_profile_parse(copy, (size_t)n, &q, &why) != 0, "bit flip accepted");
    memcpy(copy, buf, sizeof buf);
    copy[n - 3] = '\0';
    CHECK(fd_profile_parse(copy, (size_t)n - 3, &q, &why) != 0, "truncated accepted");
    memcpy(copy, buf, sizeof buf);
    strcat(copy, "floor 30\n");
    CHECK(fd_profile_parse(copy, strlen(copy), &q, &why) != 0, "trailing data accepted");
    memcpy(copy, buf, sizeof buf);
    copy[30] = '\0';
    CHECK(fd_profile_parse(copy, (size_t)n, &q, &why) != 0, "embedded NUL accepted");
    copy[0] = '\0';
    CHECK(fd_profile_parse(copy, 0, &q, &why) != 0, "empty accepted");
}

static void test_fuzz(void)
{
    char buf[FD_PROFILE_MAX], m[FD_PROFILE_MAX];
    struct fd_profile p = test_profile(1), q;
    const char *why;
    int n = fmt(&p, buf, sizeof buf), accepted = 0, iters = 200000;
    for (int it = 0; it < iters; it++) {
        memcpy(m, buf, (size_t)n + 1);
        size_t len = (size_t)n;
        int edits = 1 + (int)(rnd() % 4);
        for (int e = 0; e < edits; e++) {
            size_t at = rnd() % (len ? len : 1);
            switch (rnd() % 4) {
            case 0: m[at] = (char)(rnd() & 0xff); break;                      /* replace */
            case 1: m[at] = "0123456789 \n-+x#"[rnd() % 16]; break;           /* plausible char */
            case 2: if (len > 1) { memmove(m + at, m + at + 1, len - at); len--; } break;
            default: if (len + 2 < sizeof m) { memmove(m + at + 1, m + at, len - at + 1); m[at] = (char)('0' + rnd() % 10); len++; }
            }
        }
        m[len] = '\0';
        int ml = (int)strlen(m);
        if (it % 2) ml = resign(m, sizeof m);              /* half get a valid CRC */
        if (ml < 0) continue;
        if (fd_profile_parse(m, (size_t)ml, &q, &why) == 0) {
            accepted++;
            CHECK(fd_profile_check(&q, &why) == 0, "accepted invalid profile");
            for (int t = -40000; t <= 150000; t += 250) {
                int c = fd_curve(&q, t);
                if (c < 0 || c > 255 || (c > 0 && c < q.floor_pwm)) { CHECK(0, "curve out of range"); break; }
            }
        }
    }
    printf("fuzz: %d mutated profiles, %d accepted (all valid)\n", iters, accepted);
}

static int run(const struct fd_profile *p, const struct fd_timing *tm, struct fd_state *s, int t, int rpm, int ticks)
{
    int out = -1;
    for (int i = 0; i < ticks; i++) out = fd_step(p, tm, s, t, rpm);
    return out;
}

static void test_control(void)
{
    struct fd_profile p = test_profile(0);
    struct fd_timing tm;
    struct fd_state s;
    fd_timing_init(&p, 1000, &tm);
    memset(&s, 0, sizeof s);

    CHECK(run(&p, &tm, &s, 30000, -1, 5) == 0, "cold: off");
    CHECK(fd_step(&p, &tm, &s, 50000, -1) == 50, "start: kick");
    CHECK(fd_step(&p, &tm, &s, 50000, -1) == 50, "kick 2 ticks");
    CHECK(fd_step(&p, &tm, &s, 50000, -1) == 40, "after kick: curve(50C)=40");
    CHECK(fd_step(&p, &tm, &s, 60000, -1) == 105, "up immediately: curve(60C)");
    CHECK(fd_step(&p, &tm, &s, 59000, -1) == 105, "1 C drop < hyst: hold");
    int a = fd_step(&p, &tm, &s, 50000, -1);
    CHECK(a == 102, "drop: slew 3/s (got %d)", a);
    a = run(&p, &tm, &s, 50000, -1, 100);
    CHECK(a == 48, "settles at curve(T+hyst)=curve(52C)=48 (got %d)", a);

    /* min-on: a fan that just started holds its floor before stopping */
    memset(&s, 0, sizeof s);
    run(&p, &tm, &s, 50000, -1, 3);
    a = run(&p, &tm, &s, 40000, -1, 10);
    CHECK(a == 20, "cooled right after start: holds floor (got %d)", a);
    a = run(&p, &tm, &s, 40000, -1, 30);
    CHECK(a == 0, "off once min_on has passed (got %d)", a);
    CHECK(fd_step(&p, &tm, &s, 44000, -1) == 0, "44C: still off (zero-RPM)");
    CHECK(fd_step(&p, &tm, &s, 90000, -1) == 255, "critical: 255 at once from off");
    CHECK(s.mode == FD_MODE_CRIT, "mode crit");
    a = fd_step(&p, &tm, &s, 50000, -1);
    CHECK(a == 252, "after critical: slews down (got %d)", a);

    /* sensor loss */
    memset(&s, 0, sizeof s);
    run(&p, &tm, &s, 50000, -1, 10);
    CHECK(fd_step(&p, &tm, &s, FD_TEMP_INVALID, -1) == 40, "1 bad tick: hold");
    CHECK(fd_step(&p, &tm, &s, FD_TEMP_INVALID, -1) == 40, "2 bad ticks: hold");
    CHECK(fd_step(&p, &tm, &s, FD_TEMP_INVALID, -1) == 255, "3 bad ticks: failsafe 255");
    CHECK(run(&p, &tm, &s, FD_TEMP_INVALID, -1, 1000) == 255, "stays failsafe");
    CHECK(fd_step(&p, &tm, &s, 50000, -1) == 252, "sensor back: slew down");
}

static void test_stall(void)
{
    struct fd_profile p = test_profile(1);
    struct fd_timing tm;
    struct fd_state s;
    fd_timing_init(&p, 1000, &tm);
    memset(&s, 0, sizeof s);
    run(&p, &tm, &s, 50000, 1000, 10);
    CHECK(s.pwm == 40 && !s.fault, "running");
    int out = run(&p, &tm, &s, 50000, 0, FD_STALL_TICKS);
    CHECK(out == 50 && s.stall_kicks == 1, "stall -> re-kick (out %d kicks %d)", out, s.stall_kicks);
    run(&p, &tm, &s, 50000, 0, 40);
    CHECK(s.fault == 1, "3 failed kicks -> fault");
    CHECK(fd_step(&p, &tm, &s, 50000, 0) == 255, "fault: 255");
    run(&p, &tm, &s, 50000, 3000, FD_RECOVER_S);
    CHECK(s.fault == 0 && s.floor_bias == FD_BIAS_STEP, "recovered, floor raised (bias %d)", s.floor_bias);
    out = run(&p, &tm, &s, 45000, 1000, 400);
    CHECK(out == p.floor_pwm + FD_BIAS_STEP, "new floor used (got %d)", out);
    /* glitchy tach (-1) never counts as a stall */
    memset(&s, 0, sizeof s);
    run(&p, &tm, &s, 50000, -1, 500);
    CHECK(s.stall_kicks == 0 && !s.fault, "unknown rpm ignored");
}

/* Random walk: invariants must hold at every step. */
static void test_random_walk(void)
{
    struct fd_profile p = test_profile(1);
    struct fd_timing tm;
    struct fd_state s;
    fd_timing_init(&p, 1000, &tm);
    memset(&s, 0, sizeof s);
    int t = 40000, bad = 0;
    for (int i = 0; i < 2000000; i++) {
        t += (int)(rnd() % 2001) - 1000;
        if (t < -50000) t = -50000;
        if (t > 160000) t = 160000;
        int temp = rnd() % 97 == 0 ? FD_TEMP_INVALID : t;
        int rpm = rnd() % 13 == 0 ? (int)(rnd() % 2) - 1 : 1500;
        int out = fd_step(&p, &tm, &s, temp, rpm);
        if (out < 0 || out > 255) bad++;
        if (out > 0 && out < p.floor_pwm) bad++;
        if (temp != FD_TEMP_INVALID && temp >= p.crit_mC && out != 255) bad++;
        if (temp != FD_TEMP_INVALID && out < fd_curve(&p, temp)) bad++;   /* never below the curve */
    }
    CHECK(bad == 0, "random walk: %d invariant violations", bad);
}

static int emit(const char *file, int tach)
{
    char buf[FD_PROFILE_MAX];
    struct fd_profile p = test_profile(tach);
    int n = fmt(&p, buf, sizeof buf);
    FILE *f = fopen(file, "w");
    if (!f || n < 0) return 1;
    fwrite(buf, 1, (size_t)n, f);
    return fclose(f) != 0;
}

int main(int argc, char **argv)
{
    if (argc == 3 && !strcmp(argv[1], "--emit")) return emit(argv[2], 0);
    if (argc == 3 && !strcmp(argv[1], "--emit-tach")) return emit(argv[2], 1);
    test_parser();
    test_control();
    test_stall();
    test_random_walk();
    test_fuzz();
    printf("%d checks, %d failures\n", checks, fails);
    return fails != 0;
}
