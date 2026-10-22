#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Integration tests: run the real fandynd against a fake sysfs tree and
# inject faults. Uses a fast tick (20 ms) so minutes of behaviour take seconds.
set -u
cd "$(dirname "$0")/.."
D=$(mktemp -d)
trap 'kill $PID 2>/dev/null; rm -rf "$D"' EXIT
PASS=0; FAIL=0
ok()   { PASS=$((PASS+1)); echo "  ok   $1"; }
bad()  { FAIL=$((FAIL+1)); echo "  FAIL $1"; }
check() { if eval "$2"; then ok "$1"; else bad "$1 ($3)"; fi; }

R=$D/root; FAN=$R/sys/devices/pwm-fan; RUN=$D/run
cp build/alloc_count.so $D/alloc_count.so   # LD_PRELOAD paths cannot contain spaces
mkfake() {
    mkdir -p $FAN $RUN
    for z in 0 1; do mkdir -p $R/sys/class/thermal/thermal_zone$z; done
    echo CPU-therm > $R/sys/class/thermal/thermal_zone0/type
    echo GPU-therm > $R/sys/class/thermal/thermal_zone1/type
    mkdir -p $R/sys/class/thermal/thermal_zone2; echo AO-therm > $R/sys/class/thermal/thermal_zone2/type
    echo 99000 > $R/sys/class/thermal/thermal_zone2/temp   # must be ignored
    temp 30000 30000
    echo 0 > $FAN/target_pwm; echo 1 > $FAN/temp_control; echo 0 > $FAN/tach_enable; echo 1500 > $FAN/rpm_measured
}
temp() { echo $1 > $R/sys/class/thermal/thermal_zone0/temp; echo $2 > $R/sys/class/thermal/thermal_zone1/temp; }
pwm()  { cat $FAN/target_pwm 2>/dev/null; }
wait_for() {  # wait_for "<condition>" seconds
    for i in $(seq $(( $2 * 20 ))); do eval "$1" && return 0; sleep 0.05; done; return 1
}
start() {
    LD_PRELOAD=$D/alloc_count.so ./fandynd --root $R --run-dir $RUN --profile $1 --tick-ms ${2:-20} 2> $D/log &
    PID=$!
}
stop() { kill -TERM $PID; wait $PID; RC=$?; }
# open fds: minimum of 20 samples. One sample can catch a file that is open
# mid-tick (the daemon opens/closes a few per tick); a leak raises the minimum.
fds() { local m=999 n; for i in $(seq 20); do n=$(ls /proc/$PID/fd 2>/dev/null | wc -l); [ "$n" -lt "$m" ] && m=$n; sleep 0.01; done; echo $m; }

./build/test_core --emit $D/good.profile || exit 1
./build/test_core --emit-tach $D/tach.profile || exit 1

echo "normal operation"
mkfake; start $D/good.profile
check "takes control from kernel"      'wait_for "[ \$(cat $FAN/temp_control) = 0 ]" 2' "temp_control=$(cat $FAN/temp_control)"
check "cold board: fan off"           'wait_for "[ \"\$(pwm)\" = 0 ]" 2' "pwm=$(pwm)"
check "ignores other zones (AO 99C)"  '[ "$(pwm)" = 0 ]' "pwm=$(pwm)"
temp 60000 50000
check "60 C: kick then curve(60)=105" 'wait_for "[ \"\$(pwm)\" = 105 ]" 3' "pwm=$(pwm)"
check "status file"                   'grep -q "state=on" $RUN/status' "$(cat $RUN/status 2>/dev/null | tr '\n' ' ')"
temp 30000 70000
check "hottest zone wins (GPU 70C)"   'wait_for "[ \"\$(pwm)\" = 255 ]" 3' "pwm=$(pwm)"

echo "sensor faults"
temp 50000 50000; sleep 0.5
rm $R/sys/class/thermal/thermal_zone0/temp
check "one sensor gone: uses other"   'wait_for "[ \"\$(pwm)\" -lt 255 ]" 8' "pwm=$(pwm)"
echo garbage > $R/sys/class/thermal/thermal_zone1/temp
check "no valid sensor: 255"          'wait_for "[ \"\$(pwm)\" = 255 ]" 2' "pwm=$(pwm)"
for v in "" "-" "12abc" "999999999999999999999" "-99999999" "  " "1e5"; do printf '%s' "$v" > $R/sys/class/thermal/thermal_zone1/temp; sleep 0.1; done
check "garbage values: still 255"     '[ "$(pwm)" = 255 ]' "pwm=$(pwm)"
temp 50000 50000
check "sensors back: slows down"      'wait_for "[ \"\$(pwm)\" -lt 100 ]" 8' "pwm=$(pwm)"

echo "kernel and device faults"
echo 1 > $FAN/temp_control
check "kernel control re-enabled: retaken" 'wait_for "[ \$(cat $FAN/temp_control) = 0 ]" 2' "temp_control=$(cat $FAN/temp_control)"
mv $FAN $FAN.gone
sleep 1
check "fan device gone: still running" 'kill -0 $PID' "daemon died"
mv $FAN.gone $FAN; echo 7 > $FAN/target_pwm
check "fan device back: resumes"      'wait_for "[ \"\$(pwm)\" != 7 ]" 3' "pwm=$(pwm)"
temp 95000 95000
check "critical 95 C: 255"            'wait_for "[ \"\$(pwm)\" = 255 ]" 1' "pwm=$(pwm)"
temp 30000 30000

echo "profile reload"
cp $D/good.profile $D/live.profile
sed 's/floor 20/floor 25/' $D/good.profile > $D/corrupt.profile   # CRC no longer matches
kill -TERM $PID; wait $PID
start $D/live.profile
wait_for "grep -q 'loaded (calibrated' $D/log" 3 || bad "daemon did not start"   # no swap before startup load
cp $D/corrupt.profile $D/live.profile; kill -HUP $PID; sleep 0.3
check "corrupt profile on reload: kept old" 'grep -q "rejected (crc mismatch" $D/log && kill -0 $PID' "$(tail -2 $D/log)"
cp $D/good.profile $D/live.profile; kill -HUP $PID; sleep 0.3
check "valid profile on reload: loaded" '[ $(grep -c "loaded (calibrated" $D/log) = 2 ]' "$(tail -2 $D/log)"
stop
check "SIGTERM: exit 0"               '[ $RC = 0 ]' "rc=$RC"
check "SIGTERM: kernel control back"  '[ $(cat $FAN/temp_control) = 1 ]' "temp_control=$(cat $FAN/temp_control)"
check "SIGTERM: status removed"       '[ ! -e $RUN/status ]' "status still there"

echo "bad or missing profile"
start $D/nonexistent.profile
check "missing profile: built-in"     'wait_for "grep -q \"using built-in safe profile\" $D/log" 2' "$(cat $D/log)"
temp 50000 50000
check "built-in profile drives fan"   'wait_for "[ \"\$(pwm)\" -ge 40 ]" 3' "pwm=$(pwm)"
stop
head -c 3000 /dev/urandom > $D/random.profile
start $D/random.profile
check "random bytes profile: built-in" 'wait_for "grep -q \"using built-in\" $D/log" 2' "$(cat $D/log)"
stop

echo "stall supervision"
mkfake; echo 0 > $FAN/rpm_measured; temp 50000 50000
start $D/tach.profile
# 3 kicks x 2 s + stall checks: fault after ~6.3 s
check "dead fan: re-kicks then fault mode" 'wait_for "grep -qs \"state=fault\" $RUN/status" 12' "$(cat $RUN/status 2>/dev/null | tr '\n' ' ')"
check "fault: full speed"             '[ "$(pwm)" = 255 ]' "pwm=$(pwm)"
check "fault is logged"               'grep -q "mode fault" $D/log' "$(cat $D/log)"
echo 3000 > $FAN/rpm_measured; sleep 1
check "no flapping: fault holds until 120 s of good RPM" 'grep -q "state=fault" $RUN/status' "$(cat $RUN/status | tr '\n' ' ')"
check "tach enabled"                  '[ $(cat $FAN/tach_enable) = 1 ]' "tach_enable=$(cat $FAN/tach_enable)"
stop

echo "resources (heap, fds, memory)"
mkfake; temp 55000 50000
start $D/tach.profile 20; sleep 1; stop
SHORT=$(grep ALLOC_COUNT $D/log)
start $D/tach.profile 1
sleep 1; FD1=$(fds); RSS1=$(awk '/VmRSS/{print $2}' /proc/$PID/status)
for i in $(seq 20); do temp $((40000 + (i % 7) * 6000)) 45000; sleep 1; done
FD2=$(fds); RSS2=$(awk '/VmRSS/{print $2}' /proc/$PID/status)
stop
LONG=$(grep ALLOC_COUNT $D/log)
check "heap counter active"           '[ -n "$SHORT" ] && [ -n "$LONG" ]' "no ALLOC_COUNT output"
check "no heap use per tick (50 ticks: ${SHORT#ALLOC_COUNT } | ~20000 ticks: ${LONG#ALLOC_COUNT })" '[ "$SHORT" = "$LONG" ]' "differs"
check "no fd leak (baseline $FD1 -> $FD2)" '[ $FD2 -le $FD1 ]' "fds grew"
# a leak of even 1 byte per tick would add ~20 kB over this run; allow 2 pages of noise
check "no memory growth ($RSS1 -> $RSS2 kB)" '[ $RSS2 -le $((RSS1 + 8)) ]' "rss grew"

echo
echo "$PASS passed, $FAIL failed"
[ $FAIL = 0 ]
