#!/bin/bash
# Fan calibration: sweeps PWM down from full speed and reads the tach (RPM sensor),
# so we can find the lowest PWM where the fan still spins.
# Run with: bash calibrate.sh (needs write access to the fan files)
F=/sys/devices/pwm-fan
[ -w $F/target_pwm ] || { echo "no write access to $F (run with sudo or install contrib/99-fandyn.rules)"; exit 1; }

restore() { echo 1 > $F/temp_control; echo "restored kernel fan control"; }
trap restore EXIT

# the driver ramps cur_pwm toward target_pwm gradually; wait for it, then let RPM settle
settle() { for i in $(seq 100); do [ "$(cat $F/cur_pwm)" = "$1" ] && break; sleep 0.2; done; sleep 2; }

echo 0 > $F/temp_control     # take control from the kernel
echo 1 > $F/tach_enable 2>/dev/null

echo "full speed..."
echo 255 > $F/target_pwm; settle 255
echo "PWM  RPM"
for p in 255 200 160 130 110 90 75 60 50 40 30 20; do
    echo $p > $F/target_pwm
    settle $p
    echo "$p  $(cat $F/rpm_measured)"
done

echo "start-up test from stopped (does it start without a kick?)"
for p in 40 60 80 100; do
    echo 0 > $F/target_pwm; settle 0; sleep 3
    echo $p > $F/target_pwm; settle $p
    echo "start at $p -> $(cat $F/rpm_measured) RPM"
done
