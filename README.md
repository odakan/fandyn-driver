# fandyn driver

An adaptive, low-noise fan control daemon for the NVIDIA Jetson Nano: silent when the board is idle, full power under load.

The whole daemon is one small C file (`src/fandynd.c`). It uses about 0.15% of one CPU core and under 1 MB of RAM.

## How it works

Once per second, `fandynd`:

1. **Reads sensors.** It takes the hottest of `CPU-therm` and `GPU-therm`, plus CPU load (`/proc/stat`), GPU load (`/sys/devices/gpu.0/load`) and RAM pressure (`/proc/meminfo`).
2. **Predicts.** It smooths the temperature, measures how fast it's rising, and builds a *predicted* temperature:
   `predicted = temp + rising_trend × lookahead + load_gain × load`
   Load counts as "heat on the way", so the fan starts ramping before the temperature climbs.
3. **Maps to a fan speed.** It applies a curve from `t_start` to `t_full`, with an exponent (`curve`) that keeps the low end quiet.
4. **Adapts.** An integral *trim* slowly adds speed while the board stays above `t_limit`, then bleeds away once the board cools. This adjusts automatically for a hot room, dust, or a weaker fan.
5. **Shapes the output for low noise:**
   - fast ramp up (`slew_up`) and very slow ramp down (`slew_down`), so speed changes are hard to hear
   - a deadband, so the fan doesn't keep shifting speed by small amounts
   - zero-RPM mode: the fan stops when cool and restarts with hysteresis, after at least 30 s of running
   - a spin-up kick (`kick_pwm`) when starting from a stop
   - stall detection: if the tach reads 0 RPM while the fan should be spinning, the daemon kicks it again
6. **Fails safe.** At `t_crit`, or if the sensors can't be read, the fan goes to full speed immediately. On exit or crash, fan control goes back to the kernel (`temp_control=1`).

## Profiles

| profile | fan stops when cool | starts ramping | full speed | ramp down |
|---|---|---|---|---|
| `quiet` | yes | 50 °C | 82 °C | very slow |
| `balanced` (default) | yes | 45 °C | 75 °C | slow |
| `performance` | no, min PWM 30 | 38 °C | 65 °C | moderate |

Every setting can be overridden in `/etc/fandyn/fandyn.conf`. See the comments in that file.

## Install

```sh
make deb
sudo apt install ./build/fandyn_0.1.0_arm64.deb
```

The package installs and starts the `fandyn` systemd service. Remove it with `sudo apt remove fandyn`; the kernel takes back control of the fan.

Without a package: `make && sudo make install && sudo systemctl enable --now fandyn`.

## Use

```sh
fandynd status                       # current temperature, prediction, load, PWM, RPM
sudo nano /etc/fandyn/fandyn.conf    # change the profile or tune settings
sudo systemctl reload fandyn         # apply config without restarting
journalctl -u fandyn                 # logs
```

To tune interactively, stop the service and run the daemon in the foreground with a log line per second:

```sh
sudo systemctl stop fandyn
sudo ./fandynd -v -c fandyn.conf -s /tmp/fandyn.status
```

## Calibration

`tools/calibrate.sh` sweeps the PWM range and reads the tach, to find your fan's stall point and start-up kick. Results for the stock fan on the development board:

| PWM | 255 | 100 | 40 | 20 | 16 | ≤12 |
|---|---|---|---|---|---|---|
| RPM | 8529 | ~3400 | 1192 | 412 | 239 | 0 (stall) |

If your fan is different, set `min_pwm` and `kick_pwm` to match.

`contrib/99-fandyn.rules` is a udev rule for development only. It lets a normal user write the fan files so you can run tests without sudo. The installed service runs as root and doesn't need it.

## Hardware notes

- Tested on Jetson Nano Developer Kit, L4T R32.7.6 (kernel 4.9), 4-pin fan on header J15.
- Uses NVIDIA's `pwm-fan` sysfs interface (`/sys/devices/pwm-fan/target_pwm`). The kernel driver still handles the hardware PWM itself.
