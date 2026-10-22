# fandyn driver

Source: https://github.com/odakan/fandyn-driver

Calibrated, low-noise fan control for the NVIDIA Jetson Nano. It's silent when the board is cool and runs at full speed under heavy load. Once installed, you can forget about it.

fandyn is split into two programs with opposite goals:

| | `fandyn-calibrate` | `fandynd` |
|---|---|---|
| runs | once, at install | always, from boot |
| job | learn the board, design and test a fixed profile | follow that profile |
| style | heavy: physics, statistics, simulation | minimal: integer math, no heap, no threads |
| noise | loud for ~16 min (full fan sweep, all CPU cores busy) | as quiet as the profile allows |

## Install

```sh
make deb                                  # runs every test first; refuses to build if one fails
sudo apt install ./build/fandyn_1.0.0_arm64.deb
```

The install calibrates the board, which takes about 16 minutes with the fan loud. It then enables and starts the service. There is nothing to configure.

- Remove: `sudo apt remove fandyn`. The kernel takes the fan back.
- Remove and delete the calibration data: `sudo apt purge fandyn`.
- Skip calibration at install: `sudo FANDYN_NO_CALIBRATE=1 apt install ./...deb`. It then runs 3 minutes after the next boot. Until a profile exists, a safe built-in profile is used.

## Calibration (`fandyn-calibrate`)

1. **Fan.** Sweeps PWM from 255 down to stall and records RPM at each step. Each step takes a median of 9 tach readings, and readings faster than the next-higher PWM step are rejected and re-measured. This tach produces 1–2% glitches of up to 2× the real speed. Monotonicity is repaired by capping from above, so an outlier can only make the fan look weaker, which leads to a louder, safer profile. Finds the lowest running speed and adds a 25% margin for the floor, which must hold for 10 s. Finds the lowest PWM that starts the fan from standstill (this varies from run to run with rotor position). The kick (start-up burst) is set to at least 2× that value and never below 40, then verified with 5 of 5 starts from standstill, using exactly the timing the runtime uses. Measures the kernel driver's own ramp rate.
2. **Thermal.** Runs 10 minutes of load on/off at four fan levels, from off to full, while logging temperature, heat input, RPM and PWM every second. The heat source is 8 independent NEON multiply-add chains per core, which maximizes heat for a clean signal. Airflow for the model comes from the drive level through the validated fan curve, not from the raw tach.
3. **Model.** Fits a lumped thermal network to the log:

   ```
   C·dTs/dt = P − G(u)·(Ts − Ta)        heatsink node
   Td       = Ts + Rjc·P                die sensor
   G(u)     = G0 + G1·u^n               natural + forced convection, u = RPM / max RPM
   ```

   - P is the board input power from the INA3221 monitor. If that sensor is unavailable, P is CPU load. Heat is aligned with the interval it was measured over.
   - The fit simulates the full time series with the exact zero-order-hold solution and minimizes output error. It uses multi-start Nelder–Mead with weak physical priors (a MAP estimate), then refits once with the estimated noise variance (empirical Bayes), so the estimate and its covariance come from the same objective.
   - It reports residual σ, R², max error, the Durbin–Watson statistic, and parameter covariance from the Jacobian, inflated for autocorrelated residuals. Confidence intervals for thermal resistance and time constant come from the delta method.
   - The model is accepted only if the fit is good, the parameters are physically plausible, the fan's cooling effect is statistically significant, and the key uncertainties are small. Otherwise a safe fallback curve is used.
4. **Design.** Decides where the fan turns on: the upper 95% bound of the predicted fan-off idle temperature plus 1 °C, but never below 50 °C. That keeps the fan off during light everyday use, not just at true idle. From there up to `--max-temp` (default 70 °C), each temperature gets the *slowest* fan speed whose cooling capacity holds that temperature. Capacity grows linearly up to full speed, and a pessimistic (upper 95%) thermal resistance is used. Hysteresis comes from the measured noise, slow-down rate from the time constant, and minimum on-time from the floor-speed time constant.
5. **Test on the board.** Runs the real control code (`fd_step`, the same as the runtime) in closed loop under load, then idle. Also simulates the same controller against the fitted model and compares it to the board. An actuator sweep then feeds the controller a temperature ramp across the whole curve, checking that the real fan and tach follow it without stalls. If a test fails, the calibrator redesigns with safer settings and tests again, up to 3 attempts.
6. **Simulated tests**, one hour each, using the real control code and the fitted model: design power (≥10 W), the same in a room 10 °C hotter, idle, and bursty load (30 s on, 30 s off).

Output in `/var/lib/fandyn/`:

| file | contents |
|---|---|
| `profile` | the fixed profile. Plain text, CRC-32 protected, written atomically. The runtime parser checks it before saving. |
| `calibration.csv` | every raw sample: the measurement database |
| `report.txt` | fan table, model parameters with confidence intervals, profile, all test results |
| `history.csv` | one line per calibration. Rising thermal resistance over time means dust. |
| `recalibrate` | present only while a recalibration request is pending |

Recalibrate on request, for example after changing the fan, cleaning dust, or moving the board somewhere hotter. The service is stopped during the run and restarted automatically:

```sh
sudo fandyn-calibrate --force         # now, with live progress (add --max-temp 65 for a cooler, louder profile)
sudo fandyn-calibrate --background    # now, as a service; follow with: journalctl -fu fandyn-calibrate
sudo fandyn-calibrate --next-boot     # unattended, 3 minutes after the next boot
```

Without `--force`, the calibrator refuses to replace an existing profile, so a stray command can't start 16 loud minutes. `--background` and `--next-boot` leave a request file, `/var/lib/fandyn/recalibrate`, that the calibration service picks up. It's deleted once a new profile is written, and you can delete it yourself to cancel.

## Runtime (`fandynd`)

Once per second: hottest of CPU/GPU temperature → `fd_step()` → `target_pwm`. The control law:

- **Speeding up is immediate.** The kernel driver ramps the fan smoothly by itself.
- **Slowing down** needs the board to cool by the hysteresis band first, then happens at a limited rate.
- **Zero-RPM.** The fan stops below the calibrated threshold, but only after running at least the minimum on-time. Starting from a stop always uses the verified kick.

Safety behavior:

| situation | behavior |
|---|---|
| temperature ≥ critical (calibrated, ≤ 90 °C) | full speed immediately |
| no readable sensor for 3 s | full speed until sensors return |
| one sensor missing or garbage | uses the others |
| fan reads 0 RPM while it should spin | re-kick; after 3 failed kicks, full speed (fault mode) |
| fan recovers (120 s of good RPM) | leaves fault mode and raises its own floor by 8 |
| kernel fan control re-enabled (driver reset, resume, nvpmodel) | takes control back within 10 s |
| fan device disappears | keeps running and retries quietly; resumes when it's back |
| profile missing, corrupt (CRC) or out of range | built-in safe profile |
| bad profile on `systemctl reload fandyn` | keeps the current one |
| daemon hangs | systemd watchdog (15 s) restarts it |
| daemon stops or crashes | systemd hands the fan back to the kernel, then restarts the daemon |

Engineering rules for the runtime:

- no heap at all: 0 allocations, verified with an allocation counter
- no floating point, no threads, no recursion
- every loop has a fixed bound, every system call is checked, all file input is range-checked
- memory locked (`mlockall`), protected from the OOM killer
- logs only abnormal events, rate-limited to one per kind per 10 min
- 29 KB binary, about 450 KB resident, fewer than 20 system calls per second

```sh
fandynd status       # live state: temperature, pwm, rpm, mode
fandynd --check      # profile, sensors, fan, and the curve
journalctl -u fandyn
```

## Tests

```sh
make test
```

- `tests/test_core.c` covers the control law (kick, hysteresis, slew, min-on, zero-RPM, critical, sensor loss, stall, fault, recovery). It also runs a 2-million-step random walk checking invariants, rejection tests for every kind of profile corruption, and a fuzzer that sends 200,000 mutated profiles through the parser.
- `fandyn-calibrate --refit calibration.csv` refits the model from a saved database without touching the fan. It was used to check the fitting fixes against stored runs.
- `tests/test_runtime.sh` runs the real daemon against a fake sysfs tree and injects faults: missing and garbage sensors, the fan device disappearing, the kernel taking control back, critical temperature, corrupt and valid profile reloads, a missing profile, random-bytes profiles, a dead fan, and SIGTERM hand-back. It also proves there's no heap use per tick and no fd or memory growth over about 20,000 ticks.

## Layout

```
src/fandyn.h            shared core: profile format, control law, sysfs io
src/fandynd.c           runtime daemon
src/fandyn-calibrate.c  calibration
systemd/                fandyn.service, fandyn-calibrate.service/.timer
debian/                 package metadata and maintainer scripts
tests/                  unit, fuzz and fault-injection tests
contrib/99-fandyn.rules udev rule for development only (fan control without sudo)
```

Tested on Jetson Nano Developer Kit, L4T R32.7.6 (kernel 4.9), 4-pin fan on J15.

## License

Apache License 2.0. See [LICENSE](LICENSE).
