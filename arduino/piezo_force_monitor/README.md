# Clamp-force & alignment monitor — Arduino UNO Q + 3 × PI PL033.31

Firmware for the design in [`docs/heat-sealer-force-monitor.md`](docs/heat-sealer-force-monitor.md)
(schematic: [`docs/heat-sealer-charge-amp.svg`](docs/heat-sealer-charge-amp.svg)): three PL033.31
piezo chips between the jaws, each feeding a charge amplifier (MCP6004, Cf 6.8 µF, Rf 10 MΩ) into
A0 / A1 / A2.

For every squeeze it reports:

- **Hit order**: first, second, third chip to reach the onset force, with interpolated timing,
  per-channel timing offsets, and a **± uncertainty**. If the skew is inside the uncertainty it
  says "effectively simultaneous".
- **Clamp force** = peak of F1 + F2 + F3, computed sample by sample from the charge-amplifier
  equation (not by integrating raw piezo voltage).
- **Force share** of each chip, in %, at the moment of peak clamp force.
- A **self-check** (force returns to zero after release) and warnings (overstress, clipping,
  sampling gaps).

On start-up the serial monitor walks you through **calibrating each piezo on its own**. You can
also **skip** calibration and use standard parameters, or paste back a saved calibration.

---

## 1. Hardware (from the design spec)

| Channel | Op amp (MCP6004) | ADC pin | Sensor |
|---|---|---|---|
| CH1 | U1A, out pin 1 | **A0** | PL033.31 #1 |
| CH2 | U1B, out pin 7 | **A1** | PL033.31 #2 |
| CH3 | U1C, out pin 8 | **A2** | PL033.31 #3 |
| Reset button (optional) | — | **D2** → GND | momentary, internal pull-up |

- Resting output ≈ **VREF ≈ 0.30 V**. The output **rises on squeeze**. If one falls, swap that
  sensor's red and black wires. The firmware warns if a channel goes strongly negative.
- Keep each chip's two wires shorted whenever it is disconnected. Turn the heater off while
  measuring (piezo ceramic is pyroelectric). See spec sections 3 and 9.

## 2. Install and run

1. **Arduino App Lab** (recommended for the UNO Q): create an app and paste
   `piezo_force_monitor.ino` as its sketch. The Bridge library is already available. Run it, then
   open the Serial Monitor.
   **Arduino IDE 2**: install the *Arduino UNO Q* board package and the **Arduino_RouterBridge**
   library, upload, then open the Serial Monitor.
2. Any line-ending setting works ("Newline" is best). The baud rate setting is irrelevant.

> **UNO Q specifics.** The serial monitor is reached through the Bridge `Monitor` object. A plain
> `Serial` goes to the D0/D1 UART pins instead. Every `Monitor` call is an RPC to the board's Linux
> side, so this sketch samples in its own high-priority Zephyr thread at 2 kHz and sends text in
> queued chunks. Serial traffic never delays or drops a sample.

## 3. The physics used (spec sections 3, 4.3, 7)

| Step | Equation | Meaning |
|---|---|---|
| Piezo | **Q = S·F** | Charge proportional to force. S ≈ 13–22 nC/N for the PL033.31 (estimate). |
| Charge amp | **ΔV = Q / Cf** | ΔV = output − pre-closure baseline. Sensor and cable capacitance drop out. |
| Leak through Rf | **Cf·dV/dt = dQ/dt − (V − V_eq)/Rf** | τ = Rf·Cf = 68 s. V_eq is the resting output level. |
| Force per chip | **F = k·(ΔV + (1/τ)·∫(V − V_eq) dt)**, **k = Cf/S** | Restores the charge that leaked away (~2.2 % after a 1.5 s hold). |
| Clamp force | **F_total[n] = F1[n] + F2[n] + F3[n]**, clamp = max F_total | Summed sample by sample. Never add the three individual peaks. |
| Share | **share_i = F_i / F_total** at the clamp-force peak | ≈ 33 % each means parallel jaws under load. |
| Onset | first crossing of F_i over the threshold (interpolated) − offset_i | Earliest crossing = first contact. Last − first = closure skew. |

When the output has settled, the pre-closure baseline equals V_eq. The force equation is then
exactly the spec's discrete form **F_i[n] = k_i·(ΔV_i[n] + (Ts/τ_i)·S_i[n])**, S_i[n] = S_i[n−1] + ΔV_i[n].
Integrating against V_eq instead of the baseline keeps the result exact right after earlier
squeezes as well, while their undershoot is still fading (3–4 min, spec section 5). V_eq is taken
at the power-up self-test and refined whenever the channel sits idle. Type `eq` to print the model
with the current values.

**Standard parameters** (used when calibration is skipped):

| Parameter | Value | Origin |
|---|---|---|
| k (every chip) | **400 N/V** | Cf / S = 6.8 µF / 17 nC/N. 17 nC/N is the geometric middle of the 13–22 nC/N estimate. |
| q (curvature) | 0 | linear |
| τ (every channel) | **68 s** | Rf·Cf = 10 MΩ × 6.8 µF |
| timing offsets | 0 | — |

Uncalibrated, absolute force is good to about −25…+30 %. The shares and the timing are skewed by
however much the chips differ, and the report widens its timing uncertainty to match.

## 4. Start-up

```
Self-test (jaws open):
  CH1/A0  idle 0.302 V   noise 0.30 mV rms = 0.12 N   OK
  ...
CALIBRATION SETUP
  c = calibrate now (guided step by step, one piezo at a time)
  s = skip, use STANDARD parameters ...
  p = paste a saved CAL line from an earlier calibration
  (the button on D2 also means s; no answer within 60 s = s)
```

After calibration, skip or paste, the monitor shows `RESET done.` and then
`Waiting for actuation (squeeze)...`.

## 5. Calibration walk-through (spec section 8)

Type `c` at start-up (or `cal` later), then `1`, `2` or `3` for one piezo. Mount each chip exactly
as in the sealer (same shims, same orientation) and keep every chip with its channel.

| Step | What you do | What the firmware does |
|---|---|---|
| **1. Conditioning** (new chips) | Squeeze the chip 30–50 times to working force. `n` = next. | Counts the presses and shows each peak. |
| **2. τ** (no scale) | Hold a steady 100–270 N for ~10 s, release quickly, keep still 2 s. Two runs. | After release the true force is 0, so **τ = −∫(V−V_eq)dt / ΔV**. Shows each run, the mean, and the post-release residual (pass ≤ 0.5 %). `a` accepts. |
| **3. k** (scale + arbor press) | ~50, 100, 150, 200, 270 N, 3 times each. Ramp ~0.5 s, hold until the scale settles, release, **type the reading** (`101`, `10.3kg`, `22.7lb`). | Takes the τ-corrected plateau at the end of the hold, then fits force against corrected ΔV: **slope = k**. Checks the intercept is ≈ 0, every point is within 2 % of the line, and repeats agree within 1–2 %. If the points curve, it offers a **quadratic** fit (`aq`). |
| **4. Mounting checks** | `v` → `1/2/3`: rotate the chip 90° and repeat one level, then try a fast and a slow ramp. | Compares the reading with the scale. More than 2 % apart means uneven loading, or τ is off. |
| **5. System check** | `v` → `a`: all three chips under one plate on the scale. Centre press, then off-centre. | The total should be within ~2 % of the scale, and stay the same while the shares shift. |
| **6. Timing offsets** | `t`: centre-press the plate ~10 times. | Records each channel's crossing relative to the mean of the three, then averages. Warns above 0.5 ms (electronics alone give ~0.1 ms). |
| **7. Record** | `note 2026-10-28 22C`, then `show`. | Prints the record table and a **`CAL1 …` line**. |

> **Save the CAL line.** Sketches on the UNO Q have no EEPROM, so calibration is lost at
> power-off. Copy the `CAL1 …` line somewhere safe. After power-up, paste it into the monitor (or
> type `p` first). Recheck before each test session with `v` (one known load; more than 3 % off
> means recalibrate), after any overload, after re-mounting, or after a large temperature change.

## 6. Reading a squeeze report

Taken from the simulation in section 8 (CH2 side closing 0.88 ms before CH1 and 2.12 ms before CH3):

```
==================== SQUEEZE #1 ====================
Hit order (each chip's force crossing 5.0 N, timing offsets applied):
  1st  CH2/A1   0.000 ms
  2nd  CH1/A0  +0.866 ms
  3rd  CH3/A2  +2.124 ms
  Closure skew (first to last): 2.124 ms  +/- 0.033 ms
  -> first contact at CH2/A1: that side closes first (sits low)
Clamp force (peak of F1+F2+F3): 869.4 N  (195.4 lbf), reached 80.6 ms after first contact
Force share at that instant:
  CH1/A0  32.2 %   (279.6 N)
  CH2/A1  37.9 %   (329.5 N)
  CH3/A2  29.9 %   (260.3 N)
  -> unbalanced: shares up to 4.6 points from 33.3 %; CH2/A1 carries the most
Per-chip peaks (information only, never add them): CH1 279.7, CH2 329.7, CH3 260.6 N
Squeeze duration: 1.26 s
Self-check: after release F_total = +0.00 % of peak  PASS (<= 0.5 %)
Note: above 270 N (30 MPa constant-force limit of the PL033, spec 3): CH1/A0 280 N CH2/A1 330 N
WARNING: CH2/A1 overloaded (330 N): recheck its calibration (spec 8)
CSV,1,CH2/A1,2.124,0.033,0.866,0.000,2.124,869.4,32.16,37.90,29.94,0.00,1.258
```

- **± uncertainty**: for each channel, timing noise = force noise ÷ ramp slope at the crossing
  (both measured on that squeeze). On top of that comes the leftover channel-delay mismatch:
  0.02 ms with calibrated offsets, 0.1 ms without. Uncalibrated chips add the ±30 % sensitivity
  spread. A skew of less than 2 × the uncertainty is reported as "effectively simultaneous".
- **CSV line**: one per squeeze, for logging or a spreadsheet:
  `shot, first, skew_ms, skew_unc_ms, on1_ms, on2_ms, on3_ms, clamp_N, share1, share2, share3, resid_pct, dur_s`.
- **Self-check**: a negative residual means τ is set too large; a positive one means too small
  (spec section 7).
- **270 N note**: at the 200 lbf design load each chip sees ~297 N, above PI's 30 MPa
  constant-force limit (spec section 3, where PL055.31 is the in-spec alternative). The firmware
  notes it, and warns to recheck a chip only when it was overloaded (> 324 N).

## 7. Commands

| Command | Action |
|---|---|
| `r` / button D2 | Reset and re-arm ("RESET done", then "Waiting for actuation") |
| `cal` | Calibration menu |
| `show` | Calibration record and CAL line |
| `CAL1 …` / `p` | Load a saved calibration |
| `std` | Back to the standard parameters |
| `thr <N>` | Onset threshold per chip (default 5 N; raise it if the channels are noisy) |
| `jaw <L> <W>` | Seal length × width in mm, adds pressure to the report (`jaw 0 0` = off) |
| `auto on/off` | Re-arm automatically after each squeeze |
| `mon` | Live forces 5× per second (Enter stops) |
| `eq` / `stat` / `h` | Equations, sampler statistics, help |

If you use the optional reset switch across Cf, type `r` afterwards. Its step in the output would
otherwise look like a squeeze.

## 8. How it was verified

I could not build or flash for the real board in this environment. **The first compile on your
UNO Q is the first real-toolchain build**, so please report any compile message. What was checked:

- **Against the real UNO Q sources**: the Zephyr core (`ArduinoCore-zephyr`), the board files
  (14-bit ADC, A0–A2 = PA4/PA5/PA6), the Bridge `Monitor` class, and the loader's export table.
  Every kernel, libc and libm call the sketch makes is exported to sketches. Thread creation copies
  the Bridge library's own pattern. Identifiers that could clash with Zephyr or STM32 macros were
  renamed.
- **Host simulation** (`extras/sim/`): the unchanged sketch runs against a model of this circuit.
  The model has three chips of different sensitivity (one deliberately nonlinear), Rin·Cp lag,
  charge amplifiers with different τ, the Rout/Cflt filter, a 14-bit ADC with noise and injected
  read errors, and a sampler thread pre-empting `loop()` while every `Monitor` call costs RPC time
  at 115200 baud. A scripted operator answers the prompts. Results:

| Quantity | True (model) | Firmware |
|---|---|---|
| τ CH1 / CH2 / CH3 | 71.50 / 64.00 / 68.00 s | 71.56 / 63.90 / 68.04 s |
| Calibrated force at 100 N / 250 N, all chips | 100 / 250 N | within 0.25 % |
| CH2 (nonlinear chip) | +5 % sensitivity at 250 N | quadratic fit offered and used, 0.45 % worst |
| Misaligned squeeze: skew | 2.122 ms | 2.124 ± 0.033 ms, order correct |
| Clamp force | 870.0 N | 869.4 N |
| Shares | 32.2 / 37.9 / 29.9 % | 32.2 / 37.9 / 29.9 % |
| Aligned squeeze (true spread 7 µs) | — | 0.019 ± 0.033 ms → "effectively simultaneous" |
| Self-check residual | 0 % | 0.00 % |
| Skip / paste / "no line ending" / no-thread fallback | — | all run correctly |

Run it yourself with `extras/sim/run_sim.sh` (needs g++).

## 9. Main settings (top of the sketch)

| Constant | Default | Meaning |
|---|---|---|
| `FS_HZ` | 2000 | Sample rate per channel (spec section 7) |
| `CF_F`, `RF_OHM` | 6.8 µF, 10 MΩ | Charge-amp parts (set both standard k and τ) |
| `S_STD_CN` | 17 nC/N | Standard sensitivity behind k = Cf/S |
| `THR_DEFAULT_N` | 5 N | Onset threshold (`thr` changes it at run time) |
| `BASE_WIN_US` | 250 ms | Pre-closure baseline window (spec: ≥ 0.2 s) |
| `POST_US` | 300 ms | Recording after release (spec: ≥ 0.2 s) |
| `CAP_TIMEOUT_US` | 10 s | Longest squeeze captured |
| `CHIP_MAX_N` | 270 N | 30 MPa constant-force limit (PL033) |
| `BTN_PIN` | 2 | Reset button pin (−1 = none) |
