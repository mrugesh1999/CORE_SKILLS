# Piezo weld aligner — Arduino UNO Q (STM32U585)

> **Superseded by [`../piezo_force_monitor`](../piezo_force_monitor)** (charge amplifiers +
> PI PL033.31 chips, force from the charge equation, guided per-piezo calibration).
> Known limitation of this older sketch on the UNO Q: it prints and reads with `Serial`. Without
> the Bridge library, `Serial` is the D0/D1 UART, not the serial monitor. Its loop also checks
> for input on every pass, and on the UNO Q each serial-monitor call is an RPC to the Linux side.
> The new sketch uses the Bridge `Monitor` object and samples in its own thread.

Firmware to align the horn/anvil of an **ultrasonic welder** using three piezo
discs, and to measure the impact (compression force proxy) per side.

Three piezo discs sit between the horn and the anvil (**sonotrode OFF**). Each
feeds one analog input: `A0`, `A1`, `A2`. On horn descent the firmware reports:

1. **Which disc crosses the rising threshold first** and by how many microseconds
   the others follow — i.e. which side contacts first and how badly the horn is
   tilted.
2. **The integral of each piezo signal over time** (impulse ≈ energy /
   compression force per side), from the first crossing until **all** channels
   fall back below a second threshold **or** 10 s elapse — whichever comes first.

Balanced integrals **and** near-simultaneous crossings ⇒ the horn is flat against
the anvil.

---

## ⚠️ Hardware safety — read first

The UNO Q sketch runs on an **STM32U585**, a **3.3 V** part:

- **`A0` and `A1` are NOT 5 V tolerant.** None of `A0..A5` tolerate more than
  **~3.6 V**. Valid analog input range is **0 V … VREF+ (default 3.3 V)**.
- A **piezo disc under impact can output tens of volts** (and go negative).
  Connecting it raw **will destroy the pin / board.**

Condition every piezo so the pin only ever sees `0..VREF`. Minimum sane front end
per channel:

```
 piezo +──┬───────────────┬──────┬────────────► Ax (A0/A1/A2)
          │               │      │
         [R_load]      [D1 to 3V3]│         R_load ~1 MΩ  (bleeds charge, sets decay)
         ~1 MΩ         [D2 to GND]│         D1/D2 = small Schottky clamp diodes
          │               │      │          (optional) op-amp buffer before Ax
 piezo -──┴───────────────┴──────┴────────────► GND
```

If your piezo swings well above 3.3 V, add a resistor divider (and/or an op-amp
buffer) ahead of the clamp. Keep all three channels identical so the three sides
are comparable. A small series resistor (e.g. 1 kΩ) into the pin adds protection.

---

## Wiring

| Signal        | Pin            | Notes                                           |
|---------------|----------------|-------------------------------------------------|
| Piezo 0       | `A0` (PA4)     | conditioned to 0..3.3 V                          |
| Piezo 1       | `A1` (PA5)     | conditioned to 0..3.3 V                          |
| Piezo 2       | `A2` (PA6)     | conditioned to 0..3.3 V                          |
| Reset button  | `D2` → GND     | momentary, uses internal pull-up (pressed = LOW) |
| Status LED    | `LED_BUILTIN`  | on while integrating, off otherwise              |

Set `RESET_BTN_PIN` to `-1` in the sketch to disable the hardware button (the
serial `r` command still resets).

---

## Build & run

1. Open `piezo_weld_aligner.ino` in the Arduino IDE / Arduino App Lab for the
   UNO Q and select the UNO Q board (sketch targets the on-board STM32U585).
2. Upload.
3. Open the Serial Monitor at **115200 baud**.

Commands over serial: `r` = reset / re-arm, `h` (or `?`) = help.

### What you'll see

On reset it prints `# RESET -> ARMED ...`, settles the baseline, then waits. After
an impact it prints a result block: stop reason, integration time, measured scan
rate (= your timing resolution), the **first** channel and the per-channel delays,
and per-channel **peak**, **integral** (`cnt·s` and `V·s`) and **force-balance %**.

---

## Tuning

All knobs are `#define`/`const` at the top of the sketch:

| Setting                     | Meaning                                                            |
|-----------------------------|--------------------------------------------------------------------|
| `ADC_BITS`                  | 8 / 10 / 12 / 14. Default 12 (0..4095). Re-tune thresholds if changed. |
| `RISE_THRESHOLD`            | counts **above baseline** that start an event / count as a crossing. |
| `FALL_THRESHOLD`            | counts **above baseline** below which the event is over (< RISE).  |
| `FALL_HOLD_US`              | signal must stay below FALL this long to stop (default 2 ms).      |
| `INTEGRATION_TIMEOUT_US`    | hard cap from first crossing (default 10 s).                       |
| `BASELINE_SHIFT`            | baseline EMA speed (1/16 by default); frozen at trigger.           |
| `BASELINE_MIN_SCANS`        | scans spent settling the baseline before arming.                   |
| `VREF`                      | reference volts, display only (default 3.3).                       |

**How to pick thresholds:** send `r`, tap a disc, read the reported `peak` counts
for a light vs. a real contact, then set `RISE` above the noise floor and `FALL`
roughly half of `RISE`.

Thresholds and the integral are measured **relative to a per-channel baseline**
captured just before the impact, so a DC offset or slow drift on a piezo channel
won't bias the result.

---

## Interpreting the result

- **First channel** = the side that contacts first ⇒ that side sits low; tilt the
  head so the *later* side comes down to meet the anvil.
- **Delays (`+NN us`)** quantify the tilt: large spread = badly tilted, all `+0`
  (within one scan) = flat to within one scan period.
- **Integral per side** ≈ impulse/energy that side absorbed ⇒ compression-force
  distribution. **Force-balance %** near 33/33/33 with equal delays = well aligned.
- The printed **timing resolution** is your real limit for "which first" — if two
  sides report `+0 us`, they differ by less than one scan period, not necessarily
  zero.

---

## Overflow protection (what the code guards)

- Integral and sum accumulators are **`uint64_t` with saturating add**
  (`addSat64` clamps instead of wrapping). At 14-bit and 2.5 MS/s for 10 s the
  worst case is ≈ `1.6e11` per channel — far inside `uint64_t`.
- All time math uses **unsigned subtraction** (`now - t0`), which is correct
  across the `micros()` wrap (~71.6 min) for our ≤10 s windows.
- `amp = value − baseline` is **clamped at 0** (no unsigned underflow); peak,
  counts and scan counters are range-checked before increment.

---

## Going faster (true register / DMA level)

`analogRead()` on the STM32U585 core reconfigures the ADC on every call, so it is
not the fastest option. This sketch stays correct regardless of rate by
integrating `amplitude × dt` with `micros()` time-stamps and by **printing the
measured scan rate**, so the integral and the crossing delays are honest even at a
modest rate.

For genuine memory-address-level speed on the UNO Q, the path is **not** AVR
`ADMUX`/`ADCSRA` registers (this is an STM32, not an ATmega). Instead:

- `A0..A5` connect to **both ADC1 and ADC4**; `A0..A2` are **PA4/PA5/PA6**.
- Configure a **timer-triggered, DMA** multi-channel scan (optionally **ADC1 +
  ADC4 simultaneously**) to free-run into a RAM buffer at **up to ~2.5 MS/s**,
  then run the same threshold/first-crossing/integration logic over the buffer.
- This needs the STM32 HAL/CMSIS and must be verified on the board, so it is left
  as an upgrade rather than shipped blind (a wrong channel map would silently read
  the wrong pin on a welder). See the Arduino forum thread below for a working
  UNO Q dual-ADC + DMA capture approach.

### Sources

- [UNO Q User Manual](https://docs.arduino.cc/tutorials/uno-q/user-manual/)
- [UNO Q Power Specifications](https://docs.arduino.cc/tutorials/uno-q/power-specification/)
- [UNO Q: simultaneous ADC1 + ADC4 timer-triggered DMA capture up to 2.5 MS/s (Arduino Forum)](https://forum.arduino.cc/t/uno-q-simultaneous-adc1-adc4-timer-triggered-dma-capture-at-up-to-2-5-ms-s/1448253)
- [Arduino UNO Q pinout (QRB2210 / STM32U585)](https://esp32.co.uk/arduino-uno-q-pinout-qrb2210-stm32u585-gpio-qwiic-usb/)
- [STM32 analogRead reinitialises the ADC each call (SimpleFOC forum)](https://community.simplefoc.com/t/stm32-analogread/4476)
