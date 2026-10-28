# Heat Sealer Alignment and Clamp-Force Monitor — Design Spec

Signal conditioning for three piezo sensors read by an Arduino UNO Q.
Source: claude.ai conversation, Oct 27–28, 2026. Prepared as project context for Claude Code.

> **How to use this file**
>
> - **Part 1 is the current, authoritative design.** Use it for any wiring, math, or firmware work.
> - **Part 2 is the conversation log**, condensed and in order. Values marked *superseded* were replaced later and must not be used.
> - The Arduino sketch already exists. This document covers hardware, signal math, and calibration only.
> - Figures marked *estimate* are derived, not datasheet values. Calibration (section 8) replaces them.

---

## Part 1 — Current design

### 1. Goal and constraints

| Item | Value |
|---|---|
| Machine | Heat sealer with two sealing bars (hand impulse-sealer style, 1/2 in seal) |
| Sensors | 3 piezo sensors placed between the two sealing bars |
| Controller | Arduino UNO Q, analog inputs A0, A1, A2 |
| Alignment | Which side is hit first (threshold-crossing times) and force share at peak |
| Clamp force | About 200 lbf (890 N) total; about 297 N per sensor if evenly split |
| Squeeze duration | 1.5 s maximum |
| Build | Breadboard, through-hole components only |
| Firmware | Already written by the user; no code requested |

### 2. Arduino UNO Q analog input facts

- MCU I/O banks run at 3.3 V. Absolute maximum on the analog pins is 3.6 V. The pins are **not 5 V tolerant** when used as ADC inputs.
- ADC resolution is configurable to 8, 10, 12 or 14 bits (`analogReadResolution(14)`). Default reference is 3.3 V.
- Design consequence: the op amp is powered from the UNO Q 3.3 V pin, so its outputs physically cannot exceed the ADC range.

Sources: docs.arduino.cc — UNO Q Power Specifications; UNO Q User Manual.

### 3. Sensor — PI Ceramic PL033.31

PICMA multilayer chip actuator, used here as a compressive force sensor.

| Parameter | Value | Source |
|---|---|---|
| Size | 3 × 3 × 2 mm (4.2 mm diagonal) | PI datasheet |
| Leads | PTFE-insulated stranded wires, 100 mm, AWG 32 (".31" suffix) | PI |
| Capacitance (Cp) | 75 nF ±20% | PI |
| Blocking force | > 300 N | PI |
| Travel | 2.2 µm at 0–100 V | PI |
| Ceramic | PIC252 | PI |
| Operating voltage | −20 to +100 V | PI |
| Operating temperature | −40 to +150 °C | PI |
| Preload | 15 MPa recommended (dynamic); 30 MPa max (constant force) | PI |
| Price | $60 each (PI USA list) | PI |
| Charge sensitivity S | About 13–22 nC/N (*estimate*) | Derived |

**Sensitivity estimate.** For a stack, travel per volt equals charge per newton: 2.2 µm / 100 V = 22 nm/V = 22 nC/N. That is a large-signal value, so small-signal sensing is likely lower. Plan on 13–22 nC/N until calibrated.

**Stress check at 200 lbf**

- PL033.31: 297 N / 9 mm² = 33 MPa per chip with perfectly even loading. That exceeds PI's 30 MPa constant-force limit. In spec only if total force is at or below about 180 lbf (270 N per chip). Misalignment loads one chip harder.
- Alternative: **PL055.31** (5 × 5 × 2 mm, 250 nF, $68). 11.9 MPa at 297 N; about 750 N per chip before reaching 30 MPa. Same height and travel, so same charge per newton: **no circuit change** (channel delay rises to about 1.2 ms, still matched across channels). Does not meet the under-5 mm size limit.

**Handling**

- Open-circuit voltage at 297 N is roughly Q/Cp, about 50–90 V. Keep the two wires shorted (twisted together) whenever the chip is not connected to the circuit. A squeeze and release while open can drive the chip past its −20 V reverse limit.
- The AWG 32 wires are fragile. Strain-relieve them near the chip.

### 4. Circuit

Three identical charge amplifiers plus a buffered reference voltage, all on one MCP6004 quad op amp, powered from the UNO Q 3.3 V pin. Schematic: `heat-sealer-charge-amp.svg`.

#### 4.1 Netlist

Nets: `+3V3` (UNO Q 3.3 V pin), `GND` (UNO Q GND), `VDIV`, `VREF`, and for each channel n = 1, 2, 3: `PZn`, `INn`, `OUTn`, `An`.

**Power and decoupling**

- U1 pin 4 → `+3V3`; U1 pin 11 → `GND`.
- C4 100 nF ceramic: U1 pin 4 to U1 pin 11, placed directly at the pins (pins 4 and 11 are opposite each other across the chip).
- C5 10 µF electrolytic: `+3V3` (+) to `GND`.

**Reference (shared)**

- R1 100 kΩ: `+3V3` → `VDIV`.
- R2 10 kΩ: `VDIV` → `GND`.
- C3 10 µF electrolytic: `VDIV` (+) → `GND`.
- U1D voltage follower: pin 12 (+) = `VDIV`; pin 13 (−) tied to pin 14; pin 14 = `VREF` (about 0.30 V).

**Channel pin map**

| Channel | Op amp | − input (INn) | + input | Output (OUTn) | ADC pin |
|---|---|---|---|---|---|
| 1 | U1A | pin 2 | pin 3 | pin 1 | A0 |
| 2 | U1B | pin 6 | pin 5 | pin 7 | A1 |
| 3 | U1C | pin 9 | pin 10 | pin 8 | A2 |

**Per-channel connections** (repeat for n = 1, 2, 3)

- Sensor red wire → `VREF`. Sensor black wire → `PZn`.
- DZa and DZb, 1N5231B 5.1 V zeners: in series, anode to anode (back-to-back), between `PZn` and `VREF`.
- Rin 4.7 kΩ: `PZn` → `INn`.
- D1 1N4148: anode `INn`, cathode `+3V3`.
- D2 1N4148: anode `GND`, cathode `INn`.
- Cf 6.8 µF polyester film, 63 V: `INn` ↔ `OUTn`.
- Rf 10 MΩ: `INn` ↔ `OUTn`.
- Op amp + input → `VREF`.
- Rout 1 kΩ: `OUTn` → `An`.
- Cflt 100 nF ceramic: `An` → `GND`.
- Optional reset: momentary switch in series with 100 Ω, across Cf.

**Polarity.** The output should rise on squeeze. If it falls, swap the sensor's red and black wires.

#### 4.2 Bill of materials (all through-hole)

| Ref | Part | Qty | Notes |
|---|---|---|---|
| U1 | MCP6004-I/P quad op amp, PDIP-14 | 1 | Rail-to-rail I/O, 1.8–6 V supply, about 1 pA input bias |
| Sensor | PI Ceramic PL033.31 | 3 | Or PL055.31 (see section 3) |
| Cf | 6.8 µF polyester film, 63 V, radial (or 4.7 µF + 2.2 µF in parallel) | 3 | Film only, not ceramic |
| Rf | 10 MΩ ¼ W | 3 | |
| Rin | 4.7 kΩ ¼ W | 3 | |
| DZ1–DZ6 | 1N5231B 5.1 V zener, DO-35 | 6 | Back-to-back pair across each sensor |
| D1–D6 | 1N4148, DO-35 | 6 | Two per channel |
| Rout | 1 kΩ ¼ W | 3 | |
| Cflt | 100 nF ceramic, radial | 3 | |
| R1 | 100 kΩ ¼ W | 1 | |
| R2 | 10 kΩ ¼ W | 1 | |
| C3, C5 | 10 µF electrolytic, 16 V or higher | 2 | Observe polarity |
| C4 | 100 nF ceramic, radial | 1 | Directly at U1 pins 4 and 11 |
| Optional | Tactile switch + 100 Ω | 3 | Reset across each Cf |

#### 4.3 Why each part is there

- **Op amp at 3.3 V:** outputs cannot exceed the UNO Q ADC limit.
- **Charge amplifier:** ΔV = Q / Cf, independent of sensor and cable capacitance. Noise gain 1 + Cp/Cf ≈ 1.01.
- **Rf:** gives leakage currents a DC path so the output cannot drift into a rail. DC offset ≈ leakage × Rf (about 10 mV at 1 nA). Together with Cf it sets τ.
- **Rin with D1/D2:** limit and clamp current into the op amp input during overload.
- **Zener pair:** absorbs overload charge (for example, far more than 900 N on one chip, or a dropped sensor) so it never dumps into the 3.3 V rail. Normally about 0 V across them, so leakage is negligible.
- **Rout with Cflt:** anti-aliasing low-pass (corner about 1.6 kHz) and a stiff source for the ADC.
- **VREF ≈ 0.30 V:** nearly the whole 3.3 V range is available for squeeze, with about 0.3 V of headroom below for the small undershoot after release.
- **C4 / C5 decoupling:** C4 (100 nF ceramic) handles fast noise and keeps the op amp stable; it only works when placed at pins 4 and 11. C5 (10 µF) supplies slower current bursts during a squeeze and filters ripple from the UNO Q 3.3 V rail, which is shared with its Linux processor and Wi-Fi. Ceramic is fine for decoupling; only Cf must be film.

### 5. Key design numbers

| Quantity | Value |
|---|---|
| τ = Rf × Cf | 10 MΩ × 6.8 µF = 68 s |
| Droop during a 1.5 s hold (uncorrected) | 1 − e^(−1.5/68) ≈ 2.2% |
| Undershoot after release | About 2.2% of peak; fades with τ (about 3–4 min to clear) |
| Output at 297 N per chip | 0.57–0.96 V above VREF |
| Usable output swing | About 2.9 V (0.30 V up to about 3.2 V) |
| Clip point, single chip | About 900 N (at 22 nC/N) to 1500 N (at 13 nC/N) |
| ADC resolution (14-bit, 3.3 V reference) | 0.2 mV per count, about 0.06–0.1 N per count |
| Channel delay | Rin × Cp ≈ 0.35 ms plus Rout × Cflt = 0.1 ms; ramp lag about 0.35–0.45 ms |
| Channel delay mismatch | About 0.1 ms worst case (from Cp ±20%) |
| Nominal k_i before calibration | Cf / S ≈ 310–520 N/V |

**Effect of τ on a constant 1.5 s squeeze** (output as % of true force):

| Moment | τ = 68 s (current) | τ = 4.7 s (superseded) |
|---|---|---|
| End of hold | 97.8% | 72.7% |
| Just after release | −2.2% | −27.3% |

τ governs how long a held reading stays valid. It does **not** slow the response to a hit; that is set by the channel delay.

### 6. Cf sizing

1. Sensitivity S ≈ 13–22 nC/N (*estimate*).
2. Worst case: misaligned jaws put all 890 N on one chip, so Q ≈ 890 × 22 nC ≈ 19.6 µC.
3. Usable output swing ≈ 2.9 V.
4. Cf ≥ 19.6 µC / 2.9 V = 6.75 µF, so use the standard value 6.8 µF.
5. Resolution check at 297 N: 0.57–0.96 V is about 2,800–4,800 ADC counts, so there is no reason to use a smaller Cf.

Side benefits: a long τ with an ordinary 10 MΩ resistor; the sensor's own capacitance becomes irrelevant; Cf tolerance is absorbed by calibration (it only affects τ, and therefore the 2.2% correction term).

### 7. Force math (squeeze of 1.5 s or less)

Sample each channel at fs = 2 kHz (Ts = 0.5 ms). Record at least 0.2 s before closure (baseline) and at least 0.2 s after release.

```
For each channel i = 1, 2, 3:
  dV_i[n]  = V_i[n] - mean(V_i over pre-closure samples)      [volts]
  S_i[n]   = S_i[n-1] + dV_i[n]          (running sum, S_i[-1] = 0)
  F_i[n]   = k_i * ( dV_i[n] + (Ts / tau) * S_i[n] )           [newtons]

F_total[n]  = F_1[n] + F_2[n] + F_3[n]
Clamp force = max over n of F_total[n]

tau = 68 s (tune per channel during calibration)
Ts / tau = 7.35e-6 at fs = 2 kHz
k_i from calibration (nominal 310-520 N/V)
```

Rules:

- Sum the three forces sample by sample. Never add each channel's own peak: when the jaws are misaligned, the peaks happen at different times and the sum overstates the force.
- The correction term adds about 2.2% by the end of a 1.5 s hold. With τ known to ±10%, the remaining error is about 0.25%. It can be skipped if 2% accuracy is acceptable.
- **Self-check after every squeeze:** F_total returns to 0 within ±0.5% of the peak. A negative residual means τ in software is too large (lower it); a positive residual means it is too small.
- Pressure = F_total / (jaw length × seal width). Example: 200 lbf / (8 in × 0.5 in) = 50 psi.

**Alignment**

- Onset time per channel = first sample where F_i crosses a threshold, minus that channel's stored timing offset (section 8, step 6). Interpolate between samples for sub-sample resolution. The earliest channel is the side that hits first; the spread between earliest and latest onset is the closure skew.
- Force share at peak = F_i / F_total at the moment of peak F_total. About 33% each means the jaws are parallel under load.

**Background:** with a bare piezo and only a load resistor R, the voltage tracks the rate of change of force (Q = Cp·V + (1/R)·∫V dt), so voltage peaks reflect closing speed, not force. The charge amplifier removes that problem.

### 8. Calibration procedure

**Why it is required:** PI does not publish charge per newton for this part; chips vary from part to part (capacitance ±20% hints at similar sensitivity spread); mounting affects the response; Cf tolerance and τ vary by channel; sensitivity rises somewhat with stress. Each k_i belongs to one chip paired with one channel. Label both and keep pairs together.

**Equipment**

- Reference force rated for at least 300 N (70 lbf) per chip and 900 N (200 lbf) for the system check: a digital bathroom scale (about ±1.5% at 30 kg), or a certified force gauge or button load cell (Mark-10, Imada) for ±0.5% or better.
- A manual arbor press to apply force smoothly, pressing down onto the scale.
- The exact mounting stack used in the sealer: same shims, same chip, same orientation.

**Steps**

1. **Condition (new chips only):** squeeze each chip 30–50 times to about working force before calibrating.
2. **τ per channel (no force reference needed):** hold a steady force about 10 s, then release. Adjust τ in the force equation until the corrected force is flat during the hold and returns to zero after release. Pass: residual within ±0.5% of peak.
3. **k_i per chip:** put the shim / chip / shim stack on the scale, centered under the press. Apply about 50, 100, 150, 200 and 270 N, starting from zero each time. Ramp over about 0.5 s, then hold. Read the scale at the end of the hold (bathroom scales need 2–3 s to settle) and compare it with the τ-corrected reading at the same moment. Repeat each level 3 times. Fit force against corrected ΔV: the slope is k_i and the intercept should be near zero.
    - Pass: repeats agree within 1–2%, and every point lies within about 2% of the fitted line. If the points curve consistently, use a quadratic fit or a lookup table.
    - Stop at 270 N (the 30 MPa limit) unless you have accepted running the chips at 297 N.
4. **Mounting checks:** rotate the chip 90° and repeat one level (more than 2% difference means uneven loading; check shim flatness). Repeat one level with a fast ramp (about 0.1 s) and a slow one (about 1 s); the corrected readings should match, otherwise τ is off.
5. **System check:** place all three chips, in their sealer positions, under one rigid plate on the scale. Center press: F_1 + F_2 + F_3 within about 2% of the scale. Off-center presses: the total stays the same while the shares shift toward the pressed side. If the total changes with press position, one k_i is off.
6. **Timing offsets:** center-press the plate about 10 times. Record each channel's threshold-crossing time relative to the mean of the three, average over presses, store, and subtract from every alignment measurement.
7. **Record** for each chip/channel pair: k_i, intercept, worst fit deviation, τ_i, timing offset, date, room temperature.

**When to recheck**

- Before each test session: apply one known load through the plate. More than 3% off means recalibrate.
- After any overload (a chip saw well over 270 N).
- After re-mounting or replacing any chip, shim or Cf.
- When the test temperature differs a lot from calibration (sensitivity can shift several percent over a 20 °C swing).

### 9. Mechanical and build notes

- Load each chip between flat, parallel, hard shims (ground steel). No bending, shear or edge loading — these crack a stack long before the force does.
- The sensors must be the only contact points between the bars; otherwise they measure only part of the force.
- The 2 mm chip height (plus shims) props the jaws open. Pneumatic actuation gives the same force at any gap; spring or toggle mechanisms may not.
- Heater off during measurements. Piezo ceramic is pyroelectric: a temperature change produces charge that looks like force. The PL033 survives 150 °C, but readings during heating are invalid.
- Leads short and taped down (cable flex generates charge).
- Breadboard: keep each channel's summing node (op amp − pin, Rin, Cf, Rf, D1, D2) compact; Cf and Rf straight across the op amp's − input and output pins; C4 bridged across the chip between pins 4 and 11; keep the board clean and dry.
- Electrolytic capacitors: the longer lead is +; the stripe marks −.

### 10. Superseded first design (do not use)

The first answer targeted the user's Amazon buzzer discs, before the 200 lbf and 1.5 s constraints were given.

| Part | Superseded | Current |
|---|---|---|
| Sensor | Piezo buzzer discs (no published sensitivity) | PI PL033.31 |
| Cf | 47 nF film | 6.8 µF film |
| Rf | 100 MΩ (τ = 4.7 s) | 10 MΩ (τ = 68 s) |
| Rin | 10 kΩ | 4.7 kΩ |
| Zener clamp | None | 1N5231B back-to-back pair per channel |
| R1 / VREF | 33 kΩ / about 0.77 V | 100 kΩ / about 0.30 V |

Reasons: with τ = 4.7 s, a 1.5 s hold loses about 27% and undershoots about 27% on release; buzzer discs are bending-mode parts with no published sensitivity, and the user asked for a small, fully specified sensor.

---

## Part 2 — Conversation log (condensed)

**1. User:** Use piezo discs bought from Amazon to measure alignment in a heat sealer with two sealing bars (product image of a hand impulse sealer, 1/2 in seal). Three piezos between the bars, read by an Arduino UNO Q's analog inputs: which side hits first; all three at nearly the same time means aligned. Also find the force between the bars from the voltages — sum and find peaks, integrate, or something else? Already has the Arduino program, so no code. Needs the signal-conditioning circuit for a breadboard with through-hole parts and schematics.

**Answer (summary):** A bare disc's voltage tracks how fast force changes, so neither summing peaks nor integrating raw voltage gives force reliably. Proposed a charge amplifier per disc on an MCP6004 powered from 3.3 V (UNO Q analog pins are 3.3 V, not 5 V tolerant in analog mode), with a buffered VREF. Force per disc = k × baseline-subtracted voltage plus a leak-correction integral; sum the three time-aligned forces; peak of the sum is clamp force; calibrate against a scale; pressure = force / seal area; alignment from onset times plus force share. Cautions: heater off (pyroelectric; cheap discs rated about 70 °C), load through a small puck, sensors as the only contact points, short taped leads, compact breadboard layout. Values given then are *superseded* (section 10).

**2. User:** (1) Compression lasts 1.5 s max — design the equation for that. (2) What changes for 200 lbf between the arms? (3) Find a readily available piezo sensor under 5 mm diameter with soldered leads and a datasheet with capacitance and other characteristics. (4) Modify the circuit for its parameters.

**Answer (summary):** Selected PI PL033.31 (section 3), with the stress caveat (33 MPa vs 30 MPa limit at 200 lbf) and PL055.31 as the in-spec alternative needing no circuit change. Re-sized the circuit to the current values (section 4). Gave the discrete force equation with τ = 68 s (section 7) and build notes for the chips (section 9).

**3. User:** Explain the 100 nF / 10 µF part of the "Reference and power (shared)" diagram.

**Answer (summary):** Power-supply decoupling across U1 pins 4 and 11 (section 4.3). Covered local energy reservoir, filtering UNO Q rail noise, op amp stability, why two values, electrolytic polarity, and breadboard placement.

**4. User:** Explain τ and how it affects timing; justify the Cf value; what is channel delay?

**Answer (summary):** τ = Rf × Cf = 68 s sets droop during a hold, not response speed (section 5 table). Cf sizing walkthrough (section 6). Channel delay = Rin × Cp (≈ 0.35 ms) plus Rout × Cflt (0.1 ms); only the mismatch between channels (about 0.1 ms) affects alignment timing, and it can be measured and subtracted (section 8, step 6).

**5. User:** Is calibration required with PL033.31, and what is the procedure?

**Answer (summary):** Yes. Full procedure in section 8.

**6. User:** Create a PDF of the chat to share with Claude Code, or suggest a better way.

**Answer:** Delivered this document as a Markdown file (best for Claude Code), the standalone schematic as an SVG file, and a PDF of the same content for people.
