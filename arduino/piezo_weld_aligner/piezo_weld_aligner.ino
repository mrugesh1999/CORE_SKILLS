/* ============================================================================
 *  piezo_weld_aligner.ino
 *
 *  Target board : Arduino UNO Q  (sketch runs on the on-board STM32U585,
 *                 Cortex-M33 @ 160 MHz, 14-bit ADC).  NOT an AVR "classic" Uno.
 *
 *  Purpose
 *  -------
 *  Three piezo discs are placed between the horn and the anvil of an
 *  ultrasonic welder (sonotrode OFF).  Each disc feeds one analog input:
 *
 *        A0  ->  piezo 0
 *        A1  ->  piezo 1
 *        A2  ->  piezo 2
 *
 *  When the horn descends, the disc on the side that makes contact first
 *  produces a signal first.  This firmware does two things:
 *
 *    1) ALIGNMENT  - detects which of A0/A1/A2 crosses a rising threshold
 *                    FIRST, and the time offsets of the others relative to
 *                    it.  "First + how much later" tells you which side is
 *                    low and how badly the horn is tilted vs. the anvil.
 *
 *    2) FORCE       - integrates each piezo signal over time (impulse ~
 *                    energy / compression force per side) from the first
 *                    crossing until EITHER all channels fall back below a
 *                    second threshold OR 10 s elapse, whichever comes first.
 *                    Balanced integrals + simultaneous crossings  = flat/aligned.
 *
 *  Reset
 *  -----
 *    - Press the button on RESET_BTN_PIN (to GND, uses internal pull-up), OR
 *    - Send 'r' (or 'R') over Serial.
 *  Either re-arms the measurement from scratch.  'h' prints help.
 *
 *  SAFETY  (read before wiring!)
 *  -----------------------------
 *  A piezo disc under impact can output TENS of volts.  The UNO Q analog pins
 *  are 3.3 V parts: A0/A1 are NOT 5 V tolerant, and none of A0..A5 tolerate
 *  more than ~3.6 V.  You MUST condition each piezo so the pin only ever sees
 *  0..VREF (default 3.3 V), e.g.:
 *      piezo -> parallel load resistor (~1 MOhm) -> resistor divider / clamp
 *             -> Schottky clamp diodes to GND and to 3V3 -> (optional) op-amp
 *                buffer -> Ax
 *  Without clamping you can destroy the board.  See README.md for a circuit note.
 *
 *  "Fast / memory-address level"
 *  -----------------------------
 *  On the STM32U585 core, analogRead() reconfigures the ADC on every call, so
 *  it is not the fastest option.  This sketch therefore does NOT rely on a
 *  fixed loop rate: it time-stamps every scan with micros() and integrates by
 *  (amplitude * dt), so the result is correct regardless of the achieved rate,
 *  and it PRINTS the measured sample rate so you know your timing resolution.
 *  For true register/DMA-level speed (simultaneous ADC1+ADC4, timer-triggered
 *  DMA up to ~2.5 MS/s) see "GOING FASTER" in README.md.
 * ========================================================================= */

#include <Arduino.h>

/* ----------------------------- CONFIG ----------------------------------- */

#define NUM_CH            3                 /* number of piezo channels      */
static const uint8_t PIEZO_PIN[NUM_CH] = { A0, A1, A2 };
static const char*   PIEZO_LBL[NUM_CH] = { "A0", "A1", "A2" };

/* ADC resolution. UNO Q supports 8, 10, 12 or 14 bits. 12 is a good default
 * (fast + quiet). If you change this, re-tune the thresholds below, because
 * thresholds are expressed in ADC counts.                                   */
#define ADC_BITS          12
static const uint32_t ADC_FULLSCALE = (1UL << ADC_BITS) - 1UL;   /* e.g. 4095 */

/* ADC reference voltage, volts. Default on UNO Q is 3.3 V. Only used to turn
 * the printed results into volts; it does not affect the raw measurement.   */
static const float VREF = 3.3f;

/* Detection thresholds, in ADC COUNTS ABOVE the resting baseline (not raw).
 *   RISE  : an event has started on a channel (start integration / crossing).
 *   FALL  : the event on a channel is considered over.
 * TUNE THESE to your piezo + conditioning. Keep RISE > FALL (hysteresis).   */
static const uint16_t RISE_THRESHOLD = 200;    /* counts above baseline      */
static const uint16_t FALL_THRESHOLD = 100;    /* counts above baseline      */

/* Integration stops when ALL channels have stayed below FALL_THRESHOLD for
 * this long (confirms the impact is really over, not just a momentary dip). */
static const uint32_t FALL_HOLD_US   = 2000UL;        /* 2 ms                */

/* Hard cap: integrate at most this long from the first crossing.            */
static const uint32_t INTEGRATION_TIMEOUT_US = 10000000UL;  /* 10 s          */

/* Baseline (resting level) tracking while ARMED. The baseline is an exking
 * (exponential) moving average; it is FROZEN at the first crossing so the
 * impact itself does not drag it. Larger shift = slower/smoother baseline.  */
#define BASELINE_SHIFT    4                 /* EMA factor = 1/16             */
static const uint16_t BASELINE_MIN_SCANS = 128;  /* settle before arming     */

/* Reset button. Wire a momentary button between this pin and GND.
 * Set to -1 to disable the hardware button (serial 'r' still works).        */
#define RESET_BTN_PIN     2
static const uint32_t DEBOUNCE_MS = 25;

/* Serial speed */
#define SERIAL_BAUD       115200

/* ----------------------------- STATE ------------------------------------ */

enum State { ST_ARMED, ST_INTEGRATING, ST_DONE };
static State state = ST_ARMED;

/* per-channel working data */
static uint16_t baseline[NUM_CH];               /* resting level (counts)    */
static bool     baselineInit = false;
static bool     baselineFrozen = false;
static uint32_t armScans = 0;                   /* scans spent settling      */

static uint32_t firstCrossUs[NUM_CH];           /* micros() of 1st crossing, */
                                                /* 0 => has not crossed yet  */
static uint16_t peakAmp[NUM_CH];                /* peak amplitude (counts)   */
static uint32_t peakUs[NUM_CH];                 /* micros() of peak          */
static uint64_t sumCounts[NUM_CH];              /* Sum of amplitudes         */
static uint64_t integralCountUs[NUM_CH];        /* Sum(amplitude * dt_us)    */

static uint32_t t0Us        = 0;                /* integration start         */
static uint32_t lastScanUs  = 0;                /* for dt                    */
static uint32_t belowSinceUs= 0;                /* when all went below FALL  */
static uint32_t sampleCount = 0;                /* scans during integration  */
static uint8_t  stopReason  = 0;                /* 1=fell,2=timeout          */
static bool     resultsPrinted = false;

static bool     resetFlag = false;              /* set by button/serial      */

/* ----------------------------- HELPERS ---------------------------------- */

/* Saturating 64-bit add (overflow protection for the accumulators).        */
static inline uint64_t addSat64(uint64_t a, uint64_t b) {
  uint64_t s = a + b;
  return (s < a) ? UINT64_MAX : s;            /* wrapped -> clamp to max     */
}

static inline void ledSet(bool on) {
#ifdef LED_BUILTIN
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
#else
  (void)on;
#endif
}

/* Clear everything and re-arm. */
static void resetMeasurement() {
  state          = ST_ARMED;
  baselineInit   = false;
  baselineFrozen = false;
  armScans       = 0;
  t0Us = lastScanUs = belowSinceUs = 0;
  sampleCount    = 0;
  stopReason     = 0;
  resultsPrinted = false;
  for (uint8_t i = 0; i < NUM_CH; i++) {
    baseline[i]       = 0;
    firstCrossUs[i]   = 0;
    peakAmp[i]        = 0;
    peakUs[i]         = 0;
    sumCounts[i]      = 0;
    integralCountUs[i]= 0;
  }
  ledSet(false);
  Serial.println(F("# RESET -> ARMED (settling baseline, waiting for impact)"));
}

static void printHelp() {
  Serial.println(F("# ---------------------------------------------------"));
  Serial.println(F("# Piezo weld aligner  (UNO Q / STM32U585)"));
  Serial.println(F("#   r : reset / re-arm      h : help"));
  Serial.print  (F("#   RISE="));   Serial.print(RISE_THRESHOLD);
  Serial.print  (F(" FALL="));      Serial.print(FALL_THRESHOLD);
  Serial.print  (F(" counts above baseline, ADC "));
  Serial.print  (ADC_BITS); Serial.print(F("-bit (0.."));
  Serial.print  (ADC_FULLSCALE); Serial.println(F(")"));
  Serial.println(F("# ---------------------------------------------------"));
}

/* Convert counts -> volts for display only. */
static inline float toVolts(uint32_t counts) {
  return (float)counts * VREF / (float)ADC_FULLSCALE;
}

/* --------------------------- INPUT: serial + button --------------------- */

static void handleSerial() {
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c == 'r' || c == 'R') resetFlag = true;
    else if (c == 'h' || c == 'H' || c == '?') printHelp();
  }
}

static void checkButton() {
#if (RESET_BTN_PIN >= 0)
  static int      lastReading = HIGH;   /* idle = HIGH (pull-up)             */
  static int      stable      = HIGH;
  static uint32_t lastChange  = 0;
  int reading = digitalRead(RESET_BTN_PIN);
  if (reading != lastReading) {
    lastReading = reading;
    lastChange  = millis();
  }
  if ((uint32_t)(millis() - lastChange) >= DEBOUNCE_MS) {
    if (stable != lastReading) {
      stable = lastReading;
      if (stable == LOW) resetFlag = true;   /* pressed (to GND)             */
    }
  }
#endif
}

/* --------------------------- STATE STEPS -------------------------------- */

/* ARMED: build/track baseline, then watch for the first threshold crossing. */
static void armedStep(uint32_t now, const uint16_t a[NUM_CH]) {

  /* initialise baseline on the very first scan */
  if (!baselineInit) {
    for (uint8_t i = 0; i < NUM_CH; i++) baseline[i] = a[i];
    baselineInit = true;
    armScans = 1;
    return;                       /* need more scans before we can trigger   */
  }

  /* amplitude above the (not-yet-frozen) baseline, clamped at 0 */
  uint16_t amp[NUM_CH];
  bool crossed = false;
  for (uint8_t i = 0; i < NUM_CH; i++) {
    amp[i] = (a[i] > baseline[i]) ? (uint16_t)(a[i] - baseline[i]) : 0;
    if ((armScans >= BASELINE_MIN_SCANS) && (amp[i] >= RISE_THRESHOLD)) {
      if (firstCrossUs[i] == 0) firstCrossUs[i] = (now == 0) ? 1 : now;
      crossed = true;
    }
  }

  if (crossed) {
    /* ---- TRIGGER: lock baseline, start integrating ---- */
    baselineFrozen = true;
    t0Us           = now;
    lastScanUs     = now;
    belowSinceUs   = 0;
    sampleCount    = 0;
    state          = ST_INTEGRATING;
    ledSet(true);
    /* fold this triggering sample into the integral too (dt = 0 here) */
    for (uint8_t i = 0; i < NUM_CH; i++) {
      if (amp[i] > peakAmp[i]) { peakAmp[i] = amp[i]; peakUs[i] = now; }
      sumCounts[i] = addSat64(sumCounts[i], amp[i]);
    }
    return;
  }

  /* no crossing: keep the baseline following the resting level (EMA) */
  for (uint8_t i = 0; i < NUM_CH; i++) {
    int32_t err = (int32_t)a[i] - (int32_t)baseline[i];
    baseline[i] = (uint16_t)((int32_t)baseline[i] + (err >> BASELINE_SHIFT));
  }
  if (armScans < 0xFFFFFFFFUL) armScans++;
}

/* INTEGRATING: accumulate amplitude*dt; capture late crossings + peaks;
 * stop on all-below-FALL (held) or on the 10 s timeout.                     */
static void integrateStep(uint32_t now, const uint16_t a[NUM_CH]) {

  uint32_t dt = now - lastScanUs;      /* unsigned -> micros() wrap safe      */
  lastScanUs  = now;

  bool allBelow = true;
  for (uint8_t i = 0; i < NUM_CH; i++) {
    uint16_t amp = (a[i] > baseline[i]) ? (uint16_t)(a[i] - baseline[i]) : 0;

    if (firstCrossUs[i] == 0 && amp >= RISE_THRESHOLD)
      firstCrossUs[i] = (now == 0) ? 1 : now;          /* late crosser        */

    if (amp > peakAmp[i]) { peakAmp[i] = amp; peakUs[i] = now; }

    sumCounts[i]       = addSat64(sumCounts[i], amp);
    integralCountUs[i] = addSat64(integralCountUs[i], (uint64_t)amp * (uint64_t)dt);

    if (amp >= FALL_THRESHOLD) allBelow = false;
  }
  if (sampleCount < 0xFFFFFFFFUL) sampleCount++;

  /* stop conditions */
  if (allBelow) {
    if (belowSinceUs == 0) belowSinceUs = (now == 0) ? 1 : now;
    else if ((uint32_t)(now - belowSinceUs) >= FALL_HOLD_US) {
      stopReason = 1; state = ST_DONE; ledSet(false); return;
    }
  } else {
    belowSinceUs = 0;                 /* reset the "below" hold timer         */
  }

  if ((uint32_t)(now - t0Us) >= INTEGRATION_TIMEOUT_US) {
    stopReason = 2; state = ST_DONE; ledSet(false);
  }
}

/* DONE: print results exactly once, then idle until reset. */
static void doneStep() {
  if (resultsPrinted) return;
  resultsPrinted = true;

  uint32_t duration = (lastScanUs - t0Us);        /* us, wrap-safe            */

  /* find which channel crossed first */
  uint8_t  firstCh   = 0xFF;
  uint32_t firstTime = 0xFFFFFFFFUL;
  uint8_t  nCrossed  = 0;
  for (uint8_t i = 0; i < NUM_CH; i++) {
    if (firstCrossUs[i] != 0) {
      nCrossed++;
      if (firstCrossUs[i] < firstTime) { firstTime = firstCrossUs[i]; firstCh = i; }
    }
  }

  Serial.println(F("==================== WELD ALIGNMENT RESULT ===================="));
  Serial.print  (F("Stop reason      : "));
  Serial.println(stopReason == 1 ? F("signal fell below FALL threshold")
                                  : F("10 s timeout"));
  Serial.print  (F("Integration time : ")); Serial.print(duration); Serial.print(F(" us ("));
  Serial.print  ((float)duration / 1000.0f, 3); Serial.println(F(" ms)"));
  Serial.print  (F("Scans            : ")); Serial.print(sampleCount);
  if (sampleCount > 0 && duration > 0) {
    float rate = (float)sampleCount * 1e6f / (float)duration;      /* scans/s */
    Serial.print(F("   (~")); Serial.print(rate, 0);
    Serial.print(F(" scans/s, timing resolution ~"));
    Serial.print((float)duration / (float)sampleCount, 1); Serial.print(F(" us)"));
  }
  Serial.println();

  /* ---- ALIGNMENT: order + delays ---- */
  Serial.println(F("---- Which hit first (alignment) ----"));
  if (firstCh == 0xFF) {
    Serial.println(F("  (no channel crossed RISE threshold)"));
  } else {
    Serial.print(F("  FIRST: ")); Serial.print(PIEZO_LBL[firstCh]);
    Serial.println(F("  -> this side contacts first (sits low / tilt toward it)"));
    for (uint8_t i = 0; i < NUM_CH; i++) {
      Serial.print(F("  ")); Serial.print(PIEZO_LBL[i]); Serial.print(F(" : "));
      if (firstCrossUs[i] == 0) {
        Serial.println(F("never crossed"));
      } else {
        uint32_t d = firstCrossUs[i] - firstTime;   /* delay vs first, us     */
        if (d == 0) Serial.println(F("+0 us (first / within one scan)"));
        else { Serial.print(F("+")); Serial.print(d); Serial.println(F(" us later")); }
      }
    }
    if (nCrossed > 1) {
      Serial.println(F("  Tip: near-equal delays AND near-equal integrals => horn is flat/aligned"));
    }
  }

  /* ---- FORCE: peaks + integrals ---- */
  Serial.println(F("---- Impact magnitude (compression force proxy) ----"));
  double totalVs = 0.0;
  for (uint8_t i = 0; i < NUM_CH; i++) {
    double integ_cs = (double)integralCountUs[i] / 1e6;            /* count*s  */
    double integ_vs = integ_cs * (double)VREF / (double)ADC_FULLSCALE; /* V*s  */
    totalVs += integ_vs;
    Serial.print(F("  ")); Serial.print(PIEZO_LBL[i]);
    Serial.print(F("  peak="));    Serial.print(peakAmp[i]);
    Serial.print(F(" cnt ("));     Serial.print(toVolts(peakAmp[i]), 3); Serial.print(F(" V)"));
    Serial.print(F("  integral=")); Serial.print(integ_cs, 2); Serial.print(F(" cnt*s ("));
    Serial.print(integ_vs, 5);     Serial.print(F(" V*s)"));
    Serial.print(F("  sum="));      Serial.print((unsigned long)(sumCounts[i] & 0xFFFFFFFFUL));
    Serial.println();
  }

  /* ---- balance: integral share per side ---- */
  if (totalVs > 0.0) {
    Serial.println(F("---- Force balance (integral share) ----"));
    for (uint8_t i = 0; i < NUM_CH; i++) {
      double integ_vs = ((double)integralCountUs[i] / 1e6)
                        * (double)VREF / (double)ADC_FULLSCALE;
      Serial.print(F("  ")); Serial.print(PIEZO_LBL[i]); Serial.print(F(" : "));
      Serial.print(100.0 * integ_vs / totalVs, 1); Serial.println(F(" %"));
    }
  }

  Serial.println(F("Send 'r' or press the reset button to run again."));
  Serial.println(F("==============================================================="));
}

/* ------------------------------- SETUP ---------------------------------- */

void setup() {
  Serial.begin(SERIAL_BAUD);
  uint32_t t = millis();
  while (!Serial && (millis() - t) < 3000) { /* wait up to 3 s for USB CDC */ }

#ifdef LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
#endif
#if (RESET_BTN_PIN >= 0)
  pinMode(RESET_BTN_PIN, INPUT_PULLUP);
#endif

  analogReadResolution(ADC_BITS);
  /* prime each ADC channel once so the first real scan is not an outlier */
  for (uint8_t i = 0; i < NUM_CH; i++) (void)analogRead(PIEZO_PIN[i]);

  printHelp();
  resetMeasurement();
}

/* -------------------------------- LOOP ---------------------------------- */

void loop() {
  handleSerial();
  checkButton();
  if (resetFlag) { resetFlag = false; resetMeasurement(); }

  if (state == ST_DONE) { doneStep(); return; }   /* idle; nothing to sample */

  /* ---- fast acquisition: read all channels, integer only, no Serial ---- */
  uint32_t now = micros();
  uint16_t a[NUM_CH];
  for (uint8_t i = 0; i < NUM_CH; i++) a[i] = (uint16_t)analogRead(PIEZO_PIN[i]);

  if (state == ST_ARMED)            armedStep(now, a);
  else /* ST_INTEGRATING */         integrateStep(now, a);
}
