/* ============================================================================
 *  piezo_force_monitor.ino
 *  Clamp-force and alignment monitor:  Arduino UNO Q  +  3 x PI PL033.31
 *  piezo chips  +  3 x charge amplifier  (design: docs/heat-sealer-force-monitor.md)
 * ============================================================================
 *
 *  HARDWARE (spec Part 1, section 4)
 *    Each PL033.31 feeds one charge amplifier on an MCP6004 powered from the
 *    UNO Q 3.3 V pin: Cf 6.8 uF film || Rf 10 Mohm, Rin 4.7 k, zener and
 *    1N4148 clamps, Rout 1 k / Cflt 100 nF to the ADC pin, all referenced to a
 *    buffered VREF of about 0.30 V.  The output rises on squeeze (if one
 *    falls, swap that sensor's red and black wires).
 *        CH1 = U1A -> A0        CH2 = U1B -> A1        CH3 = U1C -> A2
 *    Optional: momentary button from D2 to GND = reset / re-arm.
 *
 *  PHYSICS (spec sections 3, 4.3 and 7)
 *    1. The piezo produces charge in proportion to force:   Q = S * F
 *         S = charge sensitivity, about 13..22 nC/N for the PL033.31.
 *    2. The charge amplifier turns that charge into voltage:  dV = Q / Cf
 *         dV = output minus its resting level (independent of sensor and
 *         cable capacitance).
 *    3. Rf slowly bleeds the charge away, tau = Rf * Cf = 68 s:
 *         Cf * dV/dt = dQ/dt - (V - V_eq) / Rf        V_eq = resting output level
 *         =>  Q / Cf = dV + (1/tau) * integral((V - V_eq) dt)
 *    4. So the force on chip i is
 *         F_i = k_i * ( dV_i + (1/tau_i) * integral((V_i - V_eq,i) dt) ),  k_i = Cf / S_i
 *       Once the output has settled the pre-closure baseline equals V_eq and
 *       this is exactly the spec's discrete form (fs = 2 kHz, Ts = 0.5 ms)
 *         F_i[n] = k_i * ( dV_i[n] + (Ts/tau_i) * S_i[n] ),  S_i[n] = S_i[n-1] + dV_i[n]
 *       Using V_eq keeps it exact right after earlier squeezes too, while their
 *       undershoot is still fading (3-4 min, spec 5).  Every sample is weighted
 *       by its measured dt (= Ts when sampling is perfectly regular).
 *    5. Clamp force = max over n of F_total[n] = F_1[n] + F_2[n] + F_3[n].
 *       The three individual peaks are never added: with misaligned jaws they
 *       happen at different moments and their sum overstates the force.
 *    6. Alignment: onset_i = first crossing of F_i over a threshold,
 *       interpolated between samples, minus the channel's timing offset.
 *       Earliest onset = the side that hits first; first-to-last = closure
 *       skew.  Force share_i = F_i / F_total at the moment F_total peaks
 *       (about 33 % each = jaws parallel under load).
 *    The running-sum term is not an integral of a raw piezo voltage: the
 *    charge amplifier already gives force-proportional voltage, and the term
 *    only restores the charge lost through Rf (~2.2 % after a 1.5 s hold).
 *    Self-check after every squeeze: F_total returns to 0 within +/-0.5 % of
 *    its peak; negative = tau set too large, positive = too small.
 *
 *  CALIBRATION (spec section 8), guided over the serial monitor, per piezo:
 *    conditioning -> tau (no reference needed) -> k against a scale, then
 *    timing offsets (three chips under one plate) and a verification tool.
 *    Skipping it uses the STANDARD parameters k = Cf / 17 nC/N = 400 N/V,
 *    tau = 68 s and zero offsets (absolute force then within about
 *    -25..+30 %).  Sketches on the UNO Q have no EEPROM, so the calibration is
 *    printed as one "CAL1 ..." line to copy and paste back after power-up.
 *
 *  UNO Q NOTES
 *    - The serial monitor is the Bridge "Monitor" object (Arduino_RouterBridge
 *      library); plain Serial is the D0/D1 UART.  Every Monitor call is an RPC
 *      to the Linux side, so sampling runs in its own high-priority Zephyr
 *      thread and text goes out through a queue in chunks: serial traffic
 *      never delays or skips a sample.
 *    - analogRead() returns a negative error code on failure; such samples are
 *      counted and replaced by the previous good value.
 *    - Any serial monitor line-ending setting works (Newline recommended).
 * ========================================================================== */

#include <Arduino.h>
#include <Arduino_RouterBridge.h>
#include <zephyr/kernel.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* ================================ CONFIG ================================== */

#define NCH 3
static const uint8_t PIN[NCH] = { A0, A1, A2 };
static const char*   CHN[NCH] = { "CH1/A0", "CH2/A1", "CH3/A2" };

// Sampling and ADC
static const uint32_t FS_HZ      = 2000;      // spec 7: fs = 2 kHz
static const uint8_t  ADCRES_BITS   = 14;        // the UNO Q ADC is natively 14-bit
static const float    ADCREF_V = 3.300f;    // ADC reference = UNO Q 3.3 V rail
static const int      BTN_PIN    = 2;         // reset button to GND; -1 = none

// Charge amplifier and sensor (spec 3 and 4.1)
static const double   CF_F       = 6.8e-6;    // Cf, 6.8 uF polyester film
static const double   RF_OHM     = 10.0e6;    // Rf, 10 Mohm
static const double   S_STD_CN   = 17.0e-9;   // standard sensitivity, C/N (13..22 nC/N)
static const float    CHIP_MAX_N = 270.0f;    // 30 MPa constant-force limit, 3 x 3 mm chip
static const float    CHIP_OVL_N = 324.0f;    // "well over" 270 N: recheck that chip (spec 8)

// Event detection (times in microseconds)
static const float    THR_DEFAULT_N   = 5.0f;     // onset threshold per chip (cmd: thr)
static const uint32_t BASE_WIN_US     = 250000;   // baseline = mean over 0.25 s (spec: >= 0.2 s)
static const uint32_t BASE_GUARD_US   = 20000;    // ... ending 20 ms before the force started
static const float    QUIET_FRAC      = 0.2f;     // "force started" = above 0.2 x threshold
static const float    TRACK_SLEW_VPS  = 0.010f;   // trigger baseline follows drift <= 10 mV/s, not presses
static const float    RELEASE_FRAC    = 0.05f;    // released below 5 % of the peak
static const uint32_t POST_US         = 300000;   // keep recording 0.3 s after release
static const uint32_t RESID_WIN_US    = 100000;   // residual = mean over the last 0.1 s
static const uint32_t CAP_TIMEOUT_US      = 10000000; // longest squeeze captured
static const uint32_t SMALL_HOLD_US   = 200000;   // small event: back to rest for 0.2 s
static const float    SQUEEZE_SPEC_S  = 1.5f;     // design squeeze length (spec 1)
static const float    CLIP_HI_V       = 3.20f;    // top of the usable output swing
static const float    CLIP_LO_V       = 0.02f;    // output pinned at 0 V
static const float    BALANCE_TOL_PCT = 3.0f;     // balanced if each share is 33.3 +/- 3
static const uint32_t AUTOSKIP_MS     = 60000;    // start-up question auto-skips
static const float    EMA_ALPHA       = 1.0f / 128.0f;  // trigger baseline, ~64 ms

// Calibration (spec 8)
static const float    FIT_PASS_PCT     = 2.0f;    // every point within 2 % of the line
static const float    REPEAT_PASS_PCT  = 2.0f;    // repeats agree within 1-2 %
static const float    RESID_PASS_PCT   = 0.5f;    // back to zero within 0.5 % of peak
static const float    VERIFY_OK_PCT    = 2.0f;
static const float    VERIFY_RECAL_PCT = 3.0f;    // more than 3 % off: recalibrate
static const float    CAL_MIN_N        = 20.0f;   // lighter calibration presses ignored
static const float    PL_STEADY_PCT    = 3.0f;    // plateau steadiness needed for a point
static const uint32_t TAU_W0_US        = 500000;  // tau: average 0.5 s ...
static const uint32_t TAU_W1_US        = 1500000; // ... to 1.5 s after release
static const float    OFFSET_WARN_US   = 500.0f;  // spec: ~100 us electronics mismatch
static const float    KLEVELS[5]       = { 50.0f, 100.0f, 150.0f, 200.0f, 270.0f };

// Threads and buffers
static const int      SAMPLER_PRIO  = 2;          // above Bridge (5) and loop() (14)
static const size_t   SAMPLER_STACK = 2048;
static const uint32_t QN            = 1024;       // sample queue, ~0.5 s (power of 2)
static const uint32_t HN            = 2048;       // pre-trigger history, ~1 s (power of 2)

// Derived
static const uint32_t ADCFULL   = (1UL << ADCRES_BITS) - 1UL;   // 16383
static const float    V_PER_CNT = ADCREF_V / (float)ADCFULL;
static const float    TAU_STD   = (float)(RF_OHM * CF_F);    // 68 s
static const float    STD_K     = (float)(CF_F / S_STD_CN);  // 400 N/V
static const float    N_PER_LBF = 4.4482216f;
static const float    N_PER_KGF = 9.80665f;

/* ================================= TYPES ================================== */
// Every type is defined here, before the first function.  The Arduino build
// generates a prototype for each function of a .ino file and inserts them all
// just above the first function definition, so a type used in any function
// signature must already be defined there (otherwise: "'X' does not name a
// type").  Keep new types in this section.

// one scan of the three channels (written by the sampler thread)
struct Sample {
  uint32_t t0;            // microseconds, end of channel 0's conversion
  uint16_t dt[NCH - 1];   // microseconds after t0 for channels 1 and 2
  int16_t  raw[NCH];      // ADC counts, -1 = conversion error
};

// calibration of the three chip/channel pairs
struct Cal {
  float k[NCH];        // N/V                F = k*X + q*X*|X|,  X = corrected dV
  float q[NCH];        // N/V^2              0 = linear (spec: quadratic if curved)
  float b[NCH];        // N, fit intercept   (record only; should be near 0)
  float dev[NCH];      // %, worst fit deviation (record only; -1 = none)
  float tau[NCH];      // s, leak time constant Rf*Cf per channel
  float off[NCH];      // us, timing offset subtracted from each onset
  bool  kCal[NCH], tauCal[NCH], offCal;
  char  note[40];
};

// capture engine: one press or squeeze
enum CapPurpose : uint8_t { CP_MEASURE, CP_COND, CP_TAU, CP_KPOINT, CP_VERIFY, CP_OFFSET };
enum EndReason  : uint8_t { END_NONE, END_RELEASED, END_TIMEOUT, END_SMALL };

struct CapCfg {
  CapPurpose purpose;
  int8_t     focus;          // chip index, or -1 = all three (F_total)
  uint8_t    trigMask;       // channels allowed to trigger
  uint32_t   postUs, timeoutUs;
  float      minEvent;       // N; lighter events count as "small"
  bool       quiet;          // don't announce "ready" again
};

struct CapArm {
  bool     init, warm, warmEvent;
  uint32_t t0, tLast;
  float    trk[NCH];           // slew-limited resting level (trigger only)
  uint32_t tQuiet[NCH];        // last time each channel carried no force
  uint8_t  negWarned, negEvent;
  float    negF[NCH];
};

struct Blk { float x[NCH], f[NCH], ftot; uint32_t tEnd; };
static const uint8_t  NBLK   = 16;           // 16 x 50 ms
static const uint32_t BLK_US = 50000;
static const uint32_t PL_GUARD_US = 100000;  // plateau ends 0.1 s before release ...
static const uint32_t PL_WIN_US   = 500000;  // ... and is 0.5 s long

struct CapRun {
  uint32_t tStart;                 // us; onset times are relative to this
  float    base[NCH];              // V, pre-closure mean
  double   I[NCH];                 // V*s, integral of (V - V_eq) (leak correction)
  float    dV[NCH], X[NCH], F[NCH], Ftot;
  bool     havePrev;
  uint32_t tPrev[NCH];
  float    Fprev[NCH];
  bool     on[NCH];
  float    tOn[NCH];               // us after tStart, interpolated
  float    sigV[NCH];              // V, baseline noise (from the pre-closure window)
  float    sigT[NCH];              // us, onset timing noise = noise / slope at crossing
  float    sgMax;                  // peak of the focus signal (F_i or F_total)
  float    rawMax;                 // same without leak correction (event detection)
  float    peakTot, tPeakTot, FatPk[NCH];
  float    peakF[NCH], peakX[NCH];
  bool     clipHi[NCH], clipLo[NCH];
  uint32_t nSmp, gaps, drops0, ovr0, err0;
  bool     t90Set;
  Blk      blk[NBLK];
  uint8_t  blkHead, blkCount;
  uint32_t blkT0;
  double   bx[NCH], bf[NCH], bt;
  uint16_t bn;
  bool     plOk;
  uint8_t  plN;
  float    plX[NCH], plF[NCH], plFtot, plSpread;
  bool     rel, belowSet;
  uint32_t tRel, tBelow;
  double   rSum;
  uint32_t rN;
  double   tI[NCH], tDV[NCH], tX[NCH];
  uint32_t tN;
  uint8_t  endReason;
};

struct Cap { CapCfg cfg; CapArm arm; CapRun run; bool armed, active, done; };

// user-interface state machines
enum Mode   : uint8_t { MODE_SELFTEST, MODE_STARTUP, MODE_MEASURE, MODE_CAL };
enum MState : uint8_t { M_ARMED, M_DONE };
enum CState : uint8_t {
  C_MENU, C_COND, C_TAU_PRESS, C_TAU_DECIDE, C_K_PRESS, C_K_REF, C_K_FIT,
  C_OFF_PRESS, C_OFF_DECIDE, C_VER_SELECT, C_VER_PRESS, C_VER_REF
};

// calibration wizard
struct KPt { float x, f; };
struct FitRes { bool ok; float k, b, worst, worstUse, rep, qk, qq, qWorst; uint8_t levels; bool qOk; };

/* ============================ SMALL HELPERS =============================== */

static inline float absf(float v) { return v < 0 ? -v : v; }
static inline float maxf(float a, float b) { return a > b ? a : b; }
static inline char  lc(char ch) { return (ch >= 'A' && ch <= 'Z') ? (char)(ch + 32) : ch; }

static bool eqCI(const char* a, const char* b) {
  while (*a && *b) { if (lc(*a) != lc(*b)) return false; a++; b++; }
  return *a == 0 && *b == 0;
}
static bool preCI(const char* s, const char* pre) {
  while (*pre) { if (lc(*s) != lc(*pre)) return false; s++; pre++; }
  return true;
}
// first word of the line equals w (case-insensitive)
static bool wordIs(const char* ln, const char* w) {
  while (*w) { if (lc(*ln) != lc(*w)) return false; ln++; w++; }
  return *ln == 0 || *ln == ' ';
}
// text after the first word
static const char* argOf(const char* ln) {
  while (*ln && *ln != ' ') ln++;
  while (*ln == ' ') ln++;
  return ln;
}
static bool parseNum(const char* s, float& out, const char** rest) {
  char* end;
  double v = strtod(s, &end);
  if (end == s || v != v) return false;
  out = (float)v;
  if (rest) *rest = end;
  return true;
}
// "101", "101 N", "10.3kg", "22.7 lb" -> newtons
static bool parseForce(const char* s, float& n) {
  const char* rest; float v;
  if (!parseNum(s, v, &rest)) return false;
  while (*rest == ' ') rest++;
  if (*rest == 0 || eqCI(rest, "n")) n = v;
  else if (eqCI(rest, "kg") || eqCI(rest, "kgf")) n = v * N_PER_KGF;
  else if (eqCI(rest, "lb") || eqCI(rest, "lbs") || eqCI(rest, "lbf")) n = v * N_PER_LBF;
  else return false;
  return n > 0.0f && n < 20000.0f;
}
// "1.5,2,3" -> up to maxN floats; returns how many were read
static uint8_t parseList(const char* s, float* out, uint8_t maxN) {
  uint8_t n = 0;
  while (*s && n < maxN) {
    const char* rest;
    if (!parseNum(s, out[n], &rest)) return 0;
    n++;
    s = rest;
    if (*s == ',') s++;
    else break;
  }
  return n;
}

/* ================================ OUTPUT ================================== */
// Monitor text is queued here and sent in chunks (one RPC each) from loop(),
// so a long report never stops sample processing for longer than one chunk.

static const uint16_t OUT_N = 8192, OUT_CHUNK = 200;
static char     gOut[OUT_N];
static uint16_t gOutHead = 0, gOutTail = 0, gOutUsed = 0;

static void pumpOutput() {
  if (gOutUsed == 0) return;
  uint8_t  buf[OUT_CHUNK];
  uint16_t n = 0;
  while (n < OUT_CHUNK && gOutUsed > 0) {
    buf[n++]  = (uint8_t)gOut[gOutTail];
    gOutTail  = (uint16_t)((gOutTail + 1) % OUT_N);
    gOutUsed--;
  }
  Monitor.write(buf, n);              // dropped if no monitor is attached
}

static void outRaw(const char* s, uint16_t n) {
  while ((uint16_t)(OUT_N - gOutUsed) < n) pumpOutput();   // queue full: send now
  for (uint16_t i = 0; i < n; i++) {
    gOut[gOutHead] = s[i];
    gOutHead = (uint16_t)((gOutHead + 1) % OUT_N);
  }
  gOutUsed = (uint16_t)(gOutUsed + n);
}

// One output line:  Line().s("F = ").f(12.345, 1).s(" N").end();
class Line {
 public:
  Line& s(const char* t) { while (*t) put(*t++); return *this; }
  Line& c(char ch) { put(ch); return *this; }
  Line& u(unsigned long v) {
    char t[11]; uint8_t k = 0;
    do { t[k++] = (char)('0' + v % 10UL); v /= 10UL; } while (v && k < sizeof(t));
    while (k) put(t[--k]);
    return *this;
  }
  Line& i(long v) {
    if (v < 0) { put('-'); return u(0UL - (unsigned long)v); }
    return u((unsigned long)v);
  }
  Line& f(double v, uint8_t dec);
  Line& sf(double v, uint8_t dec) {                   // with sign; "+0.00" for rounded zero
    double half = 0.5;
    for (uint8_t k = 0; k < dec; k++) half *= 0.1;
    if (v > -half) put('+');
    return f(v, dec);
  }
  Line& pad(uint8_t col) { while (n < col && n < LINE_N - 2) put(' '); return *this; }
  void  end() { b[n++] = '\r'; b[n++] = '\n'; outRaw(b, n); n = 0; }
 private:
  static const uint16_t LINE_N = 320;          // fits the longest CAL1 line (~255 chars)
  void put(char ch) { if (n < LINE_N - 2) b[n++] = ch; }
  char     b[LINE_N];
  uint16_t n = 0;
};

Line& Line::f(double v, uint8_t dec) {
  if (v != v) return s("nan");
  if (dec > 6) dec = 6;
  double scale = 1.0;
  for (uint8_t k = 0; k < dec; k++) scale *= 10.0;
  double a = (v < 0 ? -v : v) * scale + 0.5;
  if (a >= 1.0e18) return s(v < 0 ? "-ovf" : "ovf");
  uint64_t r = (uint64_t)a, sc = (uint64_t)scale;
  uint64_t ip = r / sc, fp = r % sc;
  if (v < 0 && r != 0) put('-');
  char t[20]; uint8_t k = 0;
  do { t[k++] = (char)('0' + (int)(ip % 10)); ip /= 10; } while (ip && k < sizeof(t));
  while (k) put(t[--k]);
  if (dec) {
    char d[6];
    put('.');
    for (int8_t j = (int8_t)dec - 1; j >= 0; j--) { d[j] = (char)('0' + (int)(fp % 10)); fp /= 10; }
    for (uint8_t j = 0; j < dec; j++) put(d[j]);
  }
  return *this;
}

static void rule(char ch = '-') {
  Line l;
  for (uint8_t k = 0; k < 66; k++) l.c(ch);
  l.end();
}

/* ================================ INPUT =================================== */

static const uint16_t IN_N = 320;     // fits a pasted CAL1 line
static char     gIn[IN_N];
static uint16_t gInLen = 0;
static bool     gInCR = false;
static uint32_t gInLastMs = 0, gPollMs = 0;
static void onLine(char* ln);

static void dispatchLine() {
  gIn[gInLen] = 0;
  gInLen = 0;
  char* p = gIn;
  while (*p == ' ' || *p == '\t') p++;
  char* e = p + strlen(p);
  while (e > p && (e[-1] == ' ' || e[-1] == '\t')) *--e = 0;
  onLine(p);
}

static void feedChar(char ch) {
  if (ch == '\n' && gInCR) { gInCR = false; return; }   // "\r\n" ends one line
  gInCR = (ch == '\r');
  if (ch == '\r' || ch == '\n') { dispatchLine(); return; }
  if (ch >= 32 && ch < 127 && gInLen < IN_N - 1) gIn[gInLen++] = ch;
  gInLastMs = millis();
}

static void pollInput() {
  uint32_t now = millis();
  if ((uint32_t)(now - gPollMs) < 25) return;     // each available() is an RPC
  gPollMs = now;
  int n = Monitor.available();
  while (n-- > 0) {
    int ch = Monitor.read();
    if (ch < 0) break;
    feedChar((char)ch);
  }
  // monitor set to "No line ending": a 0.4 s pause ends the line
  if (gInLen > 0 && (uint32_t)(millis() - gInLastMs) > 400) dispatchLine();
}

/* =============================== SAMPLER ================================== */
// A Zephyr thread (priority 2) reads A0..A2 every 1/FS_HZ, stamps each channel
// with a microsecond time, and pushes the scan into a lock-free queue that
// loop() drains.  If the thread cannot be created the same scan runs from
// loop() instead (slower serial output then causes gaps; reported).

static inline uint32_t sT(const Sample& s, uint8_t i) { return i == 0 ? s.t0 : s.t0 + s.dt[i - 1]; }

static Sample   gQ[QN];
static uint32_t gQHead = 0;                 // written by the sampler only
static uint32_t gQTail = 0;                 // written by loop() only
static volatile uint32_t gDrops = 0, gOverruns = 0, gAdcErrors = 0;
static uint32_t gHz = 0;                    // cycle counter frequency
static uint32_t gPeriodUs = 1000000UL / FS_HZ;
static bool     gThreaded = false;
static uint32_t gScanUs = 0;                // measured duration of one 3-channel scan
static uint32_t gTickNs = 0;                // measured cycle-counter resolution

// Microsecond time base built from 32-bit cycle-counter deltas, so it wraps
// cleanly at 2^32 us whatever the counter width or frequency.  Called from
// the sampling context only.
static uint32_t gTbLastCyc = 0;
static uint64_t gTbUs = 0, gTbRem = 0;

static inline uint32_t tbNowUs() {
  uint32_t c   = k_cycle_get_32();
  uint64_t num = (uint64_t)(uint32_t)(c - gTbLastCyc) * 1000000ULL + gTbRem;
  gTbLastCyc   = c;
  gTbUs       += num / gHz;
  gTbRem       = num % gHz;
  return (uint32_t)gTbUs;
}

static void scanOnce() {
  Sample s;
  uint32_t t[NCH];
  for (uint8_t i = 0; i < NCH; i++) {
    int v = analogRead(PIN[i]);
    t[i] = tbNowUs();
    if (v < 0 || v > (int)ADCFULL) { s.raw[i] = -1; gAdcErrors++; }
    else s.raw[i] = (int16_t)v;
  }
  s.t0 = t[0];
  for (uint8_t i = 1; i < NCH; i++) {
    uint32_t d = t[i] - t[0];
    s.dt[i - 1] = d > 65535UL ? (uint16_t)65535 : (uint16_t)d;
  }
  uint32_t head = gQHead;
  uint32_t tail = __atomic_load_n(&gQTail, __ATOMIC_ACQUIRE);
  if ((uint32_t)(head - tail) >= QN) { gDrops++; return; }   // loop() fell behind
  gQ[head & (QN - 1)] = s;
  __atomic_store_n(&gQHead, head + 1, __ATOMIC_RELEASE);
}

static bool popSample(Sample& s) {
  uint32_t tail = gQTail;
  uint32_t head = __atomic_load_n(&gQHead, __ATOMIC_ACQUIRE);
  if (tail == head) return false;
  s = gQ[tail & (QN - 1)];
  __atomic_store_n(&gQTail, tail + 1, __ATOMIC_RELEASE);
  return true;
}

static struct k_timer  gTimer;
static struct k_thread gThread;

static void samplerEntry(void*, void*, void*) {
  k_timer_start(&gTimer, K_USEC(gPeriodUs), K_USEC(gPeriodUs));
  for (;;) {
    uint32_t n = k_timer_status_sync(&gTimer);    // waits for the next tick
    if (n > 1) gOverruns += n - 1;                // a scan took longer than 1/fs
    scanOnce();
  }
}

static bool startSamplerThread() {
  k_timer_init(&gTimer, NULL, NULL);
  k_thread_stack_t* stk = k_thread_stack_alloc(SAMPLER_STACK, 0);
  if (stk == NULL) return false;
  k_tid_t tid = k_thread_create(&gThread, stk, SAMPLER_STACK, samplerEntry,
                                NULL, NULL, NULL, SAMPLER_PRIO, 0, K_NO_WAIT);
  return tid != NULL;
}

static uint32_t gNextUs = 0;
static void samplerInlinePoll() {                 // fallback: sample from loop()
  uint32_t now = tbNowUs();
  if ((int32_t)(now - gNextUs) < 0) return;
  uint32_t late = now - gNextUs;
  if (late >= gPeriodUs) { gOverruns += late / gPeriodUs; gNextUs = now + gPeriodUs; }
  else gNextUs += gPeriodUs;
  scanOnce();
}

/* ======================= HISTORY AND IDLE STATISTICS ====================== */

static Sample   gH[HN];                     // last HN processed samples
static uint32_t gHCount = 0;
static int16_t  gLastGood[NCH] = { 0, 0, 0 };

static inline float toV(int16_t raw) { return (float)raw * V_PER_CNT; }
static void histPush(const Sample& s) { gH[gHCount & (HN - 1)] = s; gHCount++; }
static uint32_t histSize() { return gHCount < HN ? gHCount : HN; }
static const Sample& histBack(uint32_t k) { return gH[(gHCount - 1 - k) & (HN - 1)]; }

static float histMeanV(uint8_t ch, uint32_t n) {
  uint32_t m = histSize();
  if (n > m) n = m;
  if (n == 0) return 0.0f;
  double acc = 0;
  for (uint32_t k = 0; k < n; k++) acc += toV(histBack(k).raw[ch]);
  return (float)(acc / n);
}

// Equilibrium level V_eq of each amplifier output: the level at which no
// current flows through Rf (VREF plus leakage x Rf).  Leakage through Rf is
// (V - V_eq) / Rf, so the leak term integrates V - V_eq.  When the output has
// settled the pre-closure baseline equals V_eq and this is exactly the spec's
// formula; right after earlier squeezes (the output recovers from their
// undershoot with tau, spec 5) it keeps the result exact instead of drifting.
// V_eq is taken at the self-test and refined while idle: at rest the output
// relaxes as V(t) = V_eq + A * exp(-t/tau), which is linear in V_eq and A,
// so a least-squares fit over the last 8..16 one-second means gives V_eq.
static float    gVeq[NCH];
static uint32_t gVeqMs = 0;            // millis() of the last refinement; 0 = self-test value

static const uint8_t IDLE_N = 16;      // one-second means of the idle output
static float    gIdle[IDLE_N][NCH];
static uint8_t  gIdleHead = 0, gIdleCount = 0;
static double   gIdleAcc[NCH];
static uint32_t gIdleN = 0, gIdleT0 = 0;
static float    calTau(uint8_t i);

static void idleReset() { gIdleCount = 0; gIdleN = 0; for (uint8_t i = 0; i < NCH; i++) gIdleAcc[i] = 0; }

static void veqRefine() {
  uint8_t n = gIdleCount;
  if (n < 8) return;
  bool any = false;
  for (uint8_t i = 0; i < NCH; i++) {
    float  y[IDLE_N];
    double z[IDLE_N], sz = 0, sy = 0, szz = 0, szy = 0;
    for (uint8_t j = 0; j < n; j++) {
      y[j] = gIdle[(gIdleHead + IDLE_N - n + j) % IDLE_N][i];
      z[j] = exp((double)(n - 1 - j) / calTau(i));            // newest second: z = 1
      sz += z[j]; sy += y[j]; szz += z[j] * z[j]; szy += z[j] * y[j];
    }
    double den = n * szz - sz * sz;
    if (den <= 0) continue;
    double A   = (n * szy - sz * sy) / den;                   // what is left to recover now
    double veq = (sy - A * sz) / n;
    double ss = 0;
    for (uint8_t j = 0; j < n; j++) { double e = y[j] - (veq + A * z[j]); ss += e * e; }
    if (sqrtf((float)(ss / n)) > 0.0003f) continue;           // touched or disturbed: skip
    if (fabs(A) > 0.030) continue;                            // still far from settled: wait
    gVeq[i] += 0.5f * ((float)veq - gVeq[i]);
    any = true;
  }
  if (any) { gVeqMs = millis(); if (gVeqMs == 0) gVeqMs = 1; }
}

static void idleFeed(const Sample& s) {
  if (gIdleN == 0) gIdleT0 = s.t0;
  for (uint8_t i = 0; i < NCH; i++) gIdleAcc[i] += toV(s.raw[i]);
  gIdleN++;
  if ((uint32_t)(s.t0 - gIdleT0) >= 1000000UL) {
    for (uint8_t i = 0; i < NCH; i++) { gIdle[gIdleHead][i] = (float)(gIdleAcc[i] / gIdleN); gIdleAcc[i] = 0; }
    gIdleHead = (uint8_t)((gIdleHead + 1) % IDLE_N);
    if (gIdleCount < IDLE_N) gIdleCount++;
    gIdleN = 0;
    veqRefine();
  }
}

/* ============================ CALIBRATION MODEL =========================== */

static Cal   gCal;
static float gThrN = THR_DEFAULT_N;
static bool  gAuto = false;
static float gJawL = 0, gJawW = 0;     // mm, for pressure; 0 = off

static void calDefaults(Cal& c) {
  for (uint8_t i = 0; i < NCH; i++) {
    c.k[i] = STD_K; c.q[i] = 0; c.b[i] = 0; c.dev[i] = -1;
    c.tau[i] = TAU_STD; c.off[i] = 0;
    c.kCal[i] = false; c.tauCal[i] = false;
  }
  c.offCal = false;
  c.note[0] = 0;
}

static inline float forceFromX(uint8_t i, float x) {
  return gCal.k[i] * x + gCal.q[i] * x * absf(x);
}
static float calTau(uint8_t i) { return gCal.tau[i]; }

static bool anyCalibrated() {
  for (uint8_t i = 0; i < NCH; i++) if (gCal.kCal[i] || gCal.tauCal[i]) return true;
  return gCal.offCal;
}

/* ============================ CAPTURE ENGINE ============================== */
// One engine handles every press: measurement squeezes and calibration
// presses alike.  While armed it follows the resting level with a slew-limited
// tracker (it follows slow settling but not a press, so even slow ramps
// trigger) and notes when each channel last carried no force.  At the trigger
// the baseline is the mean of the 0.25 s before the force started (spec 7),
// the samples since then are replayed, and the force equation runs on every
// sample until release + 0.3 s.

static Cap gCap;

static void capArm(CapPurpose p, int8_t focus, bool quiet) {
  gCap.cfg.purpose   = p;
  gCap.cfg.focus     = focus;
  gCap.cfg.trigMask  = (focus < 0) ? (uint8_t)0x07 : (uint8_t)(1u << focus);
  gCap.cfg.postUs    = (p == CP_TAU) ? (TAU_W1_US + 50000UL) : POST_US;
  gCap.cfg.timeoutUs = (p == CP_MEASURE || p == CP_OFFSET) ? CAP_TIMEOUT_US
                     : (p == CP_TAU ? 40000000UL : 20000000UL);
  gCap.cfg.minEvent  = (p == CP_MEASURE || p == CP_OFFSET) ? 3.0f * gThrN : CAL_MIN_N;
  gCap.cfg.quiet     = quiet;
  gCap.arm    = CapArm();
  gCap.run    = CapRun();
  gCap.armed  = true;
  gCap.active = false;
  gCap.done   = false;
}

static void capCancel() { gCap.armed = gCap.active = gCap.done = false; }

static void capFinish(uint8_t why) {
  gCap.run.endReason = why;
  gCap.active = false;
  gCap.armed  = false;
  gCap.done   = true;
}

static float blkFocus(const Blk& b) { return gCap.cfg.focus < 0 ? b.ftot : b.f[gCap.cfg.focus]; }

// Plateau = mean of the 50 ms blocks from 0.6 s to 0.1 s before tEnd
// (the end of the hold); its spread tells how steady the hold was.
static void capPlateau(uint32_t tEnd) {
  CapRun& r = gCap.run;
  double sx[NCH] = { 0, 0, 0 }, sf[NCH] = { 0, 0, 0 }, st = 0;
  float lo = 1e30f, hi = -1e30f;
  uint8_t n = 0;
  for (uint8_t j = 0; j < r.blkCount; j++) {
    const Blk& b = r.blk[(uint8_t)(r.blkHead + NBLK - 1 - j) % NBLK];
    uint32_t age = tEnd - b.tEnd;
    if (age < PL_GUARD_US) continue;
    if (age > PL_GUARD_US + PL_WIN_US) break;
    for (uint8_t i = 0; i < NCH; i++) { sx[i] += b.x[i]; sf[i] += b.f[i]; }
    st += b.ftot;
    float g = blkFocus(b);
    if (g < lo) lo = g;
    if (g > hi) hi = g;
    n++;
  }
  r.plN = n;
  if (n == 0) { r.plOk = false; return; }
  for (uint8_t i = 0; i < NCH; i++) { r.plX[i] = (float)(sx[i] / n); r.plF[i] = (float)(sf[i] / n); }
  r.plFtot = (float)(st / n);
  float m = gCap.cfg.focus < 0 ? r.plFtot : r.plF[gCap.cfg.focus];
  r.plSpread = (m > 0) ? 100.0f * (hi - lo) / m : 999.0f;
  r.plOk = (n >= 4) && (m >= gCap.cfg.minEvent) && (r.plSpread <= PL_STEADY_PCT);
}

// The force equation (spec 7) on one sample, plus onsets, peaks, release.
static void capStep(const Sample& s) {
  CapRun& r = gCap.run;
  const CapCfg& cf = gCap.cfg;
  if (r.nSmp == 0) { r.tStart = s.t0; r.blkT0 = s.t0; }
  else if ((uint32_t)(s.t0 - r.tPrev[0]) > 3 * gPeriodUs) r.gaps++;

  float ftot = 0.0f, fr[NCH], frTot = 0.0f;
  for (uint8_t i = 0; i < NCH; i++) {
    uint32_t ti = sT(s, i);
    float v = toV(s.raw[i]);
    if (v >= CLIP_HI_V) r.clipHi[i] = true;
    if (v <= CLIP_LO_V) r.clipLo[i] = true;
    float dv = v - r.base[i];                                       // dV_i[n]
    fr[i] = forceFromX(i, dv);                                      // uncorrected, for events
    frTot += fr[i];
    if (r.havePrev) r.I[i] += (double)(v - gVeq[i]) * (double)(uint32_t)(ti - r.tPrev[i]) * 1e-6;
    float x = dv + (float)(r.I[i] / (double)gCal.tau[i]);           // dV + (1/tau)*integral
    float F = forceFromX(i, x);                                     // F_i[n]
    if (!r.on[i] && F >= gThrN) {                                   // onset, interpolated
      float tc = (float)(uint32_t)(ti - r.tStart);
      r.sigT[i] = 1000.0f;                                          // unknown slope: 1 ms
      if (r.havePrev && r.Fprev[i] < gThrN && F > r.Fprev[i]) {
        float tp = (float)(uint32_t)(r.tPrev[i] - r.tStart);
        float slope = (F - r.Fprev[i]) / maxf(tc - tp, 1.0f);       // N per us over the sample step
        tc = tp + (gThrN - r.Fprev[i]) / (F - r.Fprev[i]) * (tc - tp);
        float sigF  = gCal.k[i] * r.sigV[i];
        // an uncalibrated chip may be 30 % more or less sensitive, so its
        // threshold sits at a different true force: that shifts the crossing
        float kU    = gCal.kCal[i] ? 0.0f : 0.3f * gThrN;
        r.sigT[i] = slope > 0 ? sqrtf(sigF * sigF + kU * kU) / slope : 1000.0f;
      }
      r.on[i] = true;
      r.tOn[i] = tc;
    }
    if (F > r.peakF[i]) r.peakF[i] = F;
    if (x > r.peakX[i]) r.peakX[i] = x;
    r.dV[i] = dv; r.X[i] = x; r.F[i] = F;
    r.Fprev[i] = F; r.tPrev[i] = ti;
    ftot += F;
  }
  r.Ftot = ftot;                                                    // F_total[n]
  r.havePrev = true;
  r.nSmp++;
  uint32_t t = s.t0;
  if (ftot > r.peakTot) {                                           // clamp force + shares
    r.peakTot = ftot;
    r.tPeakTot = (float)(uint32_t)(t - r.tStart);
    for (uint8_t i = 0; i < NCH; i++) r.FatPk[i] = r.F[i];
  }

  // 50 ms block means (for the plateau of calibration presses)
  for (uint8_t i = 0; i < NCH; i++) { r.bx[i] += r.X[i]; r.bf[i] += r.F[i]; }
  r.bt += ftot;
  r.bn++;
  if ((uint32_t)(t - r.blkT0) >= BLK_US) {
    Blk& b = r.blk[r.blkHead];
    for (uint8_t i = 0; i < NCH; i++) {
      b.x[i] = (float)(r.bx[i] / r.bn); b.f[i] = (float)(r.bf[i] / r.bn); r.bx[i] = r.bf[i] = 0;
    }
    b.ftot = (float)(r.bt / r.bn);
    b.tEnd = t;
    r.bt = 0; r.bn = 0; r.blkT0 = t;
    r.blkHead = (uint8_t)((r.blkHead + 1) % NBLK);
    if (r.blkCount < NBLK) r.blkCount++;
  }

  // End of the hold (for the plateau) is judged on the corrected force.
  // Release is judged on the uncorrected one: it always undershoots on
  // release, even when tau is set far off (e.g. while calibrating tau).
  float sgC = cf.focus < 0 ? ftot  : r.F[cf.focus];
  float sgR = cf.focus < 0 ? frTot : fr[cf.focus];
  if (sgC > r.sgMax) { r.sgMax = sgC; r.t90Set = false; }
  else if (!r.t90Set && r.sgMax >= cf.minEvent && sgC < 0.9f * r.sgMax) { r.t90Set = true; capPlateau(t); }
  if (sgR > r.rawMax) r.rawMax = sgR;

  if (r.rawMax < cf.minEvent) {                      // a bump that never got big
    if (sgR < 0.5f * gThrN) {
      if (!r.belowSet) { r.belowSet = true; r.tBelow = t; }
      else if ((uint32_t)(t - r.tBelow) >= SMALL_HOLD_US) { capFinish(END_SMALL); return; }
    } else r.belowSet = false;
  } else {
    float relLvl = maxf(RELEASE_FRAC * r.rawMax, 0.5f * gThrN);
    if (!r.rel) {
      if (sgR < relLvl) {
        r.rel = true; r.tRel = t; r.rSum = 0; r.rN = 0; r.tN = 0;
        for (uint8_t i = 0; i < NCH; i++) r.tI[i] = r.tDV[i] = r.tX[i] = 0;
        if (!r.t90Set) { r.t90Set = true; capPlateau(t); }
      }
    } else if (sgR > 3.0f * relLvl) {
      r.rel = false;                                 // pressed again: keep going
    } else {
      uint32_t since = t - r.tRel;
      if (since + RESID_WIN_US >= cf.postUs) { r.rSum += sgC; r.rN++; }   // self-check
      if (since >= TAU_W0_US && since <= TAU_W1_US) {
        for (uint8_t i = 0; i < NCH; i++) { r.tI[i] += r.I[i]; r.tDV[i] += r.dV[i]; r.tX[i] += r.X[i]; }
        r.tN++;
      }
      if (since >= cf.postUs) { capFinish(END_RELEASED); return; }
    }
  }
  if ((uint32_t)(t - r.tStart) >= cf.timeoutUs) {
    if (!r.t90Set) capPlateau(t);
    capFinish(END_TIMEOUT);
  }
}

// Trigger: baseline = mean of the BASE_WIN_US ending BASE_GUARD_US before the
// force started, then replay every sample from there on.
static void capStart(const Sample& trig) {
  CapRun& r = gCap.run;
  CapArm& a = gCap.arm;
  uint32_t tq = trig.t0;                              // when the force started
  for (uint8_t i = 0; i < NCH; i++)
    if ((gCap.cfg.trigMask & (1u << i)) && (int32_t)(a.tQuiet[i] - tq) < 0) tq = a.tQuiet[i];
  uint32_t tEnd = tq - BASE_GUARD_US;
  double acc[NCH] = { 0, 0, 0 }, acc2[NCH] = { 0, 0, 0 };
  uint32_t n = 0, m = histSize(), kStart = 0;
  for (uint32_t k = 0; k < m; k++) {
    uint32_t t = histBack(k).t0;
    if ((int32_t)(t - tEnd) > 0) { kStart = k; continue; }   // replay from here
    if ((uint32_t)(tEnd - t) > BASE_WIN_US) break;
    for (uint8_t i = 0; i < NCH; i++) { double v = toV(histBack(k).raw[i]); acc[i] += v; acc2[i] += v * v; }
    n++;
  }
  for (uint8_t i = 0; i < NCH; i++) {
    float b = n >= 50 ? (float)(acc[i] / n) : a.trk[i];
    double var = n >= 50 ? acc2[i] / n - (acc[i] / n) * (acc[i] / n) : 0.0;
    r.sigV[i] = n >= 50 ? sqrtf(var > 0 ? (float)var : 0.0f) : 0.0005f;
    // the window's mean belongs to its centre; carry it to the window end
    // along the natural recovery towards V_eq
    r.base[i] = b + (gVeq[i] - b) * (0.5e-6f * (float)BASE_WIN_US) / gCal.tau[i];
  }
  idleReset();
  r.drops0 = gDrops; r.ovr0 = gOverruns; r.err0 = gAdcErrors;
  gCap.active = true;
  for (uint32_t k = kStart + 1; k-- > 0;) {
    capStep(histBack(k));
    if (!gCap.active) return;
  }
}

static void capFeed(const Sample& s) {
  if (gCap.active) { capStep(s); return; }
  CapArm& a = gCap.arm;
  if (!a.init) {
    for (uint8_t i = 0; i < NCH; i++) { a.trk[i] = histMeanV(i, 64); a.tQuiet[i] = sT(s, i); }
    a.init = true; a.t0 = a.tLast = s.t0;
    return;
  }
  if (!a.warm && (uint32_t)(s.t0 - a.t0) >= BASE_WIN_US + BASE_GUARD_US) { a.warm = true; a.warmEvent = true; }
  float maxStep = TRACK_SLEW_VPS * (float)(uint32_t)(s.t0 - a.tLast) * 1e-6f;
  a.tLast = s.t0;
  bool trig = false;
  float v[NCH];
  for (uint8_t i = 0; i < NCH; i++) {
    v[i] = toV(s.raw[i]);
    float F = forceFromX(i, v[i] - a.trk[i]);
    if (F < QUIET_FRAC * gThrN) a.tQuiet[i] = sT(s, i);
    if (!(gCap.cfg.trigMask & (1u << i))) continue;
    if (a.warm && F >= gThrN) trig = true;
    if (a.warm && F <= -3.0f * gThrN && !(a.negWarned & (1u << i))) {
      a.negWarned |= (uint8_t)(1u << i); a.negEvent |= (uint8_t)(1u << i); a.negF[i] = F;
    }
  }
  if (trig) { capStart(s); return; }
  for (uint8_t i = 0; i < NCH; i++) {
    float d = (v[i] - a.trk[i]) * EMA_ALPHA;
    if (d > maxStep) d = maxStep; else if (d < -maxStep) d = -maxStep;
    a.trk[i] += d;
  }
}

/* ============================= MODES / STATE ============================== */

static Mode     gMode   = MODE_SELFTEST;
static MState   gMState = M_ARMED;
static CState   gCState = C_MENU;
static uint8_t  gCalCh  = 0;
static bool     gPasteNext = false;     // next line is a CAL line
static uint32_t gStartupMs = 0;
static uint32_t gShot = 0;

// self-test accumulators
static uint32_t gStN = 0, gStT0 = 0, gStT1 = 0;
static double   gStMean[NCH], gStM2[NCH];
static const uint32_t SELFTEST_US = 500000;

// live view
static bool     gMon = false;
static uint32_t gMonMs = 0;
static float    gMonBase[NCH];

static void enterMeasure();
static void calMenu();
static void onCaptureDone();

static void calStatusLine() {
  Line l;
  l.s("Calibration: ");
  for (uint8_t i = 0; i < NCH; i++) {
    l.s(CHN[i]).s(" k=").f(gCal.k[i], 1).s(gCal.kCal[i] ? "" : "(std)")
     .s(" tau=").f(gCal.tau[i], 1).s(gCal.tauCal[i] ? "" : "(std)");
    l.s(i < NCH - 1 ? " | " : "");
  }
  l.end();
  Line().s("             timing offsets ").s(gCal.offCal ? "calibrated" : "0 (std)")
        .s(", onset threshold ").f(gThrN, 1).s(" N").end();
}

/* ============================ MEASUREMENT ================================= */

static void measureArm(bool quiet) { capArm(CP_MEASURE, -1, quiet); gMState = M_ARMED; }

static void measureReset() {
  Line().s("RESET done.").end();
  measureArm(false);            // "Waiting for actuation" follows once the baseline is ready
}

static void reportMeasurement() {
  CapRun& r = gCap.run;
  gShot++;
  Line().s("==================== SQUEEZE #").u(gShot).s(" ====================").end();

  // ---- hit order ----
  float   tc[NCH];
  uint8_t ord[NCH], n = 0;
  for (uint8_t i = 0; i < NCH; i++) if (r.on[i]) { tc[i] = r.tOn[i] - gCal.off[i]; ord[n++] = i; }
  for (uint8_t a = 1; a < n; a++)                       // sort by corrected onset
    for (uint8_t b = a; b > 0 && tc[ord[b]] < tc[ord[b - 1]]; b--) { uint8_t t = ord[b]; ord[b] = ord[b - 1]; ord[b - 1] = t; }
  Line().s("Hit order (each chip's force crossing ").f(gThrN, 1).s(" N, ")
        .s(gCal.offCal ? "timing offsets applied):" : "no timing offsets):").end();
  static const char* ORD[3] = { "1st", "2nd", "3rd" };
  float t0 = n ? tc[ord[0]] : 0.0f;
  for (uint8_t k = 0; k < n; k++) {
    uint8_t i = ord[k];
    Line l; l.s("  ").s(ORD[k]).s("  ").s(CHN[i]).s("  ");
    if (k == 0) l.s(" 0.000 ms"); else l.c('+').f((tc[i] - t0) / 1000.0f, 3).s(" ms");
    l.end();
  }
  for (uint8_t i = 0; i < NCH; i++)
    if (!r.on[i]) Line().s("  --   ").s(CHN[i]).s("  never reached ").f(gThrN, 1).s(" N (no contact on that side?)").end();
  // timing uncertainty: onset noise of the two channels (noise / slope at the
  // crossing) plus the residual channel-delay mismatch (spec 5: ~0.1 ms
  // without offset calibration)
  float skew = 0.0f, uSkew = 0.0f;
  if (n >= 2) {
    uint8_t a0 = ord[0], a1 = ord[n - 1];
    float sys = gCal.offCal ? 20.0f : 100.0f;
    skew  = tc[a1] - t0;
    uSkew = sqrtf(r.sigT[a0] * r.sigT[a0] + r.sigT[a1] * r.sigT[a1] + sys * sys);
    Line().s("  Closure skew (first to last): ").f(skew / 1000.0f, 3).s(" ms  +/- ")
          .f(uSkew / 1000.0f, 3).s(" ms").end();
    if (skew <= 2.0f * uSkew)
      Line().s("  -> within the timing uncertainty: effectively simultaneous contact").end();
    else
      Line().s("  -> first contact at ").s(CHN[a0]).s(": that side closes first (sits low)").end();
  }

  // ---- clamp force and shares ----
  float tFirstRaw = 1e30f;
  for (uint8_t i = 0; i < NCH; i++) if (r.on[i] && r.tOn[i] < tFirstRaw) tFirstRaw = r.tOn[i];
  Line l0;
  l0.s("Clamp force (peak of F1+F2+F3): ").f(r.peakTot, 1).s(" N  (").f(r.peakTot / N_PER_LBF, 1).s(" lbf)");
  if (tFirstRaw < 1e29f) l0.s(", reached ").f((r.tPeakTot - tFirstRaw) / 1000.0f, 1).s(" ms after first contact");
  l0.end();
  float maxDev = 0; uint8_t heavy = 0;
  Line().s("Force share at that instant:").end();
  for (uint8_t i = 0; i < NCH; i++) {
    float sh = r.peakTot > 0 ? 100.0f * r.FatPk[i] / r.peakTot : 0.0f;
    if (absf(sh - 100.0f / NCH) > maxDev) maxDev = absf(sh - 100.0f / NCH);
    if (r.FatPk[i] > r.FatPk[heavy]) heavy = i;
    Line().s("  ").s(CHN[i]).s("  ").f(sh, 1).s(" %   (").f(r.FatPk[i], 1).s(" N)").end();
  }
  if (r.peakTot > 0) {
    if (maxDev <= BALANCE_TOL_PCT) Line().s("  -> balanced: every share within 33.3 +/- ").f(BALANCE_TOL_PCT, 0).s(" %").end();
    else Line().s("  -> unbalanced: shares up to ").f(maxDev, 1).s(" points from 33.3 %; ")
               .s(CHN[heavy]).s(" carries the most").end();
  }
  Line().s("Per-chip peaks (information only, never add them): ")
        .s("CH1 ").f(r.peakF[0], 1).s(", CH2 ").f(r.peakF[1], 1).s(", CH3 ").f(r.peakF[2], 1).s(" N").end();

  // ---- duration and self-check ----
  float durS = -1.0f;
  if (r.endReason == END_RELEASED && tFirstRaw < 1e29f)
    durS = ((float)(uint32_t)(r.tRel - r.tStart) - tFirstRaw) * 1e-6f;
  float resid = (r.rN && r.sgMax > 0) ? (float)(100.0 * (r.rSum / r.rN) / r.sgMax) : 0.0f;
  if (r.endReason == END_RELEASED) {
    Line().s("Squeeze duration: ").f(durS, 2).s(" s").end();
    Line l; l.s("Self-check: after release F_total = ").sf(resid, 2).s(" % of peak  ");
    if (absf(resid) <= RESID_PASS_PCT) l.s("PASS (<= 0.5 %)");
    else if (resid < 0) l.s("CHECK: negative -> tau set too large (lower it) or jaws not fully open");
    else l.s("CHECK: positive -> tau set too small (raise it) or jaws not fully open");
    l.end();
    if (durS > SQUEEZE_SPEC_S)
      Line().s("  note: longer than the 1.5 s design squeeze; the leak correction still applies").end();
  } else {
    Line().s("Squeeze still closed after 10 s: result taken at the time-out. Open the jaws before re-arming.").end();
  }
  if (gJawL > 0 && gJawW > 0) {
    float pa = r.peakTot / (gJawL * gJawW * 1e-6f);
    Line().s("Seal pressure: ").f(pa / 1000.0f, 1).s(" kPa (").f(pa / 6894.757f, 1).s(" psi) over ")
          .f(gJawL, 1).s(" x ").f(gJawW, 1).s(" mm").end();
  }

  // ---- warnings ----
  bool over = false;
  for (uint8_t i = 0; i < NCH; i++) over |= r.peakF[i] > CHIP_MAX_N;
  if (over) {
    Line l; l.s("Note: above 270 N (30 MPa constant-force limit of the PL033, spec 3):");
    for (uint8_t i = 0; i < NCH; i++) if (r.peakF[i] > CHIP_MAX_N) l.c(' ').s(CHN[i]).c(' ').f(r.peakF[i], 0).s(" N");
    l.end();
  }
  for (uint8_t i = 0; i < NCH; i++) {
    if (r.peakF[i] > CHIP_OVL_N)
      Line().s("WARNING: ").s(CHN[i]).s(" overloaded (").f(r.peakF[i], 0).s(" N): recheck its calibration (spec 8)").end();
    if (r.clipHi[i])
      Line().s("WARNING: ").s(CHN[i]).s(" output reached 3.2 V: force above the measurable range, result too low").end();
    if (r.clipLo[i])
      Line().s("WARNING: ").s(CHN[i]).s(" output hit 0 V: check wiring / polarity").end();
  }
  uint32_t dDrop = gDrops - r.drops0, dOvr = gOverruns - r.ovr0, dErr = gAdcErrors - r.err0;
  if (r.gaps || dDrop || dOvr || dErr)
    Line().s("WARNING: sampling gaps during this squeeze (gaps ").u(r.gaps).s(", dropped ").u(dDrop)
          .s(", late ").u(dOvr).s(", ADC errors ").u(dErr).s(")").end();
  bool allK = gCal.kCal[0] && gCal.kCal[1] && gCal.kCal[2];
  if (!allK) {
    Line().s("Note: ").s(anyCalibrated() ? "some chips use" : "all chips use")
          .s(" STANDARD k = 400 N/V: force about -25..+30 %, and the shares").end();
    Line().s("      shift as much as the chips differ (spec: +/-20 %). Calibrate for alignment work.").end();
  }

  // ---- one CSV line for logging ----
  Line l;
  l.s("CSV,").u(gShot).s(",").s(n ? CHN[ord[0]] : "none").s(",")
   .f(skew / 1000.0f, 3).c(',').f(uSkew / 1000.0f, 3);
  for (uint8_t i = 0; i < NCH; i++) {
    l.c(',');
    if (r.on[i]) l.f((tc[i] - t0) / 1000.0f, 3); else l.s("NA");
  }
  l.c(',').f(r.peakTot, 1);
  for (uint8_t i = 0; i < NCH; i++) l.c(',').f(r.peakTot > 0 ? 100.0f * r.FatPk[i] / r.peakTot : 0.0f, 2);
  l.c(',').f(resid, 2).c(',').f(durS, 3);
  l.end();
  Line().s("  (CSV: shot,first,skew_ms,skew_unc_ms,on1_ms,on2_ms,on3_ms,clamp_N,share1,share2,share3,resid_pct,dur_s)").end();
}

static void measureDone() {
  CapRun& r = gCap.run;
  if (r.endReason == END_SMALL) {
    Line().s("(small event ignored: peak ").f(r.sgMax, 1).s(" N < ").f(gCap.cfg.minEvent, 1).s(" N)").end();
    measureArm(true);
    return;
  }
  reportMeasurement();
  if (gAuto) { measureReset(); return; }
  gMState = M_DONE;
  Line().s("Send r (or press the button) to reset for the next squeeze.").end();
}

static void enterMeasure() {
  gMode = MODE_MEASURE;
  rule('=');
  Line().s("MEASUREMENT MODE   (h = help, cal = calibration menu)").end();
  calStatusLine();
  measureReset();
}

/* ======================= CALIBRATION RECORD / CAL LINE ==================== */

static void printCalLine() {
  Line l;
  l.s("CAL1 k=");
  for (uint8_t i = 0; i < NCH; i++) { if (i) l.c(','); l.f(gCal.k[i], 2); }
  l.s(" q=");
  for (uint8_t i = 0; i < NCH; i++) { if (i) l.c(','); l.f(gCal.q[i], 3); }
  l.s(" tau=");
  for (uint8_t i = 0; i < NCH; i++) { if (i) l.c(','); l.f(gCal.tau[i], 2); }
  l.s(" off=");
  for (uint8_t i = 0; i < NCH; i++) { if (i) l.c(','); l.f(gCal.off[i], 1); }
  l.s(" b=");
  for (uint8_t i = 0; i < NCH; i++) { if (i) l.c(','); l.f(gCal.b[i], 2); }
  l.s(" dev=");
  for (uint8_t i = 0; i < NCH; i++) { if (i) l.c(','); l.f(gCal.dev[i], 2); }
  l.s(" kc=");
  for (uint8_t i = 0; i < NCH; i++) { if (i) l.c(','); l.u(gCal.kCal[i]); }
  l.s(" tc=");
  for (uint8_t i = 0; i < NCH; i++) { if (i) l.c(','); l.u(gCal.tauCal[i]); }
  l.s(" oc=").u(gCal.offCal).s(" thr=").f(gThrN, 1);
  if (gCal.note[0]) l.s(" note=").s(gCal.note);
  l.end();
}

static void showRecord() {
  rule();
  Line().s("CALIBRATION RECORD (each chip stays paired with its channel)").end();
  Line().s("        k N/V    q N/V2   icpt N   worst %  tau s    offset ms   source").end();
  for (uint8_t i = 0; i < NCH; i++) {
    Line l;
    l.s(CHN[i]).pad(8).f(gCal.k[i], 2).pad(17).f(gCal.q[i], 3).pad(26);
    if (gCal.kCal[i]) l.sf(gCal.b[i], 2); else l.s("-");
    l.pad(35);
    if (gCal.dev[i] >= 0) l.f(gCal.dev[i], 2); else l.s("-");
    l.pad(44).f(gCal.tau[i], 2).pad(53).sf(gCal.off[i] / 1000.0f, 3).pad(65)
     .s("k:").s(gCal.kCal[i] ? "cal" : "std").s(" tau:").s(gCal.tauCal[i] ? "cal" : "std");
    l.end();
  }
  Line().s("Timing offsets: ").s(gCal.offCal ? "calibrated" : "standard (0)")
        .s(".  Onset threshold ").f(gThrN, 1).s(" N.").end();
  Line().s("Note: ").s(gCal.note[0] ? gCal.note : "(none - add date and room temperature: note 2026-10-28 22C)").end();
  Line().s("Copy the next line and keep it; paste it back after a power cycle:").end();
  printCalLine();
  rule();
}

static bool loadCalLine(const char* ln) {
  Cal   c   = gCal;
  float thr = gThrN;
  bool  any = false;
  const char* p = ln;
  while (*p) {
    while (*p == ' ') p++;
    const char* key = p;
    while (*p && *p != '=' && *p != ' ') p++;
    if (*p != '=') { while (*p && *p != ' ') p++; continue; }   // e.g. "CAL1"
    size_t klen = (size_t)(p - key);
    const char* val = ++p;
    while (*p && *p != ' ') p++;
    size_t vlen = (size_t)(p - val);
    char kb[8], vb[100];
    if (klen == 0 || klen >= sizeof(kb) || vlen >= sizeof(vb)) return false;
    memcpy(kb, key, klen); kb[klen] = 0;
    memcpy(vb, val, vlen); vb[vlen] = 0;
    if (eqCI(kb, "note")) {
      strncpy(c.note, vb, sizeof(c.note) - 1); c.note[sizeof(c.note) - 1] = 0;
      continue;
    }
    float a[NCH];
    uint8_t n = parseList(vb, a, NCH);
    if      (eqCI(kb, "k")   && n == NCH) { for (uint8_t i = 0; i < NCH; i++) c.k[i] = a[i]; any = true; }
    else if (eqCI(kb, "q")   && n == NCH) { for (uint8_t i = 0; i < NCH; i++) c.q[i] = a[i]; }
    else if (eqCI(kb, "tau") && n == NCH) { for (uint8_t i = 0; i < NCH; i++) c.tau[i] = a[i]; }
    else if (eqCI(kb, "off") && n == NCH) { for (uint8_t i = 0; i < NCH; i++) c.off[i] = a[i]; }
    else if (eqCI(kb, "b")   && n == NCH) { for (uint8_t i = 0; i < NCH; i++) c.b[i] = a[i]; }
    else if (eqCI(kb, "dev") && n == NCH) { for (uint8_t i = 0; i < NCH; i++) c.dev[i] = a[i]; }
    else if (eqCI(kb, "kc")  && n == NCH) { for (uint8_t i = 0; i < NCH; i++) c.kCal[i] = a[i] != 0; }
    else if (eqCI(kb, "tc")  && n == NCH) { for (uint8_t i = 0; i < NCH; i++) c.tauCal[i] = a[i] != 0; }
    else if (eqCI(kb, "oc")  && n == 1)   { c.offCal = a[0] != 0; }
    else if (eqCI(kb, "thr") && n == 1)   { thr = a[0]; }
    else return false;
  }
  if (!any) return false;
  for (uint8_t i = 0; i < NCH; i++) {
    if (!(c.k[i] > 50.0f && c.k[i] < 5000.0f)) return false;
    if (!(c.tau[i] > 5.0f && c.tau[i] < 1000.0f)) return false;
    if (!(absf(c.off[i]) < 5000.0f)) return false;
    if (!(absf(c.q[i]) < 100000.0f)) return false;
  }
  if (!(thr >= 0.5f && thr <= 100.0f)) return false;
  gCal = c;
  gThrN = thr;
  return true;
}

static void tryLoadCal(const char* ln) {
  if (loadCalLine(ln)) {
    Line().s("Calibration loaded.").end();
    calStatusLine();
  } else {
    Line().s("That CAL line could not be read (format: CAL1 k=a,b,c tau=a,b,c off=a,b,c ...).").end();
  }
}

/* ============================ CALIBRATION WIZARD ========================== */

static uint16_t gCondN = 0;
static float    gTauRuns[8];
static uint8_t  gTauN = 0;
static KPt      gPts[32];
static uint8_t  gPtN = 0;
static float    gPendX = 0;
static FitRes   gFit;
static double   gOffSum[NCH], gOffSq[NCH];
static uint8_t  gOffN = 0;
static float    gOffMean[NCH];
static int8_t   gVerFocus = -1;
static float    gPendVer[NCH + 1];

static void calMenu() {
  gMode = MODE_CAL;
  gCState = C_MENU;
  capCancel();
  rule('=');
  Line().s("CALIBRATION MENU  (design spec section 8)").end();
  Line().s("One piezo at a time, mounted exactly as in the sealer (same shims, same").end();
  Line().s("orientation). Label each chip with its channel and keep the pairs together.").end();
  Line().s("  1 / 2 / 3   calibrate the piezo on CH1/A0, CH2/A1 or CH3/A2").end();
  Line().s("              (conditioning -> tau -> sensitivity k against a scale)").end();
  Line().s("  t           timing offsets (all three chips under one plate, ~10 centre presses)").end();
  Line().s("  v           verify a known load (one chip, or system check with all three)").end();
  Line().s("  show        calibration record and the CAL line to save").end();
  Line().s("  note <txt>  add date / room temperature to the record").end();
  Line().s("  mon         live forces;   std = back to standard parameters").end();
  Line().s("  x           finish -> measurement").end();
  calStatusLine();
}

// ---- step 1: conditioning ----
static void condStart() {
  gCState = C_COND;
  gCondN = 0;
  rule();
  Line().s("PIEZO ").s(CHN[gCalCh]).s("  step 1 of 3: CONDITIONING (new chips only)").end();
  Line().s("Squeeze this chip 30-50 times to about working force (100-270 N).").end();
  Line().s("Presses are counted below.   n = next step (also to skip),  q = menu").end();
  capArm(CP_COND, (int8_t)gCalCh, false);
}

static void condDone() {
  CapRun& r = gCap.run;
  if (r.endReason == END_SMALL) Line().s("  (light press ignored)").end();
  else {
    gCondN++;
    Line().s("  press ").u(gCondN).s(": peak ").f(r.peakF[gCalCh], 1).s(" N").end();
    if (gCondN == 30) Line().s("  30 presses done - continue to 50, or type n for the next step").end();
    if (gCondN == 50) Line().s("  50 presses done - type n for the next step").end();
  }
  capArm(CP_COND, (int8_t)gCalCh, true);
}

// ---- step 2: tau ----
static void tauStart() {
  gCState = C_TAU_PRESS;
  gTauN = 0;
  rule();
  Line().s("PIEZO ").s(CHN[gCalCh]).s("  step 2 of 3: TAU (charge leak time constant, no scale needed)").end();
  Line().s("1. Best: let the output settle, >= 3 min after earlier squeezes (what is").end();
  Line().s("   left of their undershoot is corrected for, using the equilibrium level).").end();
  Line().s("2. Apply a steady force (~100-270 N), HOLD about 10 s, release quickly,").end();
  Line().s("   then keep everything still for 2 s.").end();
  Line().s("After release the true force is 0, so tau = -integral(V - V_eq) dt / dV.").end();
  Line().s("   q = keep the current tau (").f(gCal.tau[gCalCh], 1).s(" s) and go to step 3").end();
  capArm(CP_TAU, (int8_t)gCalCh, false);
}

static void tauDone() {
  CapRun& r = gCap.run;
  uint8_t i = gCalCh;
  if (r.endReason == END_SMALL) { Line().s("  (light press ignored)").end(); capArm(CP_TAU, (int8_t)i, true); return; }
  if (r.endReason != END_RELEASED || r.tN < 50) {
    Line().s("  no clean release within 40 s - press, hold ~10 s and release again").end();
    capArm(CP_TAU, (int8_t)i, true);
    return;
  }
  double mI = r.tI[i] / r.tN, mDV = r.tDV[i] / r.tN, mX = r.tX[i] / r.tN;
  float hold = ((float)(uint32_t)(r.tRel - r.tStart) - r.tOn[i]) * 1e-6f;
  Line().s("  hold ").f(hold, 1).s(" s, peak ").f(r.peakF[i], 1).s(" N, undershoot after release ")
        .f(mDV * 1000.0, 2).s(" mV").end();
  if (r.peakX[i] > 0) {
    float res = (float)(100.0 * mX / r.peakX[i]);
    Line().s("  residual after release with tau ").f(gCal.tau[i], 1).s(" s: ").sf(res, 2).s(" % of peak  ")
          .s(absf(res) <= RESID_PASS_PCT ? "PASS" : "(limit 0.5 %)").end();
  }
  if (gTauN > 0 && r.peakX[i] > 0) {
    float mean = 0;
    for (uint8_t k = 0; k < gTauN; k++) mean += gTauRuns[k];
    mean /= gTauN;
    float res = (float)(100.0 * (mDV + mI / mean) / r.peakX[i]);
    Line().s("  residual with tau from the earlier runs (").f(mean, 1).s(" s): ").sf(res, 2).s(" %  ")
          .s(absf(res) <= RESID_PASS_PCT ? "PASS" : "(limit 0.5 %)").end();
  }
  if (mDV > -0.002) {
    Line().s("  undershoot too small to measure (needs < -2 mV): hold longer and/or press harder").end();
    capArm(CP_TAU, (int8_t)i, true);
    return;
  }
  float tauRun = (float)(-mI / mDV);
  if (!(tauRun > 10.0f && tauRun < 1000.0f)) {
    Line().s("  implausible tau ").f(tauRun, 1).s(" s (expected ~68 s): check Cf/Rf, then repeat").end();
    capArm(CP_TAU, (int8_t)i, true);
    return;
  }
  if (gTauN < 8) gTauRuns[gTauN++] = tauRun;
  float mean = 0, lo = 1e9f, hi = 0;
  for (uint8_t k = 0; k < gTauN; k++) { mean += gTauRuns[k]; if (gTauRuns[k] < lo) lo = gTauRuns[k]; if (gTauRuns[k] > hi) hi = gTauRuns[k]; }
  mean /= gTauN;
  Line().s("  tau from this run: ").f(tauRun, 2).s(" s   (nominal Rf*Cf = ").f(TAU_STD, 1).s(" s)").end();
  Line l; l.s("  runs: ").u(gTauN).s("   mean ").f(mean, 2).s(" s");
  if (gTauN > 1) l.s("   spread ").f(100.0f * (hi - lo) / mean, 1).s(" %");
  l.end();
  Line l2;
  l2.s("  equilibrium level used for the leak: ").f(gVeq[i], 4).s(" V (");
  if (gVeqMs) l2.s("refined from idle drift ").u((millis() - gVeqMs) / 1000UL).s(" s ago)");
  else l2.s("from the self-test)");
  l2.end();
  if (absf(r.base[i] - gVeq[i]) > 0.010f)
    Line().s("  note: output was ").f((r.base[i] - gVeq[i]) * 1000.0f, 1)
          .s(" mV from equilibrium before the press (earlier squeezes) - corrected for").end();
  Line().s("  m = another run (repeat until two runs agree within ~5 %)   a = accept the mean").end();
  Line().s("  r = clear the runs   q = keep the current tau and go to step 3").end();
  gCState = C_TAU_DECIDE;
}

// ---- step 3: sensitivity k ----
static void kStart() {
  gCState = C_K_PRESS;
  gPtN = 0;
  rule();
  Line().s("PIEZO ").s(CHN[gCalCh]).s("  step 3 of 3: SENSITIVITY k (needs a reference scale)").end();
  Line().s("Put the shim / chip / shim stack centred on the scale, under the press.").end();
  Line().s("Levels ~50, 100, 150, 200 and 270 N, each 3 times, starting from zero each time:").end();
  Line().s("ramp ~0.5 s, HOLD until the scale settles (2-3 s), release, then type the").end();
  Line().s("scale reading.  Units: N (default), kg or lb  ->  101   10.3kg   22.7lb").end();
  Line().s("Do not exceed 270 N (30 MPa limit for the PL033).").end();
  Line().s("   f = fit now   u = undo last point   s = list points   q = menu (discards points)").end();
  capArm(CP_KPOINT, (int8_t)gCalCh, false);
}

static void kPrompt() {
  Line l;
  l.s("[ready - press ").u(gPtN + 1).s(": ");
  if (gPtN < 15) l.s("target ~").f(KLEVELS[gPtN / 3], 0).s(" N (").u(gPtN % 3 + 1).s(" of 3)");
  else l.s("any level, or f = fit");
  l.s(" on ").s(CHN[gCalCh]).s("]");
  l.end();
}

static void kDone() {
  CapRun& r = gCap.run;
  uint8_t i = gCalCh;
  if (r.endReason == END_SMALL) { Line().s("  (light press ignored)").end(); capArm(CP_KPOINT, (int8_t)i, false); return; }
  if (r.clipHi[i]) { Line().s("  output clipped at 3.2 V - use a lower force").end(); capArm(CP_KPOINT, (int8_t)i, false); return; }
  if (!r.plOk) {
    Line().s("  hold not steady or too short (plateau spread ").f(r.plSpread, 1).s(" %, ")
          .u(r.plN).s(" x 50 ms) - repeat this press").end();
    capArm(CP_KPOINT, (int8_t)i, false);
    return;
  }
  gPendX = r.plX[i];
  Line().s("  plateau: corrected dV = ").f(gPendX, 5).s(" V  (steady within ").f(r.plSpread, 1)
        .s(" %, ~").f(r.plF[i], 0).s(" N at current k)").end();
  Line().s("  Type the scale reading (or x to discard this press):").end();
  gCState = C_K_REF;
}

static void kListPoints() {
  for (uint8_t j = 0; j < gPtN; j++)
    Line().s("  #").u(j + 1).s("  ").f(gPts[j].f, 1).s(" N  <->  ").f(gPts[j].x, 5).s(" V   ratio ")
          .f(gPts[j].f / gPts[j].x, 1).s(" N/V").end();
  if (!gPtN) Line().s("  (no points yet)").end();
}

static void kFit() {
  FitRes& f = gFit;
  f = FitRes();
  uint8_t n = gPtN;
  if (n < 2) { Line().s("  need at least 2 points (better 15: 5 levels x 3)").end(); return; }
  double Sx = 0, Sy = 0, Sxx = 0, Sxy = 0, S3 = 0, S4 = 0, Sx2y = 0;
  for (uint8_t j = 0; j < n; j++) {
    double x = gPts[j].x, y = gPts[j].f;
    Sx += x; Sy += y; Sxx += x * x; Sxy += x * y;
    S3 += x * x * x; S4 += x * x * x * x; Sx2y += x * x * y;
  }
  double den = n * Sxx - Sx * Sx;
  if (den <= 1e-12 * n * Sxx) { Line().s("  all points are at one level - add points at other forces").end(); return; }
  f.k = (float)((n * Sxy - Sx * Sy) / den);           // slope = k (spec 8 step 3)
  f.b = (float)((Sy - f.k * Sx) / n);                 // intercept, should be ~0
  double det = Sxx * S4 - S3 * S3;                     // F = qk*x + qq*x^2 (through 0)
  if (n >= 3 && det > 1e-12 * Sxx * S4) {
    f.qk = (float)((Sxy * S4 - Sx2y * S3) / det);
    f.qq = (float)((Sxx * Sx2y - S3 * Sxy) / det);
    f.qOk = true;
  }
  float maxRef = 0;
  for (uint8_t j = 0; j < n; j++) {
    double x = gPts[j].x, y = gPts[j].f;
    float e1 = (float)(100.0 * (f.k * x + f.b - y) / y);
    float e2 = (float)(100.0 * (f.k * x - y) / y);
    if (absf(e1) > f.worst) f.worst = absf(e1);
    if (absf(e2) > f.worstUse) f.worstUse = absf(e2);
    if (f.qOk) { float e3 = (float)(100.0 * (f.qk * x + f.qq * x * x - y) / y); if (absf(e3) > f.qWorst) f.qWorst = absf(e3); }
    if (y > maxRef) maxRef = (float)y;
  }
  // repeats: points within 10 % of each other are one level
  uint8_t idx[32];
  for (uint8_t j = 0; j < n; j++) idx[j] = j;
  for (uint8_t a = 1; a < n; a++)
    for (uint8_t b = a; b > 0 && gPts[idx[b]].f < gPts[idx[b - 1]].f; b--) { uint8_t t = idx[b]; idx[b] = idx[b - 1]; idx[b - 1] = t; }
  uint8_t start = 0;
  while (start < n) {
    uint8_t end = start + 1;
    while (end < n && gPts[idx[end]].f <= 1.10f * gPts[idx[start]].f) end++;
    f.levels++;
    if (end - start >= 2) {
      float lo = 1e30f, hi = 0, sum = 0;
      for (uint8_t j = start; j < end; j++) { float rr = gPts[idx[j]].f / gPts[idx[j]].x; sum += rr; if (rr < lo) lo = rr; if (rr > hi) hi = rr; }
      float sp = 100.0f * (hi - lo) / (sum / (end - start));
      if (sp > f.rep) f.rep = sp;
    }
    start = end;
  }
  f.ok = true;

  rule();
  Line().s("FIT of reference force vs corrected dV for ").s(CHN[gCalCh]).s(" (").u(n).s(" points, ").u(f.levels).s(" levels)").end();
  bool bOk = absf(f.b) <= maxf(1.0f, 0.01f * maxRef);
  Line().s("  k = ").f(f.k, 2).s(" N/V   (nominal 310-520 N/V;  S = Cf/k = ").f(CF_F / f.k * 1e9, 1).s(" nC/N)").end();
  Line().s("  intercept ").sf(f.b, 2).s(" N  ").s(bOk ? "(near zero: OK)" : "CHECK: not near zero - baseline or mounting?").end();
  Line().s("  worst point vs fitted line ").f(f.worst, 2).s(" %  (limit 2 %)  ").s(f.worst <= FIT_PASS_PCT ? "PASS" : "FAIL").end();
  if (f.levels < n)
    Line().s("  repeats at the same level  ").f(f.rep, 2).s(" %  (limit 1-2 %)  ").s(f.rep <= REPEAT_PASS_PCT ? "PASS" : "FAIL").end();
  else
    Line().s("  repeats: none yet (repeat each level 3 times to check)").end();
  Line().s("  as used in measurements (F = k*dV, through zero): worst ").f(f.worstUse, 2).s(" %").end();
  bool suggestQ = f.qOk && f.worstUse > FIT_PASS_PCT && f.qWorst < f.worstUse;
  if (f.qOk)
    Line().s("  quadratic through zero: k ").f(f.qk, 2).s(" N/V, q ").f(f.qq, 2).s(" N/V^2, worst ").f(f.qWorst, 2)
          .s(" %").s(suggestQ ? "   <- points curve: aq recommended" : "").end();
  Line().s("  a = accept linear").s(f.qOk ? "   aq = accept quadratic" : "")
        .s("   m = more points   u = undo last point   x = discard").end();
  gCState = C_K_FIT;
}

// fit, or go back to pressing if there is nothing valid to fit
static void kFitOrPress() {
  capCancel();
  kFit();
  if (!gFit.ok) { gCState = C_K_PRESS; capArm(CP_KPOINT, (int8_t)gCalCh, false); }
}

static void kAccept(bool quad) {
  uint8_t i = gCalCh;
  if (!gFit.ok || (quad && !gFit.qOk)) return;
  if (quad) { gCal.k[i] = gFit.qk; gCal.q[i] = gFit.qq; gCal.b[i] = 0; gCal.dev[i] = gFit.qWorst; }
  else      { gCal.k[i] = gFit.k;  gCal.q[i] = 0;       gCal.b[i] = gFit.b; gCal.dev[i] = gFit.worst; }
  gCal.kCal[i] = true;
  Line().s(CHN[i]).s(" calibrated: k = ").f(gCal.k[i], 2).s(" N/V").s(quad ? " (quadratic)" : "")
        .s(", tau = ").f(gCal.tau[i], 2).s(" s").end();
  Line().s("Next: the other chips, then t (timing offsets) and v (system check).").end();
  Line().s("Then type show and save the CAL line - it is lost at power-off otherwise.").end();
  calMenu();
}

// ---- timing offsets ----
static void offStart() {
  gCState = C_OFF_PRESS;
  gOffN = 0;
  for (uint8_t i = 0; i < NCH; i++) gOffSum[i] = gOffSq[i] = 0;
  rule();
  Line().s("TIMING OFFSETS (spec 8 step 6)").end();
  Line().s("All three chips in their sealer positions under ONE rigid, flat plate.").end();
  Line().s("Centre-press the plate about 10 times (each press well above ").f(3.0f * gThrN, 0).s(" N, then release).").end();
  Line().s("Each channel's crossing time is taken relative to the mean of the three.").end();
  Line().s("   f = finish now (needs 3 presses)   q = menu (discard)").end();
  capArm(CP_OFFSET, -1, false);
}

static void offFinish() {
  rule();
  Line().s("Timing offsets from ").u(gOffN).s(" centre presses (mean +/- std):").end();
  bool big = false;
  for (uint8_t i = 0; i < NCH; i++) {
    double m = gOffSum[i] / gOffN;
    double var = gOffSq[i] / gOffN - m * m;
    gOffMean[i] = (float)m;
    if (absf((float)m) > OFFSET_WARN_US) big = true;
    Line().s("  ").s(CHN[i]).s("  ").sf(m / 1000.0, 3).s(" ms  +/- ").f(sqrtf(var > 0 ? (float)var : 0.0f) / 1000.0f, 3).s(" ms").end();
  }
  if (big) {
    Line().s("  WARNING: an offset above 0.5 ms (electronics alone give ~0.1 ms): is the plate").end();
    Line().s("  seating evenly? Accepting would hide a real misalignment.").end();
  }
  Line().s("  a = accept   r = redo   q = menu (discard)").end();
  gCState = C_OFF_DECIDE;
}

static void offDone() {
  CapRun& r = gCap.run;
  if (r.endReason == END_SMALL) { Line().s("  (light press ignored)").end(); capArm(CP_OFFSET, -1, true); return; }
  bool all = r.on[0] && r.on[1] && r.on[2];
  if (!all) {
    Line l; l.s("  press ignored:");
    for (uint8_t i = 0; i < NCH; i++) if (!r.on[i]) l.c(' ').s(CHN[i]);
    l.s(" did not reach the threshold - press harder / more centrally").end();
    capArm(CP_OFFSET, -1, true);
    return;
  }
  float mean = (r.tOn[0] + r.tOn[1] + r.tOn[2]) / 3.0f;
  Line l; l.s("  press ").u(gOffN + 1).s(":");
  for (uint8_t i = 0; i < NCH; i++) {
    float rel = r.tOn[i] - mean;
    gOffSum[i] += rel; gOffSq[i] += (double)rel * rel;
    l.s("  ").s(CHN[i]).c(' ').sf(rel / 1000.0f, 3).s(" ms");
  }
  l.end();
  gOffN++;
  if (gOffN >= 10) { offFinish(); return; }
  capArm(CP_OFFSET, -1, true);
}

// ---- verification ----
static void verStart() {
  gCState = C_VER_SELECT;
  rule();
  Line().s("VERIFY A KNOWN LOAD (spec 8 steps 4-5, and before each test session)").end();
  Line().s("  1 / 2 / 3 = one chip on the scale (single-chip check)").end();
  Line().s("  a         = system check: all three chips under one plate on the scale").end();
  Line().s("              (centre press: total within ~2 %; off-centre: same total, shares shift)").end();
  Line().s("  q         = menu").end();
}

static void verDone() {
  CapRun& r = gCap.run;
  if (r.endReason == END_SMALL) { Line().s("  (light press ignored)").end(); capArm(CP_VERIFY, gVerFocus, true); return; }
  if (!r.plOk) {
    Line().s("  hold not steady or too short (spread ").f(r.plSpread, 1).s(" %) - repeat").end();
    capArm(CP_VERIFY, gVerFocus, true);
    return;
  }
  for (uint8_t i = 0; i < NCH; i++) gPendVer[i] = r.plF[i];
  gPendVer[NCH] = r.plFtot;
  if (gVerFocus < 0) {
    Line l; l.s("  measured F_total ").f(r.plFtot, 1).s(" N  (shares");
    for (uint8_t i = 0; i < NCH; i++) l.c(' ').s(CHN[i]).c(' ').f(r.plFtot > 0 ? 100.0f * r.plF[i] / r.plFtot : 0.0f, 1).s("%");
    l.s(")").end();
  } else {
    Line().s("  measured ").f(r.plF[gVerFocus], 1).s(" N on ").s(CHN[gVerFocus]).end();
  }
  Line().s("  Type the scale reading (or x to discard):").end();
  gCState = C_VER_REF;
}

/* ============================= HELP TEXTS ================================= */

static void printEquations() {
  rule();
  Line().s("FORCE MODEL (design spec sections 3, 4.3, 7)").end();
  Line().s("  Q  = S * F                 piezo charge; S ~13-22 nC/N for the PL033.31").end();
  Line().s("  dV = Q / Cf                charge amplifier, Cf = 6.8 uF (sensor C irrelevant)").end();
  Line().s("  Cf*dV/dt = dQ/dt - (V - V_eq)/Rf   charge leaks through Rf: tau = Rf*Cf = 68 s").end();
  Line().s("  => F = k * (dV + (1/tau) * integral((V - V_eq) dt)),   k = Cf / S").end();
  Line().s("     settled output (baseline = V_eq): F[n] = k*(dV[n] + (Ts/tau)*S[n]),").end();
  Line().s("     S[n] = S[n-1] + dV[n], Ts = 0.5 ms   (spec section 7)").end();
  {
    Line l; l.s("  V_eq now:");
    for (uint8_t i = 0; i < NCH; i++) l.c(' ').s(CHN[i]).c(' ').f(gVeq[i], 4).s(" V");
    l.end();
  }
  Line().s("  F_total[n] = F1[n] + F2[n] + F3[n];  clamp force = max of F_total").end();
  Line().s("  share_i = F_i / F_total at that moment;  onset_i = threshold crossing - offset_i").end();
  Line().s("  Standard values: k = 6.8 uF / 17 nC/N = ").f(STD_K, 0).s(" N/V, tau = ").f(TAU_STD, 0).s(" s").end();
  calStatusLine();
}

static void printHelp() {
  rule();
  Line().s("COMMANDS (type, then Enter)").end();
  Line().s("  r            reset / re-arm (same as the button on D2)").end();
  Line().s("  cal          calibration menu (guided, one piezo at a time)").end();
  Line().s("  show         calibration record and the CAL line to save").end();
  Line().s("  CAL1 ...     paste a saved CAL line to load it").end();
  Line().s("  std          standard parameters (k = 400 N/V, tau = 68 s, offsets 0)").end();
  Line().s("  thr <N>      onset threshold per chip, now ").f(gThrN, 1).s(" N").end();
  Line().s("  jaw <L> <W>  seal length and width in mm for pressure (jaw 0 0 = off)").end();
  Line().s("  auto on|off  re-arm automatically after each squeeze, now ").s(gAuto ? "on" : "off").end();
  Line().s("  mon          live forces 5x per second (Enter stops)").end();
  Line().s("  eq           the force equations used").end();
  Line().s("  stat         sampler statistics").end();
}

static void printStats() {
  Line().s("Sampler: ").s(gThreaded ? "own thread, priority 2" : "inline in loop() (FALLBACK)")
        .s(", ").u(FS_HZ).s(" Hz, 3-channel scan ").u(gScanUs).s(" us, timebase ")
        .f(gHz / 1e6, 3).s(" MHz (").u(gTickNs).s(" ns)").end();
  Line().s("  dropped ").u(gDrops).s(", late ticks ").u(gOverruns).s(", ADC errors ").u(gAdcErrors).end();
}

static void startupPrompt() {
  rule();
  Line().s("CALIBRATION SETUP").end();
  Line().s("Every PL033.31 + amplifier channel has its own sensitivity, so each input").end();
  Line().s("is calibrated on its own (design spec section 8).").end();
  Line().s("  c = calibrate now (guided step by step, one piezo at a time)").end();
  Line().s("  s = skip, use STANDARD parameters:").end();
  Line().s("        k = Cf / S = 6.8 uF / 17 nC/N = ").f(STD_K, 0).s(" N/V for every chip,").end();
  Line().s("        tau = Rf * Cf = ").f(TAU_STD, 0).s(" s, timing offsets 0").end();
  Line().s("        (absolute force then about -25..+30 %; shares assume equal chips)").end();
  Line().s("  p = paste a saved CAL line from an earlier calibration").end();
  Line().s("  (the button on D2 also means s; no answer within 60 s = s)").end();
  gMode = MODE_STARTUP;
  gStartupMs = millis();
}

static void selfTestReport() {
  rule('=');
  Line().s("Self-test (jaws open):").end();
  for (uint8_t i = 0; i < NCH; i++) {
    float mean = (float)gStMean[i];
    float sd   = gStN > 1 ? sqrtf((float)(gStM2[i] / (gStN - 1))) : 0.0f;
    Line l;
    l.s("  ").s(CHN[i]).s("  idle ").f(mean, 3).s(" V   noise ").f(sd * 1000.0f, 2).s(" mV rms = ")
     .f(sd * gCal.k[i], 2).s(" N   ");
    if (mean > 3.0f)       l.s("FAULT: stuck high (amplifier saturated: Rf/Cf, wiring, polarity?)");
    else if (mean < 0.05f) l.s("FAULT: near 0 V (VREF missing, op amp unpowered, open input?)");
    else if (mean < 0.15f || mean > 0.60f) l.s("CHECK: expected about 0.30 V (VREF)");
    else if (sd > 0.003f)  l.s("CHECK: noisy");
    else l.s("OK");
    l.end();
  }
  float rate = (gStN > 1 && gStT1 != gStT0) ? (float)(gStN - 1) * 1e6f / (float)(uint32_t)(gStT1 - gStT0) : 0.0f;
  Line().s("  measured sample rate ").f(rate, 0).s(" Hz per channel").end();
  printStats();
  if (!gThreaded)
    Line().s("  WARNING: no sampler thread - serial traffic will cause sampling gaps").end();
}

/* ============================ LINE DISPATCH =============================== */

static void startMon() {
  for (uint8_t i = 0; i < NCH; i++) gMonBase[i] = histMeanV(i, 200);
  gMon = true;
  gMonMs = millis();
  Line().s("Live forces F = k*(V - V at start), 5x per second (no leak correction). Enter stops.").end();
}

static bool globalCmd(const char* ln) {              // valid in menu / measurement idle
  if (wordIs(ln, "show"))  { showRecord(); return true; }
  if (wordIs(ln, "eq"))    { printEquations(); return true; }
  if (wordIs(ln, "stat"))  { printStats(); return true; }
  if (wordIs(ln, "mon"))   { startMon(); return true; }
  if (wordIs(ln, "std"))   { calDefaults(gCal); Line().s("Standard parameters restored.").end(); calStatusLine(); return true; }
  if (wordIs(ln, "note"))  {
    const char* a = argOf(ln);
    uint8_t k = 0;
    while (*a && k < sizeof(gCal.note) - 1) { gCal.note[k++] = (*a == ' ') ? '_' : *a; a++; }
    gCal.note[k] = 0;
    Line().s("Note set: ").s(gCal.note).end();
    return true;
  }
  if (wordIs(ln, "thr")) {
    float v;
    if (parseNum(argOf(ln), v, NULL) && v >= 0.5f && v <= 100.0f) { gThrN = v; Line().s("Onset threshold = ").f(gThrN, 1).s(" N per chip").end(); }
    else Line().s("thr needs a value from 0.5 to 100 (N)").end();
    return true;
  }
  if (wordIs(ln, "jaw")) {
    const char* rest; float L, W;
    if (parseNum(argOf(ln), L, &rest) && parseNum(rest, W, NULL) && L >= 0 && W >= 0) {
      gJawL = L; gJawW = W;
      if (L > 0 && W > 0) Line().s("Seal area ").f(L, 1).s(" x ").f(W, 1).s(" mm").end(); else Line().s("Pressure output off").end();
    } else Line().s("jaw <length mm> <width mm>, e.g. jaw 203 12.7").end();
    return true;
  }
  if (wordIs(ln, "auto")) {
    const char* a = argOf(ln);
    if (eqCI(a, "on")) gAuto = true; else if (eqCI(a, "off")) gAuto = false;
    Line().s("Auto re-arm ").s(gAuto ? "on" : "off").end();
    return true;
  }
  if (wordIs(ln, "h") || wordIs(ln, "help") || wordIs(ln, "?")) { printHelp(); return true; }
  return false;
}

static void startupCmd(const char* ln) {
  if (gPasteNext) {
    gPasteNext = false;
    if (loadCalLine(ln)) { Line().s("Calibration loaded.").end(); enterMeasure(); }
    else { Line().s("Could not read that line.").end(); startupPrompt(); }
    return;
  }
  if (eqCI(ln, "c")) { calMenu(); return; }
  if (eqCI(ln, "s")) { Line().s("Skipped: using STANDARD calibration parameters.").end(); enterMeasure(); return; }
  if (eqCI(ln, "p")) { gPasteNext = true; Line().s("Paste the CAL1 line and press Enter:").end(); return; }
  if (globalCmd(ln)) return;
  if (*ln) Line().s("Type c, s or p.").end();
}

static void measureCmd(const char* ln) {
  if (gPasteNext) { gPasteNext = false; tryLoadCal(ln); return; }
  if (eqCI(ln, "r")) { measureReset(); return; }
  if (eqCI(ln, "c") || wordIs(ln, "cal")) { calMenu(); return; }
  if (eqCI(ln, "p")) { gPasteNext = true; Line().s("Paste the CAL1 line and press Enter:").end(); return; }
  if (globalCmd(ln)) return;
  if (*ln) Line().s("Unknown command - h for help").end();
}

static void calCmd(const char* ln) {
  switch (gCState) {
    case C_MENU:
      if (eqCI(ln, "1") || eqCI(ln, "2") || eqCI(ln, "3")) { gCalCh = (uint8_t)(ln[0] - '1'); condStart(); return; }
      if (eqCI(ln, "t")) { offStart(); return; }
      if (eqCI(ln, "v")) { verStart(); return; }
      if (eqCI(ln, "x")) { showRecord(); enterMeasure(); return; }
      if (globalCmd(ln)) return;
      if (*ln) Line().s("Type 1, 2, 3, t, v, show or x").end();
      return;

    case C_COND:
      if (eqCI(ln, "n")) { tauStart(); return; }
      if (eqCI(ln, "q")) { calMenu(); return; }
      return;

    case C_TAU_PRESS:
      if (eqCI(ln, "q")) { kStart(); return; }
      return;

    case C_TAU_DECIDE:
      if (eqCI(ln, "m")) { gCState = C_TAU_PRESS; capArm(CP_TAU, (int8_t)gCalCh, false); return; }
      if (eqCI(ln, "r")) { gTauN = 0; Line().s("  runs cleared").end(); gCState = C_TAU_PRESS; capArm(CP_TAU, (int8_t)gCalCh, false); return; }
      if (eqCI(ln, "a")) {
        float m = 0;
        for (uint8_t k = 0; k < gTauN; k++) m += gTauRuns[k];
        gCal.tau[gCalCh] = m / gTauN;
        gCal.tauCal[gCalCh] = true;
        Line().s("  tau for ").s(CHN[gCalCh]).s(" = ").f(gCal.tau[gCalCh], 2).s(" s").end();
        kStart();
        return;
      }
      if (eqCI(ln, "q")) { kStart(); return; }
      Line().s("  m, a, r or q").end();
      return;

    case C_K_PRESS:
      if (eqCI(ln, "f")) { kFitOrPress(); return; }
      if (eqCI(ln, "u")) { if (gPtN) gPtN--; Line().s("  points: ").u(gPtN).end(); return; }
      if (eqCI(ln, "s")) { kListPoints(); return; }
      if (eqCI(ln, "q")) { calMenu(); return; }
      return;

    case C_K_REF: {
      if (eqCI(ln, "x")) { Line().s("  discarded").end(); gCState = C_K_PRESS; capArm(CP_KPOINT, (int8_t)gCalCh, false); return; }
      float fN;
      if (!parseForce(ln, fN)) { Line().s("  could not read that - type e.g. 101, 10.3kg or 22.7lb (or x)").end(); return; }
      if (gPtN < sizeof(gPts) / sizeof(gPts[0])) { gPts[gPtN].x = gPendX; gPts[gPtN].f = fN; gPtN++; }
      Line().s("  point ").u(gPtN).s(": ").f(fN, 1).s(" N  <->  ").f(gPendX, 5).s(" V   (ratio ")
            .f(fN / gPendX, 1).s(" N/V)").end();
      if (fN > CHIP_MAX_N) Line().s("  note: above 270 N, the 30 MPa limit of the PL033").end();
      if (gPtN >= 15 && gPtN % 15 == 0) { kFitOrPress(); return; }
      gCState = C_K_PRESS;
      capArm(CP_KPOINT, (int8_t)gCalCh, false);
      return;
    }

    case C_K_FIT:
      if (eqCI(ln, "a"))  { kAccept(false); return; }
      if (eqCI(ln, "aq") && gFit.qOk) { kAccept(true); return; }
      if (eqCI(ln, "m"))  { gCState = C_K_PRESS; capArm(CP_KPOINT, (int8_t)gCalCh, false); return; }
      if (eqCI(ln, "u"))  { if (gPtN) gPtN--; Line().s("  points: ").u(gPtN).end(); kFitOrPress(); return; }
      if (eqCI(ln, "x"))  { Line().s("  points discarded, k unchanged").end(); calMenu(); return; }
      Line().s("  a, ").s(gFit.qOk ? "aq, " : "").s("m, u or x").end();
      return;

    case C_OFF_PRESS:
      if (eqCI(ln, "f")) { if (gOffN >= 3) { capCancel(); offFinish(); } else Line().s("  need at least 3 presses").end(); return; }
      if (eqCI(ln, "q")) { calMenu(); return; }
      return;

    case C_OFF_DECIDE:
      if (eqCI(ln, "a")) {
        for (uint8_t i = 0; i < NCH; i++) gCal.off[i] = gOffMean[i];
        gCal.offCal = true;
        Line().s("  timing offsets stored").end();
        calMenu();
        return;
      }
      if (eqCI(ln, "r")) { offStart(); return; }
      if (eqCI(ln, "q")) { calMenu(); return; }
      return;

    case C_VER_SELECT:
      if (eqCI(ln, "1") || eqCI(ln, "2") || eqCI(ln, "3") || eqCI(ln, "a")) {
        gVerFocus = (ln[0] == 'a' || ln[0] == 'A') ? (int8_t)-1 : (int8_t)(ln[0] - '1');
        gCState = C_VER_PRESS;
        capArm(CP_VERIFY, gVerFocus, false);
        return;
      }
      if (eqCI(ln, "q")) { calMenu(); return; }
      return;

    case C_VER_PRESS:
      if (eqCI(ln, "q")) { calMenu(); return; }
      return;

    case C_VER_REF: {
      if (eqCI(ln, "x")) { gCState = C_VER_PRESS; capArm(CP_VERIFY, gVerFocus, true); return; }
      if (eqCI(ln, "q")) { calMenu(); return; }
      float fN;
      if (!parseForce(ln, fN)) { Line().s("  could not read that - type e.g. 101, 10.3kg or 22.7lb (or x)").end(); return; }
      float meas = gVerFocus < 0 ? gPendVer[NCH] : gPendVer[gVerFocus];
      float err = 100.0f * (meas - fN) / fN;
      Line().s("  reference ").f(fN, 1).s(" N, measured ").f(meas, 1).s(" N, error ").sf(err, 2).s(" %  ")
            .s(absf(err) <= VERIFY_OK_PCT ? "PASS" : absf(err) <= VERIFY_RECAL_PCT ? "MARGINAL (recheck)" : "RECALIBRATE (> 3 %)").end();
      Line().s("  next press, or q = menu").end();
      gCState = C_VER_PRESS;
      capArm(CP_VERIFY, gVerFocus, true);
      return;
    }
  }
}

static void onLine(char* ln) {
  if (gMon) { gMon = false; Line().s("(live view stopped)").end(); if (!*ln) return; }
  if (preCI(ln, "CAL1 ") && gMode != MODE_SELFTEST) {   // a pasted CAL line, anywhere idle
    bool busy = gMode == MODE_CAL && gCState != C_MENU;
    if (!busy) {
      if (gMode == MODE_STARTUP) { gPasteNext = true; startupCmd(ln); }
      else { gPasteNext = false; tryLoadCal(ln); }
      return;
    }
  }
  switch (gMode) {
    case MODE_SELFTEST: return;
    case MODE_STARTUP:  startupCmd(ln); return;
    case MODE_MEASURE:  measureCmd(ln); return;
    case MODE_CAL:      calCmd(ln); return;
  }
}

/* ============================ EVENT HANDLING ============================== */

static void onCaptureDone() {
  switch (gCap.cfg.purpose) {
    case CP_MEASURE: measureDone(); break;
    case CP_COND:    condDone();    break;
    case CP_TAU:     tauDone();     break;
    case CP_KPOINT:  kDone();       break;
    case CP_VERIFY:  verDone();     break;
    case CP_OFFSET:  offDone();     break;
  }
}

static void onArmEvents() {
  CapArm& a = gCap.arm;
  if (a.warmEvent) {
    a.warmEvent = false;
    switch (gCap.cfg.purpose) {
      case CP_MEASURE: if (!gCap.cfg.quiet) Line().s("Waiting for actuation (squeeze)...").end(); break;
      case CP_COND:    if (!gCap.cfg.quiet) Line().s("[ready - squeeze ").s(CHN[gCalCh]).s("]").end(); break;
      case CP_TAU:     Line().s("[ready - press and HOLD ").s(CHN[gCalCh]).s(" ~10 s, then release]").end(); break;
      case CP_KPOINT:  kPrompt(); break;
      case CP_VERIFY:  if (!gCap.cfg.quiet) Line().s("[ready - press the known load]").end(); break;
      case CP_OFFSET:  Line().s("[ready - centre-press the plate (").u(gOffN + 1).s(" of 10)]").end(); break;
    }
  }
  if (a.negEvent) {
    for (uint8_t i = 0; i < NCH; i++) {
      if (!(a.negEvent & (1u << i))) continue;
      Line().s("note: ").s(CHN[i]).s(" went NEGATIVE (").f(a.negF[i], 1).s(" N) while waiting. If that").end();
      Line().s("      happened on squeeze, swap that sensor's red and black wires (spec 4.1);").end();
      Line().s("      if the jaws were just opened, ignore this.").end();
    }
    a.negEvent = 0;
  }
}

static void processSamples() {
  Sample s;
  uint16_t budget = 400;                 // keep the UI responsive
  while (budget-- && popSample(s)) {
    for (uint8_t i = 0; i < NCH; i++) {  // replace failed conversions
      if (s.raw[i] < 0) s.raw[i] = gLastGood[i]; else gLastGood[i] = s.raw[i];
    }
    histPush(s);
    if (!gCap.active) idleFeed(s);

    if (gMode == MODE_SELFTEST) {        // Welford mean / variance
      if (gStN == 0) gStT0 = s.t0;
      gStT1 = s.t0;
      gStN++;
      for (uint8_t i = 0; i < NCH; i++) {
        double v = toV(s.raw[i]), d = v - gStMean[i];
        gStMean[i] += d / gStN;
        gStM2[i] += d * (v - gStMean[i]);
      }
      if ((uint32_t)(gStT1 - gStT0) >= SELFTEST_US) {
        for (uint8_t i = 0; i < NCH; i++) gVeq[i] = (float)gStMean[i];   // settled at power-up
        selfTestReport();
        startupPrompt();
      }
      continue;
    }
    if (gCap.armed || gCap.active) capFeed(s);
    if (gCap.done) { gCap.done = false; onCaptureDone(); }
    onArmEvents();
  }
}

static void pollButton() {
  if (BTN_PIN < 0) return;
  static int      last = HIGH, stable = HIGH;
  static uint32_t tChange = 0;
  int v = digitalRead(BTN_PIN);
  if (v != last) { last = v; tChange = millis(); }
  if ((uint32_t)(millis() - tChange) >= 25 && stable != last) {
    stable = last;
    if (stable == LOW) {                                       // pressed
      if (gMode == MODE_MEASURE) measureReset();
      else if (gMode == MODE_STARTUP) { Line().s("Button: using STANDARD calibration parameters.").end(); enterMeasure(); }
    }
  }
}

static void uiTick() {
  if (gMode == MODE_STARTUP && (uint32_t)(millis() - gStartupMs) >= AUTOSKIP_MS) {
    Line().s("No answer in 60 s: using STANDARD calibration parameters.").end();
    enterMeasure();
  }
  if (gMon && (uint32_t)(millis() - gMonMs) >= 200) {
    gMonMs = millis();
    const Sample& s = histBack(0);
    Line l;
    float tot = 0;
    l.s("F");
    for (uint8_t i = 0; i < NCH; i++) {
      float F = gCap.active ? gCap.run.F[i] : forceFromX(i, toV(s.raw[i]) - gMonBase[i]);
      tot += F;
      l.s("  CH").u(i + 1).c(' ').f(F, 1);
    }
    l.s("  total ").f(tot, 1).s(" N   V");
    for (uint8_t i = 0; i < NCH; i++) l.c(' ').f(toV(s.raw[i]), 3);
    l.end();
  }
}

/* ============================ SETUP / LOOP ================================ */

void setup() {
  Monitor.begin();
  if (BTN_PIN >= 0) pinMode(BTN_PIN, INPUT_PULLUP);
  calDefaults(gCal);

  analogReadResolution(ADCRES_BITS);
  for (uint8_t i = 0; i < NCH; i++) (void)analogRead(PIN[i]);   // init ADC + pins here

  gHz = sys_clock_hw_cycles_per_sec();
  if (gHz == 0) gHz = 1;
  uint32_t best = 0xFFFFFFFFUL, c0 = k_cycle_get_32();           // timer resolution
  for (uint8_t n = 0; n < 50;) {
    uint32_t c1 = k_cycle_get_32();
    if (c1 != c0) { if (c1 - c0 < best) best = c1 - c0; c0 = c1; n++; }
  }
  gTickNs = (uint32_t)((uint64_t)best * 1000000000ULL / gHz);
  gTbLastCyc = k_cycle_get_32();
  uint32_t a = tbNowUs();                                         // one scan's duration
  for (uint8_t i = 0; i < NCH; i++) (void)analogRead(PIN[i]);
  gScanUs = tbNowUs() - a;

  gThreaded = startSamplerThread();
  if (!gThreaded) gNextUs = tbNowUs();

  rule('=');
  Line().s("Clamp-force & alignment monitor  |  Arduino UNO Q + 3 x PL033.31").end();
  Line().s("Charge amps: Cf 6.8 uF, Rf 10 Mohm (tau 68 s), VREF ~0.30 V").end();
  Line().s("Sampling ").u(FS_HZ).s(" Hz x 3 channels, 14-bit ADC (").f(V_PER_CNT * 1000.0f, 2).s(" mV per count)").end();
  Line().s("Keep the jaws open for the self-test...").end();
  gMode = MODE_SELFTEST;
}

void loop() {
  if (!gThreaded) samplerInlinePoll();
  processSamples();
  pumpOutput();                 // at most one RPC per loop
  pollInput();                  // at most one RPC every 25 ms
  pollButton();
  uiTick();
  if (gThreaded && gOutUsed == 0 && __atomic_load_n(&gQHead, __ATOMIC_ACQUIRE) == gQTail) delay(1);
}
