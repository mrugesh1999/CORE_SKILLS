// Host simulation of piezo_force_monitor.ino on the spec's circuit.
// Build: g++ -std=c++17 -O2 -Istub sim_main.cpp -o sim   (or ./run_sim.sh)
// Run:   ./sim <full|skip|paste|noeol|inline> [quiet]
#include "Arduino.h"
#include "Arduino_RouterBridge.h"
#include "zephyr/kernel.h"
#include <string>
#include <vector>
#include <deque>
#include <random>
#include <algorithm>

uint64_t g_ns = 0;
static bool     simInSampler = false, simThreadMode = false, simForceInline = false;
static uint64_t simNextScanNs = 0;
static const uint64_t SIM_PERIOD_NS = 500000;      // 2 kHz, like the k_timer
static void   (*simSamplerHook)() = nullptr;

#include "../../piezo_force_monitor.ino"            // the sketch, unchanged

/* ------------------------------ physics ---------------------------------- */
struct SimChan { double S0, alpha, Cf, tau, tauIn, vref, qin, vamp, vout, eTau, eIn, eOut; };
static SimChan simCh[3];
static uint64_t simPhysNs = 0;
static const double SIM_DT = 10e-6;
struct SimPress { double t0, ramp, hold, fall, creep; double P[3]; double delay[3]; };
static std::vector<SimPress> simPresses;

static double simForceAt(int i, double t) {
  double F = 0;
  for (const SimPress& p : simPresses) {
    double ts = p.t0 + p.delay[i], te = p.t0 + p.ramp + p.hold;
    if (t < ts || p.P[i] <= 0) continue;
    double f;
    if (t < ts + p.ramp)  f = p.P[i] * (t - ts) / p.ramp;
    else if (t < te)      f = p.P[i] * (1 - p.creep * (t - (ts + p.ramp)));
    else if (t < te + p.fall) {
      double fh = p.P[i] * (1 - p.creep * (te - (ts + p.ramp)));
      f = fh * (1 - (t - te) / p.fall);
    } else f = 0;
    if (f > 0) F += f;
  }
  return F;
}
static void simPhysInit() {
  const double S0[3]    = { 15.0e-9, 19.0e-9, 13.5e-9 };       // C/N, three different chips
  const double alpha[3] = { 0.0, 2.0e-4, 0.0 };                // CH2 gets stiffer-sensitive (+5 % at 250 N)
  const double Cf[3]    = { 6.8e-6 * 1.03, 6.8e-6 * 0.97, 6.8e-6 };
  const double tau[3]   = { 71.5, 64.0, 68.0 };                // s, Rf*Cf with tolerances
  const double tauIn[3] = { 4.7e3 * 75e-9 * 1.2, 4.7e3 * 75e-9 * 0.85, 4.7e3 * 75e-9 };  // Rin*Cp
  const double vref[3]  = { 0.302, 0.299, 0.304 };
  for (int i = 0; i < 3; i++) {
    SimChan& c = simCh[i];
    c = SimChan();
    c.S0 = S0[i]; c.alpha = alpha[i]; c.Cf = Cf[i]; c.tau = tau[i]; c.tauIn = tauIn[i]; c.vref = vref[i];
    c.eTau = exp(-SIM_DT / c.tau); c.eIn = 1 - exp(-SIM_DT / c.tauIn); c.eOut = 1 - exp(-SIM_DT / 1e-4);
  }
}
static double simTrueK(int i) { return simCh[i].Cf / simCh[i].S0; }
static void simPhysAdvanceTo(uint64_t ns) {
  while (simPhysNs + 10000 <= ns) {
    simPhysNs += 10000;
    double t = simPhysNs * 1e-9;
    for (int i = 0; i < 3; i++) {
      SimChan& c = simCh[i];
      double F = simForceAt(i, t);
      double Q = c.S0 * (1 + c.alpha * F) * F;                 // Q = S*F
      double qn = c.qin + (Q - c.qin) * c.eIn;                  // Rin*Cp lag
      double dq = qn - c.qin; c.qin = qn;
      c.vamp = c.vamp * c.eTau + dq / c.Cf;                     // Cf*dV/dt = dQ/dt - V/Rf
      c.vout += (c.vamp - c.vout) * c.eOut;                     // Rout*Cflt
    }
  }
}

/* --------------------------- platform stubs ------------------------------ */
static std::mt19937 simRng(12345);
static std::normal_distribution<double> simNoise(0.0, 1.5);     // ADC noise, counts
static std::uniform_real_distribution<double> simU(0.0, 1.0);
static long simAdcErrInjected = 0;

void simAdvance(uint64_t ns) {
  uint64_t target = g_ns + ns;
  if (simThreadMode && simSamplerHook && !simInSampler) {        // the sampler thread preempts
    while (simNextScanNs <= target) {
      if (g_ns < simNextScanNs) g_ns = simNextScanNs;
      simInSampler = true; simSamplerHook(); simInSampler = false;
      simNextScanNs += SIM_PERIOD_NS;
      if (g_ns > target) target = g_ns;
    }
  }
  if (g_ns < target) g_ns = target;
}
int simAnalogRead(int pin) {
  simAdvance(40000);                                             // 40 us per conversion
  simPhysAdvanceTo(g_ns);
  int i = pin - A0;
  if (simU(simRng) < 2e-5) { simAdcErrInjected++; return -5; }   // -EIO now and then
  double v = simCh[i].vref + simCh[i].vout;
  if (v < 0) v = 0;
  if (v > 3.29) v = 3.29;
  long c = lround(v / 3.3 * 16383.0 + simNoise(simRng));
  if (c < 0) c = 0;
  if (c > 16383) c = 16383;
  return (int)c;
}
int simDigitalRead(int) { return HIGH; }
uint32_t k_cycle_get_32() { g_ns += 10; return (uint32_t)((g_ns * 160ULL) / 1000ULL); }  // wraps every 26.8 s
static k_thread_stack_t simStack;
k_thread_stack_t* k_thread_stack_alloc(size_t, int) { return simForceInline ? nullptr : &simStack; }
k_tid_t k_thread_create(struct k_thread* t, k_thread_stack_t*, size_t, void (*)(void*, void*, void*),
                        void*, void*, void*, int, uint32_t, k_timeout_t) { return t; }

static std::string simLog;
static std::deque<char> simInQ;
static bool simEcho = true, simAtLineStart = true;
MonitorStub Monitor;
size_t MonitorStub::write(const uint8_t* b, size_t n) {
  simAdvance(2000000ULL + (uint64_t)n * 87000ULL);               // RPC + 115200 baud
  for (size_t k = 0; k < n; k++) {
    char c = (char)b[k];
    simLog.push_back(c);
    if (!simEcho) continue;
    if (simAtLineStart && c != '\r' && c != '\n') { printf("[%8.3f] ", g_ns * 1e-9); simAtLineStart = false; }
    if (c == '\n') simAtLineStart = true;
    if (c != '\r') putchar(c);
  }
  return n;
}
int MonitorStub::available() { simAdvance(2000000ULL); return (int)simInQ.size(); }
int MonitorStub::read() {
  if (simInQ.empty()) return -1;
  char c = simInQ.front(); simInQ.pop_front();
  return (unsigned char)c;
}

/* ------------------------------ operator --------------------------------- */
enum SimStepT { S_TYPE, S_RAW, S_WAIT, S_PRESS, S_SCALE, S_SLEEP, S_CHOOSE };
struct SimStep { SimStepT t; std::string a, b, c; SimPress p; double secs; int focus; bool meas; };
static std::vector<SimStep> simSteps;
static size_t simIdx = 0, simCursor = 0;
static bool   simBegun = false, simFail = false;
static double simT0 = 0, simLastTrue[3], simLastSum = 0;
static int    simUnit = 0;
static std::string simFailMsg;
static std::vector<size_t> simMeasIdx;
static double nowS() { return g_ns * 1e-9; }

static void T(const std::string& s) { SimStep st{}; st.t = S_TYPE; st.a = s; simSteps.push_back(st); }
static void RAW(const std::string& s) { SimStep st{}; st.t = S_RAW; st.a = s; simSteps.push_back(st); }
static void W(const std::string& s, double to = 150) { SimStep st{}; st.t = S_WAIT; st.a = s; st.secs = to; simSteps.push_back(st); }
static void SLEEP(double s) { SimStep st{}; st.t = S_SLEEP; st.secs = s; simSteps.push_back(st); }
static void SCALE(int focus) { SimStep st{}; st.t = S_SCALE; st.focus = focus; simSteps.push_back(st); }
static void CHOOSE(const std::string& cond, const std::string& yes, const std::string& no) {
  SimStep st{}; st.t = S_CHOOSE; st.a = cond; st.b = yes; st.c = no; simSteps.push_back(st);
}
static void PRESS(const SimPress& p, bool meas = false) { SimStep st{}; st.t = S_PRESS; st.p = p; st.meas = meas; simSteps.push_back(st); }
static SimPress single(int ch, double P, double ramp, double hold) {
  SimPress p{}; p.ramp = ramp; p.hold = hold; p.fall = 0.08; p.creep = 0.003; p.P[ch] = P; return p;
}
static SimPress all3(double P1, double P2, double P3, double d1, double d2, double d3, double ramp, double hold) {
  SimPress p{}; p.ramp = ramp; p.hold = hold; p.fall = 0.03; p.creep = 0.002;
  p.P[0] = P1; p.P[1] = P2; p.P[2] = P3; p.delay[0] = d1; p.delay[1] = d2; p.delay[2] = d3; return p;
}

static void simType(const std::string& s, const char* eol) {
  for (char c : s) simInQ.push_back(c);
  for (const char* e = eol; *e; e++) simInQ.push_back(*e);
}
struct SimRec { uint32_t t[3]; int16_t raw[3]; double qcf[3], w[3]; };
static std::vector<SimRec> simRec;
static void simScanHook() {
  scanOnce();
  const Sample& s = gQ[(gQHead - 1) & (QN - 1)];
  SimRec r; for (int i = 0; i < 3; i++) r.t[i] = sT(s, (uint8_t)i); memcpy(r.raw, s.raw, sizeof r.raw);
  for (int i = 0; i < 3; i++) { r.qcf[i] = simCh[i].qin / simCh[i].Cf; r.w[i] = simCh[i].vamp; }
  simRec.push_back(r);
}
static void simNext() { simIdx++; simBegun = false; }
static void simOperatorTick() {
  if (simFail || simIdx >= simSteps.size()) return;
  SimStep& st = simSteps[simIdx];
  if (!simBegun) {
    simBegun = true; simT0 = nowS();
    if (st.t == S_PRESS) {
      st.p.t0 = nowS() + 0.3;
      simPresses.push_back(st.p);
      if (st.meas) simMeasIdx.push_back(simPresses.size() - 1);
      double te = st.p.t0 + st.p.ramp + st.p.hold;
      simLastSum = 0;
      for (int i = 0; i < 3; i++) { simLastTrue[i] = simForceAt(i, te - 1e-6); simLastSum += simLastTrue[i]; }
    }
  }
  switch (st.t) {
    case S_TYPE:  if (nowS() - simT0 >= 0.3) { simType(st.a, (simIdx % 3 == 0) ? "\r\n" : "\n"); simNext(); } break;
    case S_RAW:   if (nowS() - simT0 >= 0.3) { simType(st.a, ""); simNext(); } break;
    case S_SLEEP: if (nowS() - simT0 >= st.secs) simNext(); break;
    case S_WAIT: {
      size_t p = simLog.find(st.a, simCursor);
      if (p != std::string::npos) { simCursor = p + st.a.size(); simNext(); }
      else if (nowS() - simT0 > st.secs) { simFail = true; simFailMsg = "timeout waiting for: " + st.a; }
      break;
    }
    case S_PRESS: if (nowS() >= st.p.t0 + st.p.ramp + st.p.hold + st.p.fall + 0.05) simNext(); break;
    case S_SCALE:
      if (nowS() - simT0 >= 0.5) {
        double v = (st.focus >= 0 ? simLastTrue[st.focus] : simLastSum) * (1.0 + 0.002 * simNoise(simRng) / 1.5);
        char buf[48];
        switch (simUnit++ % 3) {
          case 0:  snprintf(buf, sizeof buf, "%.1f", v); break;
          case 1:  snprintf(buf, sizeof buf, "%.2fkg", v / 9.80665); break;
          default: snprintf(buf, sizeof buf, "%.1f lb", v / 4.4482216); break;
        }
        simType(buf, "\n");
        if (getenv("SIMDBG") && st.focus >= 0) {
          const SimPress& p = simPresses.back();
          int i = st.focus;
          double te = p.t0 + p.ramp + p.hold, tp = te - 0.35;
          double Fp = simForceAt(i, tp);
          double Xtrue = simCh[i].S0 * (1 + simCh[i].alpha * Fp) * Fp / simCh[i].Cf;
          const CapRun& r = gCap.run;
          printf("DBG ch%d F@plateau %.2f  plX %.5f  Xtrue %.5f  err %+.2f%%  base %.5f  Veq %.5f  tau_set %.2f  nSmp %u plN %u\n",
                 i + 1, Fp, r.plX[i], Xtrue, 100 * (r.plX[i] - Xtrue) / Xtrue, r.base[i], gVeq[i], gCal.tau[i], r.nSmp, r.plN);
        }
        simNext();
      }
      break;
    case S_CHOOSE: {            // look at the last ~1.5 kB of output (the fit report)
      size_t from = simCursor > 1500 ? simCursor - 1500 : 0;
      simType(simLog.find(st.a, from) != std::string::npos ? st.b : st.c, "\n");
      printf("OPERATOR chose: %s\n", simLog.find(st.a, from) != std::string::npos ? st.b.c_str() : st.c.c_str());
    }
      simNext();
      break;
  }
}

/* ------------------------------ scenarios -------------------------------- */
static const SimPress MEAS1 = all3(280, 330, 260, 0.0008, 0.0, 0.0020, 0.030, 1.2);  // CH2 first
static const SimPress MEAS2 = all3(290, 292, 288, 0.0, 0.0, 0.0, 0.030, 1.2);        // aligned

static void buildFull() {
  W("CALIBRATION SETUP"); T("c"); W("CALIBRATION MENU");
  const double lv[5] = { 50, 100, 150, 200, 265 };
  for (int ch = 0; ch < 3; ch++) {
    T(std::to_string(ch + 1)); W("step 1 of 3");
    for (int k = 0; k < 3; k++) { PRESS(single(ch, 150, 0.3, 0.4)); W("press " + std::to_string(k + 1) + ":"); }
    T("n"); W("step 2 of 3"); SLEEP(11);
    W("[ready - press and HOLD"); PRESS(single(ch, 180, 0.5, 10.0)); W("tau from this run");
    T("m"); SLEEP(15);
    W("[ready - press and HOLD"); PRESS(single(ch, 160, 0.5, 10.0)); W("tau from this run");
    T("a"); W("step 3 of 3");
    for (int L = 0; L < 5; L++)
      for (int r = 0; r < 3; r++) {
        W("[ready - press"); PRESS(single(ch, lv[L] * (1.0 + 0.02 * (r - 1)), 0.5, 2.5));
        W("Type the scale reading"); SCALE(ch);
      }
    W("FIT of reference"); W("a = accept linear"); CHOOSE("aq recommended", "aq", "a"); W("CALIBRATION MENU");
  }
  T("t"); W("TIMING OFFSETS");
  for (int k = 0; k < 10; k++) { W("[ready - centre-press"); PRESS(all3(120, 120, 120, 0, 0, 0, 0.03, 0.3)); }
  W("Timing offsets from"); T("a"); W("CALIBRATION MENU");
  T("v"); W("VERIFY A KNOWN LOAD"); T("a"); W("[ready - press the known load]");
  PRESS(all3(200, 210, 190, 0, 0, 0, 0.5, 2.5)); W("Type the scale reading"); SCALE(-1); W("error"); T("q"); W("CALIBRATION MENU");
  T("note 2026-10-28 22C"); W("Note set");
  T("x"); W("Waiting for actuation");
  PRESS(MEAS1, true); W("CSV,");
  T("r"); W("Waiting for actuation");
  PRESS(MEAS2, true); W("CSV,");
  T("show"); W("CAL1 k=");
}
static void buildSkip() {
  W("CALIBRATION SETUP"); T("s"); W("Waiting for actuation");
  PRESS(MEAS1, true); W("CSV,"); T("r"); W("Waiting for actuation");
  PRESS(MEAS2, true); W("CSV,");
}
static void buildPaste(const std::string& cal) {
  W("CALIBRATION SETUP"); T(cal); W("Calibration loaded"); W("Waiting for actuation");
  PRESS(MEAS1, true); W("CSV,");
}
static void buildNoEol() {   // monitor set to "No line ending"
  W("CALIBRATION SETUP"); RAW("s"); W("Waiting for actuation");
  PRESS(MEAS1, true); W("CSV,");
}

/* ------------------------------- checks ---------------------------------- */
static int simChecksFailed = 0;
static void check(bool ok, const char* what, double got, double want, double tol) {
  printf("  %-44s got %10.4f  want %10.4f  (tol %.4f)  %s\n", what, got, want, tol, ok ? "ok" : "**FAIL**");
  if (!ok) simChecksFailed++;
}
static void checkMeas(size_t pi, const std::string& csv, bool calibrated) {
  const SimPress& p = simPresses[pi];
  double thr = gThrN;
  // true threshold crossings, peak of the true total, shares there
  double tc[3];
  for (int i = 0; i < 3; i++) tc[i] = p.t0 + p.delay[i] + thr / p.P[i] * p.ramp;
  double best = 0, tb = 0;
  for (double t = p.t0; t < p.t0 + p.ramp + p.hold; t += 10e-6) {
    double s = simForceAt(0, t) + simForceAt(1, t) + simForceAt(2, t);
    if (s > best) { best = s; tb = t; }
  }
  int first = 0;
  for (int i = 1; i < 3; i++) if (tc[i] < tc[first]) first = i;
  // parse CSV,shot,first,skew,on1,on2,on3,clamp,s1,s2,s3,resid,dur
  std::vector<std::string> f;
  size_t a = 0;
  while (true) { size_t b = csv.find(',', a); f.push_back(csv.substr(a, b == std::string::npos ? std::string::npos : b - a)); if (b == std::string::npos) break; a = b + 1; }
  printf("Measurement check (%s): %s\n", calibrated ? "calibrated" : "standard k", csv.c_str());
  double spread = 0; for (int i = 0; i < 3; i++) spread = std::max(spread, (tc[i] - tc[first]) * 1000.0);
  double unc = atof(f[4].c_str());
  if (spread > 2 * unc) check(f[2] == std::string(CHN[first]), "first hit channel (1=match)", f[2] == std::string(CHN[first]), 1, 0);
  else printf("  true spread %.3f ms <= 2 x reported uncertainty %.3f ms: order not meaningful (firmware should say simultaneous)\n", spread, unc);
  check(fabs(atof(f[3].c_str()) - spread) <= std::max(0.06, 2 * unc), "skew [ms] vs truth (<= max(0.06, 2u))", atof(f[3].c_str()), spread, std::max(0.06, 2 * unc));
  for (int i = 0; i < 3; i++) {
    char nm[48]; snprintf(nm, sizeof nm, "onset %s rel. first [ms]", CHN[i]);
    double want = (tc[i] - tc[first]) * 1000.0, got = atof(f[5 + i].c_str());
    double tol = calibrated ? 0.06 : std::max(0.06, 2 * unc);   // uncalibrated: the firmware's own +/- 2u
    check(fabs(got - want) <= tol, nm, got, want, tol);
  }
  double clamp = atof(f[8].c_str());
  if (calibrated) check(fabs(clamp - best) / best <= 0.015, "clamp force [N] (1.5 %)", clamp, best, best * 0.015);
  else printf("  clamp force (standard k)  got %.1f, true %.1f  (%.1f %%)\n", clamp, best, 100 * (clamp - best) / best);
  for (int i = 0; i < 3; i++) {
    double want = 100.0 * simForceAt(i, tb) / best;
    char nm[48]; snprintf(nm, sizeof nm, "share %s at peak [%%]", CHN[i]);
    if (calibrated) check(fabs(atof(f[9 + i].c_str()) - want) <= 0.6, nm, atof(f[9 + i].c_str()), want, 0.6);
    else printf("  %s (standard k)  got %s, true %.2f\n", nm, f[9 + i].c_str(), want);
  }
  check(fabs(atof(f[12].c_str())) <= 0.5, "self-check residual [%] (|r| <= 0.5)", atof(f[12].c_str()), 0, 0.5);
}

int main(int argc, char** argv) {
  std::string sc = argc > 1 ? argv[1] : "full";
  simEcho = !(argc > 2 && std::string(argv[2]) == "quiet");
  if (sc == "inline") simForceInline = true;
  simPhysInit();
  simSamplerHook = &simScanHook;
  if (sc == "full") buildFull();
  else if (sc == "skip" || sc == "inline") buildSkip();
  else if (sc == "paste") buildPaste(argc > 3 ? argv[3] : "");
  else if (sc == "noeol") buildNoEol();
  setup();
  if (gThreaded) { simThreadMode = true; simNextScanNs = g_ns + SIM_PERIOD_NS; }
  while (nowS() < 3600 && !simFail && simIdx < simSteps.size()) {
    loop();
    simAdvance(5000);
    simOperatorTick();
  }
  for (int k = 0; k < 4000; k++) { loop(); simAdvance(5000); }    // flush output
  printf("\n================= SIMULATION SUMMARY (%s) =================\n", sc.c_str());
  printf("sim time %.1f s, steps %zu/%zu, ADC errors injected %ld (firmware counted %u), drops %u, overruns %u\n",
         nowS(), simIdx, simSteps.size(), simAdcErrInjected, (unsigned)gAdcErrors, (unsigned)gDrops, (unsigned)gOverruns);
  if (simFail) { printf("OPERATOR FAILED: %s\n", simFailMsg.c_str()); return 2; }
  // collect CSV lines
  std::vector<std::string> csv;
  for (size_t a = simLog.find("\nCSV,"); a != std::string::npos; a = simLog.find("\nCSV,", a + 1)) {
    size_t b = simLog.find('\r', a + 1); csv.push_back(simLog.substr(a + 1, b - a - 1));
  }
  bool calibrated = (sc == "full" || sc == "paste");
  for (size_t m = 0; m < simMeasIdx.size() && m < csv.size(); m++) checkMeas(simMeasIdx[m], csv[m], calibrated);
  if (sc == "full") {
    size_t a = simLog.rfind("CAL1 k=");
    size_t b = simLog.find('\r', a);
    std::string cal = simLog.substr(a, b - a);
    printf("CAL line: %s\n", cal.c_str());
    FILE* fp = fopen("cal_line.txt", "w"); if (fp) { fputs(cal.c_str(), fp); fclose(fp); }
    for (int i = 0; i < 3; i++) {
      char nm[48];
      snprintf(nm, sizeof nm, "tau %s [s] (2 %%)", CHN[i]);
      check(fabs(gCal.tau[i] - simCh[i].tau) / simCh[i].tau <= 0.02, nm, gCal.tau[i], simCh[i].tau, simCh[i].tau * 0.02);
      // compare force predicted by the calibration with the truth at 100 N and 250 N
      for (double Ft : { 100.0, 250.0 }) {
        double X = simCh[i].S0 * (1 + simCh[i].alpha * Ft) * Ft / simCh[i].Cf;
        double Fc = gCal.k[i] * X + gCal.q[i] * X * fabs(X);
        snprintf(nm, sizeof nm, "calibrated F %s at %.0f N (1 %%)", CHN[i], Ft);
        check(fabs(Fc - Ft) / Ft <= 0.01, nm, Fc, Ft, Ft * 0.01);
      }
      printf("  %s: k %.2f N/V (true small-signal %.2f), q %.3f, offset %.1f us (tauIn %.1f us)\n",
             CHN[i], gCal.k[i], simTrueK(i), gCal.q[i], gCal.off[i], simCh[i].tauIn * 1e6);
    }
  }
  printf("checks failed: %d\n", simChecksFailed);
  return simChecksFailed ? 1 : 0;
}
