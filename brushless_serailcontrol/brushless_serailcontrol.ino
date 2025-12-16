#include <Servo.h>

// ====== Declare type first (fix Arduino auto-prototype issue) ======
struct Step {
  int L;
  int R;
  unsigned long ms;
};

// ====== Prototypes ======
int THR(int pct);
void setMotors(int leftUs, int rightUs);
Step* getPattern(int &len);
void printHelp();

void ensureAttachedAndArmed();   // <-- important fix
void disarmOutputs();

void startRun();
void stopRun();

Servo escL, escR;

// ---------- TOP VIEW: LEFT / RIGHT ----------
const int PIN_LEFT  = 10;
const int PIN_RIGHT = 11;

// ESC signal range (IMPORTANT: allow 1000 for arming)
const int ESC_SIGNAL_MIN_US = 1000;  // <-- used to ARM most ESCs
const int ESC_SIGNAL_MAX_US = 2500;  // <-- as you requested

// Requested values for behavior
const int CALM_US = 1500;  // idle/calm (your requirement)
const int MAX_US  = 2500;  // max (your requirement)

// Arming sequence timings
const unsigned long ARM_LOW_MS   = 3000;  // hold 1000us to arm
const unsigned long ARM_CALM_MS  = 800;   // settle at 1500us after arm

// Idle behavior (no job): detach outputs after some time
const unsigned long IDLE_DISARM_MS = 2000; // idle 2s -> detach (no PWM)

// ---------- Runtime state ----------
bool outputsAttached = false;
bool escArmed = false;

unsigned long idleSinceMs = 0;

// ✅ รวม N1+N2 เป็นโหมดเดียว: MODE_N
enum Mode { MODE_N, MODE_STORM };
Mode currentMode = MODE_N;

unsigned long runDurationMs = 30000; // default 30s
bool running = false;

int stepIndex = 0;
unsigned long stepStart = 0;
unsigned long stopAt = 0;

String line;

// ---------- Helpers ----------
int clampUs(int us) {
  if (us < ESC_SIGNAL_MIN_US) return ESC_SIGNAL_MIN_US;
  if (us > ESC_SIGNAL_MAX_US) return ESC_SIGNAL_MAX_US;
  return us;
}

int THR(int pct) {
  // percent 0..100 -> CALM..MAX
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  long us = CALM_US + (long)(MAX_US - CALM_US) * pct / 100;
  return (int)us;
}

// ===== Motor ON/OFF happens right here =====
// - CALM_US (1500) means calm/idle (your setting)
// - > CALM_US means running (throttle up)
void setMotors(int leftUs, int rightUs) {
  if (!outputsAttached) return;
  escL.writeMicroseconds(clampUs(leftUs));   // <-- LEFT motor command
  escR.writeMicroseconds(clampUs(rightUs));  // <-- RIGHT motor command
}

// ---------- Patterns ----------
// ✅ N = N1 ต่อด้วย N2 แล้ววนซ้ำ
Step N[] = {
  // ---- N1 ----
  { CALM_US, CALM_US, 1200 },     // idle both
  { THR(15), CALM_US,  600 },     // LEFT gentle, RIGHT idle
  { CALM_US, CALM_US,  500 },     // idle both
  { CALM_US, THR(15),  600 },     // RIGHT gentle, LEFT idle
  { CALM_US, CALM_US,  700 },     // idle both

  // ---- N2 ----
  { THR(10), THR(10),  700 },     // both very gentle
  { CALM_US, CALM_US,  900 },     // idle both
  { THR(18), THR(18),  500 },     // both gentle
  { CALM_US, CALM_US, 1200 },     // idle both
};
const int N_LEN = sizeof(N) / sizeof(N[0]);

// ===== STORM: 15s per loop (15000 ms exactly) =====
// LEFT = มอเตอร์ซ้าย, RIGHT = มอเตอร์ขวา (Top view)
// CALM_US = 1500 (นิ่ง), MAX_US = 2500 (แรงสุด)
Step ST[] = {
  { CALM_US,  CALM_US,  800 }, // (0.8s)  lull

  { THR(55),  CALM_US, 1200 }, // (1.2s)  gust: left
  { CALM_US,  CALM_US,  400 }, // (0.4s)  lull

  { CALM_US,  THR(65), 1200 }, // (1.2s)  gust: right
  { CALM_US,  CALM_US,  400 }, // (0.4s)  lull

  { THR(80),  CALM_US,  400 }, // (0.4s)  left jerk
  { CALM_US,  CALM_US,  150 }, // (0.15s) off
  { CALM_US,  THR(80),  400 }, // (0.4s)  right jerk
  { CALM_US,  CALM_US,  150 }, // (0.15s) off
  { THR(80),  CALM_US,  400 }, // (0.4s)  left jerk
  { CALM_US,  CALM_US,  150 }, // (0.15s) off
  { CALM_US,  THR(80),  400 }, // (0.4s)  right jerk
  { CALM_US,  CALM_US,  150 }, // (0.15s) off

  { THR(40),  THR(85), 1200 }, // (1.2s)  both (R stronger)
  { THR(85),  THR(40), 1200 }, // (1.2s)  both (L stronger)

  { THR(100), THR(100), 1500 }, // (1.5s) peak

  { CALM_US,  CALM_US,  800 },  // (0.8s) recovery

  { THR(25),  THR(25), 1500 },  // (1.5s) rolling swell

  { CALM_US,  CALM_US,  500 },  // (0.5s) lull
  { CALM_US,  THR(90),  700 },  // (0.7s) last gust: right
  { CALM_US,  CALM_US,  500 },  // (0.5s) end calm
};
const int ST_LEN = sizeof(ST) / sizeof(ST[0]);

Step* getPattern(int &len) {
  if (currentMode == MODE_N) { len = N_LEN; return N; }
  len = ST_LEN; return ST;
}

// ---------- Attach + ARM (FIXED) ----------
void ensureAttachedAndArmed() {
  if (!outputsAttached) {
    escL.attach(PIN_LEFT,  ESC_SIGNAL_MIN_US, ESC_SIGNAL_MAX_US);
    escR.attach(PIN_RIGHT, ESC_SIGNAL_MIN_US, ESC_SIGNAL_MAX_US);
    outputsAttached = true;
    escArmed = false;
  }

  if (!escArmed) {
    setMotors(ESC_SIGNAL_MIN_US, ESC_SIGNAL_MIN_US);
    delay(ARM_LOW_MS);

    setMotors(CALM_US, CALM_US);
    delay(ARM_CALM_MS);

    escArmed = true;
  }
}

void disarmOutputs() {
  if (!outputsAttached) return;

  setMotors(CALM_US, CALM_US);
  delay(200);

  escL.detach();
  escR.detach();
  outputsAttached = false;
  escArmed = false;
}

// ---------- Run control ----------
void startRun() {
  ensureAttachedAndArmed();

  int len;
  Step* pat = getPattern(len);

  stepIndex = 0;
  stepStart = millis();
  stopAt = stepStart + runDurationMs;
  running = true;

  setMotors(pat[0].L, pat[0].R);
}

void stopRun() {
  running = false;
  ensureAttachedAndArmed();
  setMotors(CALM_US, CALM_US);
  idleSinceMs = millis();
}

// ---------- Serial help ----------
void printHelp() {
  Serial.println("Commands:");
  Serial.println("  MODE N       -> normal pattern (N1+N2 combined)");
  Serial.println("  MODE STORM   -> storm pattern");
  Serial.println("  TIME <sec>   -> run time seconds (e.g. TIME 60)");
  Serial.println("  START        -> start running");
  Serial.println("  STOP         -> stop (CALM=1500, then auto-detach)");
}

void setup() {
  Serial.begin(115200);

  ensureAttachedAndArmed();

  idleSinceMs = millis();
  Serial.println("Ready. Type HELP (set Newline or Both NL&CR).");
}

void loop() {
  // ---- Read commands as lines ----
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      line.trim();
      if (line.length()) {
        String cmd = line; cmd.toUpperCase();

        if (cmd == "HELP") {
          printHelp();

        } else if (cmd.startsWith("MODE ")) {
          String m = cmd.substring(5); m.trim();

          // ✅ โหมดใหม่: N / STORM
          // (ยังรับ N1, N2 เพื่อความเข้ากันได้ — จะพาไป N เหมือนกัน)
          if (m == "N" || m == "N1" || m == "N2") currentMode = MODE_N;
          else if (m == "STORM") currentMode = MODE_STORM;

          Serial.print("Mode set to: "); Serial.println(m);

        } else if (cmd.startsWith("TIME ")) {
          long sec = cmd.substring(5).toInt();
          if (sec < 1) sec = 1;
          runDurationMs = (unsigned long)sec * 1000UL;
          Serial.print("Time set to: "); Serial.print(sec); Serial.println(" sec");

        } else if (cmd == "START") {
          startRun();
          Serial.println("RUNNING");

        } else if (cmd == "STOP") {
          stopRun();
          Serial.println("STOPPED");

        } else {
          Serial.println("Unknown. Type HELP");
        }
      }
      line = "";
    } else {
      line += c;
    }
  }

  unsigned long now = millis();

  // ---- If no job running, auto-disarm (detach) after some idle time ----
  if (!running) {
    if (outputsAttached && (now - idleSinceMs >= IDLE_DISARM_MS)) {
      disarmOutputs();
    }
    return;
  }

  // ---- If running, update pattern ----
  if ((long)(now - stopAt) >= 0) {
    stopRun();
    Serial.println("DONE");
    return;
  }

  int len;
  Step* pat = getPattern(len);

  if (now - stepStart >= pat[stepIndex].ms) {
    stepIndex++;
    if (stepIndex >= len) stepIndex = 0;
    stepStart = now;

    setMotors(pat[stepIndex].L, pat[stepIndex].R);
  }
}
