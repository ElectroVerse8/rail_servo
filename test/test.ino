/*
  Rail Servo – ESP32 + AccelStepper + AsyncWebServer
  - Creates a Wi-Fi AP for control
  - Minimal, modern web UI (all inline; no external deps)
  - Homing to 3 switches (SW1..SW3), with full startup homing
  - Position units in the UI: mm; label shows cm
  - Live position updates via Server-Sent Events (SSE)
  - Auto-disable stepper after idle

  Notes:
  - Limit switches are ACTIVE LOW.
  - Positions are referenced such that Home1 is a known physical offset (home1PosCm).
  - Slider bounds come from railMinCm .. railMaxCm (converted to mm in UI).
*/

#include <WiFi.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AccelStepper.h>

// ====== Wi-Fi AP credentials ======
static const char* kAP_SSID     = "rail_servo";
static const char* kAP_PASSWORD = "123456789";

// ====== Pins ======
static const int STEP_PIN = 21;
static const int DIR_PIN  = 22;
static const int EN_PIN   = 23;

static const int SW1_PIN  = 17;  // Home1
static const int SW2_PIN  = 18;  // Home2
static const int SW3_PIN  = 19;  // Home3

// ====== Mechanics / Motion ======
static const float SCREW_LEAD_MM   = 4.0f;  // mm/rev
static const int   MOTOR_STEPS     = 200;   // full steps/rev
static const int   MICROSTEPS      = 8;     // driver microstepping
static const float ACCEL_MM_S2     = 50.0f; // mm/s^2
static const float STEPS_PER_MM    = (MOTOR_STEPS * MICROSTEPS) / SCREW_LEAD_MM;

// Travel limits and Home1 location (in centimeters; UI works in mm)
static float railMinCm   = -12.5f;
static float railMaxCm   =  12.0f;
static float home1PosCm  = -11.8f;   // physical Home1 position (user measured)

static int   uiSpeedPercent = 30;    // 1..100; mapped to real max speed

// Startup full-homing speed (constant speed search)
static float startupHomeSpeedMmS = 15.0f;

// ====== Server / Stepper ======
AsyncWebServer        server(80);
AsyncEventSource      events("/events");
AccelStepper          stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Saved positions for Home2/Home3 (in steps, relative to current machine 0)
long home2PosSteps = 0;
long home3PosSteps = 0;

// Flags / timers
volatile bool wantFullHome = false;
volatile bool abortAll     = false;
unsigned long lastEnableMs = 0;

// SSE push cadence
unsigned long lastSseMs = 0;

// ====== Homing State Machine ======
enum class HomeState { NONE, SEEK1, SEEK2, SEEK3 };
HomeState homeState = HomeState::NONE;

// ====== Helpers: units ======
inline long mmToSteps(long mm) {
  return lroundf(mm * STEPS_PER_MM);
}
inline long stepsToMm(long steps) {
  return lroundf(steps / STEPS_PER_MM);
}
inline long pos10FromSteps(long steps) {
  // pos10 = (mm) where 10 mm = 1 cm in UI label math
  // UI expects: mm value as integer; label shows val/10 cm
  long mm = stepsToMm(steps) + lroundf(home1PosCm * 10.0f); // add mm offset of Home1
  return mm;
}
inline long stepsFromPos10(long pos10) {
  // Convert mm (pos10) minus Home1 offset into steps
  long mm = pos10 - lroundf(home1PosCm * 10.0f);
  return mmToSteps(mm);
}

// ====== Minimal, modern UI (no external assets) ======
static const char index_html[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<meta name="theme-color" content="#0f172a">
<title>Rail Servo</title>
<style>
:root{
  --bg: #0b1220;
  --panel: #0f172a;
  --ink: #e5e7eb;
  --muted:#94a3b8;
  --accent:#2563eb;
  --accent-2:#60a5fa;
  --danger:#ef4444;
  --ok:#10b981;
  --ring:#1f2a44;
  --shadow: 0 10px 30px rgba(0,0,0,.35);
  --radius: 16px;
}
*{box-sizing:border-box}
html,body{height:100%}
body{
  margin:0; background:radial-gradient(1200px 800px at 20% -10%, #11193a 0%, var(--bg) 60%),
            radial-gradient(800px 600px at 100% 0%, #0a1028 0%, var(--bg) 70%);
  font-family: ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Ubuntu, "Helvetica Neue", Arial, "Noto Sans", "Apple Color Emoji", "Segoe UI Emoji";
  color:var(--ink); display:flex; align-items:center; justify-content:center; padding:18px;
}
.wrap{
  width:100%; max-width:860px;
  background:linear-gradient(180deg, rgba(255,255,255,.02), rgba(255,255,255,.00));
  border:1px solid rgba(255,255,255,.06);
  border-radius:var(--radius);
  box-shadow:var(--shadow);
  backdrop-filter:blur(6px);
}
.header{
  display:flex; align-items:center; justify-content:space-between;
  padding:18px 20px; border-bottom:1px solid rgba(255,255,255,.06);
}
.brand{display:flex; gap:12px; align-items:center}
.dot{
  width:10px; height:10px; border-radius:50%;
  background:conic-gradient(from 180deg, var(--accent), var(--accent-2));
  box-shadow:0 0 16px rgba(96,165,250,.5);
}
.title{font-weight:600; letter-spacing:.3px}
.header .posRead{
  font-variant-numeric: tabular-nums; font-weight:700; letter-spacing:.4px;
  background:rgba(255,255,255,.03); border:1px solid rgba(255,255,255,.06);
  padding:8px 12px; border-radius:12px;
}
.main{padding:18px 20px; display:grid; grid-template-columns:1fr; gap:18px}
@media (min-width:760px){ .main{ grid-template-columns: 1.5fr .9fr } }

.card{
  background:linear-gradient(180deg, rgba(255,255,255,.02), rgba(255,255,255,.00));
  border:1px solid rgba(255,255,255,.06);
  border-radius:14px; padding:16px; position:relative;
}

.sectionTitle{font-size:14px; color:var(--muted); margin-bottom:8px; letter-spacing:.4px}
.readout{
  font-size:28px; font-weight:700; margin:6px 0 10px; letter-spacing:.5px;
}
.marks{
  display:flex; justify-content:space-between; color:var(--muted); font-size:12px; margin-top:6px;
}

/* Range base */
.range{ width:100%; appearance:none; height:10px; background:transparent; margin:10px 0 6px; }
.range:focus{ outline:none }
/* Track */
.range::-webkit-slider-runnable-track{
  background:linear-gradient(90deg, rgba(96,165,250,.5), rgba(37,99,235,.6));
  height:10px; border-radius:999px; border:1px solid rgba(255,255,255,.08);
  box-shadow: inset 0 0 0 1px rgba(255,255,255,.04), 0 6px 18px rgba(37,99,235,.25);
}
.range::-moz-range-track{
  background:linear-gradient(90deg, rgba(96,165,250,.5), rgba(37,99,235,.6));
  height:10px; border-radius:999px; border:1px solid rgba(255,255,255,.08);
}
/* Thumb */
.range::-webkit-slider-thumb{
  appearance:none; width:22px; height:22px; border-radius:50%;
  background:radial-gradient(8px 8px at 40% 35%, #fff 0%, #dbeafe 35%, #93c5fd 70%, #60a5fa 100%);
  border:1px solid rgba(255,255,255,.4);
  box-shadow:0 6px 14px rgba(0,0,0,.35);
  margin-top:-6px;
}
.range::-moz-range-thumb{
  width:22px; height:22px; border-radius:50%;
  background:radial-gradient(8px 8px at 40% 35%, #fff 0%, #dbeafe 35%, #93c5fd 70%, #60a5fa 100%);
  border:1px solid rgba(255,255,255,.4);
}

/* Buttons */
.btnRow{ display:flex; flex-wrap:wrap; gap:10px; margin-top:8px }
.btn{
  border:none; cursor:pointer; padding:10px 14px; border-radius:12px; font-weight:600; letter-spacing:.2px;
  background:linear-gradient(180deg, rgba(37,99,235,.85), rgba(37,99,235,.75));
  color:white; box-shadow:0 8px 20px rgba(37,99,235,.35);
  transition:transform .05s ease, box-shadow .2s ease, filter .2s ease;
}
.btn:active{ transform:translateY(1px) }
.btn.muted{
  background:linear-gradient(180deg, rgba(148,163,184,.35), rgba(148,163,184,.25));
  color:#0b1220; box-shadow:none; border:1px solid rgba(255,255,255,.08);
}
.btn.success{
  background:linear-gradient(180deg, rgba(16,185,129,.9), rgba(16,185,129,.75));
  box-shadow:0 8px 20px rgba(16,185,129,.35);
}

/* +/- inc controls */
.inc{
  display:flex; align-items:center; gap:10px; margin-top:8px; flex-wrap:wrap;
}
.inc .radio{
  display:flex; align-items:center; gap:6px; padding:6px 10px; border-radius:999px;
  background:rgba(255,255,255,.04); border:1px solid rgba(255,255,255,.06); color:var(--muted);
}
.inc .radio input{ accent-color: var(--accent) }

/* STOP button */
.stopDock{
  position:sticky; bottom:0; display:flex; justify-content:center; padding:14px 0 4px;
}
.stop{
  width:92px; height:92px; border-radius:50%; border:none; cursor:pointer; color:#fff; font-weight:800; letter-spacing:.5px;
  background:radial-gradient(60px 60px at 30% 30%, #fecaca 0%, #ef4444 55%, #b91c1c 100%);
  box-shadow:0 16px 40px rgba(239,68,68,.55), inset 0 0 0 3px rgba(255,255,255,.25);
  transition:transform .06s ease, box-shadow .2s ease, filter .2s ease;
}
.stop:active{ transform:translateY(2px) }

/* Footer note */
.footer{ color:var(--muted); font-size:12px; padding:12px 20px 16px; text-align:center }

.code{
  font-family:ui-monospace,SFMono-Regular,Menlo,Monaco,Consolas,"Liberation Mono","Courier New",monospace;
  color:#a5b4fc; background:rgba(255,255,255,.03);
  padding:2px 6px; border-radius:6px; border:1px solid rgba(255,255,255,.06);
}
</style>
</head>
<body>
  <div class="wrap">
    <div class="header">
      <div class="brand">
        <div class="dot"></div>
        <div class="title">Rail Servo</div>
      </div>
      <div class="posRead"><span id="posLabel">0.0</span> cm</div>
    </div>

    <div class="main">
      <!-- LEFT: Positioning -->
      <div class="card">
        <div class="sectionTitle">Position</div>
        <div class="readout"><span id="posLabel2">0.0</span> cm</div>

        <input class="range" type="range" id="pos" list="marks" min="%MIN10%" max="%MAX10%" step="1" value="0">
        <datalist id="marks">
          <option value="%MIN10%"></option>
          <option value="0"></option>
          <option value="%MAX10%"></option>
        </datalist>
        <div class="marks"><span>%MIN% cm</span><span>0</span><span>%MAX% cm</span></div>

        <div class="inc">
          <button class="btn muted" id="btnDec">–</button>
          <label class="radio"><input type="radio" name="step" value="1"  checked> 1 mm</label>
          <label class="radio"><input type="radio" name="step" value="10"> 10 mm</label>
          <label class="radio"><input type="radio" name="step" value="50"> 50 mm</label>
          <button class="btn muted" id="btnInc">+</button>
        </div>

        <div class="btnRow" style="margin-top:14px">
          <button class="btn" data-home="1">Home 1</button>
          <button class="btn" data-home="2">Home 2</button>
          <button class="btn" data-home="3">Home 3</button>
          <button class="btn success" id="homeAll">Home All</button>
        </div>
      </div>

      <!-- RIGHT: Speed / Status -->
      <div class="card">
        <div class="sectionTitle">Speed</div>
        <div class="readout"><span id="spdLabel">30</span>%</div>
        <input class="range" type="range" id="spd" min="1" max="100" step="1" value="30">
        <div class="marks"><span>1%</span><span>50%</span><span>100%</span></div>

        <div class="btnRow" style="margin-top:14px">
          <button class="btn muted" id="btnRefresh">Refresh Pos</button>
          <button class="btn muted" id="btnCenter">Go 0</button>
        </div>
      </div>
    </div>

    <div class="stopDock">
      <button class="stop" id="stopBtn">STOP</button>
    </div>

    <div class="footer">
      Reachable at <span class="code">http://rail_servo.local</span> (mDNS), or AP IP shown on serial.
    </div>
  </div>

<script>
const pos = document.getElementById('pos');
const spd = document.getElementById('spd');
const posLabel = document.getElementById('posLabel');
const posLabel2 = document.getElementById('posLabel2');
const spdLabel = document.getElementById('spdLabel');
const btnDec = document.getElementById('btnDec');
const btnInc = document.getElementById('btnInc');
const btnCenter = document.getElementById('btnCenter');
const btnRefresh = document.getElementById('btnRefresh');
const stopBtn = document.getElementById('stopBtn');
const homeAll = document.getElementById('homeAll');

let debounceTimer = null;
function mmToCmString(mm){ return (mm/10).toFixed(1); } // UI uses mm integers; label shows cm

function updateLabelsFromSlider(){
  const mm = parseInt(pos.value || '0', 10);
  const cmTxt = mmToCmString(mm);
  posLabel.textContent = cmTxt;
  posLabel2.textContent = cmTxt;
}

function sendMoveDebounced(){
  if(debounceTimer) clearTimeout(debounceTimer);
  debounceTimer = setTimeout(() => {
    fetch(`/move?pos=${encodeURIComponent(pos.value)}&spd=${encodeURIComponent(spd.value)}`).catch(()=>{});
  }, 80);
}

// Position slider interaction
pos.addEventListener('input', updateLabelsFromSlider);
pos.addEventListener('change', sendMoveDebounced);

// Speed slider interaction
spd.addEventListener('input', () => { spdLabel.textContent = spd.value; });
spd.addEventListener('change', sendMoveDebounced);

// +/- buttons
function currentStep() {
  const el = document.querySelector('input[name="step"]:checked');
  return el ? parseInt(el.value, 10) : 1;
}
btnDec.addEventListener('click', () => {
  const step = currentStep();
  let v = parseInt(pos.value || '0', 10) - step;
  v = Math.max(v, parseInt(pos.min, 10));
  pos.value = String(v);
  updateLabelsFromSlider();
  sendMoveDebounced();
});
btnInc.addEventListener('click', () => {
  const step = currentStep();
  let v = parseInt(pos.value || '0', 10) + step;
  v = Math.min(v, parseInt(pos.max, 10));
  pos.value = String(v);
  updateLabelsFromSlider();
  sendMoveDebounced();
});

// Home buttons (1..3)
document.querySelectorAll('[data-home]').forEach(btn=>{
  btn.addEventListener('click', ()=>{
    const n = btn.getAttribute('data-home');
    fetch(`/home?n=${encodeURIComponent(n)}`).catch(()=>{});
  });
});

// Home all
homeAll.addEventListener('click', ()=>{ fetch('/homeall').catch(()=>{}); });

// STOP
stopBtn.addEventListener('click', ()=>{
  fetch('/stop').then(r=>r.text()).then(mm=>{
    pos.value = parseInt(mm||'0',10);
    updateLabelsFromSlider();
  }).catch(()=>{});
});

// Refresh pos
btnRefresh.addEventListener('click', ()=>{
  fetch('/pos').then(r=>r.text()).then(mm=>{
    pos.value = parseInt(mm||'0',10);
    updateLabelsFromSlider();
  }).catch(()=>{});
});

// Go zero (center 0 mm)
btnCenter.addEventListener('click', ()=>{
  pos.value = 0;
  updateLabelsFromSlider();
  sendMoveDebounced();
});

// SSE live position
try{
  const es = new EventSource('/events');
  es.addEventListener('pos', e => {
    const mm = parseInt(e.data||'0',10);
    pos.value = String(mm);
    updateLabelsFromSlider();
  });
}catch(_){}

// initial label sync
updateLabelsFromSlider();
spdLabel.textContent = spd.value;
</script>
</body>
</html>
)HTML";

// ====== Page templating ======
String buildIndexHtml(){
  String page = FPSTR(index_html);
  const int min10 = lroundf(railMinCm * 10.0f); // convert cm -> mm
  const int max10 = lroundf(railMaxCm * 10.0f);

  page.replace("%MIN10%", String(min10));
  page.replace("%MAX10%", String(max10));
  page.replace("%MIN%", String(railMinCm, 1));
  page.replace("%MAX%", String(railMaxCm, 1));
  return page;
}

// ====== Prototypes ======
void startHome(int n);
void runHoming();
void fullHoming();

// ====== Setup ======
void setup(){
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[Rail Servo] Booting…");

  // Pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);
  pinMode(EN_PIN,   OUTPUT);
  pinMode(SW1_PIN,  INPUT_PULLUP);
  pinMode(SW2_PIN,  INPUT_PULLUP);
  pinMode(SW3_PIN,  INPUT_PULLUP);

  // Stepper base config
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true); // invert EN only
  stepper.enableOutputs();
  stepper.setAcceleration(ACCEL_MM_S2 * STEPS_PER_MM);
  stepper.setMaxSpeed(30.0f * STEPS_PER_MM);

  // Wi-Fi AP
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(kAP_SSID, kAP_PASSWORD)) {
    Serial.println("[WiFi] Failed to start AP!");
  }
  Serial.print("[WiFi] AP IP: "); Serial.println(WiFi.softAPIP());

  // mDNS
  if (MDNS.begin("rail_servo")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("[mDNS] http://rail_servo.local");
  } else {
    Serial.println("[mDNS] failed");
  }

  // ====== HTTP Handlers ======
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req){
    req->send(200, "text/html; charset=utf-8", buildIndexHtml());
  });

  server.on("/move", HTTP_GET, [](AsyncWebServerRequest* req){
    abortAll = false;
    stepper.enableOutputs();

    if (req->hasParam("spd")) {
      uiSpeedPercent = constrain(req->getParam("spd")->value().toInt(), 1, 100);
      float maxSpeedMmS = 0.3f * uiSpeedPercent; // simple linear map
      stepper.setMaxSpeed(maxSpeedMmS * STEPS_PER_MM);
      Serial.printf("[MOVE] Speed %% -> %d (max ~ %.1f mm/s)\n", uiSpeedPercent, maxSpeedMmS);
    }

    if (req->hasParam("pos")) {
      long minmm = lroundf(railMinCm * 10.0f);
      long maxmm = lroundf(railMaxCm * 10.0f);
      long mm = constrain(req->getParam("pos")->value().toInt(), minmm, maxmm);
      long tgtSteps = stepsFromPos10(mm);
      stepper.moveTo(tgtSteps);
      Serial.printf("[MOVE] Target -> %ld mm (steps=%ld)\n", mm, tgtSteps);
    }

    req->send(200, "text/plain", "OK");
  });

  server.on("/home", HTTP_GET, [](AsyncWebServerRequest* req){
    abortAll = false;
    if (!req->hasParam("n")) {
      req->send(400, "text/plain", "Missing n");
      return;
    }
    int n = req->getParam("n")->value().toInt();
    startHome(n);
    req->send(200, "text/plain", "Homing");
    Serial.printf("[HOME] Command n=%d\n", n);
  });

  server.on("/homeall", HTTP_GET, [](AsyncWebServerRequest* req){
    abortAll = false;
    wantFullHome = true;
    req->send(200, "text/plain", "Full homing");
    Serial.println("[HOME] Full homing requested");
  });

  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest* req){
    homeState   = HomeState::NONE;
    wantFullHome= false;
    abortAll    = true;
    stepper.moveTo(stepper.currentPosition());
    stepper.setSpeed(0);
    long mm = pos10FromSteps(stepper.currentPosition());
    req->send(200, "text/plain", String(mm));
    Serial.println("[STOP] Immediate stop issued");
  });

  server.on("/pos", HTTP_GET, [](AsyncWebServerRequest* req){
    long mm = pos10FromSteps(stepper.currentPosition());
    req->send(200, "text/plain", String(mm));
  });

  server.addHandler(&events);
  events.onConnect([](AsyncEventSourceClient* c){
    Serial.println("[SSE] Client connected");
  });

  server.begin();
  Serial.println("[HTTP] Server started");

  // Initial full homing
  fullHoming();
}

// ====== Main loop ======
void loop(){
  if (wantFullHome) fullHoming();

  // Run state
  if (homeState == HomeState::NONE) {
    stepper.run();
  } else {
    runHoming();
  }

  // Auto-disable outputs after idle
  if (stepper.distanceToGo() != 0 || stepper.isRunning() || homeState != HomeState::NONE) {
    stepper.enableOutputs();
    lastEnableMs = millis();
  } else {
    if (millis() - lastEnableMs > 4000) {
      stepper.disableOutputs();
    }
  }

  // Stream live position via SSE
  const bool moving = (stepper.distanceToGo() != 0) || (homeState != HomeState::NONE);
  const uint32_t interval = moving ? 60 : 250;
  if (millis() - lastSseMs >= interval) {
    lastSseMs = millis();
    long mm = pos10FromSteps(stepper.currentPosition());
    events.send(String(mm).c_str(), "pos");
  }
}

// ====== IO helpers ======
inline bool switchHit(int pin){ return digitalRead(pin) == LOW; }

// ====== Homing control ======
void startHome(int n){
  stepper.enableOutputs();
  switch (n) {
    case 1: homeState = HomeState::SEEK1; break;
    case 2: homeState = HomeState::SEEK2; break;
    case 3: homeState = HomeState::SEEK3; break;
    default: homeState = HomeState::NONE; break;
  }
  // Conservative profile for homing moves from UI
  stepper.setAcceleration(ACCEL_MM_S2 * STEPS_PER_MM);
  stepper.setMaxSpeed(0.3f * uiSpeedPercent * STEPS_PER_MM);
}

void runHoming(){
  stepper.enableOutputs();
  switch (homeState) {
    case HomeState::SEEK1:
      stepper.moveTo(0);
      stepper.runToPosition(); // blocking homing hop
      Serial.println("[HOME] Home1 reached");
      events.send(String(pos10FromSteps(stepper.currentPosition())).c_str(), "pos");
      homeState = HomeState::NONE;
      break;

    case HomeState::SEEK2:
      stepper.moveTo(home2PosSteps);
      stepper.runToPosition();
      Serial.println("[HOME] Home2 reached");
      events.send(String(pos10FromSteps(stepper.currentPosition())).c_str(), "pos");
      homeState = HomeState::NONE;
      break;

    case HomeState::SEEK3:
      stepper.moveTo(home3PosSteps);
      stepper.runToPosition();
      Serial.println("[HOME] Home3 reached");
      events.send(String(pos10FromSteps(stepper.currentPosition())).c_str(), "pos");
      homeState = HomeState::NONE;
      break;

    default: break;
  }
}

// ====== Full homing sequence ======
void fullHoming(){
  Serial.println("[HOME] Starting Full Homing…");
  wantFullHome = false;
  abortAll     = false;
  stepper.enableOutputs();

  // 1) Seek SW1 at constant speed (negative direction)
  stepper.setAcceleration(0); // constant-speed search
  stepper.setMaxSpeed(startupHomeSpeedMmS * STEPS_PER_MM);
  stepper.setSpeed(-startupHomeSpeedMmS * STEPS_PER_MM);
  while (!switchHit(SW1_PIN)) {
    stepper.runSpeed();
    if (abortAll) break;
  }
  if (!abortAll) {
    stepper.setCurrentPosition(0);
    Serial.println("[HOME] Home1 triggered, position set to 0");
  } else {
    Serial.println("[HOME] Aborted during SEEK1");
  }

  // 2) Sweep positive to find SW2 and SW3; record positions
  bool found2 = false;
  long maxSweepSteps = mmToSteps(lroundf((railMaxCm + fabsf(home1PosCm)) * 10.0f));
  stepper.setSpeed(startupHomeSpeedMmS * STEPS_PER_MM);
  while (stepper.currentPosition() < maxSweepSteps && !abortAll) {
    stepper.runSpeed();
    if (!found2 && switchHit(SW2_PIN)) {
      home2PosSteps = stepper.currentPosition();
      found2 = true;
      Serial.printf("[HOME] Home2 at steps=%ld\n", home2PosSteps);
    }
    if (switchHit(SW3_PIN)) {
      home3PosSteps = stepper.currentPosition();
      Serial.printf("[HOME] Home3 at steps=%ld\n", home3PosSteps);
      break;
    }
  }

  // 3) Move to machine zero (defined by Home1 physical offset)
  if (!abortAll) {
    long zeroSteps = mmToSteps(lroundf(-home1PosCm * 10.0f));
    stepper.setAcceleration(ACCEL_MM_S2 * STEPS_PER_MM);
    stepper.setMaxSpeed(20.0f * STEPS_PER_MM);
    stepper.moveTo(zeroSteps);
    stepper.runToPosition();
    Serial.println("[HOME] Moved to machine zero");
  }

  // Restore runtime acceleration
  stepper.setAcceleration(ACCEL_MM_S2 * STEPS_PER_MM);

  // Broadcast final position
  events.send(String(pos10FromSteps(stepper.currentPosition())).c_str(), "pos");
  Serial.println("[HOME] Full homing complete");
}
