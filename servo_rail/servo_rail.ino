#include <WiFi.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AccelStepper.h>

// WiFi credentials
const char* ssid = "your_ssid";
const char* password = "your_password";

// Stepper driver pins
const int STEP_PIN = 14;
const int DIR_PIN  = 12;
const int EN_PIN   = 13;

// Limit switch pins
const int SW1_PIN = 25; // Home1 at -15 cm
const int SW2_PIN = 26; // Home2
const int SW3_PIN = 27; // Home3

// Motion parameters
const float SCREW_LEAD_MM = 5.0;    // mm per revolution
const int MOTOR_STEPS = 200;        // full steps per rev
const int MICROSTEPS = 16;          // driver microstepping
const float ACCEL_MM_S2 = 50.0;     // acceleration mm/s^2
const float STEPS_PER_MM = (MOTOR_STEPS * MICROSTEPS) / SCREW_LEAD_MM;

AsyncWebServer server(80);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

long home2Pos = 0;
long home3Pos = 0;

// HTML page served to the client
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<style>
body{background:#e0f0ff;font-family:Arial;text-align:center;color:#024;}
.slider{width:90%;margin:20px auto;}
.marks{width:90%;margin:-10px auto 10px;display:flex;justify-content:space-between;font-size:12px;color:#024;}
button{background:#028;color:#fff;padding:10px 20px;margin:5px;border:none;border-radius:5px;}
.inc{display:flex;justify-content:center;align-items:center;margin:10px;}
.inc label{margin:0 5px;}
.pos{font-size:24px;margin:10px;}
</style>
</head>
<body>
<h3>Position</h3>
<div class='pos' id='posLabel'>0 cm</div>
<input type='range' id='pos' min='-150' max='150' value='0' class='slider' list='posmarks'>
<datalist id='posmarks'>
  <option value='-150'></option>
  <option value='0'></option>
  <option value='150'></option>
</datalist>
<div class='marks'><span>-15</span><span>0</span><span>15</span></div>
<div class='inc'>
<button onclick='inc(-1)'>-</button>
<label><input type='radio' name='step' value='1' checked>1mm</label>
<label><input type='radio' name='step' value='10'>10mm</label>
<label><input type='radio' name='step' value='50'>50mm</label>
<button onclick='inc(1)'>+</button>
</div>
<h3>Speed <span id='spdLabel'>50</span></h3>
<input type='range' id='spd' min='1' max='100' value='50' class='slider'>
<div>
<button onclick='home(1)'>Home1</button>
<button onclick='home(2)'>Home2</button>
<button onclick='home(3)'>Home3</button>
</div>
<script>
let pos=document.getElementById('pos');
let spd=document.getElementById('spd');
let label=document.getElementById('posLabel');
let spdLabel=document.getElementById('spdLabel');
pos.oninput=()=>{update();};
spd.oninput=()=>{spdLabel.innerHTML=spd.value; update();};
function update(){
  let val=parseInt(pos.value);
  pos.value=val;
  label.innerHTML=(val/10)+' cm';
  fetch('/move?pos='+val+'&spd='+spd.value);
}
function inc(dir){
  let step=document.querySelector('input[name="step"]:checked').value;
  let v=parseInt(pos.value)+dir*step;
  if(v>150)v=150;if(v<-150)v=-150;
  pos.value=v;
  update();
}
function home(n){fetch('/home?n='+n);}
</script>
</body>
</html>
)rawliteral";

void startHome(int n);
void runHoming();

enum HomeState { NONE, SEEK1, SEEK2, SEEK3 };
HomeState homeState = NONE;

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(SW1_PIN, INPUT_PULLUP);
  pinMode(SW2_PIN, INPUT_PULLUP);
  pinMode(SW3_PIN, INPUT_PULLUP);
  digitalWrite(EN_PIN, LOW);

  stepper.setEnablePin(EN_PIN);
  stepper.setAcceleration(ACCEL_MM_S2 * STEPS_PER_MM);
  stepper.setMaxSpeed(100 * STEPS_PER_MM); // default speed

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  MDNS.begin("servo_rail");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){
    req->send_P(200, "text/html", index_html);
  });
  server.on("/move", HTTP_GET, [](AsyncWebServerRequest *req){
    if(req->hasParam("pos")) {
      long pos = req->getParam("pos")->value().toInt();
      stepper.moveTo(pos * STEPS_PER_MM / 10);
    }
    if(req->hasParam("spd")) {
      int spd = req->getParam("spd")->value().toInt();
      stepper.setMaxSpeed(spd * STEPS_PER_MM);
    }
    req->send(200, "text/plain", "OK");
  });
  server.on("/home", HTTP_GET, [](AsyncWebServerRequest *req){
    int n = req->getParam("n")->value().toInt();
    startHome(n);
    req->send(200, "text/plain", "Homing");
  });
  server.begin();

  startHome(1);       // Home1 at startup
  while(homeState != NONE) runHoming();
  startHome(2);       // move to detect switch2
  while(homeState != NONE) runHoming();
  startHome(3);       // move to detect switch3
  while(homeState != NONE) runHoming();
  stepper.setCurrentPosition(0);
}

void loop() {
  if(homeState == NONE) {
    stepper.run();
  } else {
    runHoming();
  }
}

bool switchHit(int pin){
  return digitalRead(pin) == LOW;
}

void startHome(int n){
  if(n==1) homeState = SEEK1;
  else if(n==2) homeState = SEEK2;
  else if(n==3) homeState = SEEK3;
  stepper.setMaxSpeed(50 * STEPS_PER_MM);
  stepper.setAcceleration(ACCEL_MM_S2 * STEPS_PER_MM);
}

void runHoming(){
  switch(homeState){
    case SEEK1:
      stepper.move(-100000);
      while(!switchHit(SW1_PIN)) stepper.run();
      stepper.stop(); stepper.runToPosition();
      stepper.setCurrentPosition(-150 * STEPS_PER_MM / 10);
      homeState = NONE;
      break;
    case SEEK2:
      stepper.move(100000);
      while(!switchHit(SW2_PIN)) stepper.run();
      stepper.stop(); stepper.runToPosition();
      home2Pos = stepper.currentPosition();
      homeState = NONE;
      break;
    case SEEK3:
      stepper.move(100000);
      while(!switchHit(SW3_PIN)) stepper.run();
      stepper.stop(); stepper.runToPosition();
      home3Pos = stepper.currentPosition();
      homeState = NONE;
      break;
    default:
      break;
  }
}
