#include <WiFi.h>              // Wi-Fi connectivity
#include <ESPmDNS.h>           // mDNS responder
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h> // asynchronous web server
#include <AccelStepper.h>      // stepper control library

// Credentials for the Wi-Fi access point created by the ESP32
const char* ssid = "rail_servo";
const char* password = "123456789";

// Pins connected to the stepper driver
const int STEP_PIN = 21;
const int DIR_PIN  = 22;
const int EN_PIN   = 23;
 
// Limit switch pins (active LOW)
const int SW1_PIN = 17; // Home1
const int SW2_PIN = 18; // Home2
const int SW3_PIN = 19; // Home3

// Mechanical parameters for the linear rail
const float SCREW_LEAD_MM = 4.0;    // mm per revolution
const int MOTOR_STEPS = 200;        // full steps per rev
const int MICROSTEPS = 16;          // driver microstepping
const float ACCEL_MM_S2 = 50.0;     // acceleration mm/s^2
const float STEPS_PER_MM = (MOTOR_STEPS * MICROSTEPS) / SCREW_LEAD_MM;

// Adjustable travel limits and Home1 location in centimeters
float railMinCm  = -12.5;
float railMaxCm  =  12.0;
float home1PosCm = -11.8;   // user-provided position of Home1

// Startup homing speed in mm/s (slow constant speed)
float startupHomeSpeedMmS = 15.0;

// Web server for the control interface
AsyncWebServer server(80);
// Stepper object controlling the motor
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Positions of switch 2 and switch 3 after homing
long home2Pos = 0;
long home3Pos = 0;

long onDelay = 0;
bool flag = 0;
bool abrt = 0;
bool oflag = 1;

// HTML page served to the client with placeholders for limits
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
/* large red stop button fixed to bottom center */
#stopBtn{position:fixed;bottom:20px;left:50%;transform:translateX(-50%);background:#d00;color:#fff;width:80px;height:80px;font-size:20px;border:none;border-radius:40px;}
</style>
</head>
<body>
<h3>Position</h3>
<div class='pos' id='posLabel'>0 cm</div>
<input type='range' id='pos' min='%MIN10%' max='%MAX10%' value='0' class='slider' list='posmarks'>
<datalist id='posmarks'>
  <option value='%MIN10%'></option>
  <option value='0'></option>
  <option value='%MAX10%'></option>
</datalist>
<div class='marks'><span>%MIN%</span><span>0</span><span>%MAX%</span></div>
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
<button onclick='homeAll()'>Home all</button>
</div>
<button id='stopBtn' onclick='stopMotion()'>STOP</button>
<script>
let pos=document.getElementById('pos');
let spd=document.getElementById('spd');
let label=document.getElementById('posLabel');
let spdLabel=document.getElementById('spdLabel');
pos.oninput=()=>{updateLabel();};
pos.onchange=()=>{sendMove();};
spd.oninput=()=>{spdLabel.innerHTML=spd.value;};
spd.onchange=()=>{sendMove();};
function updateLabel(){
  let val=parseInt(pos.value);
  pos.value=val;
  label.innerHTML=(val/10)+' cm';
}
function sendMove(){
  updateLabel();
  fetch('/move?pos='+pos.value+'&spd='+spd.value);
}
function inc(dir){
  let step=document.querySelector('input[name="step"]:checked').value;
  let v=parseInt(pos.value)+dir*step;
  if(v>%MAX10%)v=%MAX10%;if(v<%MIN10%)v=%MIN10%;
  pos.value=v;
  sendMove();
}
function home(n){fetch('/home?n='+n);}
function homeAll(){fetch('/homeall');}
function stopMotion(){
  fetch('/stop')
    .then(r=>r.text())
    .then(t=>{pos.value=parseInt(t);updateLabel();});
}
function updatePos(){
  fetch('/pos')
    .then(r=>r.text())
    .then(t=>{pos.value=parseInt(t);updateLabel();});
}
setInterval(updatePos, 500);
</script>
</body>
</html>
)rawliteral";

// Build HTML page with current travel limits
String getIndexHtml(){
  String page = String(index_html);
  int min10 = railMinCm * 10;
  int max10 = railMaxCm * 10;
  page.replace("%MIN10%", String(min10));
  page.replace("%MAX10%", String(max10));
  page.replace("%MIN%", String(railMinCm,1));
  page.replace("%MAX%", String(railMaxCm,1));
  return page;
}

// Start a homing sequence to switch n (1..3)
void startHome(int n);
// Execute one step of the homing state machine
void runHoming();
// Perform the entire homing routine at startup
void fullHoming();

// Possible states during homing
enum HomeState { NONE, SEEK1, SEEK2, SEEK3 };
// Current homing state
HomeState homeState = NONE;

void setup() {
  Serial.begin(115200);        // open serial port
  delay(1000);                 // allow time for port to open
  Serial.println("Rail servo starting...");

  // configure GPIOs for the stepper driver and switches
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(SW1_PIN, INPUT_PULLUP);
  pinMode(SW2_PIN, INPUT_PULLUP);
  pinMode(SW3_PIN, INPUT_PULLUP);
  digitalWrite(EN_PIN, LOW);   // enable driver

  // configure the stepper library
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true);
  stepper.setAcceleration(ACCEL_MM_S2 * STEPS_PER_MM);
  stepper.setMaxSpeed(30 * STEPS_PER_MM); // default speed

  // start the Wi-Fi access point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.print("Access Point IP: ");
  Serial.println(WiFi.softAPIP());

  // enable mDNS so the interface is reachable at http://rail_servo.local
  MDNS.begin("rail_servo");

  // HTTP handlers for the control interface
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){
    req->send(200, "text/html", getIndexHtml());
  });
  server.on("/move", HTTP_GET, [](AsyncWebServerRequest *req){
    stepper.enableOutputs();
    abrt = 0;
    if(req->hasParam("pos")) {
      long pos = req->getParam("pos")->value().toInt(); // value in mm
      long minmm = railMinCm * 10;
      long maxmm = railMaxCm * 10;
      pos = constrain(pos, minmm, maxmm);
      long target = (pos - home1PosCm * 10) * STEPS_PER_MM; // offset from Home1
      stepper.moveTo(target);
      Serial.print("Move to ");
      Serial.println(pos);
    }
    if(req->hasParam("spd")) {
      int spd = req->getParam("spd")->value().toInt();
      stepper.setMaxSpeed(0.3 * spd * STEPS_PER_MM);
      Serial.print("Set speed ");
      Serial.println(spd);
    }
    req->send(200, "text/plain", "OK");
  });
  server.on("/home", HTTP_GET, [](AsyncWebServerRequest *req){
    abrt = 0;
    int n = req->getParam("n")->value().toInt();
    startHome(n);
    Serial.print("Home command ");
    Serial.println(n);
    req->send(200, "text/plain", "Homing");
  });
  server.on("/homeall", HTTP_GET, [](AsyncWebServerRequest *req){
    abrt = 0;
    flag = 1;
    req->send(200, "text/plain", "Full homing");
  });
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *req){
    // immediate stop and cancel sequences
    homeState = NONE;
    flag = 0;
    abrt = 1;
    stepper.moveTo(stepper.currentPosition());
    stepper.setSpeed(0);
    long pos10 = stepper.currentPosition() / STEPS_PER_MM + home1PosCm * 10;
    req->send(200, "text/plain", String(pos10));
  });
  server.on("/pos", HTTP_GET, [](AsyncWebServerRequest *req){
    long pos10 = stepper.currentPosition() / STEPS_PER_MM + home1PosCm * 10;
    req->send(200, "text/plain", String(pos10));
  });
  server.begin();             // start web server
  Serial.println("Web server started");
  delay(1000);

  stepper.enableOutputs();
  // run full homing sequence on boot
  fullHoming();
}

void loop() {
  if(flag) fullHoming();

  // run the current motion
  if(homeState == NONE) {
    stepper.run();            // normal movement
  } else {
    runHoming();              // finish homing operation
  }

  if(stepper.distanceToGo() != 0 || stepper.isRunning()){
    stepper.enableOutputs();
    onDelay = millis();
  }else{
    if((onDelay + 4000) <= millis()){
      stepper.disableOutputs();
    }
  }
}

// Returns true if the given limit switch is pressed
bool switchHit(int pin){
  return digitalRead(pin) == LOW;
}

// Initiate homing toward switch n
void startHome(int n){
  stepper.enableOutputs();
  if(n==1) homeState = SEEK1;
  else if(n==2) homeState = SEEK2;
  else if(n==3) homeState = SEEK3;
  // slower speed for reliable homing
  stepper.setMaxSpeed(30 * STEPS_PER_MM);
  stepper.setAcceleration(ACCEL_MM_S2 * STEPS_PER_MM);
}

// Execute the homing state machine
void runHoming(){
  stepper.enableOutputs();
  switch(homeState){
    case SEEK1: // search for switch 1
      stepper.moveTo(0);
      stepper.runToPosition();
      Serial.println("Home1 reached");
      homeState = NONE;
      break;
    case SEEK2: // move to stored Home2 position
      stepper.moveTo(home2Pos);
      stepper.runToPosition();
      Serial.println("Home2 reached");
      homeState = NONE;
      break;
    case SEEK3: // move to stored Home3 position
      stepper.moveTo(home3Pos);
      stepper.runToPosition();
      Serial.println("Home3 reached");
      homeState = NONE;
      break;
    default:
      break;
  }
}

// Perform the full homing sequence on startup
void fullHoming(){
  Serial.println("Starting Full Homing...");
  stepper.enableOutputs();

  // run slow constant-speed homing toward switch 1
  stepper.setAcceleration(0); // disable acceleration for startup homing
  stepper.setMaxSpeed(startupHomeSpeedMmS * STEPS_PER_MM);
  stepper.setSpeed(-startupHomeSpeedMmS * STEPS_PER_MM);
  while(!switchHit(SW1_PIN)){ 
    stepper.runSpeed();
    if(abrt) break;
  }

  if(!abrt){
    stepper.setCurrentPosition(0);
    Serial.println("Home1 reached");
  }
  Serial.println(stepper.currentPosition());

  // scan toward the positive end recording switches 2 and 3
  bool found2 = false;
  long target = (railMaxCm + abs(home1PosCm)) * 10 * STEPS_PER_MM;
  stepper.setSpeed(startupHomeSpeedMmS * STEPS_PER_MM);
  while(stepper.currentPosition() < target && !abrt){
    stepper.runSpeed();
    if(!found2 && switchHit(SW2_PIN)){
      home2Pos = stepper.currentPosition();
      Serial.println("Home2 reached");
      Serial.println(stepper.currentPosition());
      found2 = true;
    }
    if(switchHit(SW3_PIN)){
      home3Pos = stepper.currentPosition();
      Serial.println("Home3 reached");
      Serial.println(stepper.currentPosition());
      break;
    }
  }

  
  // move to machine zero (offset from Home1)
  if(!abrt){
    long zeroSteps = (-home1PosCm * 10) * STEPS_PER_MM;
    stepper.setAcceleration(ACCEL_MM_S2 * STEPS_PER_MM);
    stepper.setSpeed(20 * STEPS_PER_MM);
    stepper.setMaxSpeed(20 * STEPS_PER_MM); // default speed
    stepper.moveTo(zeroSteps);
    stepper.runToPosition();
  }
  Serial.println("Startup homing complete");

  // restore normal acceleration for regular moves
  stepper.setAcceleration(ACCEL_MM_S2 * STEPS_PER_MM);
  flag = 0;  // stop repeating full homing
}
