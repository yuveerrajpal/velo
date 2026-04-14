#include <Wire.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <HardwareSerial.h>


// ================= WIFI =================
const char* ssid = "FallDetector";
const char* password = "12345678";
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");


// ================= GSM =================
#define MODEM_RX 17
#define MODEM_TX 18
#define MODEM_PWR 33
HardwareSerial GSM(2);


// ================= MPU =================
Adafruit_MPU6050 mpu;


// ================= EEPROM =================
#define EEPROM_SIZE 512
#define CONFIG_MAGIC 0xA55A1234
struct Config {
  uint32_t magic;
  float impactThreshold;   // m/s² spike to detect impact (default 25.0)
  float jerkThreshold;     // m/s² free-fall floor (default 3.0)
  float tiltThreshold;     // degrees from vertical to confirm fall (default 50.0)
  char phone1[16];
  char phone2[16];
  char phone3[16];
  uint8_t testMode;
} config;


// ================= FALL DETECTION STATE MACHINE =================
enum FallState { IDLE, FREE_FALL, IMPACT_SEEN, CONFIRMING };
FallState fallState = IDLE;
unsigned long stateEnteredAt = 0;

#define POST_IMPACT_SAMPLES  20
float postImpactBuf[POST_IMPACT_SAMPLES];
int postImpactIdx = 0;

#define STILLNESS_VARIANCE   2.5f   // m/s² variance threshold — lying still vs bouncing
#define FREEFALL_WINDOW_MS   700    // max ms to wait for impact after free-fall
#define CONFIRM_WINDOW_MS   2500    // ms of post-impact data collected before verdict


// ================= RUNTIME VARIABLES =================
float prevAcc = 0;
unsigned long lastAlert = 0;
unsigned long lastDebugPush = 0;
unsigned long lastMinuteLog = 0;
unsigned long lastImpactLog = 0;
float lastAcc = 0;
float lastJerk = 0;
float lastTilt = 0;


// ================= FORWARD DECLARATIONS =================
void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len);
void saveConfig();
void copyPhone(char* dest, const String& src);
String runAT(const String& cmd, uint16_t timeoutMs = 1200);
String escapeJson(String s);
void pushLog(const String& message);
String webpage();
void detectFall();
void sendDebug(float acc, float jerk, float tilt);
void sendAlert(float tilt, float acc);
void sendSMS(String num, String msg);
String getGPS();
float angleFromVertical(float ax, float ay, float az);


// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  pushLog("Boot: device setup started");

  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, config);

  // Default values on first boot or invalid EEPROM
  if (config.magic != CONFIG_MAGIC) {
    memset(&config, 0, sizeof(config));
    config.magic           = CONFIG_MAGIC;
    config.impactThreshold = 25.0f;  // m/s² impact spike
    config.jerkThreshold   = 3.0f;   // m/s² free-fall floor
    config.tiltThreshold   = 50.0f;  // degrees from vertical
    config.testMode        = 1;
    saveConfig();
    pushLog("EEPROM: initialized with defaults");
  }
  pushLog("EEPROM: config loaded");

  // MPU6050
  Wire.begin(15, 16);
  if (!mpu.begin()) {
    pushLog("MPU6050: init failed — check wiring on SDA=15 SCL=16");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    pushLog("MPU6050: started — range 8G, filter 21Hz");
  }

  // GSM / SIM7670G
  pinMode(MODEM_PWR, OUTPUT);
  digitalWrite(MODEM_PWR, HIGH);
  delay(5000);
  GSM.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  pushLog("GSM: serial started");
  runAT("AT");
  runAT("AT+CMGF=1");   // SMS text mode
  runAT("AT+CGSMS=1");  // prefer GSM for SMS
  runAT("AT+CGNSSPWR=1"); // power on GNSS
  pushLog("GSM: AT init complete");

  // WiFi Access Point
  WiFi.softAP(ssid, password);
  pushLog("WiFi AP started: FallDetector / 12345678");

  // WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // API: get current config
  server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest* req) {
    String json = "{";
    json += "\"impact\":"   + String(config.impactThreshold, 2) + ",";
    json += "\"jerk\":"     + String(config.jerkThreshold, 2)   + ",";
    json += "\"tilt\":"     + String(config.tiltThreshold, 2)   + ",";
    json += "\"testMode\":" + String(config.testMode ? "true" : "false") + ",";
    json += "\"phone1\":\""  + String(config.phone1) + "\",";
    json += "\"phone2\":\""  + String(config.phone2) + "\",";
    json += "\"phone3\":\""  + String(config.phone3) + "\"";
    json += "}";
    req->send(200, "application/json", json);
  });

  // API: save all settings
  server.on("/api/save", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (req->hasParam("impact"))    config.impactThreshold = req->getParam("impact")->value().toFloat();
    if (req->hasParam("jerk"))      config.jerkThreshold   = req->getParam("jerk")->value().toFloat();
    if (req->hasParam("tilt"))      config.tiltThreshold   = req->getParam("tilt")->value().toFloat();
    if (req->hasParam("testMode"))  config.testMode = (req->getParam("testMode")->value().toInt() > 0) ? 1 : 0;
    if (req->hasParam("phone1"))    copyPhone(config.phone1, req->getParam("phone1")->value());
    if (req->hasParam("phone2"))    copyPhone(config.phone2, req->getParam("phone2")->value());
    if (req->hasParam("phone3"))    copyPhone(config.phone3, req->getParam("phone3")->value());
    saveConfig();
    pushLog("Config: settings saved from web UI");
    req->send(200, "text/plain", "Saved");
  });

  // API: SIM check (test mode only)
  server.on("/api/simcheck", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!config.testMode) {
      pushLog("SIM check blocked: test mode is OFF");
      req->send(403, "application/json", "{\"error\":\"Enable test mode first\"}");
      return;
    }
    pushLog("SIM check started");
    String json = "{";
    json += "\"network\":\"" + escapeJson(runAT("AT+CREG?"))    + "\",";
    json += "\"sim\":\""     + escapeJson(runAT("AT+CPIN?"))    + "\",";
    json += "\"sms\":\""     + escapeJson(runAT("AT+CSCS?"))    + "\",";
    json += "\"gps\":\""     + escapeJson(runAT("AT+CGNSSPWR?"))+ "\"";
    json += "}";
    pushLog("SIM check completed");
    req->send(200, "application/json", json);
  });

  // Serve web UI
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/html", webpage());
  });

  server.begin();
  pushLog("Web server started — connect to FallDetector WiFi then open 192.168.4.1");
}


// ================= LOOP =================
void loop() {
  detectFall();

  // 1-minute heartbeat log
  if (millis() - lastMinuteLog >= 60000) {
    lastMinuteLog = millis();
    String msg = "Heartbeat: mode=" + String(config.testMode ? "TEST" : "LIVE");
    msg += " state=" + String(fallState);
    msg += " acc="   + String(lastAcc,  2);
    msg += " jerk="  + String(lastJerk, 2);
    msg += " tilt="  + String(lastTilt, 1) + "deg";
    pushLog(msg);
  }
}


// ================= FALL DETECTION =================

// Returns angle in degrees between the accel vector and vertical (upright = 0°, on side = 90°)
float angleFromVertical(float ax, float ay, float az) {
  float mag = sqrt(ax*ax + ay*ay + az*az);
  if (mag < 0.1f) return 0.0f;
  return acos(constrain(az / mag, -1.0f, 1.0f)) * 180.0f / PI;
}

void detectFall() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float acc  = sqrt(ax*ax + ay*ay + az*az);  // total magnitude (m/s²) — ~9.8 when still
  float jerk = abs(acc - prevAcc);            // rate of change
  float tilt = angleFromVertical(ax, ay, az); // 0° = upright, grows as bike tips over
  prevAcc = acc;

  // Cache for heartbeat log
  lastAcc  = acc;
  lastJerk = jerk;
  lastTilt = tilt;

  // Stream telemetry to web UI in test mode
  sendDebug(acc, jerk, tilt);

  unsigned long now = millis();

  switch (fallState) {

    // ---- Phase 1: Watch for free-fall ----
    // When the bike tips over, it goes briefly weightless.
    // With gravity removed this would be ~0, but raw MPU reads ~9.8 when still.
    // A significant drop toward 0 means something is very wrong (free-fall / airborne).
    // config.jerkThreshold is repurposed as this free-fall floor (default 3.0 m/s²).
    case IDLE:
      if (acc < config.jerkThreshold) {
        fallState = FREE_FALL;
        stateEnteredAt = now;
        pushLog("FallFSM: free-fall phase — acc=" + String(acc, 2));
      }
      break;

    // ---- Phase 2: Wait for impact spike ----
    // After free-fall, expect a hard impact spike.
    // If no spike in 700ms, it was just a bump or road dip — re-arm.
    case FREE_FALL:
      if (now - stateEnteredAt > FREEFALL_WINDOW_MS) {
        fallState = IDLE;
        pushLog("FallFSM: free-fall timeout, back to IDLE");
        break;
      }
      if (acc > config.impactThreshold) {
        fallState = IMPACT_SEEN;
        stateEnteredAt = now;
        postImpactIdx = 0;
        pushLog("FallFSM: impact phase — acc=" + String(acc, 2));
      }
      break;

    // ---- Phase 3: Confirm fall (tilted + still) ----
    // Collect 2.5s of post-impact data.
    // A real fall = device lies tilted and stops moving (low variance).
    // A pothole or curb = device bounces back upright (high variance or low tilt).
    case IMPACT_SEEN:
      if (postImpactIdx < POST_IMPACT_SAMPLES) {
        postImpactBuf[postImpactIdx++] = acc;
      }

      if (now - stateEnteredAt > CONFIRM_WINDOW_MS) {
        // Compute variance of post-impact acceleration samples
        float sum = 0, sumSq = 0;
        for (int i = 0; i < postImpactIdx; i++) {
          sum   += postImpactBuf[i];
          sumSq += postImpactBuf[i] * postImpactBuf[i];
        }
        float mean     = sum / postImpactIdx;
        float variance = (sumSq / postImpactIdx) - (mean * mean);

        bool isStill  = (variance < STILLNESS_VARIANCE);
        bool isTilted = (tilt > config.tiltThreshold);

        pushLog("FallFSM: eval — tilt=" + String(tilt, 1) +
                "deg still=" + String(isStill) +
                " var=" + String(variance, 2));

        if (isStill && isTilted) {
          fallState = CONFIRMING;
          stateEnteredAt = now;
          pushLog("FallFSM: FALL CONFIRMED");
          if (now - lastAlert > 30000) {
            lastAlert = now;
            sendAlert(tilt, acc);
          }
        } else {
          fallState = IDLE;
          pushLog("FallFSM: false positive rejected (still=" +
                  String(isStill) + " tilted=" + String(isTilted) + ")");
        }
      }
      break;

    // ---- Cool-down: 30s before re-arming ----
    case CONFIRMING:
      if (now - stateEnteredAt > 30000) {
        fallState = IDLE;
        pushLog("FallFSM: re-armed");
      }
      break;
  }
}


// ================= DEBUG STREAM =================
void sendDebug(float acc, float jerk, float tilt) {
  if (!config.testMode) return;
  if (millis() - lastDebugPush < 200) return;
  lastDebugPush = millis();

  String json = "{";
  json += "\"type\":\"telemetry\",";
  json += "\"acc\":"  + String(acc,  3) + ",";
  json += "\"jerk\":" + String(jerk, 3) + ",";
  json += "\"tilt\":" + String(tilt, 1) + ",";
  json += "\"state\":" + String((int)fallState);
  json += "}";

  ws.textAll(json);
}


// ================= ALERT =================
void sendAlert(float tilt, float acc) {
  String gps = getGPS();
  pushLog("Alert: preparing SMS");

  String msg = "FALL DETECTED\n";
  msg += "Tilt: " + String(tilt, 1) + " deg\n";
  msg += "Acc: "  + String(acc,  1) + " m/s2\n";
  msg += gps;

  pushLog("Alert: sending to configured contacts");
  sendSMS(String(config.phone1), msg);
  sendSMS(String(config.phone2), msg);
  sendSMS(String(config.phone3), msg);
  pushLog("Alert: SMS sequence complete");
}


// ================= SMS =================
void sendSMS(String num, String msg) {
  num.trim();
  if (num.length() < 6 || num[0] == '\0') {
    pushLog("SMS skipped: empty/invalid number");
    return;
  }

  pushLog("SMS sending to: " + num);
  GSM.print("AT+CMGS=\"");
  GSM.print(num);
  GSM.println("\"");
  delay(1000);
  GSM.print(msg);
  GSM.write(26); // Ctrl+Z to send

  // Wait for confirmation or error
  String resp = "";
  unsigned long t = millis();
  while (millis() - t < 5000) {
    while (GSM.available()) resp += (char)GSM.read();
    if (resp.indexOf("+CMGS:") != -1) { pushLog("SMS confirmed sent to: " + num); return; }
    if (resp.indexOf("ERROR")  != -1) { pushLog("SMS ERROR for: " + num); return; }
    delay(10);
  }
  pushLog("SMS timeout (no confirm) for: " + num);
}


// ================= GPS =================
String getGPS() {
  while (GSM.available()) GSM.read(); // flush stale data
  GSM.println("AT+CGNSSINFO");

  String res = "";
  unsigned long t = millis();
  while (millis() - t < 2000) {
    while (GSM.available()) res += (char)GSM.read();
  }
  pushLog("GPS raw: " + res);

  int idx = res.indexOf("+CGNSSINFO:");
  // Comma runs like ",,,," indicate no fix
  if (idx == -1 || res.indexOf(",,,,") != -1) {
    pushLog("GPS: no fix available");
    return "Location: GPS fix unavailable";
  }

  String payload = res.substring(idx + 12);
  payload.trim();

  // SIM7670G CGNSSINFO fields (0-indexed):
  // 0=mode, 1=GPS_SVs, 2=GLONASS_SVs, 3=BEIDOU_SVs,
  // 4=lat, 5=N/S, 6=lon, 7=E/W, 8=date, 9=UTC, 10=alt, ...
  String fields[12];
  int f = 0;
  fields[0] = "";
  for (int i = 0; i < (int)payload.length() && f < 11; i++) {
    if (payload[i] == ',') { f++; fields[f] = ""; }
    else fields[f] += payload[i];
  }

  String latStr = fields[4];
  String latDir = fields[5];
  String lonStr = fields[6];
  String lonDir = fields[7];

  if (latStr.length() < 4) {
    pushLog("GPS: fix fields too short");
    return "Location: GPS fix unavailable";
  }

  // Convert DDMM.MMMMM → decimal degrees
  float rawLat = latStr.toFloat();
  int   latDeg = (int)(rawLat / 100);
  float lat    = latDeg + (rawLat - latDeg * 100) / 60.0f;
  if (latDir == "S") lat = -lat;

  float rawLon = lonStr.toFloat();
  int   lonDeg = (int)(rawLon / 100);
  float lon    = lonDeg + (rawLon - lonDeg * 100) / 60.0f;
  if (lonDir == "W") lon = -lon;

  String url = "https://maps.google.com/?q=" + String(lat, 6) + "," + String(lon, 6);
  pushLog("GPS fix: " + url);
  return url;
}


// ================= WEBSOCKET =================
void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  // No inbound messages expected from the UI; events handled silently
}


// ================= HELPERS =================
void saveConfig() {
  EEPROM.put(0, config);
  EEPROM.commit();
}

void copyPhone(char* dest, const String& src) {
  String cleaned = src;
  cleaned.trim();
  cleaned.replace(" ", "");
  cleaned.replace("-", "");
  cleaned.replace("(", "");
  cleaned.replace(")", "");
  cleaned.replace(".", "");
  strncpy(dest, cleaned.c_str(), 15);
  dest[15] = '\0';
}

String runAT(const String& cmd, uint16_t timeoutMs) {
  while (GSM.available()) GSM.read();
  GSM.println(cmd);
  unsigned long start = millis();
  String out = "";
  while (millis() - start < timeoutMs) {
    while (GSM.available()) out += (char)GSM.read();
    delay(5);
  }
  out.replace("\r", " ");
  out.replace("\n", " ");
  out.trim();
  if (out.length() == 0) out = "No response";
  return out;
}

String escapeJson(String s) {
  s.replace("\\", "\\\\");
  s.replace("\"", "\\\"");
  return s;
}

void pushLog(const String& message) {
  Serial.println(message); // also print to Serial for USB debugging
  String json = "{";
  json += "\"type\":\"log\",";
  json += "\"ts\":"   + String(millis()) + ",";
  json += "\"msg\":\"" + escapeJson(message) + "\"";
  json += "}";
  ws.textAll(json);
}


// ================= WEB PAGE =================
String webpage() {
  return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Fall Detector</title>
  <style>
    :root { --bg:#0b1220; --card:#121b2e; --muted:#8ea3c5; --txt:#e9f0ff; --accent:#34d399; --line:#23324f; --warn:#f59e0b; --danger:#ef4444; }
    * { box-sizing:border-box; font-family:Arial,sans-serif; margin:0; padding:0; }
    body { background:linear-gradient(135deg,#09101e,#0f1a31); color:var(--txt); padding-bottom:40px; }
    .wrap { max-width:900px; margin:0 auto; padding:28px 16px; }
    h2 { font-size:26px; font-weight:700; margin-bottom:20px; }
    h3 { font-size:15px; font-weight:600; margin-bottom:14px; color:var(--accent); }
    .card { background:var(--card); border:1px solid var(--line); border-radius:14px; padding:18px; margin-bottom:14px; }
    .row2 { display:grid; grid-template-columns:1fr 1fr; gap:10px; }
    .row3 { display:grid; grid-template-columns:1fr 1fr 1fr; gap:10px; }
    .row4 { display:grid; grid-template-columns:1fr 1fr 1fr 1fr; gap:10px; }
    label { font-size:12px; color:var(--muted); display:block; margin-bottom:5px; }
    input { width:100%; border:1px solid var(--line); background:#0e1628; color:var(--txt); border-radius:8px; padding:9px 10px; font-size:14px; }
    button { border:0; border-radius:8px; padding:10px 16px; cursor:pointer; font-weight:600; font-size:14px; }
    .btn-primary   { background:var(--accent); color:#042214; }
    .btn-secondary { background:#243657; color:#d5e3ff; }
    .mono { font-family:monospace; white-space:pre-wrap; background:#0c1424; border:1px solid var(--line); border-radius:8px; padding:12px; min-height:80px; font-size:13px; line-height:1.5; }
    .hint { color:var(--muted); font-size:12px; margin-top:8px; }
    .status { margin-top:10px; color:#bde9d3; font-size:13px; min-height:18px; }
    .state-badge { display:inline-block; padding:3px 10px; border-radius:20px; font-size:12px; font-weight:700; margin-left:8px; }
    .s0 { background:#1e3a5f; color:#90c4ff; }
    .s1 { background:#3b2a00; color:var(--warn); }
    .s2 { background:#3b1500; color:#ff8c42; }
    .s3 { background:#3b0000; color:var(--danger); }
    .telemetry-grid { display:grid; grid-template-columns:1fr 1fr 1fr; gap:10px; margin-bottom:10px; }
    .tel-box { background:#0c1424; border:1px solid var(--line); border-radius:8px; padding:10px; text-align:center; }
    .tel-label { font-size:11px; color:var(--muted); margin-bottom:4px; }
    .tel-val { font-size:20px; font-weight:700; font-family:monospace; }
    @media(max-width:600px){ .row2,.row3,.row4,.telemetry-grid { grid-template-columns:1fr; } }
  </style>
</head>
<body>
<div class="wrap">
  <h2>🚴 Fall Detection Control Panel</h2>

  <!-- Phone Numbers -->
  <div class="card">
    <h3>Emergency Contacts (up to 3)</h3>
    <div class="row3">
      <div><label>Phone 1</label><input id="phone1" placeholder="+91xxxxxxxxxx"></div>
      <div><label>Phone 2</label><input id="phone2" placeholder="+91xxxxxxxxxx"></div>
      <div><label>Phone 3</label><input id="phone3" placeholder="+91xxxxxxxxxx"></div>
    </div>
    <p class="hint">Stored in EEPROM. SMS alert is sent to all configured numbers on confirmed fall.</p>
  </div>

  <!-- Algorithm Thresholds -->
  <div class="card">
    <h3>Algorithm Thresholds</h3>
    <div class="row4">
      <div>
        <label>Impact Threshold (m/s²)</label>
        <input id="impact" type="number" step="0.5">
        <p class="hint">Spike needed after free-fall. Default: 25</p>
      </div>
      <div>
        <label>Free-Fall Floor (m/s²)</label>
        <input id="jerk" type="number" step="0.1">
        <p class="hint">Accel drop = weightless. Default: 3.0</p>
      </div>
      <div>
        <label>Tilt Threshold (°)</label>
        <input id="tilt" type="number" step="1">
        <p class="hint">Angle from upright. Default: 50</p>
      </div>
      <div>
        <label>Test Mode (1=ON, 0=LIVE)</label>
        <input id="testMode" type="number" min="0" max="1" step="1">
        <p class="hint">ON: streams telemetry, enables SIM tests.</p>
      </div>
    </div>
  </div>

  <!-- Live Telemetry -->
  <div class="card">
    <h3>Live Telemetry <span id="stateBadge" class="state-badge s0">IDLE</span></h3>
    <div class="telemetry-grid">
      <div class="tel-box"><div class="tel-label">Accel (m/s²)</div><div class="tel-val" id="tAcc">—</div></div>
      <div class="tel-box"><div class="tel-label">Jerk (m/s²)</div><div class="tel-val" id="tJerk">—</div></div>
      <div class="tel-box"><div class="tel-label">Tilt (°)</div><div class="tel-val" id="tTilt">—</div></div>
    </div>
    <p class="hint">Only streams when Test Mode is ON. At rest: Accel ≈ 9.8, Tilt ≈ 0°. Use these to tune your thresholds.</p>
  </div>

  <!-- SIM Check -->
  <div class="card">
    <h3>SIM & GPS Check (Test Mode only)</h3>
    <button class="btn-secondary" onclick="runSimChecks()">Run SIM Checks</button>
    <div id="simOut" class="mono" style="margin-top:10px">Press "Run SIM Checks"</div>
  </div>

  <!-- Event Log -->
  <div class="card">
    <h3>Event Log</h3>
    <p class="hint" style="margin-bottom:8px">Startup, FSM state changes, fall events, SMS confirmations, heartbeats.</p>
    <div id="logs" class="mono" style="min-height:140px">Waiting for logs...</div>
  </div>

  <!-- Save -->
  <div class="card">
    <button class="btn-primary" onclick="saveAll()">💾 Save All Settings</button>
    <div id="status" class="status"></div>
  </div>
</div>

<script>
const stateNames  = ["IDLE","FREE_FALL","IMPACT_SEEN","CONFIRMING"];
const stateClasses = ["s0","s1","s2","s3"];

let ws = new WebSocket("ws://" + location.host + "/ws");

function appendLog(line) {
  const box = document.getElementById("logs");
  if (box.innerText === "Waiting for logs...") box.innerText = "";
  box.innerText += line + "\n";
  const lines = box.innerText.split("\n");
  if (lines.length > 160) box.innerText = lines.slice(lines.length - 160).join("\n");
  box.scrollTop = box.scrollHeight;
}

ws.onmessage = (e) => {
  try {
    const d = JSON.parse(e.data);
    if (d.type === "telemetry") {
      document.getElementById("tAcc").innerText  = d.acc.toFixed(2);
      document.getElementById("tJerk").innerText = d.jerk.toFixed(2);
      document.getElementById("tTilt").innerText = d.tilt.toFixed(1) + "°";
      const badge = document.getElementById("stateBadge");
      badge.innerText = stateNames[d.state] || "?";
      badge.className = "state-badge " + (stateClasses[d.state] || "s0");
    } else if (d.type === "log") {
      appendLog("[" + Math.floor(d.ts / 1000) + "s] " + d.msg);
    }
  } catch {
    appendLog(e.data);
  }
};

async function loadConfig() {
  const r = await fetch("/api/config");
  const c = await r.json();
  document.getElementById("impact").value   = c.impact;
  document.getElementById("jerk").value     = c.jerk;
  document.getElementById("tilt").value     = c.tilt;
  document.getElementById("testMode").value = c.testMode ? 1 : 0;
  document.getElementById("phone1").value   = c.phone1 || "";
  document.getElementById("phone2").value   = c.phone2 || "";
  document.getElementById("phone3").value   = c.phone3 || "";
}

async function saveAll() {
  const q = new URLSearchParams({
    impact:   document.getElementById("impact").value,
    jerk:     document.getElementById("jerk").value,
    tilt:     document.getElementById("tilt").value,
    testMode: document.getElementById("testMode").value,
    phone1:   document.getElementById("phone1").value,
    phone2:   document.getElementById("phone2").value,
    phone3:   document.getElementById("phone3").value,
  });
  const r = await fetch("/api/save?" + q.toString());
  document.getElementById("status").innerText = await r.text();
}

async function runSimChecks() {
  document.getElementById("simOut").innerText = "Running checks...";
  const r = await fetch("/api/simcheck");
  const t = await r.text();
  try {
    const j = JSON.parse(t);
    document.getElementById("simOut").innerText =
      "Network : " + j.network + "\n" +
      "SIM     : " + j.sim     + "\n" +
      "SMS     : " + j.sms     + "\n" +
      "GPS     : " + j.gps;
  } catch {
    document.getElementById("simOut").innerText = t;
  }
}

loadConfig();
</script>
</body>
</html>
  )rawliteral";
}
