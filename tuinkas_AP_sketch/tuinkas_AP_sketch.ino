/*
 * TuinKasMeter v1.0
 * ─────────────────────────────────────────────────────────────────
 * Platform  : LOLIN Wemos D1 Mini (ESP8266)
 * I2C bus   : SDA=D2 (GPIO4), SCL=D1 (GPIO5)
 * Sensoren  : Si7021 · BH1750 · DS3231
 * Batterij  : via mw.readVBatt() (ingebouwde spanningsdeler MicroWakeupper)
 *
 * Bibliotheken (installeer via Arduino Library Manager):
 *   - Adafruit Si7021          (Adafruit)
 *   - BH1750                   (Christopher Laws / claws)
 *   - RTClib                   (Adafruit)
 *   - MicroWakeupper           (tstoegi)
 *
 * Board     : LOLIN(WEMOS) D1 mini
 * Flash     : 4MB (FS:2MB OTA:~1019KB)  ← kies dit in Tools→Flash Size
 *
 * Werking:
 *   Timer-wakeup  → Si7021 + BH1750 + DS3231 meten → LittleFS → sleep 15 min
 *   Switch-wakeup → WiFi AP "TuinKasMeter" starten → webserver 15 min → sleep
 * ─────────────────────────────────────────────────────────────────
 */

#include <Wire.h>
#include <Adafruit_Si7021.h>
#include <BH1750.h>
#include <RTClib.h>
#include <MicroWakeupper.h>
#include <LittleFS.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>

// ═══════════════════════════════════════════════════════════════════
// CONFIGURATIE
// ═══════════════════════════════════════════════════════════════════

static const char*    AP_SSID        = "TuinKasMeter";
static const char*    AP_PASS        = "";            // open netwerk
static const uint8_t  SLEEP_MIN      = 30;
static const uint8_t  AP_TIMEOUT_MIN = 15;
static const uint8_t  MAX_DAYS       = 90;
static const char*    DATA_DIR       = "/data";

// ── Batterij ─────────────────────────────────────────────────────
// mw.readVBatt() gebruikt de ingebouwde spanningsdeler van MicroWakeupper
static const float    BAT_FULL_V     = 4.20f;
static const float    BAT_EMPTY_V    = 3.00f;

// ═══════════════════════════════════════════════════════════════════
// DATA STRUCTUUR
// ═══════════════════════════════════════════════════════════════════

struct Meting {
  uint32_t  timestamp;
  float     luchtTemp;
  float     vochtigheid;
  float     lux;
  float     behuizingTemp;
  float     battV;
};

// ═══════════════════════════════════════════════════════════════════
// GLOBALE OBJECTEN
// ═══════════════════════════════════════════════════════════════════

static Adafruit_Si7021  si7021;
static BH1750           bh1750;
static RTC_DS3231       rtc;
static MicroWakeupper   mw;
static ESP8266WebServer server(80);
static DNSServer        dns;
static Meting           cachedMeting;

// ═══════════════════════════════════════════════════════════════════
// FORWARD DECLARATIES
// ═══════════════════════════════════════════════════════════════════

static void    doMeting(Meting& m);
static void    slaOp(const Meting& m);
static void    ruimOudDataOp();
static void    startAPModus();
static void    handleRoot();
static void    handleLive();
static void    handleData();
static String  datumNaarBestand(const DateTime& dt);
static int     batPercentage(float v);

// ═══════════════════════════════════════════════════════════════════
// SETUP  (draait opnieuw na elke deep-sleep wake-up)
// ═══════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println(F("\n── TuinKasMeter opstarten ──"));

  Wire.begin(D2, D1);   // SDA=D2, SCL=D1

  // MicroWakeupper moet als eerste geïnitialiseerd worden
  // (configureert de DS3231-alarm en leest wake-reden)
  mw.begin();

  // RTC initialiseren
  if (!rtc.begin()) {
    Serial.println(F("[FOUT] DS3231 niet gevonden!"));
  }
  if (rtc.lostPower()) {
    Serial.println(F("[WARN] RTC heeft stroom verloren, tijd instellen op compiletijd"));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  if (mw.resetedBySwitch()) {
    // ─── Schakelaar ingedrukt: start WiFi AP ───────────────────────
    Serial.println(F("Wakeup: schakelaar → AP-modus"));
    startAPModus();
    // loop() neemt het over
  } else {
    // ─── Timer wake-up: meting doen ───────────────────────────────
    Serial.println(F("Wakeup: timer → meting"));

    if (!si7021.begin()) {
      Serial.println(F("[WARN] Si7021 niet gevonden"));
    }
    bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);

    if (!LittleFS.begin()) {
      Serial.println(F("[FOUT] LittleFS fout — eenmalig formatteren"));
      LittleFS.format();  // alleen bij meting, nooit in AP-modus
      LittleFS.begin();
    }

    Meting m;
    doMeting(m);
    slaOp(m);
    ruimOudDataOp();

    Serial.println(F("Klaar, terug naar sleep"));
    Serial.flush();
    mw.reenable();
    ESP.deepSleep((uint64_t)SLEEP_MIN * 60UL * 1000000UL);
    // Hierna stopt de executie — ESP8266 wordt gereset door RTC-alarm
  }
}

// ═══════════════════════════════════════════════════════════════════
// LOOP  (alleen actief in AP-modus)
// ═══════════════════════════════════════════════════════════════════

void loop() {
  dns.processNextRequest();
  server.handleClient();

  // Ververs sensordata elke 30 seconden
  static uint32_t vorigeUpdate = 0;
  if (millis() - vorigeUpdate >= 1800000UL) {
    doMeting(cachedMeting);
    vorigeUpdate = millis();
  }

  // Na AP_TIMEOUT_MIN minuten terug naar sleep — wacht op knopdruk
  if (millis() >= (uint32_t)AP_TIMEOUT_MIN * 60000UL) {
    Serial.println(F("AP timeout → sleep tot knop"));
    dns.stop();
    WiFi.softAPdisconnect(true);
    Serial.flush();
    mw.reenable();
    ESP.deepSleep(0);
  }
}

// ═══════════════════════════════════════════════════════════════════
// METING
// ═══════════════════════════════════════════════════════════════════

static void doMeting(Meting& m) {
  m.timestamp     = rtc.now().unixtime();
  m.luchtTemp     = si7021.readTemperature();
  m.vochtigheid   = si7021.readHumidity();
  m.lux           = bh1750.readLightLevel();
  if (m.lux < 0.0f) m.lux = 0.0f;
  m.behuizingTemp = rtc.getTemperature();
  m.battV         = mw.readVBatt();

  Serial.printf("  T=%.1f°C  H=%.1f%%  L=%.0flux  BT=%.1f°C  BAT=%.2fV\n",
    m.luchtTemp, m.vochtigheid, m.lux, m.behuizingTemp, m.battV);
}

// ═══════════════════════════════════════════════════════════════════
// OPSLAAN IN LITTLEFS
// Bestand per dag: /data/YYYYMMDD.csv
// Formaat: unix,HH:MM,luchtTemp,vochtigheid,lux,behuizingTemp,battV
// ═══════════════════════════════════════════════════════════════════

static void slaOp(const Meting& m) {
  DateTime dt(m.timestamp);

  if (!LittleFS.exists(DATA_DIR)) {
    LittleFS.mkdir(DATA_DIR);
  }

  String pad = String(DATA_DIR) + "/" + datumNaarBestand(dt) + ".csv";
  bool nieuw = !LittleFS.exists(pad);

  File f = LittleFS.open(pad, "a");
  if (!f) {
    Serial.println(F("[FOUT] Kan bestand niet openen"));
    return;
  }

  if (nieuw) {
    f.println(F("ts,tijd,luchtT,vocht,lux,bhuizT,battV"));
  }

  char regel[96];
  snprintf(regel, sizeof(regel),
    "%lu,%02d:%02d,%.2f,%.2f,%.1f,%.2f,%.3f",
    (unsigned long)m.timestamp,
    dt.hour(), dt.minute(),
    m.luchtTemp, m.vochtigheid, m.lux,
    m.behuizingTemp, m.battV);
  f.println(regel);
  f.close();

  Serial.printf("  Opgeslagen: %s\n", pad.c_str());
}

// ═══════════════════════════════════════════════════════════════════
// OPRUIMEN: verwijder CSV-bestanden ouder dan MAX_DAYS
// ═══════════════════════════════════════════════════════════════════

static void ruimOudDataOp() {
  uint32_t nu = rtc.now().unixtime();
  Dir dir = LittleFS.openDir(DATA_DIR);

  while (dir.next()) {
    String naam = dir.fileName();
    if (naam.length() < 8) continue;

    int jaar  = naam.substring(0, 4).toInt();
    int maand = naam.substring(4, 6).toInt();
    int dag   = naam.substring(6, 8).toInt();

    if (jaar < 2020 || maand < 1 || maand > 12 || dag < 1 || dag > 31) continue;

    DateTime fd(jaar, maand, dag, 0, 0, 0);
    long ouderDagen = ((long)nu - (long)fd.unixtime()) / 86400L;

    if (ouderDagen > MAX_DAYS) {
      String pad = String(DATA_DIR) + "/" + naam;
      LittleFS.remove(pad);
      Serial.printf("  Verwijderd: %s (%ld dagen oud)\n", pad.c_str(), ouderDagen);
    }
  }
}

// ═══════════════════════════════════════════════════════════════════
// HULPFUNCTIES
// ═══════════════════════════════════════════════════════════════════

static String datumNaarBestand(const DateTime& dt) {
  char buf[12];
  snprintf(buf, sizeof(buf), "%04d%02d%02d", dt.year(), dt.month(), dt.day());
  return String(buf);
}

static int batPercentage(float v) {
  if (v >= BAT_FULL_V)  return 100;
  if (v <= BAT_EMPTY_V) return 0;
  return (int)((v - BAT_EMPTY_V) / (BAT_FULL_V - BAT_EMPTY_V) * 100.0f + 0.5f);
}

// ═══════════════════════════════════════════════════════════════════
// AP MODUS
// ═══════════════════════════════════════════════════════════════════

static void disableDS3231Alarm1() {
  // Zet DS3231 Alarm1 interrupt uit zodat timer-alarm niet reset tijdens AP-modus
  Wire.beginTransmission(0x68);
  Wire.write(0x0E);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 1);
  if (!Wire.available()) return;
  uint8_t ctrl = Wire.read() & ~0x01;  // wis A1IE bit
  Wire.beginTransmission(0x68);
  Wire.write(0x0E);
  Wire.write(ctrl);
  Wire.endTransmission();
  // wis ook Alarm1 vlag in statusregister
  Wire.beginTransmission(0x68);
  Wire.write(0x0F);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 1);
  if (!Wire.available()) return;
  uint8_t st = Wire.read() & ~0x01;
  Wire.beginTransmission(0x68);
  Wire.write(0x0F);
  Wire.write(st);
  Wire.endTransmission();
}

static void startAPModus() {
  disableDS3231Alarm1();

  if (!si7021.begin()) {
    Serial.println(F("[WARN] Si7021 niet gevonden"));
  }
  bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);

  if (!LittleFS.begin()) {
    Serial.println(F("[WARN] LittleFS niet beschikbaar, geen historische data"));
    // Geen format() hier — duurt te lang (watchdog reset)
  }

  doMeting(cachedMeting);
  slaOp(cachedMeting);

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(200);  // wacht tot AP volledig actief is

  Serial.print(F("AP gestart: "));
  Serial.print(AP_SSID);
  Serial.print(F("  IP: "));
  Serial.println(WiFi.softAPIP());

  // DNS: elke domeinnaam → 192.168.4.1 (captive portal)
  dns.start(53, "*", WiFi.softAPIP());

  server.on("/",           handleRoot);
  server.on("/index.html", handleRoot);
  server.on("/api/live",   handleLive);
  server.on("/api/data",   handleData);

  // Captive portal detectie — Android / iOS / Windows sturen deze URLs
  // om te checken of er internet is. Een redirect → OS toont loginprompt
  // én gebruikt dan de WiFi-route voor alle verkeer.
  auto redir = [](){
    server.sendHeader(F("Location"), F("http://192.168.4.1/"));
    server.send(302, F("text/plain"), "");
  };
  server.on(F("/generate_204"),              redir);  // Android
  server.on(F("/gen_204"),                   redir);  // Android oud
  server.on(F("/hotspot-detect.html"),       redir);  // Apple
  server.on(F("/library/test/success.html"), redir);  // Apple
  server.on(F("/ncsi.txt"),                  redir);  // Windows
  server.on(F("/connecttest.txt"),           redir);  // Windows
  server.onNotFound(redir);
  server.begin();
  Serial.println(F("Webserver actief"));
}

// ═══════════════════════════════════════════════════════════════════
// WEB: /api/live  →  JSON met actuele meting
// ═══════════════════════════════════════════════════════════════════

static void handleLive() {
  const Meting& m = cachedMeting;

  char json[192];
  snprintf(json, sizeof(json),
    "{\"ts\":%lu,\"luchtT\":%.2f,\"vocht\":%.2f,"
    "\"lux\":%.1f,\"bhuizT\":%.2f,\"battV\":%.3f,\"battPct\":%d}",
    (unsigned long)m.timestamp,
    m.luchtTemp, m.vochtigheid,
    m.lux, m.behuizingTemp, m.battV,
    batPercentage(m.battV));

  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send(200, F("application/json"), json);
}

// ═══════════════════════════════════════════════════════════════════
// WEB: /api/data?days=N  →  JSON-array met historische metingen
// ═══════════════════════════════════════════════════════════════════

static void handleData() {
  int dagen = 7;
  if (server.hasArg(F("days")))
    dagen = constrain(server.arg(F("days")).toInt(), 1, MAX_DAYS);

  uint32_t nu = rtc.now().unixtime();
  uint32_t totaalRegels = 0;

  for (int d = dagen - 1; d >= 0; d--) {
    DateTime dt(nu - (uint32_t)d * 86400UL);
    String pad = String(DATA_DIR) + "/" + datumNaarBestand(dt) + ".csv";
    if (!LittleFS.exists(pad)) continue;
    File f = LittleFS.open(pad, "r");
    if (!f) continue;
    while (f.available()) { f.readStringUntil('\n'); totaalRegels++; yield(); }
    f.close();
  }
  if (totaalRegels > 0) totaalRegels--;

  uint32_t stap = (totaalRegels > 500) ? (totaalRegels + 499) / 500 : 1;

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send(200, F("application/json"), "");
  server.sendContent("[");
  bool eerste = true;
  uint32_t teller = 0;

  for (int d = dagen - 1; d >= 0; d--) {
    DateTime dt(nu - (uint32_t)d * 86400UL);
    String pad = String(DATA_DIR) + "/" + datumNaarBestand(dt) + ".csv";
    if (!LittleFS.exists(pad)) continue;
    File f = LittleFS.open(pad, "r");
    if (!f) continue;
    f.readStringUntil('\n');
    while (f.available()) {
      String regel = f.readStringUntil('\n');
      regel.trim();
      if (regel.length() < 10) continue;
      teller++;
      if ((teller % stap) != 0) continue;
      String v[7]; int vi = 0, start = 0;
      for (int i = 0; i <= (int)regel.length() && vi < 7; i++) {
        if (i == (int)regel.length() || regel[i] == ',') {
          v[vi++] = regel.substring(start, i); start = i + 1;
        }
      }
      if (vi < 7) continue;
      char punt[160];
      snprintf(punt, sizeof(punt),
        "%s{\"ts\":%s,\"t\":%s,\"h\":%s,\"l\":%s,\"ct\":%s,\"b\":%s}",
        eerste ? "" : ",",
        v[0].c_str(), v[2].c_str(), v[3].c_str(),
        v[4].c_str(), v[5].c_str(), v[6].c_str());
      server.sendContent(punt);
      eerste = false;
      yield();
    }
    f.close();
  }
  server.sendContent("]");
  server.sendContent("");
}

// ═══════════════════════════════════════════════════════════════════
// WEB: /  →  Hoofdpagina
// ═══════════════════════════════════════════════════════════════════

static const char HTML_1[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="nl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>TuinKasMeter</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,sans-serif;background:#0f172a;color:#e2e8f0;padding:12px}
h1{color:#38bdf8;text-align:center;padding:12px 0;font-size:1.5em}
h2{color:#7dd3fc;font-size:.9em;margin-bottom:4px}
.kaarten{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:10px;margin:12px 0}
.kaart{background:#1e293b;border-radius:10px;padding:14px;text-align:center;border:1px solid #334155}
.kaart .waarde{font-size:1.9em;font-weight:700;color:#38bdf8;line-height:1}
.kaart .eenheid{font-size:.75em;color:#94a3b8;margin-top:2px}
.kaart .naam{font-size:.7em;color:#64748b;margin-top:6px;text-transform:uppercase;letter-spacing:.04em}
.bat-balk{height:6px;border-radius:3px;background:#334155;margin-top:4px;overflow:hidden}
.bat-vulling{height:100%;border-radius:3px;background:#4ade80;transition:width .5s}
.gb{background:#1e293b;border-radius:10px;padding:12px;margin:10px 0;border:1px solid #334155}
.balk{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin:10px 0}
select,button{background:#1e293b;color:#e2e8f0;border:1px solid #38bdf8;border-radius:6px;padding:5px 10px;font-size:.85em}
button{background:#38bdf8;color:#0f172a;font-weight:700;cursor:pointer}
canvas{width:100%;display:block}
#st{font-size:.78em;color:#94a3b8}
</style>
</head>
<body>
<h1>&#127807; TuinKasMeter</h1>
<div class="kaarten">
<div class="kaart"><div class="waarde" id="cT">--</div><div class="eenheid">&deg;C</div><div class="naam">Luchttemp</div></div>
<div class="kaart"><div class="waarde" id="cH">--</div><div class="eenheid">%</div><div class="naam">Vochtigheid</div></div>
<div class="kaart"><div class="waarde" id="cL">--</div><div class="eenheid">lux</div><div class="naam">Licht</div></div>
<div class="kaart"><div class="waarde" id="cBT">--</div><div class="eenheid">&deg;C</div><div class="naam">Behuizing</div></div>
<div class="kaart"><div class="waarde" id="cV">--</div><div class="eenheid">V</div><div class="naam">Batterij</div>
<div class="bat-balk"><div class="bat-vulling" id="cBB" style="width:0%"></div></div></div>
</div>
<div class="balk">
<label>Periode:<select id="sd" onchange="lg()">
<option value="1">Vandaag</option><option value="7" selected>7 dagen</option>
<option value="30">30 dagen</option><option value="90">90 dagen</option>
</select></label>
<button onclick="lg()">&#8635; Vernieuwen</button>
<span id="st"></span>
</div>
<div class="gb"><h2>Temperatuur (&deg;C)</h2><canvas id="gT"></canvas></div>
<div class="gb"><h2>Vochtigheid (%)</h2><canvas id="gH"></canvas></div>
<div class="gb"><h2>Lichtsterkte (lux)</h2><canvas id="gL"></canvas></div>
<div class="gb"><h2>Batterij (V)</h2><canvas id="gB"></canvas></div>
)rawhtml";

static const char HTML_2[] PROGMEM = R"rawhtml(<script>
function dc(id,vals,ts,kleur){
  const cv=document.getElementById(id);
  cv.width=cv.parentElement.clientWidth-24;cv.height=165;
  const W=cv.width,H=165,ctx=cv.getContext('2d');
  const P={t:8,r:8,b:36,l:42},CW=W-P.l-P.r,CH=H-P.t-P.b;
  const N=vals.length;if(N<2)return;
  let mn=vals[0],mx=vals[0];
  vals.forEach(v=>{if(v<mn)mn=v;if(v>mx)mx=v;});
  if(mn===mx){mn-=1;mx+=1;}
  ctx.clearRect(0,0,W,H);
  for(let i=0;i<=4;i++){
    const y=P.t+CH*i/4;
    ctx.strokeStyle='#334155';ctx.lineWidth=1;
    ctx.beginPath();ctx.moveTo(P.l,y);ctx.lineTo(P.l+CW,y);ctx.stroke();
    ctx.fillStyle='#94a3b8';ctx.font='10px system-ui';ctx.textAlign='right';
    ctx.fillText((mx-(mx-mn)*i/4).toFixed(1),P.l-3,y+4);
  }
  function fmt(t){
    const d=new Date(t*1000);
    return d.getUTCDate()+'/'+(d.getUTCMonth()+1)+' '+
      String(d.getUTCHours()).padStart(2,'0')+':'+String(d.getUTCMinutes()).padStart(2,'0');
  }
  const step=Math.max(1,Math.floor(N/4));
  ctx.font='9px system-ui';ctx.textAlign='center';ctx.fillStyle='#64748b';
  for(let i=0;i<N;i+=step){
    ctx.fillText(fmt(ts[i]),P.l+CW*i/(N-1),H-4);
  }
  ctx.strokeStyle=kleur;ctx.lineWidth=1.5;ctx.beginPath();
  vals.forEach((v,i)=>{
    const x=P.l+CW*i/(N-1),y=P.t+CH*(1-(v-mn)/(mx-mn));
    i?ctx.lineTo(x,y):ctx.moveTo(x,y);
  });
  ctx.stroke();
}
function lx(v){return v>=1000?(v/1000).toFixed(1)+'k':Math.round(v).toString();}
function lg(){
  const d=document.getElementById('sd').value;
  document.getElementById('st').textContent='Laden…';
  fetch('/api/data?days='+d).then(r=>r.json()).then(data=>{
    if(!data.length){document.getElementById('st').textContent='Geen data';return;}
    const ts=data.map(d=>d.ts);
    dc('gT',data.map(d=>d.t),ts,'#f87171');
    dc('gH',data.map(d=>d.h),ts,'#38bdf8');
    dc('gL',data.map(d=>d.l),ts,'#facc15');
    dc('gB',data.map(d=>d.b),ts,'#4ade80');
    document.getElementById('st').textContent=
      'Bijgewerkt '+new Date().toLocaleTimeString('nl')+' — '+data.length+' punten';
  }).catch(e=>document.getElementById('st').textContent='Fout: '+e);
}
function ll(){
  fetch('/api/live').then(r=>r.json()).then(d=>{
    document.getElementById('cT').textContent=d.luchtT.toFixed(1);
    document.getElementById('cH').textContent=d.vocht.toFixed(1);
    document.getElementById('cL').textContent=lx(d.lux);
    document.getElementById('cBT').textContent=d.bhuizT.toFixed(1);
    document.getElementById('cV').textContent=d.battV.toFixed(2);
    document.getElementById('cBB').style.width=d.battPct+'%';
    document.getElementById('cBB').style.background=
      d.battPct>50?'#4ade80':d.battPct>20?'#facc15':'#f87171';
  }).catch(console.error);
}
ll();lg();setInterval(ll,1800000);
</script></body></html>
)rawhtml";

static void handleRoot() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, F("text/html; charset=UTF-8"), "");
  server.sendContent_P(HTML_1);
  server.sendContent_P(HTML_2);
  server.sendContent("");
}