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
static const uint8_t  SLEEP_MIN      = 15;
static const uint8_t  AP_TIMEOUT_MIN = 15;
static const uint8_t  MAX_DAYS       = 90;
static const char*    DATA_DIR       = "/data";

// ── Batterij ─────────────────────────────────────────────────────
// mw.readVBatt() gebruikt de ingebouwde spanningsdeler van MicroWakeupper
static const float    BAT_FULL_V     = 4.20f;
static const float    BAT_EMPTY_V    = 3.00f;

// ═══════════════════════════════════════════════════════════════════
// GLOBALE OBJECTEN
// ═══════════════════════════════════════════════════════════════════

static Adafruit_Si7021  si7021;
static BH1750           bh1750;
static RTC_DS3231       rtc;
static MicroWakeupper   mw;
static ESP8266WebServer server(80);
static DNSServer        dns;

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

  server.on("/",              handleRoot);
  server.on("/index.html",    handleRoot);
  server.on("/api/live",      handleLive);
  server.on("/api/data",      handleData);
  server.onNotFound(handleRoot);
  server.begin();
  Serial.println(F("Webserver actief"));
}

// ═══════════════════════════════════════════════════════════════════
// WEB: /api/live  →  JSON met actuele meting
// ═══════════════════════════════════════════════════════════════════

static void handleLive() {
  Meting m;
  doMeting(m);

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
// Max 500 punten (downsampling bij veel data)
// ═══════════════════════════════════════════════════════════════════

static void handleData() {
  int dagen = 7;
  if (server.hasArg(F("days"))) {
    dagen = constrain(server.arg(F("days")).toInt(), 1, MAX_DAYS);
  }

  // Tel eerst het totale aantal regels om samplestap te bepalen
  uint32_t nu = rtc.now().unixtime();
  uint32_t totaalRegels = 0;

  for (int d = dagen - 1; d >= 0; d--) {
    DateTime dt(nu - (uint32_t)d * 86400UL);
    String pad = String(DATA_DIR) + "/" + datumNaarBestand(dt) + ".csv";
    if (!LittleFS.exists(pad)) continue;
    File f = LittleFS.open(pad, "r");
    if (!f) continue;
    while (f.available()) {
      f.readStringUntil('\n');
      totaalRegels++;
    }
    f.close();
  }
  if (totaalRegels > 0) totaalRegels--;  // trek header af per dag

  uint32_t stap = (totaalRegels > 500) ? (totaalRegels + 499) / 500 : 1;

  // Stream JSON
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

    f.readStringUntil('\n');  // skip header

    while (f.available()) {
      String regel = f.readStringUntil('\n');
      regel.trim();
      if (regel.length() < 10) continue;

      teller++;
      if ((teller % stap) != 0) continue;

      // Parse CSV: ts,tijd,luchtT,vocht,lux,bhuizT,battV
      String v[7];
      int vi = 0, start = 0;
      for (int i = 0; i <= (int)regel.length() && vi < 7; i++) {
        if (i == (int)regel.length() || regel[i] == ',') {
          v[vi++] = regel.substring(start, i);
          start = i + 1;
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
// WEB: /  →  Hoofdpagina (PROGMEM-blokken gestreamd)
// ═══════════════════════════════════════════════════════════════════

// HTML opgedeeld in stukken om PROGMEM-limiet (~32KB per literal) te
// respecteren en heap-gebruik laag te houden.

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
h2{color:#7dd3fc;font-size:1em;margin-bottom:8px}
.kaarten{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:10px;margin:12px 0}
.kaart{background:#1e293b;border-radius:10px;padding:14px;text-align:center;border:1px solid #334155}
.kaart .waarde{font-size:1.9em;font-weight:700;color:#38bdf8;line-height:1}
.kaart .eenheid{font-size:.75em;color:#94a3b8;margin-top:2px}
.kaart .naam{font-size:.7em;color:#64748b;margin-top:6px;text-transform:uppercase;letter-spacing:.04em}
.grafiek-blok{background:#1e293b;border-radius:10px;padding:14px;margin:10px 0;border:1px solid #334155}
.balk{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin:8px 0}
select,button{background:#1e293b;color:#e2e8f0;border:1px solid #38bdf8;border-radius:6px;padding:5px 10px;font-size:.85em}
button{background:#38bdf8;color:#0f172a;font-weight:700;cursor:pointer}
button:hover{background:#7dd3fc}
#status{font-size:.78em;color:#94a3b8}
.bat-balk{height:6px;border-radius:3px;background:#334155;margin-top:4px;overflow:hidden}
.bat-vulling{height:100%;border-radius:3px;background:#4ade80;transition:width .5s}
</style>
</head>
<body>
<h1>&#127807; TuinKasMeter</h1>
<div class="kaarten">
  <div class="kaart">
    <div class="waarde" id="cLuchtT">--</div>
    <div class="eenheid">&deg;C</div>
    <div class="naam">Luchttemperatuur</div>
  </div>
  <div class="kaart">
    <div class="waarde" id="cVocht">--</div>
    <div class="eenheid">%</div>
    <div class="naam">Vochtigheid</div>
  </div>
  <div class="kaart">
    <div class="waarde" id="cLux">--</div>
    <div class="eenheid">lux</div>
    <div class="naam">Licht</div>
  </div>
  <div class="kaart">
    <div class="waarde" id="cBhuizT">--</div>
    <div class="eenheid">&deg;C</div>
    <div class="naam">Behuizing</div>
  </div>
  <div class="kaart">
    <div class="waarde" id="cBattV">--</div>
    <div class="eenheid">V</div>
    <div class="naam">Batterij</div>
    <div class="bat-balk"><div class="bat-vulling" id="cBatBalk" style="width:0%"></div></div>
  </div>
</div>
<div class="balk">
  <label>Periode:
    <select id="selDagen" onchange="laadGrafieken()">
      <option value="1">Vandaag</option>
      <option value="7" selected>7 dagen</option>
      <option value="30">30 dagen</option>
      <option value="90">90 dagen</option>
    </select>
  </label>
  <button onclick="laadGrafieken()">&#8635; Vernieuwen</button>
  <span id="status"></span>
</div>
<div class="grafiek-blok">
  <h2>Temperatuur &amp; Vochtigheid</h2>
  <canvas id="gTH"></canvas>
</div>
<div class="grafiek-blok">
  <h2>Lichtsterkte (lux)</h2>
  <canvas id="gLux"></canvas>
</div>
<div class="grafiek-blok">
  <h2>Batterij (V)</h2>
  <canvas id="gBat"></canvas>
</div>
)rawhtml";

static const char HTML_2[] PROGMEM = R"rawhtml(<script>
const SCHAAL = {
  x:{ticks:{color:'#64748b',maxTicksLimit:10},grid:{color:'#1e293b'}},
  y:{ticks:{color:'#94a3b8'},grid:{color:'#334155'}}
};
const BASE_OPTS = {
  responsive:true,
  animation:false,
  plugins:{legend:{labels:{color:'#e2e8f0'}}},
  scales:{x:SCHAAL.x,y:SCHAAL.y}
};
let gTH, gLux, gBat;

function tijdLabel(ts){
  const d = new Date(ts*1000);
  return d.toLocaleDateString('nl',{month:'short',day:'numeric'})
    +' '+d.toLocaleTimeString('nl',{hour:'2-digit',minute:'2-digit'});
}

function luxLabel(v){
  return v >= 1000 ? (v/1000).toFixed(1)+'k' : Math.round(v).toString();
}

function maakGrafieken(labels, data){
  if(gTH) gTH.destroy();
  gTH = new Chart(document.getElementById('gTH'),{
    type:'line',
    data:{labels, datasets:[
      {label:'Luchttemp (°C)',data:data.map(d=>d.t),
       borderColor:'#f87171',tension:.3,pointRadius:0,borderWidth:1.5},
      {label:'Vochtigheid (%)',data:data.map(d=>d.h),
       borderColor:'#38bdf8',tension:.3,pointRadius:0,borderWidth:1.5,yAxisID:'y2'}
    ]},
    options:{...BASE_OPTS, scales:{
      x:SCHAAL.x,
      y:{...SCHAAL.y, position:'left'},
      y2:{...SCHAAL.y, position:'right', grid:{drawOnChartArea:false}}
    }}
  });

  if(gLux) gLux.destroy();
  gLux = new Chart(document.getElementById('gLux'),{
    type:'line',
    data:{labels, datasets:[
      {label:'Lux',data:data.map(d=>d.l),
       borderColor:'#facc15',tension:.3,pointRadius:0,borderWidth:1.5,
       fill:true,backgroundColor:'rgba(250,204,21,.08)'}
    ]},
    options:BASE_OPTS
  });

  if(gBat) gBat.destroy();
  gBat = new Chart(document.getElementById('gBat'),{
    type:'line',
    data:{labels, datasets:[
      {label:'Batterij (V)',data:data.map(d=>d.b),
       borderColor:'#4ade80',tension:.3,pointRadius:0,borderWidth:1.5}
    ]},
    options:BASE_OPTS
  });
}

function laadGrafieken(){
  const dagen = document.getElementById('selDagen').value;
  document.getElementById('status').textContent = 'Laden…';
  fetch('/api/data?days='+dagen)
    .then(r => r.json())
    .then(data => {
      if(!data.length){ document.getElementById('status').textContent='Geen data'; return; }
      maakGrafieken(data.map(d=>tijdLabel(d.ts)), data);
      document.getElementById('status').textContent =
        'Bijgewerkt '+new Date().toLocaleTimeString('nl')+
        ' — '+data.length+' meetpunten';
    })
    .catch(e => document.getElementById('status').textContent = 'Fout: '+e);
}

function laadLive(){
  fetch('/api/live')
    .then(r => r.json())
    .then(d => {
      document.getElementById('cLuchtT').textContent  = d.luchtT.toFixed(1);
      document.getElementById('cVocht').textContent   = d.vocht.toFixed(1);
      document.getElementById('cLux').textContent     = luxLabel(d.lux);
      document.getElementById('cBhuizT').textContent  = d.bhuizT.toFixed(1);
      document.getElementById('cBattV').textContent   = d.battV.toFixed(2);
      document.getElementById('cBatBalk').style.width = d.battPct+'%';
      const kleur = d.battPct>50?'#4ade80':d.battPct>20?'#facc15':'#f87171';
      document.getElementById('cBatBalk').style.background = kleur;
    })
    .catch(console.error);
}

laadLive();
laadGrafieken();
setInterval(laadLive, 60000);
</script>
</body></html>
)rawhtml";

static void handleRoot() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, F("text/html; charset=UTF-8"), "");
  server.sendContent_P(HTML_1);
  server.sendContent_P(HTML_2);
  server.sendContent("");
}
