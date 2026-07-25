/* ============================================================
   PAJERO NODE — firmware
   Board : Waveshare ESP32-S3-A/SIM7670X-4G  (SIM7670G variant)
   Link  : LTE Cat-1 -> MQTT/TLS -> broker -> iPhone web app

   Outputs
     1. Keyfob UNLOCK    200 ms pulse   (IO1)
     2. Keyfob LOCK      200 ms pulse   (IO2)
     3. BLACK BOX     10000 ms relay    (IO42 -> 5V relay -> 12V actuator)
     4. GATE            400 ms pulse    (IO39)

   Paths
     Cellular:  MQTT/TLS <-> broker <-> iPhone app   (unlock/lock/gate/blackbox)
     LoRa mesh: Heltec V3 (stock Meshtastic, TEXTMSG) <-UART-> this board.
                Accepts a time-based UNLOCK code ONLY while 4G is down, and
                squirts a fast text position off-grid. See the mesh section.

   Topics
     publish   pajero/<ID>/status   every STATUS_EVERY_MS
     subscribe pajero/<ID>/cmd      {"cmd":"unlock","secret":"...","ts":1700000000000}

   ---------- LIBRARIES ----------
   TinyGSM: the Library Manager version does NOT support SIM7670G.
   Use the fork:  https://github.com/lewisxhe/TinyGSM
     Download ZIP -> Sketch -> Include Library -> Add .ZIP Library
   PubSubClient : Library Manager, by Nick O'Leary
   ArduinoJson  : Library Manager, v7

   ---------- BEFORE POWER-ON ----------
   LTE antenna on MAIN. GNSS antenna on GNSS. Never transmit without them.
   DIP switch (back): 4G = ON, USB = OFF.
   ============================================================ */

#define TINY_GSM_MODEM_SIM7672        // fork's driver for SIM7670G/SIM7672
#define TINY_GSM_RX_BUFFER 1024

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Wire.h>
#include "mbedtls/md.h"
#include <sys/time.h>
#include <time.h>

/* ================= CONFIG =================
   Pre-filled for the FIRST TEST. It will connect as-is.
   HARDEN BEFORE THE CAR: change MQTT_PASS to a strong random one in HiveMQ,
   and (optionally) regenerate SECRET. Whatever you change here, change the
   SAME value in the app. APN below is the common Israeli default - if GPRS
   won't attach, set it to your carrier's exact APN.                        */
const char* APN        = "internet.rl";     // Rami Levy (MVNO on Partner)
const char* GPRS_USER  = "";                // Rami Levy attaches on the plain APN (no user)
const char* GPRS_PASS  = "";
const char* SIM_PIN    = "";                // leave empty if the SIM PIN is disabled

const char* MQTT_HOST  = "7e8b557fbc1e49c6b158e020ea18b265.s1.eu.hivemq.cloud";
const int   MQTT_PORT  = 8883;              // 8883 = MQTT over TLS (HiveMQ fixed this port)
const char* MQTT_USER  = "pajero2";
const char* MQTT_PASS  = "pajeropassword123";   // <-- weak/known: replace before the car goes live

const char* DEVICE_ID  = "pajero1";         // must match the app
const char* SECRET     = "8yQgTSIEpMzWISaVBQ6uUmQhmXVY";   // must match the app (Connection panel)

#define USE_TLS            1                // 1 = mqtts:8883. HiveMQ Cloud requires this.
#define STATUS_EVERY_MS    10000
#define CMD_MAX_AGE_MS     60000            // reject commands older than this. See note below.

/* ---- LoRa mesh (Heltec V3 running stock Meshtastic, Serial module = TEXTMSG) ----
   The Heltec has its OWN GPS and beacons its own position on the mesh.
   This board only: (a) accepts a time-based UNLOCK code over the mesh, and only
   while 4G is DOWN; (b) when 4G is down, also squirts its own GPS as a text line
   so you get an extra fast breadcrumb on top of Meshtastic's native pin.        */
#define MESH_BAUD          38400
/* GPIO 40 was DRIVEN HIGH by on-board hardware (confirmed by the GPIO survey),
   so the Heltec RX line is moved to 48, verified FREE. Wire Heltec GPIO33 (TX)
   to Waveshare GPIO48. */
#define MESH_RXD           48     // Waveshare RX  <- Heltec TX = Heltec GPIO33
#define MESH_TXD           41     // Waveshare TX  -> Heltec RX = Heltec GPIO34
//  (Heltec's own GPS lives on its GPIO47/48 - a separate UART, nothing to do with these.)
//  Set MESH_RXD/MESH_TXD to whatever Waveshare pins are free after the PCF8574 expander.
#define MESH_TOTP_STEP     60     // unlock code changes every 60 s
#define MESH_TOTP_SKEW     2      // accept codes within +/- this many steps (clock drift)
#define MESH_FASTPOS_MS    60000  // off-grid: text-position every 60 s
/* ============================================================ */

/* ---------- PINS ----------
   Confirmed from the Waveshare wiki for this board:
     Camera 24-pin FPC : XCLK 34, SIOD 15, SIOC 16, VSYNC 36, HREF 35, PCLK 37,
                         Y9..Y2 = 14,13,12,11,10,9,8,7
     TF card           : CLK 5, CMD 4, DATA 6, CD 46
     RGB LED (WS2812B) : 38
     Battery gauge     : MAX17048 over I2C
   Those are all taken. The three free header pins below avoid every one of
   them, and avoid the strapping pins (0, 3, 45, 46) and USB (19, 20).

   !! Before wiring: the MAX17048 I2C pins are not stated in the wiki text.
      If the gauge stops reporting after you wire these, move the outputs
      to IO41 / IO42 / IO40 and retest.                                     */
#define PIN_UNLOCK    1     // PhotoMOS across keyfob unlock button
#define PIN_LOCK      2     // PhotoMOS across keyfob lock button
#define PIN_BLACKBOX  42    // drives a 5V relay module; relay feeds 12V to the actuator
#define PIN_GATE      39    // PhotoMOS across the gate remote's button (add later)
/* Hidden wired unlock button: momentary switch between this pin and GND.
   Works with a dead phone, a dead SIM and no network -- the last-resort key.
   Mount it somewhere only you know (inside a wheel arch, under a trim panel). */
#define PIN_UNLOCK_BTN 47
#define BTN_DEBOUNCE_MS 60
#define BTN_HOLD_MS     10000 // must be HELD 10s: makes an accidental press impossible

// IO21 is deliberately left free: it is the only wake-capable (<=21) pin
// spare, reserved for the LIS3DH motion-interrupt that wakes deep sleep.

#define MODEM_TX 18          // swapped: this orientation gave "modem ready" on the real board
#define MODEM_RX 17

#define PULSE_KEYFOB_MS    200
#define PULSE_GATE_MS      400     // gate remotes often want a slightly longer press
#define PULSE_BLACKBOX_MS  10000   // relay held closed 10s -> actuator energised 10s

/* Relay/optocoupler polarity.
   Most cheap relay boards are ACTIVE LOW (pin low = relay closed).
   Set to 0 if yours closes on HIGH. Getting this wrong = output stuck on
   at boot, which on the unlock line means an unlocked car. Check first. */
#define FW_VERSION "1.4.0"            // bump this each build you flash/OTA
#define FW_BUILD   (__DATE__ " " __TIME__)  // auto: compile date+time
#define OUTPUT_ACTIVE_LOW  1

/* ---- cell diagnostics + 24h location history + cell-based geolocation ----
   OCID_TOKEN: OpenCellID access token (the "pk...." string from
   my.opencellid.org/dashboard#accesstoken). Hardcoded here for bench testing --
   ROTATE IT before this is anything but a bench unit, since the firmware repo is
   public. Coverage is community-sourced: dense in towns, thin in the desert, so
   a lookup can legitimately come back "not in database" -- that is not a bug.
   Note: the current API wants the FULL token incl. the "pk." prefix. If you get
   "API Key not known", try again with the pk. removed (older endpoints differ). */
#define OCID_TOKEN   "pk.c1c604245282802cc296e9a940d23552"
#define HIST_SLOTS   144          // 24h at one sample / 10 min
#define HIST_EVERY_MS 600000UL    // 10 min
#define OUT_ON   (OUTPUT_ACTIVE_LOW ? LOW  : HIGH)
#define OUT_OFF  (OUTPUT_ACTIVE_LOW ? HIGH : LOW)

HardwareSerial modemSerial(1);
TinyGsm        modem(modemSerial);
#if USE_TLS
  TinyGsmClientSecure netClient(modem, 0);   // mux 0; fork's SIM7672 TLS socket (+CCH)
#else
  TinyGsmClient       netClient(modem);
#endif
PubSubClient mqtt(netClient);

/* Telegram goes straight from the car — no server in the middle. Uses its own
   TLS socket (mux 1) so it doesn't disturb the MQTT link on mux 0. */
#define TG_ENABLE      1     // 0 = compile Telegram out entirely
/* MQTT holds a TLS socket on mux 0; Telegram needs a second on mux 1. If this
   modem won't run both, MQTT drops with state=-2. Pausing MQTT during the send
   sidesteps that. Set to 0 to try both sockets at once. */
#define TG_PAUSE_MQTT  1
const char* TG_TOKEN = "8768067779:AAF67VdHLw61SamevCP2OIy2k9zQJUQLV_Q";
const char* TG_CHAT  = "7979565535";
const char* TG_HOST  = "api.telegram.org";
/* Result of the last Telegram attempt. Held here and flushed to the app's Node
   log once MQTT is back, since MQTT is down while the message is being sent. */
char tgPending[180] = "";

/* Defined up here on purpose: the Arduino IDE auto-generates function prototypes
   and inserts them ABOVE the first function definition. checkLimit() takes a
   Reading*, so the type must exist before that insertion point -- otherwise you
   get "variable or field 'checkLimit' declared void". */
struct Reading { float v; bool ok; uint8_t over; uint32_t lastAlert; };

/* ---------- battery gauge (MAX17048) ----------
   Found by scanning: SDA=15, SCL=16. Verified by reading its registers -- the
   value tracks the cell (3.35V fitted, 4.28V with the holder empty).
   We report VOLTS, which the chip measures directly and reliably. Its own SOC
   output was clearly wrong on first contact (0.1% at 3.35V, 2% at 4.28V), so we
   kick it with a QuickStart at boot to rebuild its model, and we also compute a
   percentage ourselves from the discharge curve as a sanity figure. */
#define GAUGE_SDA   15
#define GAUGE_SCL   16
#define GAUGE_ADDR  0x36
bool  gaugeUp = false;
float battVolts = 0;
int   battPct   = -1;

/* Forward declarations. The Arduino IDE usually generates these automatically,
   but it can silently fail on some constructs -- and tgSend() is called from the
   command handler above where it is defined. Declaring them explicitly removes
   any dependence on that. */
bool tgSend(const char* text);
void trySyncClockFromGNSS();
bool otaUpdate(const char* url, const char* wantSha);
void nlog(const char* fmt, ...);
void cmdCell(); void cmdData(); void cmdHist(); void cmdLocate();
void histSample();
void otaProbe(const char* url);
String httpsGet(const char* url);
String httpsPost(const char* url, const char* jsonBody);
#if TG_ENABLE
  TinyGsmClientSecure tgClient(modem, 1);
#endif

HardwareSerial meshSerial(2);
Preferences    prefs;
bool           cellUp = false;
uint32_t       lastMeshStep = 0;   // last TOTP step already used (anti-replay), persisted

char topicStatus[64], topicCmd[64], topicEvent[64], topicLog[64], topicCell[64];

/* Mirror of Serial output, also published to MQTT so the app can show the log
   when the board is in the car with no laptop attached. */
/* Defined above the first function on purpose: Arduino auto-generates prototypes
   (e.g. for readCell(), which RETURNS a CellInfo) and inserts them above the
   first function definition. The type must exist before that point. */
struct CellInfo {
  char  rat[12], oper[12];
  long  tac, eci, pcid, earfcn;
  int   mcc, mnc;
  float rsrp, rsrq, sinr;
  int   state;           // 0=parse fail, 1=camped, 2=no service
};

void nlog(const char* fmt, ...) {
  char buf[200];
  va_list ap; va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.println(buf);
  if (mqtt.connected()) mqtt.publish(topicLog, buf);
}

/* ---------- battery gauge helpers ---------- */
static bool gaugeRead16(uint8_t reg, uint16_t* out) {
  Wire.beginTransmission(GAUGE_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)GAUGE_ADDR, 2) != 2) return false;
  uint8_t hi = Wire.read(), lo = Wire.read();
  *out = ((uint16_t)hi << 8) | lo;
  return true;
}

/* Li-ion 18650 discharge curve, open-circuit. Coarse but honest -- and far more
   trustworthy than the gauge's SOC was when we first read it. */
static int voltsToPct(float v) {
  static const float V[] = {3.00f,3.30f,3.50f,3.65f,3.75f,3.85f,3.95f,4.05f,4.15f,4.20f};
  static const int   P[] = {   0,    5,   15,   30,   45,   60,   75,   88,   97,  100};
  if (v <= V[0]) return 0;
  if (v >= V[9]) return 100;
  for (int i = 1; i < 10; i++)
    if (v < V[i]) {
      float f = (v - V[i-1]) / (V[i] - V[i-1]);
      return (int)(P[i-1] + f * (P[i] - P[i-1]) + 0.5f);
    }
  return 100;
}

void gaugeBegin() {
  Wire.begin(GAUGE_SDA, GAUGE_SCL, 50000);   // 50kHz: this bus has weak pull-ups
  delay(50);
  uint16_t v;
  gaugeRead16(0x02, &v);                     // discard first transaction
  delay(20);
  gaugeUp = gaugeRead16(0x02, &v);
  if (gaugeUp) {
    // QuickStart: rebuild the fuel model, since its SOC was clearly wrong
    Wire.beginTransmission(GAUGE_ADDR);
    Wire.write(0x06); Wire.write(0x40); Wire.write(0x00);
    Wire.endTransmission();
    nlog("[batt] gauge ready");
  } else {
    nlog("[batt] gauge NOT responding");
  }
}

void gaugeUpdate() {
  if (!gaugeUp) return;
  static uint32_t last = 0;
  if (last && (uint32_t)(millis() - last) < 30000) return;
  last = millis();
  uint16_t raw;
  if (!gaugeRead16(0x02, &raw)) return;
  battVolts = raw * 0.000078125f;
  battPct   = voltsToPct(battVolts);
}

/* ================= CELL DIAGNOSTICS + 24h HISTORY + GEOLOCATION =============
   All hidden in the Node log. Printed only when you send a command:
     cell   -> full serving-cell diagnostics with short explanations
     data   -> data consumed (session + lifetime)
     hist   -> dump the 24h location history (GPS + cell)
     locate -> cell-tower geolocation via OpenCellID (POST, token baked in), returns lat/lon
   Nothing here runs unless asked, except the history sampler (cheap, local). */


struct Fix {
  uint32_t ts;
  float    lat, lon;
  uint8_t  gpsFix;
  long     cellid;
  int16_t  rsrp10;       // dBm * 10
};
Fix      histBuf[HIST_SLOTS];
int      histHead = 0, histCount = 0;
uint32_t histLastMs = 0;

uint32_t dataRxTotal = 0, dataTxTotal = 0;   // lifetime, persisted
uint32_t dataRxSession = 0, dataTxSession = 0;

/* last known GPS fix, refreshed by the status publisher (which already calls
   getGPS every cycle). The history sampler reads these rather than hitting the
   modem a second time. 0/0 = no fix yet. */
float lastFixLat = 0, lastFixLon = 0;
bool  lastFixValid = false;

/* Deferred cell diagnostics. readCell()/cmdLocate() send raw AT commands on the
   modem UART. onMessage() runs INSIDE mqtt.loop(), i.e. while PubSubClient may be
   mid-packet on the same UART (via the TLS socket). Running AT there can corrupt
   both. So the MQTT handler only sets a flag; the work happens in loop() after
   mqtt.loop() has returned and the socket is idle. 0=none. */
volatile char pendingCellCmd = 0;   // 'c'=cell 'd'=data 'h'=hist 'l'=locate
volatile bool pendingOtaTest = false;
char          otaTestUrl[300] = "";

/* ---- raw AT: send a command, capture the reply text into buf ----
   TinyGSM has no public sendAT/waitResponse we can rely on across the fork, so
   we talk to the modem stream directly. Safe: reads are bounded and timed out. */
bool rawAT(const char* cmd, char* buf, size_t cap, uint32_t timeout = 1500) {
  while (modemSerial.available()) modemSerial.read();   // flush
  modemSerial.print(cmd);
  modemSerial.print("\r\n");
  size_t n = 0; uint32_t t0 = millis();
  buf[0] = 0;
  while (millis() - t0 < timeout) {
    while (modemSerial.available()) {
      char c = modemSerial.read();
      if (n < cap - 1) buf[n++] = c;
    }
    buf[n] = 0;
    if (strstr(buf, "OK\r\n") || strstr(buf, "ERROR")) return strstr(buf, "OK") != NULL;
    delay(2);
  }
  return n > 0;
}

static int splitCsv(char* s, char* f[], int max) {
  int n = 0; char* p = strtok(s, ",");
  while (p && n < max) { f[n++] = p; p = strtok(NULL, ","); }
  return n;
}

static const char* rsrpQuality(float dbm) {
  if (dbm >= -80)  return "excellent";
  if (dbm >= -90)  return "good";
  if (dbm >= -100) return "fair";
  if (dbm >= -110) return "poor";
  return "very poor / cell edge";
}

/* parse +CPSI? reply into CellInfo (LTE field layout per SIMCom manual) */
CellInfo readCell() {
  CellInfo c; memset(&c, 0, sizeof(c));
  char buf[256];
  if (!rawAT("AT+CPSI?", buf, sizeof(buf))) { c.state = 0; return c; }
  char* p = strstr(buf, "+CPSI:");
  if (!p) { c.state = 0; return c; }
  p += 6; while (*p == ' ') p++;
  char line[224]; strncpy(line, p, 223); line[223] = 0;
  char* nl = strpbrk(line, "\r\n"); if (nl) *nl = 0;
  if (strstr(line, "NO SERVICE")) { strcpy(c.rat, "NO SERVICE"); c.state = 2; return c; }
  char* f[20]; int n = splitCsv(line, f, 20);
  if (n < 3) { c.state = 0; return c; }
  strncpy(c.rat, f[0], 11);
  strncpy(c.oper, f[2], 11);
  char op[16]; strncpy(op, f[2], 15); op[15] = 0;
  char* dash = strchr(op, '-');
  if (dash) { *dash = 0; c.mcc = atoi(op); c.mnc = atoi(dash + 1); }
  if (!strcmp(c.rat, "LTE") && n >= 14) {
    c.tac    = strtol(f[3], NULL, 16);
    c.eci    = strtol(f[4], NULL, 10);
    c.pcid   = strtol(f[5], NULL, 10);
    c.earfcn = strtol(f[7], NULL, 10);
    c.rsrq   = atoi(f[10]) / 10.0f;
    c.rsrp   = atoi(f[11]) / 10.0f;
    c.sinr   = atoi(f[13]) / 10.0f;
  }
  c.state = 1;
  return c;
}

void cmdCell() {
  CellInfo c = readCell();
  if (c.state == 0) { nlog("[cell] could not read (modem busy?)"); return; }
  if (c.state == 2) { nlog("[cell] NO SERVICE - not camped on any cell"); return; }
  nlog("[cell] --- serving cell ---");
  nlog("[cell] network %d-%d  RAT %s", c.mcc, c.mnc, c.rat);
  if (!strcmp(c.rat, "LTE")) {
    nlog("[cell] eNodeB %ld  cell %ld  (ECI %ld)", c.eci >> 8, c.eci & 0xFF, c.eci);
    nlog("[cell] TAC %ld  PCI %ld  EARFCN %ld", c.tac, c.pcid, c.earfcn);
    nlog("[cell] RSRP %.1f dBm (%s) = signal strength", c.rsrp, rsrpQuality(c.rsrp));
    nlog("[cell] RSRQ %.1f dB = signal quality (>-10 good, <-15 poor)", c.rsrq);
    nlog("[cell] SINR %.1f dB = noise margin (>10 clean, <0 struggling)", c.sinr);
    long cid = c.eci & 0xFF;
    nlog("[cell] cell type: %s", cid == 0 ? "likely macro (outdoor, wide area)"
                                : cid < 8 ? "macro/sector (outdoor)"
                                          : "high cell id -> possibly micro/indoor BTS");
  }
  // registration + reject cause
  char rb[128];
  if (rawAT("AT+CEREG?", rb, sizeof(rb))) {
    char* q = strstr(rb, "+CEREG:");
    if (q) {
      int nrm = 0, st = 0;
      sscanf(q + 7, " %d,%d", &nrm, &st);
      const char* meaning = st == 1 ? "registered (home)"
                          : st == 5 ? "registered (roaming)"
                          : st == 2 ? "searching"
                          : st == 3 ? "DENIED by network (data plan? not provisioned?)"
                          : st == 0 ? "not searching"
                                    : "unknown";
      nlog("[cell] registration = %d (%s)", st, meaning);
    }
  }

  // structured copy for the app (parsed fields, not text)
  char j[300];
  int reg = 0;
  { char rb2[96];
    if (rawAT("AT+CEREG?", rb2, sizeof(rb2))) {
      char* q = strstr(rb2, "+CEREG:"); int nr=0;
      if (q) sscanf(q+7, " %d,%d", &nr, &reg);
    } }
  snprintf(j, sizeof(j),
    "{\"rat\":\"%s\",\"mcc\":%d,\"mnc\":%d,\"enb\":%ld,\"cid\":%ld,"
    "\"eci\":%ld,\"tac\":%ld,\"pci\":%ld,\"earfcn\":%ld,"
    "\"rsrp\":%.1f,\"rsrq\":%.1f,\"sinr\":%.1f,\"reg\":%d}",
    c.rat, c.mcc, c.mnc, c.eci>>8, c.eci&0xFF, c.eci, c.tac, c.pcid, c.earfcn,
    c.rsrp, c.rsrq, c.sinr, reg);
  if (mqtt.connected()) mqtt.publish(topicCell, j);
}

void cmdData() {
  // pull the modem's own byte counters; formats vary, so we also keep our own
  char buf[160];
  if (rawAT("AT+CGCONTRDP", buf, sizeof(buf))) { /* context info, ignored for counts */ }
  nlog("[data] session: %lu B down / %lu B up",
       (unsigned long)dataRxSession, (unsigned long)dataTxSession);
  nlog("[data] lifetime: %.2f MB down / %.2f MB up",
       dataRxTotal / 1048576.0, dataTxTotal / 1048576.0);
  nlog("[data] (at 10s status the node uses ~87 MB/month; 'parked' rate cuts that ~5x)");
}

void histPush(uint32_t ts, float lat, float lon, int fix, long cell, int rsrp10) {
  Fix* s = &histBuf[histHead];
  s->ts = ts; s->lat = lat; s->lon = lon; s->gpsFix = fix; s->cellid = cell; s->rsrp10 = rsrp10;
  histHead = (histHead + 1) % HIST_SLOTS;
  if (histCount < HIST_SLOTS) histCount++;
}

void histSample() {
  if (histLastMs && (uint32_t)(millis() - histLastMs) < HIST_EVERY_MS) return;
  histLastMs = millis();
  uint32_t ts = nowMs() / 1000;
  if (ts < 1700000000UL) return;                 // clock not set yet
  float lat = 0, lon = 0; int fix = 0;
  if (lastFixValid) { lat = lastFixLat; lon = lastFixLon; fix = 1; }
  CellInfo c = readCell();
  long cell = (c.state == 1) ? c.eci : 0;
  int  r10  = (c.state == 1) ? (int)(c.rsrp * 10) : 0;
  histPush(ts, lat, lon, fix, cell, r10);
}

void cmdHist() {
  if (histCount == 0) { nlog("[hist] empty (fills one sample per 10 min)"); return; }
  int start = (histHead - histCount + HIST_SLOTS) % HIST_SLOTS;
  int gps = 0, cellOnly = 0;
  nlog("[hist] --- last %d samples (oldest first) ---", histCount);
  for (int i = 0; i < histCount; i++) {
    Fix* s = &histBuf[(start + i) % HIST_SLOTS];
    if (s->gpsFix) {
      gps++;
      nlog("[hist] %lu  %.5f,%.5f  cell=%ld %.0fdBm",
           (unsigned long)s->ts, s->lat, s->lon, s->cellid, s->rsrp10 / 10.0);
    } else {
      cellOnly++;
      nlog("[hist] %lu  (no GPS)  cell=%ld %.0fdBm",
           (unsigned long)s->ts, s->cellid, s->rsrp10 / 10.0);
    }
  }
  nlog("[hist] %d GPS fixes, %d cell-only over the window", gps, cellOnly);
}

/* Minimal HTTPS POST over the mux-1 TLS socket (the one Telegram uses).
   Parses "host" and "path" out of an https:// URL, sends a JSON body, returns
   just the response body (headers stripped). Pauses MQTT to free the socket,
   exactly like tgSend does, then lets the main loop reconnect. */
String httpsGet(const char* url) {
  const char* h = strstr(url, "://"); if (!h) return String(); h += 3;
  const char* slash = strchr(h, '/');
  char host[128]; int hlen = slash ? (int)(slash - h) : (int)strlen(h);
  if (hlen <= 0 || hlen >= (int)sizeof(host)) return String();
  memcpy(host, h, hlen); host[hlen] = 0;
  const char* path = slash ? slash : "/";
#if TG_PAUSE_MQTT
  bool hadMqtt = mqtt.connected();
  if (hadMqtt) { mqtt.disconnect(); netClient.stop(); delay(100); }
#endif
  if (!tgClient.connect(host, 443)) { nlog("[https] TLS connect failed to %s", host); return String(); }
  tgClient.print(String("GET ") + path + " HTTP/1.1\r\nHost: " + host +
                 "\r\nUser-Agent: pajero\r\nConnection: close\r\n\r\n");
  String resp; resp.reserve(400); uint32_t t0 = millis();
  while (millis() - t0 < 15000) {
    while (tgClient.available()) { char ch = tgClient.read(); if (resp.length() < 800) resp += ch; t0 = millis(); }
    if (!tgClient.connected() && !tgClient.available()) break;
    delay(10);
  }
  tgClient.stop();
  int b = resp.indexOf("\r\n\r\n");
  return b >= 0 ? resp.substring(b + 4) : resp;
}

String httpsPost(const char* url, const char* jsonBody) {
  const char* h = strstr(url, "://");
  if (!h) return String();
  h += 3;
  const char* slash = strchr(h, '/');
  char host[96];
  int hlen = slash ? (int)(slash - h) : (int)strlen(h);
  if (hlen <= 0 || hlen >= (int)sizeof(host)) return String();
  memcpy(host, h, hlen); host[hlen] = 0;
  const char* path = slash ? slash : "/";
#if TG_PAUSE_MQTT
  bool hadMqtt = mqtt.connected();
  if (hadMqtt) { mqtt.disconnect(); netClient.stop(); delay(100); }
#endif
  if (!tgClient.connect(host, 443)) { nlog("[https] TLS connect failed to %s", host); return String(); }
  tgClient.print("POST ");
  tgClient.print(path);
  tgClient.print(" HTTP/1.1\r\nHost: ");
  tgClient.print(host);
  tgClient.print("\r\nContent-Type: application/json\r\nContent-Length: ");
  tgClient.print(strlen(jsonBody));
  tgClient.print("\r\nConnection: close\r\n\r\n");
  tgClient.print(jsonBody);
  
  String resp;
  resp.reserve(700);
  uint32_t t0 = millis();
  while (millis() - t0 < 15000) {
    while (tgClient.available()) { char c = tgClient.read(); if (resp.length() < 1200) resp += c; t0 = millis(); }
    if (!tgClient.connected() && !tgClient.available()) break;
    delay(10);
  }
  tgClient.stop();
  int b = resp.indexOf("\r\n\r\n");
  return b >= 0 ? resp.substring(b + 4) : resp;
}



/* cell-tower geolocation via OpenCellID (free, no billing).
   GET /cell/get with mcc,mnc,lac(=TAC),cellid(=ECI) in DECIMAL. Returns lat/lon
   and a range estimate, or 0,0 / stat:err when the cell isn't in the database. */
void cmdLocate() {
  CellInfo c = readCell();
  if (!cellUp)                              { nlog("[locate] needs data - modem offline right now"); return; }
  if (c.state != 1 || strcmp(c.rat, "LTE")) { nlog("[locate] no LTE serving cell to look up"); return; }

  // Current working endpoint: OpenCellID's own AJAX GET. No token needed, returns
  // {"lon":"..","lat":"..","range":".."} with values as QUOTED strings.
  char url[224];
  snprintf(url, sizeof(url),
    "https://opencellid.org/ajax/searchCell.php?mcc=%d&mnc=%d&lac=%ld&cellid=%ld",
    c.mcc, c.mnc, c.tac, c.eci);

  nlog("[locate] asking OpenCellID about cell %ld (TAC %ld) ...", c.eci, c.tac);
  String resp = httpsGet(url);
  if (resp.length() == 0) { nlog("[locate] no response from server"); return; }

  // "false" or missing = cell not in database
  if (resp.indexOf("false") >= 0 && resp.indexOf("lat") < 0) {
    nlog("[locate] cell not in the OpenCellID database (common outside towns)");
    return;
  }
  int li = resp.indexOf("\"lat\"");
  int oi = resp.indexOf("\"lon\"");
  if (li < 0 || oi < 0) { nlog("[locate] no location in reply"); return; }
  // values are strings: skip ':' then a quote
  int lc = resp.indexOf(':', li); int oc = resp.indexOf(':', oi);
  double lat = atof(resp.c_str() + lc + 2);   // +2 skips :"
  double lon = atof(resp.c_str() + oc + 2);
  if (lat == 0.0 && lon == 0.0) { nlog("[locate] cell not mapped (0,0)"); return; }
  int ri = resp.indexOf("\"range\"");
  double range = ri >= 0 ? atof(resp.c_str() + resp.indexOf(':', ri) + 2) : -1;

  nlog("[locate] tower at %.5f, %.5f  (coverage radius ~%.0f m)", lat, lon, range);

  // Direction + distance from the car's last GPS fix to the tower.
  if (lastFixValid) {
    double toR = 3.14159265358979 / 180.0;
    double y = sin((lon - lastFixLon) * toR) * cos(lat * toR);
    double x = cos(lastFixLat * toR) * sin(lat * toR) -
               sin(lastFixLat * toR) * cos(lat * toR) * cos((lon - lastFixLon) * toR);
    double brg = atan2(y, x) / toR; if (brg < 0) brg += 360;
    double dlat = (lat - lastFixLat) * toR, dlon = (lon - lastFixLon) * toR;
    double a = sin(dlat/2)*sin(dlat/2) + cos(lastFixLat*toR)*cos(lat*toR)*sin(dlon/2)*sin(dlon/2);
    double dist = 6371000.0 * 2 * atan2(sqrt(a), sqrt(1-a));
    const char* dirs[] = {"N","NE","E","SE","S","SW","W","NW"};
    const char* dir = dirs[(int)((brg + 22.5) / 45.0) % 8];
    nlog("[locate] tower is %.0f m to your %s (bearing %.0f deg) from last GPS fix", dist, dir, brg);
  } else {
    nlog("[locate] (no GPS fix yet, so no heading - tower coords above are absolute)");
  }
  nlog("[locate] https://maps.google.com/?q=%.5f,%.5f", lat, lon);

  // structured payload for the app map: tower point + coverage radius + car heading
  char jl[220];
  if (lastFixValid) {
    snprintf(jl, sizeof(jl),
      "{\"kind\":\"celloc\",\"lat\":%.6f,\"lon\":%.6f,\"range\":%.0f,\"clat\":%.6f,\"clon\":%.6f}",
      lat, lon, range, lastFixLat, lastFixLon);
  } else {
    snprintf(jl, sizeof(jl),
      "{\"kind\":\"celloc\",\"lat\":%.6f,\"lon\":%.6f,\"range\":%.0f}", lat, lon, range);
  }
  if (mqtt.connected()) mqtt.publish(topicCell, jl);
}

/* ---------- non-blocking pulse engine ----------
   A 10-second delay() would stall the MQTT keepalive and drop the link.
   Each output runs on its own timer instead. */
struct Pulse {
  uint8_t  pin;
  uint32_t endsAt = 0;
  bool     active = false;
  void begin(uint8_t p){ pin = p; pinMode(pin, OUTPUT); digitalWrite(pin, OUT_OFF); }
  void fire(uint32_t ms){ digitalWrite(pin, OUT_ON); endsAt = millis() + ms; active = true; }
  void tick(){ if (active && (int32_t)(millis() - endsAt) >= 0) { digitalWrite(pin, OUT_OFF); active = false; } }
  uint32_t remaining(){ return active ? (endsAt - millis()) : 0; }
};
Pulse pUnlock, pLock, pBlackbox, pGate;

/* ---------- replay / stale-command guard ----------
   Two real problems this solves:
   1. You tap Unlock while the car is offline. The broker holds it. Three
      hours later the car wakes in a car park and unlocks itself. Bad.
   2. Someone replays a captured packet.
   Commands older than CMD_MAX_AGE_MS are dropped, and each ts must be newer
   than the last one accepted. Needs the modem clock, which we sync from the
   network below. */
uint64_t lastAcceptedTs = 0;

/* ---------- geofence + movement ----------
   Home point is stored in NVS so it survives reboots. Set it from the app with
   {"cmd":"sethome"}          -> uses the car's CURRENT GPS position, or
   {"cmd":"setgeo","lat":..,"lon":..,"radius":120,"auto":true}
   The gate fires automatically when the car ARRIVES inside the fence.          */
double   geoLat = 0, geoLon = 0;      // 0,0 = geofence not configured
uint16_t geoRadiusM  = 120;           // metres
bool     geoGateAuto = false;         // auto-open the gate on arrival
#define  GEO_HYST_M        40         // must travel this far past the edge to flip
#define  GATE_COOLDOWN_MS  120000UL   // don't re-fire the gate within 2 min

bool     geoInside   = false;         // current state
bool     geoKnown    = false;         // have we established a state yet?
uint32_t lastGateAuto = 0;

#define  DRIVE_START_KMH   8          // above this = driving
#define  DRIVE_STOP_KMH    3
#define  DRIVE_STOP_MS     60000UL    // slow for this long = trip ended
bool     driving = false;
uint32_t slowSince = 0;

/* metres between two lat/lon points (equirectangular — plenty accurate <10 km) */
double distanceM(double la1, double lo1, double la2, double lo2) {
  double mid = radians((la1 + la2) * 0.5);
  double dx  = radians(lo2 - lo1) * cos(mid) * 6371000.0;
  double dy  = radians(la2 - la1) * 6371000.0;
  return sqrt(dx * dx + dy * dy);
}

/* Pure reader: returns the current epoch-ms, or 0 if the clock isn't set yet.
   Deliberately does NOT set the clock -- a function used inside comparisons must
   not have side effects, and this modem's network time can't be trusted blindly. */
/* Convert a UTC calendar date/time to epoch seconds without touching mktime() or
   timegm(). mktime() interprets its input as LOCAL time (silently shifting the
   result) and timegm() isn't available in this toolchain, so we do it directly. */
static int64_t utcToEpoch(int y, int mo, int d, int h, int mi, int s) {
  // days-from-civil: valid for any Gregorian date, no library or TZ involvement
  y -= (mo <= 2);
  const int64_t era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);                       // [0,399]
  const unsigned doy = (153 * (mo + (mo > 2 ? -3 : 9)) + 2) / 5 + d - 1;// [0,365]
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;           // [0,146096]
  const int64_t days = era * 146097 + (int64_t)doe - 719468;
  return days * 86400LL + h * 3600LL + mi * 60LL + s;
}

uint64_t nowMs() {
  time_t t = time(nullptr);
  return (t > 1700000000) ? (uint64_t)t * 1000ULL : 0ULL;
}

/* MEASURED ON THIS HARDWARE (see log): the modem reported a time exactly 45 min
   (2700 s) off. tz*900 with the modem's tz=3 is precisely 2700 s -- i.e. the time
   it returns is ALREADY UTC, so applying the offset at all is wrong. Rather than
   guess this modem's tz semantics, network time is off by default: the phone sets
   the clock on the first command and is always right.

   Set USE_NETWORK_TIME to 1 only if you need a clock with no phone contact at all
   (long off-grid LoRa use). If you do, drop the tz term -- do NOT subtract it. */
#define USE_NETWORK_TIME 0

void trySyncClockFromNetwork() {
#if !USE_NETWORK_TIME
  return;                                   // phone is the only trusted clock
#else
  if (nowMs() != 0) return;                 // already have a clock; phone owns it
  int y, mo, d, h, mi, s; float tz;
  if (!modem.getNetworkTime(&y, &mo, &d, &h, &mi, &s, &tz)) return;
  // NOTE: mktime() interprets the struct as LOCAL time, which silently shifts the
  // result if TZ isn't UTC -- a documented ESP32 footgun and the likely source of
  // the odd partial-hour offsets we saw. Convert to epoch directly instead.
  (void)tz;                                 // deliberately NOT applied -- see note above
  time_t e = (time_t)utcToEpoch(y, mo, d, h, mi, s);
  // reject nonsense: before 2024-01-01 or after 2035-01-01
  if (e < 1704067200L || e > 2051222400L) {
    nlog("[time] modem clock rejected (%ld)", (long)e);
    return;
  }
  struct timeval tv = { .tv_sec = e, .tv_usec = 0 };
  settimeofday(&tv, nullptr);
  nlog("[time] clock set from network");
#endif
}

/* ---------- GNSS clock: the right time source for this build ----------

   Why GNSS and not the alternatives:
     - It is UTC by definition. No timezone field, no quarter-hour ambiguity,
       none of the modem network-time nonsense that put us exactly 45 min out.
     - It works with NO cellular at all -- which is precisely the off-grid case
       where the LoRa unlock needs a valid clock. Without this, a node that
       reboots in the desert has no clock, and meshTryUnlock() refuses every
       token: the fallback path would be dead exactly when it's needed.
     - It self-corrects drift, so a long parked period can't break the TOTP.

   Authority order:  GNSS  >  phone (via MQTT command)  >  network time (off).

   IF THIS WON'T COMPILE with "too many arguments to function ... getGPS":
   this fork's getGPS() doesn't take the trailing date/time pointers. Change
   GNSS_TIME_FROM_RAW to 1 below -- that path parses the raw GNSS sentence
   instead and needs nothing from the library beyond getGPSraw(). */

#define GNSS_TIME_FROM_RAW  0    // 0 = getGPS() with date/time args, 1 = parse raw

void trySyncClockFromGNSS() {
  static uint32_t last = 0;
  // hunt hard while we have no clock at all, then just trim drift occasionally
  uint32_t iv = (nowMs() == 0) ? 15000UL : 600000UL;
  if (last != 0 && (uint32_t)(millis() - last) < iv) return;
  last = millis();

  int y = 0, mo = 0, d = 0, h = 0, mi = 0, s = 0;

#if GNSS_TIME_FROM_RAW
  /* Raw +CGNSSINFO fields (comma separated):
       0 fixmode, 1..3 sat counts, 4 lat, 5 N/S, 6 lon, 7 E/W,
       8 date ddmmyy, 9 UTC time hhmmss.s, ...                              */
  String raw = modem.getGPSraw();
  if (raw.length() < 20) return;
  char buf[160];
  strncpy(buf, raw.c_str(), sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;

  char* field[16] = {0};
  int   nf = 0;
  for (char* p = strtok(buf, ","); p && nf < 16; p = strtok(NULL, ",")) field[nf++] = p;
  if (nf < 10 || !field[8] || !field[9]) return;
  if (strlen(field[8]) < 6 || strlen(field[9]) < 6) return;

  d  = (field[8][0]-'0')*10 + (field[8][1]-'0');
  mo = (field[8][2]-'0')*10 + (field[8][3]-'0');
  y  = 2000 + (field[8][4]-'0')*10 + (field[8][5]-'0');
  h  = (field[9][0]-'0')*10 + (field[9][1]-'0');
  mi = (field[9][2]-'0')*10 + (field[9][3]-'0');
  s  = (field[9][4]-'0')*10 + (field[9][5]-'0');
#else
  float lat = 0, lon = 0, sp = 0, al = 0, ac = 0;
  int   vs = 0, us = 0;
  uint8_t st = 0;
  if (!modem.getGPS(&st, &lat, &lon, &sp, &al, &vs, &us, &ac,
                    &y, &mo, &d, &h, &mi, &s)) return;
#endif

  // sanity: a fix with no date yet reports zeros
  if (y < 2024 || y > 2035 || mo < 1 || mo > 12 || d < 1 || d > 31) return;

  int64_t gnss = utcToEpoch(y, mo, d, h, mi, s);
  int64_t cur  = (int64_t)(nowMs() / 1000ULL);
  int64_t diff = cur - gnss;
  if (diff < 0) diff = -diff;
  if (cur != 0 && diff < 30) return;          // already accurate enough

  struct timeval tv = { .tv_sec = (time_t)gnss, .tv_usec = 0 };
  settimeofday(&tv, nullptr);
  if (cur == 0) nlog("[time] clock set from GNSS (was unset)");
  else          nlog("[time] clock corrected from GNSS (drift %lld s)", (long long)diff);
}

/* ---------- command handler ---------- */
void onMessage(char* topic, byte* payload, unsigned int len) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, len)) { Serial.println("[cmd] bad json"); return; }

  const char* cmd = doc["cmd"]  | "";
  const char* sec = doc["secret"] | "";
  uint64_t    ts  = doc["ts"]   | 0ULL;

  if (strlen(SECRET) == 0 || strcmp(sec, SECRET) != 0) {
    Serial.println("[cmd] REJECTED — bad secret"); return;
  }

  // ---- clock handling ----
  // The phone is the ONLY clock we trust. The ESP32 RTC drifts and this modem's
  // network time has proved unreliable, so we never reject on clock disagreement
  // -- we re-sync to the phone and carry on. Replay protection does not depend on
  // this: it comes from the strictly increasing timestamp check below, which a
  // captured old command cannot pass.
  if (ts <= 1700000000000ULL) {           // not a plausible epoch-ms from the app
    nlog("[cmd] REJECTED - implausible timestamp"); return;
  }
  {
    uint64_t now  = nowMs();              // read ONCE
    int64_t  skew = (now == 0) ? 0 : (int64_t)now - (int64_t)ts;
    if (now == 0 || skew > 30000 || skew < -30000) {
      struct timeval tv = { .tv_sec = (time_t)(ts / 1000ULL), .tv_usec = 0 };
      settimeofday(&tv, nullptr);
      nlog("[time] clock synced from phone (was off %lld ms)", skew);
    }
  }

  if (ts <= lastAcceptedTs) { nlog("[cmd] REJECTED - replay (ts not newer)"); return; }
  lastAcceptedTs = ts;
  nlog("[ack] %s", cmd);          // app listens for this to confirm the button worked
  { char a[96]; snprintf(a,sizeof(a),"{\"ack\":\"%s\",\"ts\":%llu}", cmd, (unsigned long long)ts);
    mqtt.publish(topicLog, a); }

  if      (!strcmp(cmd, "unlock"))   { pUnlock.fire(PULSE_KEYFOB_MS);      nlog("[cmd] UNLOCK 200ms"); }
  else if (!strcmp(cmd, "lock"))     { pLock.fire(PULSE_KEYFOB_MS);        nlog("[cmd] LOCK 200ms"); }
  else if (!strcmp(cmd, "blackbox")) { pBlackbox.fire(PULSE_BLACKBOX_MS);  nlog("[cmd] BLACKBOX relay 10s"); }
  else if (!strcmp(cmd, "gate"))     { pGate.fire(PULSE_GATE_MS);          nlog("[cmd] GATE 400ms"); }
  else if (!strcmp(cmd, "sethome")) {
    // Park where you want the fence centred, then send this — uses current GPS.
    float la=0, lo=0, sp=0, al=0, ac=0; int vs=0, us=0; uint8_t st=0;
    if (modem.getGPS(&st,&la,&lo,&sp,&al,&vs,&us,&ac) && st && la != 0) {
      geoLat = la; geoLon = lo;
      if (doc["radius"].is<uint16_t>()) geoRadiusM = doc["radius"];
      prefs.putDouble("glat", geoLat); prefs.putDouble("glon", geoLon);
      prefs.putUShort("grad", geoRadiusM);
      geoKnown = false;                       // re-evaluate on next fix
      Serial.printf("[geo] home set %.6f,%.6f r=%um\n", geoLat, geoLon, geoRadiusM);
      publishEvent("home_set", geoLat, geoLon, -1);
    } else nlog("[geo] sethome FAILED - no GPS fix");
  }
  else if (!strcmp(cmd, "setgeo")) {
    geoLat = doc["lat"] | geoLat;
    geoLon = doc["lon"] | geoLon;
    if (doc["radius"].is<uint16_t>()) geoRadiusM = doc["radius"];
    if (doc["auto"].is<bool>())       geoGateAuto = doc["auto"];
    prefs.putDouble("glat", geoLat); prefs.putDouble("glon", geoLon);
    prefs.putUShort("grad", geoRadiusM); prefs.putBool("gauto", geoGateAuto);
    geoKnown = false;
    Serial.printf("[geo] set %.6f,%.6f r=%um auto=%d\n",
                  geoLat, geoLon, geoRadiusM, geoGateAuto);
    publishEvent("geo_set", geoLat, geoLon, -1);
  }
  else if (!strcmp(cmd, "gateauto")) {
    geoGateAuto = doc["on"] | false;
    prefs.putBool("gauto", geoGateAuto);
    Serial.printf("[geo] gate auto-open = %s\n", geoGateAuto ? "ON" : "OFF");
  }
  else if (!strcmp(cmd, "clearhome")) {
    geoLat = 0; geoLon = 0; geoGateAuto = false; geoKnown = false;
    prefs.putDouble("glat", 0); prefs.putDouble("glon", 0); prefs.putBool("gauto", false);
    Serial.println("[geo] geofence cleared");
  }
  else if (!strcmp(cmd, "tgtest")) {
    Serial.println("[tg] test requested");
    tgSend("Pajero node: Telegram test OK");
  }
  else if (!strcmp(cmd, "ver"))    { nlog("[ver] fw %s  built %s", FW_VERSION, FW_BUILD); }
  else if (!strcmp(cmd, "otatest")) {
    // probe an OTA URL WITHOUT flashing: reports connect + HTTP status + size.
    const char* u = doc["url"] | "";
    if (!u[0]) { nlog("[otatest] give a url field"); }
    else       { pendingOtaTest = true; snprintf(otaTestUrl, sizeof(otaTestUrl), "%s", u); }
  }
  else if (!strcmp(cmd, "cell"))   { pendingCellCmd = 'c'; }   // deferred: see note at flag
  else if (!strcmp(cmd, "data"))   { pendingCellCmd = 'd'; }
  else if (!strcmp(cmd, "hist"))   { pendingCellCmd = 'h'; }
  else if (!strcmp(cmd, "locate")) { pendingCellCmd = 'l'; }
  else if (!strcmp(cmd, "ota")) {
    const char* u = doc["url"]    | "";
    const char* h = doc["sha256"] | "";
    if (!u[0]) nlog("[ota] no url given");
    else       otaUpdate(u, h);
  }
#if CAN_ENABLE
  else if (!strcmp(cmd, "obd"))  { obdReport("requested"); }
  else if (!strcmp(cmd, "dtc"))  {
    char d[120]; uint8_t n = readDTCs(d, sizeof(d));
    char m[160]; snprintf(m, sizeof(m), n ? "Fault codes: %s" : "No fault codes stored", d);
    nlog("[can] %s", m); tgSend(m);
  }
  else if (!strcmp(cmd, "canscan")) { canDiscover(); }
#endif
  else Serial.printf("[cmd] unknown: %s\n", cmd);

  publishStatus();                       // echo state back immediately
}

/* percent-encode for a URL query value (handles UTF-8 byte-by-byte) */
static void urlEncode(const char* in, char* out, size_t outSz) {
  static const char* hex = "0123456789ABCDEF";
  size_t o = 0;
  for (const unsigned char* p = (const unsigned char*)in; *p && o + 4 < outSz; p++) {
    if (isalnum(*p) || *p=='-' || *p=='_' || *p=='.' || *p=='~') out[o++] = *p;
    else { out[o++]='%'; out[o++]=hex[*p >> 4]; out[o++]=hex[*p & 0x0F]; }
  }
  out[o] = 0;
}

/* Send one Telegram message straight from the modem -- no server in the middle.

   Debugging aids, because "nothing arrived" has many causes:
     - every stage is logged (TLS connect, request, HTTP status, response body)
     - Telegram's own error JSON is printed, which names the real problem
       (401 = bad token, 400 "chat not found" = wrong chat id or you never
        pressed Start in the bot chat, 403 = bot blocked)
     - the result is mirrored to the app's Node log, so this is debuggable in
       the car with no laptop attached

   TG_PAUSE_MQTT exists because MQTT holds a TLS socket on mux 0 and this opens a
   second on mux 1. If the modem can't run both, MQTT drops with state=-2. With
   the pause enabled we close MQTT, send, then let loop() reconnect -- costs a
   couple of seconds, and events are rare. */
bool tgSend(const char* text) {
#if !TG_ENABLE
  Serial.println("[tg] disabled (TG_ENABLE is 0)");
  return false;
#else
  if (!cellUp) { nlog("[tg] skipped - no network"); return false; }

  char enc[420];
  urlEncode(text, enc, sizeof(enc));

  bool ok = false;

#if TG_PAUSE_MQTT
  bool hadMqtt = mqtt.connected();
  if (hadMqtt) {
    Serial.println("[tg] pausing MQTT (freeing the TLS socket)");
    mqtt.disconnect();
    netClient.stop();
    delay(100);
  }
#endif

  Serial.printf("[tg] connecting %s:443 …\n", TG_HOST);
  if (!tgClient.connect(TG_HOST, 443)) {
    Serial.println("[tg] TLS CONNECT FAILED");
    snprintf(tgPending, sizeof(tgPending),
             "[tg] FAILED - TLS connect refused (modem may not allow 2 TLS sockets)");
  } else {
    Serial.println("[tg] TLS connected, sending request");

    String req = String("GET /bot") + TG_TOKEN + "/sendMessage?chat_id=" + TG_CHAT +
                 "&disable_web_page_preview=true&text=" + enc + " HTTP/1.1\r\n" +
                 "Host: " + TG_HOST + "\r\n" +
                 "Connection: close\r\n\r\n";
    tgClient.print(req);
    Serial.printf("[tg] request sent (%u bytes)\n", (unsigned)req.length());

    // read the whole reply: headers tell us the HTTP code, body tells us why
    String resp;
    resp.reserve(600);
    uint32_t t0 = millis();
    while (millis() - t0 < 15000) {
      while (tgClient.available()) {
        char c = tgClient.read();
        if (resp.length() < 600) resp += c;
        t0 = millis();                       // reset idle timer while data flows
      }
      if (!tgClient.connected() && !tgClient.available()) break;
      delay(10);
    }
    tgClient.stop();

    if (resp.length() == 0) {
      Serial.println("[tg] NO RESPONSE (TLS opened but nothing came back)");
      snprintf(tgPending, sizeof(tgPending), "[tg] FAILED - no HTTP response");
    } else {
      int nl = resp.indexOf('\n');
      String status = resp.substring(0, nl > 0 ? nl : 40);
      status.trim();
      ok = (resp.indexOf("\"ok\":true") >= 0);

      Serial.println("[tg] --- response ---");
      Serial.println(resp);
      Serial.println("[tg] -----------------");

      if (ok) {
        snprintf(tgPending, sizeof(tgPending), "[tg] delivered");
      } else {
        // pull Telegram's own description, which names the actual fault
        int dp = resp.indexOf("\"description\":\"");
        String why = (dp >= 0)
                       ? resp.substring(dp + 15, resp.indexOf('"', dp + 15))
                       : status;
        snprintf(tgPending, sizeof(tgPending), "[tg] REJECTED - %s", why.c_str());
      }
    }
  }

#if TG_PAUSE_MQTT
  if (hadMqtt) Serial.println("[tg] MQTT will reconnect on the next loop");
#endif

  Serial.println(tgPending);
  return ok;
#endif
}

/* ---------- events (these are what the Telegram bot listens for) ---------- */
void publishEvent(const char* type, double lat, double lon, float spd) {
  StaticJsonDocument<256> e;
  e["ev"]  = type;
  e["ts"]  = nowMs();
  if (lat != 0 || lon != 0) { e["lat"] = lat; e["lon"] = lon; }
  if (spd >= 0) e["spd"] = spd;
  char buf[256];
  size_t n = serializeJson(e, buf);
  bool ok = mqtt.publish(topicEvent, (const uint8_t*)buf, n, false);
  Serial.printf("[event] %s %s\n", type, ok ? "sent" : "FAILED");

  /* --- and straight to Telegram, no server involved --- */
  const char* nice = type;
  if      (!strcmp(type, "drive_start"))    nice = "Pajero started moving";
  else if (!strcmp(type, "drive_stop"))     nice = "Pajero stopped";
  else if (!strcmp(type, "geofence_exit"))  nice = "LEFT the home area";
  else if (!strcmp(type, "geofence_enter")) nice = "Arrived home";
  else if (!strcmp(type, "gate_auto"))      nice = "Gate opened on arrival";
  else if (!strcmp(type, "home_set"))       nice = "Geofence centre set";
  else if (!strcmp(type, "geo_set"))        nice = "Geofence updated";
  else if (!strcmp(type, "boot"))           nice = "Node powered up";

  char msg[380];
  if (lat != 0 || lon != 0)
    snprintf(msg, sizeof(msg),
             "%s\nhttps://maps.google.com/?q=%.6f,%.6f", nice, lat, lon);
  else
    snprintf(msg, sizeof(msg), "%s", nice);
  tgSend(msg);
}

/* Called once per status cycle with a fresh fix. Handles geofence crossings,
   automatic gate opening on arrival, and drive start/stop detection. */
void updateGeoMotion(double lat, double lon, float spdKmh) {
  /* --- geofence --- */
  if (geoLat != 0 || geoLon != 0) {
    double dist = distanceM(lat, lon, geoLat, geoLon);
    bool   in   = geoInside ? (dist < geoRadiusM + GEO_HYST_M)   // hysteresis stops
                            : (dist < geoRadiusM);               // edge flapping

    if (!geoKnown) { geoInside = in; geoKnown = true; }          // first fix: no event
    else if (in != geoInside) {
      geoInside = in;
      publishEvent(in ? "geofence_enter" : "geofence_exit", lat, lon, spdKmh);

      if (in && geoGateAuto &&
          (uint32_t)(millis() - lastGateAuto) > GATE_COOLDOWN_MS) {
        lastGateAuto = millis();
        pGate.fire(PULSE_GATE_MS);
        publishEvent("gate_auto", lat, lon, -1);
        Serial.println("[geo] arrived home -> GATE fired");
      }
    }
  }

  /* --- drive start / stop --- */
  if (!driving && spdKmh >= DRIVE_START_KMH) {
    driving = true; slowSince = 0;
    publishEvent("drive_start", lat, lon, spdKmh);
  } else if (driving) {
    if (spdKmh <= DRIVE_STOP_KMH) {
      if (slowSince == 0) slowSince = millis();
      else if ((uint32_t)(millis() - slowSince) > DRIVE_STOP_MS) {
        driving = false; slowSince = 0;
        publishEvent("drive_stop", lat, lon, spdKmh);
      }
    } else slowSince = 0;
  }
}

/* ---------- status ---------- */
void publishStatus() {
  StaticJsonDocument<384> d;

  float lat = 0, lon = 0, spd = 0, alt = 0, acc = 0;
  int   vsat = 0, usat = 0;
  uint8_t fixStatus = 0;   // lewisxhe fork: getGPS() takes a status byte first
  if (modem.getGPS(&fixStatus, &lat, &lon, &spd, &alt, &vsat, &usat, &acc) && fixStatus) {
    d["lat"] = lat; d["lon"] = lon; d["fix"] = "3D"; d["sats"] = usat; d["spd"] = spd;
    lastFixLat = lat; lastFixLon = lon; lastFixValid = true;   // for history sampler
    updateGeoMotion(lat, lon, spd);
  } else {
    d["fix"] = "NO FIX";
  }

  d["csq"] = modem.getSignalQuality();          // 0..31, 99 = unknown
  d["op"]  = modem.getOperator();
  d["net"] = "LTE";
  d["bb"]  = pBlackbox.active;
  d["gate"]= pGate.active;                  // true while the 10s press runs
  d["bbLeft"] = pBlackbox.remaining();
  d["home"]  = (geoLat != 0 || geoLon != 0);   // geofence configured?
  d["inside"]= geoInside;
  d["drive"] = driving;
  d["gauto"] = geoGateAuto;
  if (gaugeUp && battVolts > 0) { d["bv"] = battVolts; d["bat"] = battPct; }
  d["fw"] = FW_VERSION;
  d["ts"]  = nowMs();

  char buf[384];
  size_t n = serializeJson(d, buf);
  bool ok = mqtt.publish(topicStatus, (const uint8_t*)buf, n, false);
  Serial.printf("[pub] %s (%u B) -> %s | mqtt.state=%d\n",
                ok ? "OK" : "FAIL", (unsigned)n, topicStatus, mqtt.state());
  if (n + 2 > 256) Serial.println("[pub] WARNING payload larger than default buffer");
}

/* Node battery reporting is deliberately absent. The board carries a MAX17048
   fuel gauge on I2C, but it is not wired and the I2C pins are not confirmed for
   this unit, so there is no real value to publish. Showing a placeholder is
   worse than showing nothing. When the gauge is wired, add it back here and to
   the app together.
   (The CAR's battery voltage IS real and comes from OBD PID 0x42 -- see the
   obd report, field "volt".) */

/* ---------- connection ----------
   Split in two: modemInit() runs ONCE (restart + SIM + GPS on, so GNSS/time work
   even with no cellular). netAttach() brings up data and is safe to call whenever
   the network is present -- it never blocks for minutes, so the LoRa unlock in
   loop() stays responsive while off-grid. */
void modemInit() {
  Serial.println("[net] modem init…");
  // AT already works, so init() (no reboot) is faster and won't stall the way
  // restart() can while it waits for a full modem power-cycle.
  if (modem.init()) {
    Serial.println("[net] modem ready");
  } else {
    Serial.println("[net] init failed; testing UART…");
    if (!modem.testAT(3000))
      Serial.println("[net] modem NOT answering on GPIO17/18 — check 4G DIP=ON + antenna");
  }
  if (strlen(SIM_PIN) && modem.getSimStatus() != 3) modem.simUnlock(SIM_PIN);
  modem.enableGPS();                     // GNSS on regardless of cellular
}

void netAttach() {
  if (!modem.isGprsConnected()) {
    Serial.printf("[net] GPRS connect APN=%s… ", APN);
    Serial.println(modem.gprsConnect(APN, GPRS_USER, GPRS_PASS) ? "ok" : "fail");
  }
}

void connectMqtt() {
  static uint32_t lastTry = 0;
  if (mqtt.connected() || millis() - lastTry < 5000) return;
  lastTry = millis();
  String cid = String("pajero-") + DEVICE_ID + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  bool ok = (strlen(MQTT_USER))
              ? mqtt.connect(cid.c_str(), MQTT_USER, MQTT_PASS)
              : mqtt.connect(cid.c_str());
  if (ok) { Serial.println("[mqtt] connected"); mqtt.subscribe(topicCmd, 1); publishStatus(); }
  else    { Serial.printf("[mqtt] failed rc=%d\n", mqtt.state()); }
}


/* ================== OVER-THE-AIR FIRMWARE UPDATE =======================

   Triggered from the app:
       {"cmd":"ota","url":"https://...firmware.bin","sha256":"<64 hex chars>"}

   SAFETY MODEL -- this matters, because firmware on this node can unlock the car:

     The binary is streamed straight into the spare app partition while its
     SHA-256 is computed on the fly. The image is only committed if the hash
     matches the one you sent. A truncated download, a corrupted transfer or a
     substituted file all fail the check and are discarded -- the running
     firmware is untouched and the node keeps working.

     The hash arrives inside the MQTT command, which is TLS-protected and
     authenticated with SECRET. So even if the .bin itself were fetched over
     plain HTTP, its integrity is still guaranteed by a hash you sent over a
     trusted channel. That is deliberate: it removes any dependence on our
     certificate checking, which is still disabled.

   MQTT is disconnected during the download (this modem dislikes two TLS
   sockets at once) and progress goes to Serial; the result is reported over
   MQTT/Telegram afterwards.
   ====================================================================== */

#include <Update.h>

#define OTA_MAX_REDIRECTS 3

/* Parse "https://host:port/path" -- returns false if it isn't a URL we can use. */
static bool splitUrl(const char* url, bool* secure, char* host, size_t hostSz,
                     uint16_t* port, char* path, size_t pathSz) {
  const char* p = url;
  if      (!strncmp(p, "https://", 8)) { *secure = true;  *port = 443; p += 8; }
  else if (!strncmp(p, "http://", 7))  { *secure = false; *port = 80;  p += 7; }
  else return false;

  const char* slash = strchr(p, '/');
  const char* colon = strchr(p, ':');
  if (colon && (!slash || colon < slash)) {
    size_t n = colon - p; if (n >= hostSz) return false;
    memcpy(host, p, n); host[n] = 0;
    *port = (uint16_t)atoi(colon + 1);
  } else {
    size_t n = slash ? (size_t)(slash - p) : strlen(p);
    if (n >= hostSz) return false;
    memcpy(host, p, n); host[n] = 0;
  }
  snprintf(path, pathSz, "%s", slash ? slash : "/");
  return host[0] != 0;
}

static void hexOf(const uint8_t* h, char* out65) {
  static const char* d = "0123456789abcdef";
  for (int i = 0; i < 32; i++) { out65[i*2] = d[h[i] >> 4]; out65[i*2+1] = d[h[i] & 0x0F]; }
  out65[64] = 0;
}

/* Returns true only if the new image was written AND verified. */
/* otaProbe: exercise the exact download path (connect, TLS, HTTP headers,
   redirects) and REPORT what happens, without ever touching flash. This is the
   tool for debugging an OTA that fails before flashing -- run `otatest` with a
   url and read the Node log. Safe to run anytime; it only reads. */
void otaProbe(const char* url) {
  char urlbuf[300]; snprintf(urlbuf, sizeof(urlbuf), "%s", url);
  bool secure; char host[128]; uint16_t port; char path[200];
  int redirects = 0;

  nlog("[otatest] probing %s", url);
  bool hadMqtt = mqtt.connected();
  if (hadMqtt) { mqtt.disconnect(); netClient.stop(); delay(100); }

  while (redirects <= 3) {
    if (!splitUrl(urlbuf, &secure, host, sizeof(host), &port, path, sizeof(path))) {
      nlog("[otatest] bad url"); break;
    }
    nlog("[otatest] host=%s port=%d %s path=%s", host, (int)port, secure ? "TLS" : "plain", path);
    Client* c = secure ? (Client*)&tgClient : (Client*)&netClient;

    uint32_t tc = millis();
    if (!c->connect(host, port)) {
      nlog("[otatest] CONNECT FAILED after %lums - host unreachable or TLS refused",
           (unsigned long)(millis() - tc));
      break;
    }
    nlog("[otatest] connected in %lums", (unsigned long)(millis() - tc));

    c->print(String("GET ") + path + " HTTP/1.1\r\nHost: " + host +
             "\r\nUser-Agent: pajero-node\r\nConnection: close\r\n\r\n");

    int status = 0; long length = -1; char loc[256] = ""; long seen = 0;
    String line; bool done = false; uint32_t t0 = millis();
    while (!done && millis() - t0 < 25000) {
      if (c->available()) {
        char ch = c->read(); t0 = millis(); seen++;
        if (ch == '\n') {
          line.trim();
          if (!line.length()) { done = true; break; }
          if (!status && line.startsWith("HTTP/")) status = line.substring(line.indexOf(' ')+1, line.indexOf(' ')+4).toInt();
          if (line.startsWith("Content-Length:")) length = line.substring(15).toInt();
          if (line.startsWith("Location:")) snprintf(loc, sizeof(loc), "%s", line.substring(9).c_str());
          line = "";
        } else if (ch != '\r') line += ch;
      } else { if (!c->connected() && seen > 0) break; delay(20); }
    }
    c->stop();

    if (seen == 0) { nlog("[otatest] 0 BYTES back -> TLS handshake failed to %s", host); break; }
    nlog("[otatest] HTTP %d, %ld header bytes, content-length=%ld", status, seen, length);
    if (status >= 300 && status < 400 && loc[0]) {
      nlog("[otatest] redirect -> %s", loc);
      if (!strncmp(loc, "http", 4)) snprintf(urlbuf, sizeof(urlbuf), "%s", loc);
      else snprintf(urlbuf, sizeof(urlbuf), "%s://%s%s", secure ? "https" : "http", host, loc);
      redirects++; continue;
    }
    if (status == 200 && length > 0) nlog("[otatest] OK - download path works, %ld bytes ready", length);
    else nlog("[otatest] final status %d (expected 200 with a length)", status);
    break;
  }
  if (redirects > 3) nlog("[otatest] too many redirects");
  nlog("[otatest] done (no flash written). MQTT will reconnect.");
}

bool otaUpdate(const char* url, const char* wantSha) {
  if (!wantSha || strlen(wantSha) != 64) {
    nlog("[ota] refused - need a 64-char sha256");
    return false;
  }
  if (!cellUp) { nlog("[ota] refused - no network"); return false; }

  char urlbuf[256];
  snprintf(urlbuf, sizeof(urlbuf), "%s", url);

  bool     secure = true;
  char     host[96], path[160];
  uint16_t port = 443;

  nlog("[ota] starting: pausing MQTT for the download");
  mqtt.disconnect();
  netClient.stop();
  delay(150);

  bool     ok       = false;
  char     result[160] = "[ota] failed";
  int      redirects = 0;

  while (redirects <= OTA_MAX_REDIRECTS) {
    if (!splitUrl(urlbuf, &secure, host, sizeof(host), &port, path, sizeof(path))) {
      snprintf(result, sizeof(result), "[ota] bad url"); break;
    }
    nlog("[ota] GET %s%s (%s)", host, path, secure ? "https" : "http");

    Client* c;
    if (secure) c = &tgClient;               // reuse the mux-1 TLS socket
    else        c = &netClient;              // plain TCP on mux 0 (MQTT is down)

    if (!c->connect(host, port)) {
      snprintf(result, sizeof(result), "[ota] connect failed to %s", host); break;
    }
    c->print(String("GET ") + path + " HTTP/1.1\r\n" +
             "Host: " + host + "\r\n" +
             "User-Agent: pajero-node\r\n" +
             "Connection: close\r\n\r\n");

    // ---- headers ----
    // Wait for the response. On the SIM7670G TLS mux, c->connected() can read
    // false for a moment right after connect() while c->available() is still 0,
    // so we must NOT bail the instant the socket looks idle -- only give up when
    // the full timeout elapses with zero bytes ever received. Otherwise a slow
    // first byte (common on GitHub's redirect) looks like "HTTP 0".
    int      status = 0;
    long     length = -1;
    char     location[256] = "";
    uint32_t t0 = millis();
    String   line;
    bool     headersDone = false;
    long     bytesSeen = 0;

    while (!headersDone && millis() - t0 < 25000) {
      int avail = c->available();
      if (avail > 0) {
        while (c->available()) {
          char ch = c->read(); t0 = millis(); bytesSeen++;
          if (ch == '\n') {
            line.trim();
            if (line.length() == 0) { headersDone = true; break; }
            if (!status && line.startsWith("HTTP/")) {
              int sp = line.indexOf(' ');
              status = line.substring(sp + 1, sp + 4).toInt();
            }
            if (line.startsWith("Content-Length:")) length = line.substring(15).toInt();
            if (line.startsWith("Location:"))
              snprintf(location, sizeof(location), "%s", line.substring(9).c_str());
            line = "";
          } else if (ch != '\r') line += ch;
        }
      } else {
        // only give up if the socket is truly closed AND we already got data
        if (!c->connected() && bytesSeen > 0 && c->available() == 0) break;
        delay(20);
      }
    }
    if (bytesSeen == 0) {
      c->stop();
      nlog("[ota] DIAG: connected=%d but 0 bytes in 25s from %s:%d (%s)",
           (int)c->connected(), host, (int)port, secure ? "TLS" : "plain");
      nlog("[ota] DIAG: this is a TLS-handshake or server-response failure, not a URL problem");
      nlog("[ota] DIAG: Telegram uses the same TLS path - if TG works but this doesn't, it's GitHub-specific");
      snprintf(result, sizeof(result),
               "[ota] no response from %s (see DIAG lines above)", host);
      break;
    }

    if (status >= 300 && status < 400 && location[0]) {
      c->stop();
      // Location may be relative
      if (!strncmp(location, "http", 4)) snprintf(urlbuf, sizeof(urlbuf), "%s", location);
      else snprintf(urlbuf, sizeof(urlbuf), "%s://%s%s", secure ? "https" : "http", host, location);
      nlog("[ota] redirect %d -> %s", status, location);
      redirects++;
      continue;
    }
    if (status != 200) {
      c->stop();
      snprintf(result, sizeof(result), "[ota] HTTP %d", status); break;
    }
    if (length <= 0) {
      c->stop();
      snprintf(result, sizeof(result), "[ota] no Content-Length"); break;
    }
    nlog("[ota] %ld bytes incoming", length);

    if (!Update.begin((size_t)length)) {
      c->stop();
      snprintf(result, sizeof(result),
               "[ota] no room - partition scheme has no OTA slot"); break;
    }

    // ---- stream: write to flash and hash at the same time ----
    mbedtls_md_context_t md;
    mbedtls_md_init(&md);
    mbedtls_md_setup(&md, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 0);
    mbedtls_md_starts(&md);

    uint8_t buf[1024];
    long    got = 0;
    int     lastPct = -10;
    t0 = millis();
    while (got < length && millis() - t0 < 180000) {
      int n = c->read(buf, sizeof(buf));
      if (n > 0) {
        t0 = millis();
        if (Update.write(buf, n) != (size_t)n) { break; }
        mbedtls_md_update(&md, buf, n);
        got += n;
        int pct = (int)(got * 100 / length);
        if (pct >= lastPct + 10) { lastPct = pct; nlog("[ota] %d%% (%ld/%ld)", pct, got, length); }
      } else {
        if (!c->connected() && !c->available()) break;
        delay(5);
      }
    }
    c->stop();

    uint8_t hash[32]; char hex[65];
    mbedtls_md_finish(&md, hash);
    mbedtls_md_free(&md);
    hexOf(hash, hex);

    if (got != length) {
      Update.abort();
      snprintf(result, sizeof(result), "[ota] short read %ld/%ld", got, length);
      break;
    }
    if (strcasecmp(hex, wantSha) != 0) {
      Update.abort();
      nlog("[ota] got  %s", hex); nlog("[ota] want %s", wantSha);
      snprintf(result, sizeof(result), "[ota] SHA MISMATCH - update discarded");
      break;
    }
    if (!Update.end(true)) {
      snprintf(result, sizeof(result), "[ota] finalise failed (%d)", Update.getError());
      break;
    }
    ok = true;
    snprintf(result, sizeof(result), "[ota] verified %ld bytes - rebooting", length);
    break;
  }

  if (redirects > OTA_MAX_REDIRECTS)
    snprintf(result, sizeof(result), "[ota] too many redirects");

  Serial.println(result);
  nlog("%s", result);          // mirror outcome to app Node log
  snprintf(tgPending, sizeof(tgPending), "%s", result);

  if (ok) {
    tgSend("Pajero node: firmware updated, rebooting.");
    delay(500);
    esp_restart();
  }
  return ok;   // MQTT reconnects on the next loop; tgPending is flushed then
}

/* ================== CAN bus / OBD-II ==================================

   Hardware: SN65HVD230 transceiver (the ESP32-S3 has the CAN controller built
   in -- TWAI -- so no MCP2515 is needed).
       SN65HVD230 VCC -> 3.3V      CANH -> OBD pin 6
       SN65HVD230 GND -> GND       CANL -> OBD pin 14
       SN65HVD230 TXD -> CAN_TX    (ESP32)
       SN65HVD230 RXD -> CAN_RX    (ESP32)
       OBD pin 4/5 -> GND,  pin 16 -> +12V (do NOT power the board from this
       without a proper buck converter and fuse)

   >>> DESOLDER the 120R termination resistor on the module. The vehicle bus is
       already terminated at both ends; a third resistor can corrupt it. <<<

   WHAT IS AND ISN'T AVAILABLE -- read this before expecting a reading:
     Standard OBD-II (mode 01) reliably gives coolant temp, RPM, speed, intake
     air temp, engine load, run time, fuel level, module voltage.
     Oil temp (0x5C) and exhaust gas temp (0x78) are standard PIDs but many
     vehicles simply do not implement them.
     GEARBOX / TRANSMISSION FLUID TEMP IS NOT A STANDARD OBD-II PID. On the
     A750F it lives behind a Mitsubishi-specific request. This code therefore
     auto-discovers what the car actually supports, reports what's missing, and
     provides CUSTOM_PID hooks you can fill in once you know the real address.

   BATTERY DRAIN WARNING: polling keeps ECUs awake. This module only polls while
   the engine is running, and backs right off when the bus goes quiet.
   ====================================================================== */

#define CAN_ENABLE      1
/* NOT 15/16 -- those are the board's own I2C bus, where the MAX17048 battery
   gauge lives (confirmed by reading its registers). Using them for CAN would
   have broken both. VERIFY these against the survey's GPIO MAP before soldering. */
#define CAN_TX_PIN      4         // -> SN65HVD230 TXD
#define CAN_RX_PIN      5         // -> SN65HVD230 RXD
#define CAN_POLL_MS     2000      // while engine running
#define CAN_IDLE_MS     30000     // when the bus is asleep
#define ALERT_COOLDOWN  600000UL  // 10 min between repeats of the same alert
#define ALERT_CONFIRM   3         // consecutive over-threshold reads before alerting

#if CAN_ENABLE
#include "driver/twai.h"

/* ---- standard mode-01 PIDs we care about ---- */
#define PID_LOAD        0x04
#define PID_COOLANT     0x05
#define PID_RPM         0x0C
#define PID_SPEED       0x0D
#define PID_INTAKE_T    0x0F
#define PID_RUNTIME     0x1F
#define PID_FUEL_LVL    0x2F
#define PID_VOLTAGE     0x42
#define PID_AMBIENT     0x46
#define PID_OIL_T       0x5C
#define PID_EGT         0x78

/* Fill these in if you discover the Mitsubishi-specific addresses (a scan tool
   or a Torque extended-PID profile will tell you). 0 = disabled. */
#define CUSTOM_TRANS_T_PID   0x00
#define CUSTOM_TRANS_T_MODE  0x22

bool     canUp = false;
uint32_t canSupported[4] = {0,0,0,0};   // bitmasks for PIDs 0x01-0x20, 21-40, 41-60, 61-80
bool     engineRunning = false;
bool     startReportDone = true;
uint32_t engineStartMs = 0;

Reading rCoolant{}, rOil{}, rEgt{}, rIntake{}, rTrans{}, rVolt{}, rFuel{}, rRpm{}, rLoad{};

/* ---- thresholds. Conservative for a 4M41 3.2 DiD. ---- */
#define LIM_COOLANT   105.0f   // degC  (normal 85-95)
#define LIM_OIL       125.0f
#define LIM_TRANS     120.0f
#define LIM_EGT       700.0f   // post-turbo
#define LIM_INTAKE     75.0f
#define LIM_VOLT_LO    11.8f
#define LIM_VOLT_HI    15.2f
#define LIM_FUEL_LO    10.0f   // percent

bool canInit() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  g.rx_queue_len = 16;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) { nlog("[can] driver install failed"); return false; }
  if (twai_start() != ESP_OK)                    { nlog("[can] start failed");          return false; }
  nlog("[can] TWAI up at 500kbps");
  return true;
}

/* One OBD request/response. Returns payload length, 0 on timeout. */
uint8_t obdQuery(uint8_t mode, uint8_t pid, uint8_t* out, uint8_t outSz) {
  twai_message_t tx = {};
  tx.identifier = 0x7DF;                 // functional broadcast
  tx.data_length_code = 8;
  tx.data[0] = (mode == 0x03) ? 0x01 : 0x02;
  tx.data[1] = mode;
  tx.data[2] = pid;
  for (int i = 3; i < 8; i++) tx.data[i] = 0x55;
  if (twai_transmit(&tx, pdMS_TO_TICKS(100)) != ESP_OK) return 0;

  uint32_t t0 = millis();
  while (millis() - t0 < 300) {
    twai_message_t rx;
    if (twai_receive(&rx, pdMS_TO_TICKS(100)) != ESP_OK) continue;
    if (rx.identifier < 0x7E8 || rx.identifier > 0x7EF) continue;
    if ((rx.data[0] & 0xF0) != 0x00) continue;          // want a single frame here
    uint8_t len = rx.data[0] & 0x0F;
    if (len < 2 || rx.data[1] != (mode + 0x40)) continue;
    if (mode != 0x03 && rx.data[2] != pid) continue;
    uint8_t n = 0;
    for (uint8_t i = 3; i < len + 1 && n < outSz; i++) out[n++] = rx.data[i];
    return n;
  }
  return 0;
}

bool pidSupported(uint8_t pid) {
  if (pid == 0 || pid > 0x80) return false;
  uint8_t blk = (pid - 1) / 32;
  uint8_t bit = (pid - 1) % 32;
  return (canSupported[blk] >> (31 - bit)) & 1;
}

void canDiscover() {
  const uint8_t base[4] = {0x00, 0x20, 0x40, 0x60};
  for (int i = 0; i < 4; i++) {
    uint8_t d[4];
    if (obdQuery(0x01, base[i], d, 4) == 4)
      canSupported[i] = ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) |
                        ((uint32_t)d[2] << 8)  |  (uint32_t)d[3];
    else canSupported[i] = 0;
  }
  nlog("[can] supported PIDs %08X %08X %08X %08X",
       canSupported[0], canSupported[1], canSupported[2], canSupported[3]);
  // tell the user plainly what the car will and won't give us
  nlog("[can] oil_temp=%s egt=%s runtime=%s",
       pidSupported(PID_OIL_T) ? "yes" : "NO",
       pidSupported(PID_EGT)   ? "yes" : "NO",
       pidSupported(PID_RUNTIME) ? "yes" : "NO");
}

/* Read one PID and convert to engineering units. */
bool readPid(uint8_t pid, float* out) {
  uint8_t d[8];
  uint8_t n = obdQuery(0x01, pid, d, sizeof(d));
  if (n == 0) return false;
  switch (pid) {
    case PID_COOLANT: case PID_INTAKE_T: case PID_AMBIENT: case PID_OIL_T:
      if (n < 1) return false; *out = (float)d[0] - 40.0f; return true;
    case PID_RPM:
      if (n < 2) return false; *out = ((float)d[0] * 256.0f + d[1]) / 4.0f; return true;
    case PID_SPEED:
      if (n < 1) return false; *out = (float)d[0]; return true;
    case PID_LOAD:
      if (n < 1) return false; *out = (float)d[0] * 100.0f / 255.0f; return true;
    case PID_FUEL_LVL:
      if (n < 1) return false; *out = (float)d[0] * 100.0f / 255.0f; return true;
    case PID_RUNTIME:
      if (n < 2) return false; *out = (float)d[0] * 256.0f + d[1]; return true;
    case PID_VOLTAGE:
      if (n < 2) return false; *out = ((float)d[0] * 256.0f + d[1]) / 1000.0f; return true;
    case PID_EGT: {
      // 0x78: first byte is a support mask, then signed 16-bit sensors,
      // value = raw/10 - 40. Take the highest reported sensor.
      if (n < 3) return false;
      float best = -999;
      for (uint8_t i = 1; i + 1 < n; i += 2) {
        float t = ((float)((d[i] << 8) | d[i+1])) / 10.0f - 40.0f;
        if (t > best) best = t;
      }
      if (best < -100) return false;
      *out = best; return true;
    }
    default:
      if (n < 1) return false; *out = (float)d[0]; return true;
  }
}

/* Threshold check with confirmation and cooldown, so a single noisy frame
   can't wake you at 3am. */
void checkLimit(Reading* r, bool high, float limit, const char* name,
                const char* unit) {
  if (!r->ok) return;
  bool bad = high ? (r->v > limit) : (r->v < limit);
  if (!bad) { r->over = 0; return; }
  if (r->over < 255) r->over++;
  if (r->over < ALERT_CONFIRM) return;
  if (r->lastAlert && (uint32_t)(millis() - r->lastAlert) < ALERT_COOLDOWN) return;
  r->lastAlert = millis();
  char msg[140];
  snprintf(msg, sizeof(msg), "WARNING: %s %.1f%s (limit %.1f%s)",
           name, r->v, unit, limit, unit);
  nlog("[can] %s", msg);
  tgSend(msg);
  publishEvent("obd_alert", 0, 0, -1);
}

/* Read stored DTCs (mode 03). Handles a single frame and the first frame of a
   multi-frame reply; deeper replies need ISO-TP flow control, which we request. */
uint8_t readDTCs(char* out, size_t outSz) {
  out[0] = 0;
  twai_message_t tx = {};
  tx.identifier = 0x7DF; tx.data_length_code = 8;
  tx.data[0] = 0x01; tx.data[1] = 0x03;
  for (int i = 2; i < 8; i++) tx.data[i] = 0x55;
  if (twai_transmit(&tx, pdMS_TO_TICKS(100)) != ESP_OK) return 0;

  uint8_t codes = 0;
  size_t  used  = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 600) {
    twai_message_t rx;
    if (twai_receive(&rx, pdMS_TO_TICKS(150)) != ESP_OK) continue;
    if (rx.identifier < 0x7E8 || rx.identifier > 0x7EF) continue;

    uint8_t pci = rx.data[0] & 0xF0;
    uint8_t start, end;
    if (pci == 0x00) {                       // single frame
      if (rx.data[1] != 0x43) continue;
      start = 3; end = (rx.data[0] & 0x0F) + 1;
    } else if (pci == 0x10) {                // first frame -> ask for the rest
      if (rx.data[2] != 0x43) continue;
      twai_message_t fc = {};
      fc.identifier = 0x7E0; fc.data_length_code = 8;
      fc.data[0] = 0x30; fc.data[1] = 0x00; fc.data[2] = 0x00;
      twai_transmit(&fc, pdMS_TO_TICKS(50));
      start = 4; end = 8;
    } else if (pci == 0x20) {                // consecutive frame
      start = 1; end = 8;
    } else continue;

    for (uint8_t i = start; i + 1 < end; i += 2) {
      uint8_t a = rx.data[i], b = rx.data[i+1];
      if (a == 0 && b == 0) continue;
      char t = "PCBU"[(a >> 6) & 0x03];
      int written = snprintf(out + used, outSz - used, "%s%c%d%d%d%d",
                             used ? " " : "", t, (a >> 4) & 0x03,
                             a & 0x0F, (b >> 4) & 0x0F, b & 0x0F);
      if (written > 0 && used + written < outSz) { used += written; codes++; }
    }
    if (pci == 0x00) break;
  }
  return codes;
}

/* Poll everything once and publish a full report. */
void obdReport(const char* reason) {
  StaticJsonDocument<512> d;
  d["rep"] = reason;
  if (gaugeUp && battVolts > 0) { d["bv"] = battVolts; d["bat"] = battPct; }
  d["fw"] = FW_VERSION;
  d["ts"]  = nowMs();

  float v;
  if (readPid(PID_COOLANT, &v))  { rCoolant = {v,true,rCoolant.over,rCoolant.lastAlert}; d["cool"] = v; }
  if (readPid(PID_OIL_T, &v))    { rOil     = {v,true,rOil.over,rOil.lastAlert};         d["oil"]  = v; }
  if (readPid(PID_EGT, &v))      { rEgt     = {v,true,rEgt.over,rEgt.lastAlert};         d["egt"]  = v; }
  if (readPid(PID_INTAKE_T, &v)) { rIntake  = {v,true,rIntake.over,rIntake.lastAlert};   d["iat"]  = v; }
  if (readPid(PID_VOLTAGE, &v))  { rVolt    = {v,true,rVolt.over,rVolt.lastAlert};       d["volt"] = v; }
  if (readPid(PID_FUEL_LVL, &v)) { rFuel    = {v,true,rFuel.over,rFuel.lastAlert};       d["fuel"] = v; }
  if (readPid(PID_RPM, &v))      { rRpm     = {v,true,0,0};                              d["rpm"]  = v; }
  if (readPid(PID_LOAD, &v))     { rLoad    = {v,true,0,0};                              d["load"] = v; }
  if (readPid(PID_AMBIENT, &v))  d["amb"] = v;
  if (readPid(PID_RUNTIME, &v))  d["run"] = v;

#if CUSTOM_TRANS_T_PID
  { uint8_t b[8];
    if (obdQuery(CUSTOM_TRANS_T_MODE, CUSTOM_TRANS_T_PID, b, sizeof(b)) >= 1) {
      float t = (float)b[0] - 40.0f;
      rTrans = {t,true,rTrans.over,rTrans.lastAlert}; d["trans"] = t;
    } }
#endif

  char dtc[120];
  uint8_t n = readDTCs(dtc, sizeof(dtc));
  d["dtcn"] = n;
  if (n) d["dtc"] = dtc;

  char buf[512];
  size_t len = serializeJson(d, buf);
  char topic[64];
  snprintf(topic, sizeof(topic), "pajero/%s/obd", DEVICE_ID);
  mqtt.publish(topic, (const uint8_t*)buf, len, false);
  nlog("[can] report (%s): %s", reason, buf);
}

/* Human-readable health summary, sent to Telegram 3 min after start. */
void obdHealthTelegram() {
  char msg[400];
  int p = 0;
  p += snprintf(msg + p, sizeof(msg) - p, "Pajero warm-up check (3 min):");
  if (rCoolant.ok) p += snprintf(msg+p, sizeof(msg)-p, "\nCoolant %.0fC", rCoolant.v);
  if (rOil.ok)     p += snprintf(msg+p, sizeof(msg)-p, "\nOil %.0fC", rOil.v);
  if (rTrans.ok)   p += snprintf(msg+p, sizeof(msg)-p, "\nGearbox %.0fC", rTrans.v);
  if (rEgt.ok)     p += snprintf(msg+p, sizeof(msg)-p, "\nEGT %.0fC", rEgt.v);
  if (rIntake.ok)  p += snprintf(msg+p, sizeof(msg)-p, "\nIntake %.0fC", rIntake.v);
  if (rVolt.ok)    p += snprintf(msg+p, sizeof(msg)-p, "\nBattery %.1fV", rVolt.v);
  if (rFuel.ok)    p += snprintf(msg+p, sizeof(msg)-p, "\nFuel %.0f%%", rFuel.v);

  char dtc[120];
  uint8_t n = readDTCs(dtc, sizeof(dtc));
  if (n) p += snprintf(msg+p, sizeof(msg)-p, "\nFAULT CODES: %s", dtc);
  else   p += snprintf(msg+p, sizeof(msg)-p, "\nNo fault codes");

  bool allOk = (!rCoolant.ok || rCoolant.v < LIM_COOLANT) &&
               (!rOil.ok     || rOil.v     < LIM_OIL)     &&
               (!rEgt.ok     || rEgt.v     < LIM_EGT)     &&
               (!rTrans.ok   || rTrans.v   < LIM_TRANS)   &&
               (!rVolt.ok    || (rVolt.v > LIM_VOLT_LO && rVolt.v < LIM_VOLT_HI)) && n == 0;
  snprintf(msg + p, sizeof(msg) - p, "\n%s", allOk ? "All within range." : "CHECK THE ABOVE.");
  tgSend(msg);
}

void canLoop() {
  if (!canUp) return;
  static uint32_t last = 0;
  uint32_t iv = engineRunning ? CAN_POLL_MS : CAN_IDLE_MS;
  if ((uint32_t)(millis() - last) < iv) return;
  last = millis();

  float rpm = 0;
  bool  got = readPid(PID_RPM, &rpm);

  if (got && rpm > 300) {
    if (!engineRunning) {                       // engine just started
      engineRunning = true;
      engineStartMs = millis();
      startReportDone = false;
      if (!canSupported[0]) canDiscover();      // first run: learn the car
      nlog("[can] engine started");
      publishEvent("engine_start", 0, 0, -1);
    }
  } else if (!got) {
    if (engineRunning) {
      engineRunning = false;
      nlog("[can] engine stopped / bus asleep");
      publishEvent("engine_stop", 0, 0, -1);
    }
    return;                                     // bus asleep: don't keep poking it
  }

  if (!engineRunning) return;

  // routine monitoring
  float v;
  if (readPid(PID_COOLANT, &v))  { rCoolant.v = v; rCoolant.ok = true; }
  if (readPid(PID_OIL_T, &v))    { rOil.v     = v; rOil.ok     = true; }
  if (readPid(PID_EGT, &v))      { rEgt.v     = v; rEgt.ok     = true; }
  if (readPid(PID_INTAKE_T, &v)) { rIntake.v  = v; rIntake.ok  = true; }
  if (readPid(PID_VOLTAGE, &v))  { rVolt.v    = v; rVolt.ok    = true; }
  if (readPid(PID_FUEL_LVL, &v)) { rFuel.v    = v; rFuel.ok    = true; }
#if CUSTOM_TRANS_T_PID
  { uint8_t b[8];
    if (obdQuery(CUSTOM_TRANS_T_MODE, CUSTOM_TRANS_T_PID, b, sizeof(b)) >= 1) {
      rTrans.v = (float)b[0] - 40.0f; rTrans.ok = true; } }
#endif

  checkLimit(&rCoolant, true,  LIM_COOLANT, "Coolant temp", "C");
  checkLimit(&rOil,     true,  LIM_OIL,     "Oil temp",     "C");
  checkLimit(&rTrans,   true,  LIM_TRANS,   "Gearbox temp", "C");
  checkLimit(&rEgt,     true,  LIM_EGT,     "EGT",          "C");
  checkLimit(&rIntake,  true,  LIM_INTAKE,  "Intake temp",  "C");
  checkLimit(&rVolt,    false, LIM_VOLT_LO, "Battery volts","V");
  checkLimit(&rVolt,    true,  LIM_VOLT_HI, "Charging volts","V");
  checkLimit(&rFuel,    false, LIM_FUEL_LO, "Fuel level",   "%");

  // new fault codes while running
  static uint32_t lastDtcScan = 0;
  if ((uint32_t)(millis() - lastDtcScan) > 60000UL) {
    lastDtcScan = millis();
    char dtc[120];
    if (readDTCs(dtc, sizeof(dtc))) {
      static char lastSeen[120] = "";
      if (strcmp(dtc, lastSeen) != 0) {
        strncpy(lastSeen, dtc, sizeof(lastSeen) - 1);
        char m[160]; snprintf(m, sizeof(m), "Pajero FAULT CODE: %s", dtc);
        nlog("[can] %s", m);
        tgSend(m);
        publishEvent("dtc", 0, 0, -1);
      }
    }
  }

  // the 3-minute post-start health report
  if (!startReportDone && (uint32_t)(millis() - engineStartMs) > 180000UL) {
    startReportDone = true;
    nlog("[can] 3-minute health report");
    obdReport("startup");
    obdHealthTelegram();
  }
}
#endif  // CAN_ENABLE

/* ================= LoRa mesh: time-based unlock + off-grid position =================

   UNLOCK code is a TOTP-style token both sides derive from a shared secret and the
   current UTC minute, so nothing has to be typed by counter and nothing persists on
   the phone. Message the phone sends over the mesh:

        PJRO UNLOCK <token>

   token = first 6 hex chars of SHA-256("<SECRET>:<step>"), step = floor(unixUTC/60).
   The app (offline) shows the current token with a countdown; you paste it into the
   Meshtastic app. This board accepts it only if:
     - 4G is currently DOWN   (the whole point; zero attack surface while online)
     - the token matches this step or +/- MESH_TOTP_SKEW steps (clock drift)
     - that step hasn't been used before (anti-replay within the window)

   "Mildly secure": the private-channel AES stops eavesdrop/forge; the time window +
   used-step check stops replay outside ~1-2 min. It is not hardware-grade, and it is
   deliberately unreachable whenever the cellular path (with its full token stack) is
   available.
   ==================================================================================== */

uint32_t nowUnix() {
  time_t t = time(nullptr);
  return (t > 1700000000) ? (uint32_t)t : 0;   // 0 = clock not yet valid -> fail closed
}

void sha256hex(const char* in, char* out65) {
  const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  uint8_t h[32];
  mbedtls_md(info, (const unsigned char*)in, strlen(in), h);
  for (int i = 0; i < 32; i++) sprintf(out65 + i*2, "%02x", h[i]);
  out65[64] = 0;
}

void tokenForStep(uint32_t step, char* out7) {   // 6 hex chars + NUL
  char in[80], full[65];
  snprintf(in, sizeof(in), "%s:%u", SECRET, step);
  sha256hex(in, full);
  memcpy(out7, full, 6); out7[6] = 0;
}

// Returns true and fires unlock if the token is valid for an as-yet-unused step.
bool meshTryUnlock(const char* token) {
  if (cellUp)                     { Serial.println("[mesh] unlock ignored (4G up)"); return false; }
  if (strlen(SECRET) == 0)        { Serial.println("[mesh] no secret set");          return false; }
  uint32_t now = nowUnix();
  if (now == 0)                   { Serial.println("[mesh] no clock -> reject");      return false; }
  uint32_t step = now / MESH_TOTP_STEP;
  for (int32_t k = -MESH_TOTP_SKEW; k <= MESH_TOTP_SKEW; k++) {
    uint32_t s = step + k;
    if (s <= lastMeshStep) continue;             // already used / too old
    char exp[7]; tokenForStep(s, exp);
    if (strncasecmp(token, exp, 6) == 0) {
      lastMeshStep = s;
      prefs.putUInt("meshstep", s);              // persist so a reboot can't replay
      pUnlock.fire(PULSE_KEYFOB_MS);
      Serial.printf("[mesh] UNLOCK ok (step %u)\n", s);
      return true;
    }
  }
  Serial.println("[mesh] unlock: bad/expired token");
  return false;
}

// Read lines from the Heltec. Meshtastic prefixes incoming msgs with the sender short
// name + ':' -- and that prefix is known to arrive corrupted, so we take only the text
// AFTER THE LAST ':' and never parse the prefix.
void meshLoop() {
  static char buf[96];
  static uint8_t n = 0;
  while (meshSerial.available()) {
    char c = meshSerial.read();
    if (c == '\n' || c == '\r') {
      if (n) {
        buf[n] = 0; n = 0;
        char* colon = strrchr(buf, ':');
        char* body  = colon ? colon + 1 : buf;
        while (*body == ' ') body++;
        char token[16];
        if (sscanf(body, "PJRO UNLOCK %15s", token) == 1) meshTryUnlock(token);
      }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    } else { n = 0; }                            // overflow -> drop line
  }
}

// Off-grid only: push our own GPS as a text line so Meshtastic broadcasts a fast
// breadcrumb on top of the Heltec's own (slower) native position pin.
void meshBeacon() {
  static uint32_t last = 0;
  if (millis() - last < MESH_FASTPOS_MS) return;
  last = millis();
  float lat=0,lon=0,sp=0,al=0,ac=0; int vs=0,us=0; uint8_t st=0;
  if (modem.getGPS(&st,&lat,&lon,&sp,&al,&vs,&us,&ac) && lat != 0) {
    meshSerial.printf("PJRO %.5f,%.5f\n", lat, lon);
  }
}

/* Hidden button: debounced, must be held briefly, and rate-limited so a stuck
   or shorted switch can't machine-gun the keyfob output. */
void buttonLoop() {
  static uint32_t downAt = 0, lastFire = 0;
  static bool     wasDown = false;
  bool down = (digitalRead(PIN_UNLOCK_BTN) == LOW);

  if (down && !wasDown) {
    downAt = millis();
    nlog("[btn] pressed - hold %us to unlock", BTN_HOLD_MS / 1000);
  }
  if (!down && wasDown && downAt) {
    nlog("[btn] released early after %ums - ignored",
         (unsigned)(millis() - downAt));
    downAt = 0;
  }
  if (down && wasDown && downAt && (uint32_t)(millis() - downAt) >= BTN_HOLD_MS) {
    downAt = 0;
    if ((uint32_t)(millis() - lastFire) > 2000) {     // 2s lockout
      lastFire = millis();
      pUnlock.fire(PULSE_KEYFOB_MS);
      nlog("[btn] hidden button -> UNLOCK");
      publishEvent("button_unlock", 0, 0, -1);
    }
  }
  if (!down && wasDown) delay(BTN_DEBOUNCE_MS);
  wasDown = down;
}

/* ---------- setup / loop ---------- */
void setup() {
  Serial.printf("\n[boot] Pajero node fw %s  built %s\n", FW_VERSION, FW_BUILD);
  Serial.begin(115200);
  setenv("TZ", "UTC0", 1); tzset();     // keep all internal time strictly UTC
  delay(1000);

  /* Outputs first, before anything can go wrong. A relay must never sit
     closed while the modem is booting. */
  pUnlock.begin(PIN_UNLOCK);
  pLock.begin(PIN_LOCK);
  pBlackbox.begin(PIN_BLACKBOX);
  pGate.begin(PIN_GATE);
  pinMode(PIN_UNLOCK_BTN, INPUT_PULLUP);   // hidden hard-wired unlock

  snprintf(topicStatus, sizeof(topicStatus), "pajero/%s/status", DEVICE_ID);
  snprintf(topicCmd,    sizeof(topicCmd),    "pajero/%s/cmd",    DEVICE_ID);
  snprintf(topicEvent,  sizeof(topicEvent),  "pajero/%s/event",  DEVICE_ID);
  snprintf(topicLog,    sizeof(topicLog),    "pajero/%s/log",    DEVICE_ID);
  snprintf(topicCell,   sizeof(topicCell),   "pajero/%s/cell",   DEVICE_ID);

  meshSerial.begin(MESH_BAUD, SERIAL_8N1, MESH_RXD, MESH_TXD);
  gaugeBegin();
#if CAN_ENABLE
  canUp = canInit();
#endif
  prefs.begin("pajero", false);
  lastMeshStep = prefs.getUInt("meshstep", 0);
  geoLat       = prefs.getDouble("glat", 0);
  geoLon       = prefs.getDouble("glon", 0);
  geoRadiusM   = prefs.getUShort("grad", 120);
  geoGateAuto  = prefs.getBool("gauto", false);
  if (geoLat != 0 || geoLon != 0)
    Serial.printf("[geo] home %.6f,%.6f r=%um auto=%d\n",
                  geoLat, geoLon, geoRadiusM, geoGateAuto);

  modemSerial.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  modemInit();

  // TLS: this fork's SIM7670G secure client (mux 0, declared above) does not verify
  // the certificate by default, so the HiveMQ handshake should complete without a CA.
  // HARDEN LATER: load HiveMQ's root CA + enable authmode for real verification.

  Serial.println("[net] waiting for network (up to 20s)…");
  if (modem.waitForNetwork(20000L)) {
    Serial.println("[net] registered");
    netAttach();
  } else {
    Serial.println("[net] NOT registered after 20s — loop will keep retrying");
  }

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMessage);
  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(60);
}

void loop() {
  pUnlock.tick(); pLock.tick(); pBlackbox.tick(); pGate.tick();
  buttonLoop();
  gaugeUpdate();
  histSample();
  if (pendingCellCmd) {                 // run deferred AT diagnostics off the MQTT callback
    char c = pendingCellCmd; pendingCellCmd = 0;
    if      (c == 'c') cmdCell();
    else if (c == 'd') cmdData();
    else if (c == 'h') cmdHist();
    else if (c == 'l') cmdLocate();
  }
  if (pendingOtaTest) { pendingOtaTest = false; otaProbe(otaTestUrl); }
#if CAN_ENABLE
  canLoop();
#endif
  meshLoop();                                   // LoRa unlock works even with no cellular

  cellUp = modem.isNetworkConnected();          // fast AT+CREG query, does not block

  if (cellUp) {
    netAttach();                                // brings data up on transition; no-op if already up
    if (!mqtt.connected()) connectMqtt();
    mqtt.loop();
    if (tgPending[0] && mqtt.connected()) {   // result of a send made while MQTT was down
      nlog("%s", tgPending);
      tgPending[0] = 0;
    }
    static uint32_t last = 0, clk = 0;
    static bool clockOnce = false;
    trySyncClockFromGNSS();                     // GNSS is the primary clock
    if (!clockOnce) { trySyncClockFromNetwork(); clockOnce = true; }  // disabled by default
    uint32_t iv = pBlackbox.active ? 1000 : STATUS_EVERY_MS;
    if (millis() - last > iv) { last = millis(); publishStatus(); }
    if (millis() - clk  > 3600000UL) { clk = millis(); trySyncClockFromNetwork(); }  // no-op once set
  } else {
    trySyncClockFromGNSS();                     // off-grid the LoRa unlock NEEDS a clock
    meshBeacon();                               // off-grid: fast text position; LoRa unlock via meshLoop()

    /* Diagnostics: without this, a modem that fails to register produces total
       silence, indistinguishable from a crash. BUT isNetworkConnected() (used
       for cellUp above) and getRegistrationStatus()/getSignalQuality() are
       separate AT queries that this SIM7670G answers inconsistently -- CSQ often
       reads 99 and reg reads 2 even while data flows fine. So re-confirm we are
       genuinely offline before crying wolf, and never treat CSQ alone as truth. */
    static uint32_t lastDiag = 0;
    if (millis() - lastDiag > 5000) {
      lastDiag = millis();
      if (modem.isNetworkConnected()) {
        // false alarm: we're actually connected, cellUp just hadn't updated yet
      } else if (!modem.testAT(1000)) {
        Serial.println("[net] modem NOT ANSWERING — it has powered down or lost its supply");
      } else {
        Serial.printf("[net] connecting… reg=%d csq=%d sim=%d\n",
                      (int)modem.getRegistrationStatus(),
                      modem.getSignalQuality(),
                      (int)modem.getSimStatus());
      }
    }
  }
}
