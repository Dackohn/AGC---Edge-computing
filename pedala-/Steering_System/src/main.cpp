// --- NANO #2: CAN DRIVER (RC RECEIVER) + TRACTION CONTROL (PWM) ---
// Reconstructed from firmware binary + schematic Schematic_shield-can_2026-04-29
#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <stdint.h>

// ================================================================
// 1. PIN DEFINITIONS  (verified against schematic)
// ================================================================

// RC INPUTS вЂ” all on Port D, read via PCINT2
#define PIN_RC_THR    6   // D2 вЂ” Throttle RC channel
#define PIN_RC_ARM    3   // D3 вЂ” Arm switch RC channel
#define PIN_RC_STEER  5   // D5 вЂ” Steering RC channel
#define PIN_RC_DIR    9   // D6 вЂ” Direction RC channel (fwd/rev)

// CAN BUS
#define CAN_CS        10  // D10
#define CAN_INT       4   // D4

// RELAY OUTPUTS
#define PIN_RELAY1    7   // D7  вЂ” Direction relay 1 (forward)
#define PIN_THR_SW    8   // D8  вЂ” Throttle switch relay (main enable)
#define PIN_RELAY2    A0  // A0  вЂ” Direction relay 2 (reverse)

// PWM OUTPUT
#define PIN_THR_PWM   9   // D9  вЂ” Throttle PWM в†’ R1(1k)/C1(1u) в†’ thr_out в†’ P2

MCP_CAN CAN0(CAN_CS);

// ================================================================
// 2. CONSTANTS
// ================================================================

#define MOTOR_ID        1
#define CAN_ID_CMD      (0x06000000 + MOTOR_ID)
#define CAN_ID_FB       (0x05800000 + MOTOR_ID)
#define CAN_BAUD        CAN_250KBPS
#define MCP_CLOCK       MCP_8MHZ

#define POSITION_SCALE  39L
#define DEG_MIN         -1080
#define DEG_MAX          1080

static const uint16_t ARM_OFF_US     = 1200;
static const uint16_t ARM_ON_US      = 1800;
static const uint32_t FAILSAFE_US    = 250000; // 250ms
static const uint32_t HB_TIMEOUT_MS  = 2000;

// ================================================================
// 3. STATE
// ================================================================

volatile uint32_t rcRiseTime[8]   = {0};
volatile uint16_t rcPulse[8]      = {1500,1500, 500,1100,1500,1500, 500,1500};
//                                   [0]  [1] [2]thr [3]arm  [4]  [5]str [6]thr [7]
//                                  [0]  [1]  [2]arm[3]  [4]  [5]str[6]dir[7]
volatile uint32_t rcLastMicros[8] = {0};
volatile uint8_t  prevPinD        = 0;

float    currentTargetAngle = 0.0f;
float    motorAngle         = 0.0f;
// Steering center calibration: if wheels aren't straight at 1500Вµs RC / steer=1500,
// adjust this until they are. Positive = motor turns right to reach "straight".
static float STEER_CENTER_DEG = 0.0f;
bool     motorEnabled       = false;
bool     armed              = false;
static int8_t relayState   = 0;   // 0=coast 1=fwd -1=rev
uint32_t lastHbMillis       = 0;

// Serial command override (injected by mini computer agent)
static char     cmdBuf[40]         = {0};
static uint8_t  cmdLen             = 0;
static uint16_t cmdThr             = 1500;
static uint16_t cmdDir             = 1500;
static uint16_t cmdSteer           = 1500;
static uint16_t cmdArm             = 1000;  // default: disarmed
static uint32_t cmdLastMs          = 0;
static const uint32_t CMD_TIMEOUT_MS = 500; // fall back to RC after 500ms silence

// ================================================================
// 4. PCINT2 ISR вЂ” all 4 RC channels simultaneously, no blocking
// ================================================================

ISR(PCINT2_vect) {
  uint32_t now  = micros();
  uint8_t  curD = PIND;
  uint8_t  chg  = prevPinD ^ curD;
  prevPinD      = curD;

  const uint8_t pins[] = {PIN_RC_THR, PIN_RC_ARM, PIN_RC_STEER, PIN_RC_DIR};
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t p    = pins[i];
    uint8_t mask = (1 << p);
    if (!(chg & mask)) continue;
    if (curD & mask) {
      rcRiseTime[p] = now;
    } else {
      uint32_t w = now - rcRiseTime[p];
      bool valid = (w >= 100 && w <= 2100);
      if (valid) {
        rcPulse[p]      = (uint16_t)w;
        rcLastMicros[p] = now;
      }
    }
  }
}

// ================================================================
// 5. TIMER1 PWM вЂ” 1 kHz on pin 9 (OC1A)
// ================================================================

void setupPwm1kHz() {
  pinMode(PIN_THR_PWM, OUTPUT);
  TCCR1A = (1<<WGM11) | (1<<COM1A1);
  TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS11);
  ICR1   = 1999;
  OCR1A  = 0;
}

static inline void setDutyPercent(uint8_t d) {
  OCR1A = (uint16_t)((uint32_t)ICR1 * constrain(d, 0, 100) / 100);
}

// ================================================================
// 6. CAN HELPERS
// ================================================================

void sendCAN(uint32_t id, uint8_t data[8]) {
  CAN0.sendMsgBuf(id, 1, 8, data);
}

void sendEnable() {
  uint8_t en[8]  = {0x23,0x0D,0x20,0x01,0,0,0,0};
  uint8_t pos[8] = {0x03,0x0D,0x20,0x31,0,0,0,0};
  sendCAN(CAN_ID_CMD, en);  delay(2);
  sendCAN(CAN_ID_CMD, pos);
}

void sendDisable() {
  uint8_t dis[8] = {0x23,0x0C,0x20,0x01,0,0,0,0};
  sendCAN(CAN_ID_CMD, dis);
}

void sendPosition(float deg) {
  deg = constrain(deg, DEG_MIN, DEG_MAX);
  long pos = (long)(-deg * POSITION_SCALE / 360.0f);
  uint8_t f[8] = {0x23,0x02,0x20,0x01,
    (uint8_t)(pos),(uint8_t)(pos>>8),(uint8_t)(pos>>16),(uint8_t)(pos>>24)};
  sendCAN(CAN_ID_CMD, f);
}

void readFeedback() {
  unsigned long id; byte len = 0; byte buf[8];
  if (CAN0.checkReceive() != CAN_MSGAVAIL) return;
  CAN0.readMsgBuf(&id, &len, buf);
  if (id == CAN_ID_FB && len >= 8) {
    long posInt = ((long)buf[7]<<24)|((long)buf[6]<<16)|((long)buf[5]<<8)|buf[4];
    motorAngle  = -posInt * 360.0f / POSITION_SCALE;
    lastHbMillis = millis();
  }
}

// ================================================================
// 7. SERIAL COMMAND PARSER
//    Single-char keys  (keyboard testing, no Enter needed):
//      e/E  ARM          q/Q  DISARM+reset
//      w/W  fwd+throttleв†‘    s/S  rev+throttleв†‘
//      a/A  steer left        d/D  steer right
//      SPC  stop (thr+dir neutral)   x/X  center steer
//    CMD: line protocol (MQTT agent):
//      CMD:<thr>,<dir>,<steer>,<arm>\n  в†’  ACK\n
// ================================================================

void parseSerialCmd() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      // End of line вЂ” try to parse CMD: line
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        if (strncmp(cmdBuf, "CMD:", 4) == 0) {
          unsigned int t, d, s, a;
          if (sscanf(cmdBuf + 4, "%u,%u,%u,%u", &t, &d, &s, &a) == 4) {
            if (t >= 900 && t <= 2100 && d >= 900 && d <= 2100 &&
                s >= 900 && s <= 2100 && a >= 900 && a <= 2100) {
              cmdThr    = (uint16_t)t;
              cmdDir    = (uint16_t)d;
              cmdSteer  = (uint16_t)s;
              cmdArm    = (uint16_t)a;
              cmdLastMs = millis();
              Serial.println("ACK");
            }
          }
        }
        cmdLen = 0;
      }
    } else if (cmdLen > 0) {
      // Mid-line: accumulate only вЂ” no single-char processing
      // (prevents 'D' in "CMD:" from firing steer-right, etc.)
      if (cmdLen < 39) cmdBuf[cmdLen++] = c;
    } else {
      // Start of a new line: single-char keyboard commands (for bench testing)
      switch (c) {
        case 'e': case 'E':
          cmdArm    = 2000;
          cmdLastMs = millis();
          break;
        case 'q': case 'Q':
          cmdArm = 1000; cmdThr = 1500; cmdDir = 1500; cmdSteer = 1500;
          cmdLastMs = millis();
          break;
        case 'w': case 'W':
          if (cmdDir != 1600) cmdThr = 1500;
          cmdDir = 1600;
          cmdThr = min((uint16_t)1900, (uint16_t)(cmdThr + 40));
          cmdLastMs = millis();
          break;
        case 's': case 'S':
          if (cmdDir != 1400) cmdThr = 1500;
          cmdDir = 1400;
          cmdThr = min((uint16_t)1900, (uint16_t)(cmdThr + 40));
          cmdLastMs = millis();
          break;
        case 'a': case 'A':
          cmdSteer  = max((uint16_t)1100, (uint16_t)(cmdSteer - 60));
          cmdLastMs = millis();
          break;
        case 'd': case 'D':
          cmdSteer  = min((uint16_t)1900, (uint16_t)(cmdSteer + 60));
          cmdLastMs = millis();
          break;
        case ' ':
          cmdThr = 1500; cmdDir = 1500;
          cmdLastMs = millis();
          break;
        case 'x': case 'X':
          cmdSteer  = 1500;
          cmdLastMs = millis();
          break;
        default:
          // Start accumulating a multi-char line (e.g. "CMD:...")
          cmdBuf[cmdLen++] = c;
          break;
      }
    }
  }
}

// ================================================================
// 8. SETUP
// ================================================================

void setup() {
  Serial.begin(115200);

  pinMode(PIN_RC_THR,   INPUT);
  pinMode(PIN_RC_ARM,   INPUT);
  pinMode(PIN_RC_STEER, INPUT);
  pinMode(PIN_RC_DIR,   INPUT);

  pinMode(PIN_RELAY1,  OUTPUT); digitalWrite(PIN_RELAY1,  LOW);
  pinMode(PIN_THR_SW,  OUTPUT); digitalWrite(PIN_THR_SW,  LOW);
  pinMode(PIN_RELAY2,  OUTPUT); digitalWrite(PIN_RELAY2,  LOW);
  setupPwm1kHz();
  setDutyPercent(0);

  // Enable PCINT2 for D2, D3, D5, D6
  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1<<PCINT18)|(1<<PCINT19)|(1<<PCINT21)|(1<<PCINT22);
  prevPinD = PIND;

  if (CAN0.begin(MCP_ANY, CAN_BAUD, MCP_CLOCK) == CAN_OK)
    Serial.println("MCP2515 OK (250kbps, 8MHz)");
  else
    Serial.println("CAN init FAIL");
  CAN0.setMode(MCP_NORMAL);
  delay(200);

  // Motor init sequence
  uint8_t en[8]   = {0x23,0x0D,0x20,0x01,0,0,0,0};
  uint8_t spd[8]  = {0x03,0x0D,0x20,0x11,0,0,0,0};
  uint8_t spdv[8] = {0x23,0x00,0x20,0x01,100,0,0,0};
  uint8_t pos[8]  = {0x03,0x0D,0x20,0x31,0,0,0,0};
  sendCAN(CAN_ID_CMD, en);   delay(50);
  sendCAN(CAN_ID_CMD, spd);  delay(10);
  sendCAN(CAN_ID_CMD, spdv); delay(10);
  sendCAN(CAN_ID_CMD, pos);  delay(100);
  sendDisable();
  Serial.println("KEYA: DISABLE sent");

  lastHbMillis = millis();
}

// ================================================================
// 8. LOOP
// ================================================================

void loop() {
  // --- Atomic RC snapshot ---
  uint16_t thr, arm, steer, dir;
  uint32_t lastThr, lastDir, lastArm;
  noInterrupts();
  thr     = rcPulse[PIN_RC_THR];
  arm     = rcPulse[PIN_RC_ARM];
  steer   = rcPulse[PIN_RC_STEER];
  dir     = rcPulse[PIN_RC_DIR];
  lastThr = rcLastMicros[PIN_RC_THR];
  lastDir = rcLastMicros[PIN_RC_DIR];
  lastArm = rcLastMicros[PIN_RC_ARM];
  interrupts();

  uint32_t now   = micros();
  bool     armOk = (now - lastArm) <= FAILSAFE_US;

  // --- Serial override (mini computer takes priority over RC when active) ---
  // parseSerialCmd();
  // bool serialActive = (millis() - cmdLastMs) < CMD_TIMEOUT_MS;
  // if (serialActive) {
  //   thr   = cmdThr;
  //   dir   = cmdDir;
  //   steer = cmdSteer;
  //   arm   = cmdArm;
  //   armOk = thrOk = dirOk = true;
  // }

  // --- Arm logic ---
  bool newArmed = armed;
  if      (!armOk)          newArmed = false;
  else if (arm < ARM_OFF_US) newArmed = false;
  else if (arm > ARM_ON_US)  newArmed = true;

  // if (newArmed && !armed) {
    sendEnable();
    motorEnabled = true;
    // Serial.println("ARMED -> sent ABS_POS_MODE + ENABLE");
  // } else if (!newArmed && armed) {
  //   sendDisable();
  //   motorEnabled = false;
  //   digitalWrite(PIN_THR_SW, LOW);
  //   digitalWrite(PIN_RELAY1,  LOW);
  //   digitalWrite(PIN_RELAY2,  LOW);
  //   setDutyPercent(0);
  //   Serial.println("DISARM -> sent DISABLE");
  // }
  armed = true;//newArmed;

  if (armed) {
    // --- Throttle switch relay: ON when armed ---
    digitalWrite(PIN_THR_SW, HIGH);

    // --- Direction + speed: single thr channel (center=500us, >500=fwd, <500=rev) ---
    static const int32_t THR_CENTER = 500;
    static const int32_t THR_DEAD   = 10;   // dead zone +-10us — increase if relay chatters
    static const int32_t THR_HYST   = 5;    // hysteresis band
    static const int32_t THR_RANGE  = 300;
    static const uint8_t MAX_DUTY   = 90;    
    int32_t thr_rel = (int32_t)thr - THR_CENTER;

    switch (relayState) {
      case 0:  // coast — activate only past full dead zone
        if      (thr_rel < -THR_DEAD)  relayState =  1;
        else if (thr_rel >  THR_DEAD)  relayState = -1;
        break;
      case 1:  // forward — release only when signal returns inside hysteresis band
        if (thr_rel > -(THR_DEAD - THR_HYST))  relayState = 0;
        break;
      case -1: // reverse — release only when signal returns inside hysteresis band
        if (thr_rel < (THR_DEAD - THR_HYST))   relayState = 0;
        break;
    }

    if (relayState == 1) {
      digitalWrite(PIN_RELAY1, HIGH);
      digitalWrite(PIN_RELAY2, LOW);
    } else if (relayState == -1) {
      digitalWrite(PIN_RELAY1, LOW);
      digitalWrite(PIN_RELAY2, HIGH);
    } else {
      digitalWrite(PIN_RELAY1, LOW);
      digitalWrite(PIN_RELAY2, LOW);
    }

    uint8_t duty = (relayState != 0)
                   ? (uint8_t)constrain(abs(thr_rel) * MAX_DUTY / THR_RANGE, 0, MAX_DUTY)
                   : 0;
    setDutyPercent(duty);

    // --- Steering CAN position ---
    bool steerValid = (steer > 900 && steer < 2100); //&& (serialActive || lastSteer > 0);
    if (steerValid) {
      if (abs((long)steer - 1500) < 30) steer = 1500;
      float angle = (float)((long)steer - 1500) * 1.35f;
      if (abs(angle - currentTargetAngle) >= 2.0f || angle == 0.0f)
        currentTargetAngle = angle;
    }
    static uint32_t lastCanSend = 0;
    if (millis() - lastCanSend > 20) {
      lastCanSend = millis();
      sendPosition(currentTargetAngle + STEER_CENTER_DEG);
    }

  } else {
    // Disarmed: keep sending disable, all relays off
    static uint32_t lastDis = 0;
    if (millis() - lastDis > 100) { lastDis = millis(); sendDisable(); }
  }

  // --- CAN feedback + heartbeat ---
  readFeedback();
  static uint32_t lastWarn = 0;
  if (millis() - lastHbMillis > HB_TIMEOUT_MS && millis() - lastWarn > HB_TIMEOUT_MS) {
    lastWarn = millis();
    Serial.println("WARN: no heartbeat recently");
  }

  // --- Debug (500ms) ---
  static uint32_t lastDebug = 0;
  if (millis() - lastDebug > 500) {
    lastDebug = millis();
    // Serial.print(serialActive ? "SER" : "RC");
    Serial.print(" thr=");  Serial.print(thr);
    Serial.print(" arm=");    Serial.print(arm);
    Serial.print(" steer=");  Serial.print(steer);
    Serial.print(" relay=");  Serial.print(dir);
    Serial.print(" relay="); Serial.print(relayState);
    Serial.print(" -> deg="); Serial.println(currentTargetAngle, 2);
  }
}
