# 02-AGC Autonomous Yard Utility Vehicle — Golf Cart
**Project**: PBL26 | **Team Lead**: Denis Pleca | **Role here**: Controls & Edge Computing

## Project Context
Agricultural autonomous utility vehicle (electric golf cart) for farm logistics, inspection, and patrol.
Combines edge computing, wireless connectivity, onboard sensors, and a web-based control system.

The control system has two layers (from project architecture diagram):
- **Low-level (Arduino Nano)** — real-time RC/CAN/PWM control, millisecond response
- **High-level (Mini Computer onboard)** — edge AI, cloud sync, mission control, Serial bridge to Arduino

Claude is responsible for: Arduino firmware + mini computer control agent + internet remote control pipeline.

---

## System Architecture

```
[Remote PC / Web Dashboard]
        |
    Internet (MQTT over TLS)
        |
[Mini Computer — onboard vehicle]
   - runs MQTT client agent (Python)
   - translates commands → Serial → Arduino
   - reads Serial telemetry → publishes to MQTT
        |
      USB Serial (115200 baud)
        |
[Arduino Nano — CIUPACIUPS_314]
   - reads RC receiver (PCINT2)
   - drives KEYA steering motor (CAN)
   - drives traction (PWM + relays)
        |
   [MCP2515] → CAN Bus → [KEYA Steering Motor]
   [KY-019 ×3] → traction relays
   [RC Receiver] → J6 header
```

---

## Hardware

### MCU — Arduino Nano (U1, ATmega328P)
Connected to: MCP2515 CAN module, 3× KY-019 relay boards, RC receiver, Mini Computer (USB)

### CAN — MCP2515 Module (U2)
- Speed: 250 kbps, Crystal: 8 MHz
- Protocol: proprietary KEYA (extended 29-bit CAN frames)
- CAN-H / CAN-L → KEYA steering motor controller

### Steering Motor — KEYA Brushless
- CAN ID command: `0x06000000 + MOTOR_ID` (MOTOR_ID = 1)
- CAN ID feedback: `0x05800000 + MOTOR_ID`
- Mode: Absolute Position (ABS_POS_MODE)
- Position scale: 39 internal units per 360°
- Range: −540° to +540°
- Position sign: inverted (negative deg → positive internal units)

### Relay Boards — KY-019 (×3, active HIGH)
- U3 → relay1 (forward direction relay)
- U4 → relay2 (reverse direction relay)
- U5 → thr_sw (throttle enable — closes PWM path to output)

### PWM Filter
- thr_pwm (D9) → R1 (1kΩ) → C1 (1µF to GND) → thr_out → P2 connector
- Low-pass RC filter converts 1kHz PWM duty cycle to analog DC voltage
- That DC voltage is the speed reference input for the traction motor driver

### RC Receiver
- Connected to J6 header (8-pin HDR-M-2.54)
- J8 provides GND reference for receiver
- All 4 channels read simultaneously via PCINT2 (no pulseIn blocking)

---

## Pin Map (verified against Schematic_shield-can_2026-04-29.svg)

| Arduino Pin | Direction | Net     | Function                               |
|-------------|-----------|---------|----------------------------------------|
| D2          | INPUT     | thr     | RC throttle — speed magnitude axis     |
| D3          | INPUT     | arm     | RC arm switch                          |
| D4          | INPUT     | int     | MCP2515 INT (unused, polled instead)   |
| D5          | INPUT     | steer   | RC steering — right stick X            |
| D6          | INPUT     | dir     | RC direction axis — left stick fwd/rev |
| D7          | OUTPUT    | relay1  | Forward relay (KY-019 U3)              |
| D8          | OUTPUT    | thr_sw  | Throttle switch relay (KY-019 U5)      |
| D9          | OUTPUT    | thr_pwm | Traction PWM (Timer1 OC1A, 1 kHz)     |
| D10         | OUTPUT    | cs      | MCP2515 SPI chip select                |
| D11         | OUTPUT    | si      | SPI MOSI                               |
| D12         | INPUT     | so      | SPI MISO                               |
| D13         | OUTPUT    | sck     | SPI SCK                                |
| A0          | OUTPUT    | relay2  | Reverse relay (KY-019 U4)              |

---

## Current Arduino Control Logic

### Arming (arm, D3)
- `arm > 1800µs` → ARMED (CAN enable + ABS_POS_MODE, thr_sw relay HIGH)
- `arm < 1200µs` → DISARMED (all relays OFF, duty 0, CAN disable)
- No arm signal >250ms → DISARMED (failsafe)

### Direction (dir, D6 — left stick direction axis)
- Controls relay1/relay2, independent from speed
- `dir > 1550` → relay1 HIGH (forward), relay2 LOW
- `dir < 1450` → relay2 HIGH (reverse), relay1 LOW
- `1450 ≤ dir ≤ 1550` → both relays LOW (neutral/coasting)
- Deadband ±50µs around 1500

### Speed (thr, D2 — throttle axis)
- Controls PWM duty magnitude, independent from direction
- Center-return stick, 1500µs = stop
- `abs(thr − 1500) > 50` → duty = `abs(thr−1500) × 100 / 400`%
- Inside deadband → duty = 0%
- Range: ±400µs from center (1100–1900)
- `thr_sw` relay (D8) stays HIGH for entire armed period

### Steering (steer, D5 — CAN position control, 50 Hz)
- angle = `(steer − 1500) × 1.35` degrees, clamped ±540°
- Deadzone ±30µs → snapped to 0°
- Only updates if angle change ≥2° (prevents motor hunting)
- Ignored until first valid steer pulse received (lastSteer > 0)

### Heartbeat
- Prints `"WARN: no heartbeat recently"` if no CAN feedback for 2s

---

## CAN Message Reference (KEYA, 29-bit extended)

| Purpose          | Bytes [0..7]                    |
|------------------|---------------------------------|
| Enable           | `23 0D 20 01 00 00 00 00`       |
| Disable          | `23 0C 20 01 00 00 00 00`       |
| Speed mode       | `03 0D 20 11 00 00 00 00`       |
| Set speed 100rpm | `23 00 20 01 64 00 00 00`       |
| Position mode    | `03 0D 20 31 00 00 00 00`       |
| Set position     | `23 02 20 01 [pos 4B LE]`       |

`pos = (long)(−deg × 39 / 360)`, little-endian 32-bit signed.
Feedback `0x05800001`: position bytes [4..7], LE signed 32-bit.

---

## Serial Debug Format (Arduino → Mini Computer)
```
MCP2515 OK (250kbps, 8MHz)
KEYA: DISABLE sent
ARMED -> sent ABS_POS_MODE + ENABLE
RC thr=1548 arm=2008 steer=1504 relay=984 -> deg=0.00
WARN: no heartbeat recently
DISARM -> sent DISABLE
```

---

## Remote Control Plan — Internet Connection

### Goal
Operator on a remote PC sends joystick commands → mini computer on vehicle → Arduino.
Vehicle sends back telemetry (speed, steering angle, heartbeat).

### Chosen Approach: MQTT over TLS
MQTT is the right choice because:
- Works through NAT with no port forwarding (both sides connect OUT to broker)
- Low latency (QoS 0 fire-and-forget ~50–100ms round trip)
- Standard IoT protocol, matches project's IoT architecture
- Free cloud brokers available (HiveMQ Cloud free tier)
- Trivially extensible for future autonomous telemetry topics

### Connection Architecture

```
[Remote PC]                    [MQTT Broker — cloud]        [Mini Computer — vehicle]
  web dashboard       ←→      e.g. HiveMQ Cloud / VPS      ←→   Python agent
  publishes commands           broker.example.com:8883          subscribes to commands
  subscribes telemetry         TLS port 8883                    publishes telemetry
                                                                      |
                                                               USB Serial 115200
                                                                      |
                                                              [Arduino Nano]
```

### MQTT Topics

| Topic                    | Publisher      | Subscriber     | Payload              |
|--------------------------|----------------|----------------|----------------------|
| `agc/vehicle/cmd`        | Remote PC      | Mini Computer  | JSON command         |
| `agc/vehicle/telemetry`  | Mini Computer  | Remote PC      | JSON telemetry       |
| `agc/vehicle/status`     | Mini Computer  | Remote PC      | JSON status/heartbeat|

### Command Payload (PC → vehicle)
```json
{
  "thr": 1700,
  "dir": 1600,
  "steer": 1400,
  "arm": 2000
}
```
Values are RC-equivalent µs (1000–2000). `thr` = speed magnitude, `dir` = forward/reverse selection. Mini computer injects these into Arduino via Serial.

### Telemetry Payload (vehicle → PC)
```json
{
  "thr": 1700,
  "arm": 2000,
  "steer": 1400,
  "deg": 12.5,
  "armed": true,
  "ts": 1747123456789
}
```

### Arduino Firmware Addition Needed
The Arduino currently only reads from the physical RC receiver. When the mini computer is in control, it must inject commands via Serial. A Serial command parser needs to be added to the firmware:

```
Serial input format:  CMD:<thr>,<dir>,<steer>,<arm>\n
Example:              CMD:1700,1600,1400,2000\n
Response:             ACK\n
```

When a valid `CMD:` line is received, the parsed values override the PCINT2 RC values. A timeout (e.g., 500ms without a CMD) falls back to the physical RC receiver (failsafe to hardware).

### Mini Computer Agent (Python)
File: `agent/agent.py` (to be created)

```python
# Pseudocode structure
serial_port = Serial('/dev/ttyUSB0', 115200)
mqtt_client.connect(BROKER, 8883, tls=True)
mqtt_client.subscribe('agc/vehicle/cmd')

def on_command(msg):
    cmd = json.loads(msg.payload)
    serial_port.write(f"CMD:{cmd['thr']},{cmd['steer']},{cmd['arm']}\n")

def serial_reader_loop():
    while True:
        line = serial_port.readline()  # "RC thr=... -> deg=..."
        telemetry = parse_telemetry(line)
        mqtt_client.publish('agc/vehicle/telemetry', json.dumps(telemetry))
```

### Broker Options
| Option | Cost | Latency | Setup |
|--------|------|---------|-------|
| HiveMQ Cloud (free tier) | Free, 10 connections | ~50ms | 5 min, no server needed |
| Mosquitto on VPS | ~$5/mo VPS | ~20ms | Need to set up server |
| AWS IoT Core | Pay per message | ~30ms | Complex IAM setup |

**Recommended to start**: HiveMQ Cloud free tier — zero infrastructure, works immediately.

### Safety Rules for Remote Control
1. If MQTT command stream stops for >500ms → Arduino falls back to physical RC (or disarms)
2. Physical RC arm switch always takes priority (hardware override)
3. Max steering angle and speed limits enforced in Arduino firmware, not in agent
4. Emergency stop: publish `{"arm": 1000}` to `agc/vehicle/cmd`

---

## Implementation Phases

### Phase 1 — Remote Control (current focus)
- [ ] Add `CMD:` Serial parser to Arduino firmware with RC fallback timeout
- [ ] Write `agent/agent.py` (Serial ↔ MQTT bridge)
- [ ] Set up HiveMQ Cloud broker + credentials
- [ ] Write minimal web dashboard (HTML + JS + MQTT.js) for joystick input
- [ ] Test end-to-end: PC joystick → MQTT → agent → Arduino → vehicle moves

### Phase 2 — Telemetry & Monitoring
- [ ] Parse all Arduino Serial output in agent, publish as structured JSON
- [ ] Dashboard shows: armed state, speed, steering angle, CAN heartbeat status
- [ ] Store telemetry to SQLite on mini computer (offline buffer)

### Phase 3 — Autonomous Mode (future)
- [ ] Mission planner on mini computer sends CMD messages locally (no MQTT needed)
- [ ] GNSS/RTK integration for position feedback
- [ ] Obstacle detection (ultrasonic/LiDAR) with safety supervisor
- [ ] Remote monitoring dashboard shows live vehicle position on map

---

## Source Files
- `Motion/Steering_System/src/main.cpp` — Arduino firmware (PlatformIO, current location)
- `agent/agent.py` — mini computer MQTT↔Serial bridge (to be created)
- `Schematic_shield-can_2026-04-29.svg` — full PCB schematic
- `firmware_backup.hex` — binary dump from original working Arduino
- `PBL26-130526-1458-3887 (1).pdf` — full project specification document

## Build Tools
- PlatformIO (Arduino firmware)
- avrdude: `C:\Users\user\.platformio\packages\tool-avrdude\avrdude.exe`
- Flash read (57600 baud, CH340 chip on COM12):
  ```
  avrdude -C "%USERPROFILE%\.platformio\packages\tool-avrdude\avrdude.conf" -p atmega328p -c arduino -P COM12 -b 57600 -U flash:r:firmware_backup.hex:i
  ```
