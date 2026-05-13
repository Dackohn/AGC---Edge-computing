# Wiring Diagram / Conexiuni

## 1. RC Receiver (Receptor Radio)
| RC Channel Function | Arduino Nano Pin | Descriere |
|---------------------|------------------|-----------|
| **Throttle** (Acceleratie) | **D2** | Semnal PWM intrare (Interrupt INT0) |
| **Switch** (Armare) | **D3** | Semnal PWM intrare (Interrupt INT1) |
| **Steering** (Directie) | **D5** | Semnal PWM intrare (PulseIn) |
| GND | GND | Masa comuna obligatorie |
| VCC | 5V | Alimentare receptor |

## 2. MCP2515 CAN Module (Directie Motor CAN)
| MCP2515 Pin | Arduino Nano Pin | Descriere |
|-------------|------------------|-----------|
| **CS** | **D10** | Chip Select (SPI) |
| **INT** | **D4** | Interrupt (Modificat din D2 pentru a evita conflictul) |
| **SCK** | **D13** | SPI Clock |
| **SI** (MOSI) | **D11** | SPI Data In |
| **SO** (MISO) | **D12** | SPI Data Out |
| VCC | 5V | Alimentare |
| GND | GND | Masa |

## 3. Traction Motor & Safety (Motor Tractiune)
| Component | Arduino Nano Pin | Descriere |
|-----------|------------------|-----------|
| **PWM Input** (Motor Driver) | **D9** | Semnal PWM 1kHz (Timer1) |
| **Relay IN** (Releu Siguranta) | **D8** | Activare Releu (HIGH = ON) |

## Note Importante
1. **GND Comun**: Asigura-te ca GND-ul de la Arduino, Receptorul RC, Driverul de Motor si Modulul CAN sunt toate conectate impreuna.
2. **Conflict Pini**: Pinul D2 a fost mutat la Throttle, deci firul de INT de la CAN trebuie mutat fizic pe D4.
