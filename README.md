Plik `.ino` do budowy sterownika napełniania wody z zaworem odcinającym.

- `sterownik_napelniania.ino` – wersja Arduino Nano.
- `sterownik_napelniania_esp32_mcp2515.ino` – wersja pre-ISOBUS pod ESP32 Wemos + MCP2515 (bez LCD, sterowanie z VT/CAN).

## Pre-ISOBUS: pinout (ESP32 Wemos + MCP2515)

- MCP2515: `SCK=GPIO18`, `MISO=GPIO19`, `MOSI=GPIO23`, `CS=GPIO5`, `INT=GPIO4`
- I/O sterownika: `FLOW=GPIO27`, `VALVE=GPIO26`, `PLUS=GPIO32`, `MINUS=GPIO33`, `START=GPIO25`, `STOP/RESET=GPIO14`, `BUZZER=GPIO12`
- Zasilanie: 12V (układ wykonawczy), ESP32/MCP2515 zasilane zgodnie z modułami.

## Pre-ISOBUS: mapowanie CAN (250 kb/s, extended 29-bit)

PGN-y (proprietary):
- `0x00EF10` – komendy do ECU
- `0x00EF11` – status ECU (cyklicznie)
- `0x00EF12` – konfiguracja ECU (cyklicznie)

Komendy `0x00EF10` (`data[0]` = CMD):
- `0x01 SET_TARGET_L_X10` (`data[1..2]` little-endian, 0.1 L)
- `0x02 START`
- `0x03 STOP`
- `0x04 RESET_COUNTER` (tylko poza nalewaniem)
- `0x05 SET_STEP_L` (`data[1]`, 5..100)
- `0x06 SET_VALVE_POL` (`data[1]`: 0/1)
- `0x07 SET_IMP_PER_L` (`data[1..2]`, 10..10000)
- `0x08 SET_TANK_CAP_L` (`data[1..2]`, 1..10000)
- `0x09 SET_NO_FLOW_TIMEOUT_S` (`data[1]`, 1..120)
- `0x0A SAVE_CONFIG`

Status `0x00EF11`:
- `data[0..1]` = nalano (0.1 L, LE)
- `data[2..3]` = cel (0.1 L, LE)
- `data[4]` flags: bit0=`VALVE_OPEN`, bit1=`IN_PROGRESS`, bit2=`NO_FLOW`
- `data[5]` = krok celu [L]

Konfiguracja `0x00EF12`:
- `data[0..1]` = imp/l (LE)
- `data[2..3]` = pojemność zbiornika [L] (LE)
- `data[4]` = timeout braku przepływu [s]
- `data[5]` = polaryzacja zaworu (0/1)
- `data[6]` = krok celu [L]
