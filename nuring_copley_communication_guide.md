# NURing Teensy — Copley Amplifier Communication Reference

This document describes how the Teensy 4.1 communicates with three Copley Nano
NES-090-10-Z amplifiers, and the optimal strategy for reading motor position
and current as fast as possible. Use this alongside the existing _TEENSY.zip
codebase as the structural reference for the new firmware.

---

## Hardware Setup

- **Three amplifiers** (A, B, C), each on a dedicated Teensy hardware serial port:
  - Motor A → `Serial5` (HWSerialA)
  - Motor B → `Serial4` (HWSerialB)
  - Motor C → `Serial3` (HWSerialC)
- Because each amplifier has its own serial port, all three can be polled **in parallel**
- PWM output to amplifiers via `analogWrite` on dedicated pins at 12-bit resolution
- Enable/disable via digital pins

## Copley ASCII Protocol

The amplifiers use a simple ASCII command/response protocol over RS-232:
- Commands end with `\r` (carriage return)
- Responses end with `\r`
- Successful get: `v <value>\r`
- Successful set: `ok\r`
- Error: `e <code>\r`
- Protocol is "speak when spoken to" — the amplifier never initiates

### Commands We Use

| Command | Purpose | Response | Value Type |
|---------|---------|----------|------------|
| `g r0x32\r` | Get motor position | `v <int32>\r` | int32, encoder counts |
| `g r0x0c\r` | Get actual current | `v <int16>\r` | int16, units of 0.01 A |
| `s r0x24 3\r` | Set PWM current mode | `ok\r` | — |
| `s r0x90 115200\r` | Set baud rate | `ok\r` (at NEW baud) | — |
| `s r0x17 0\r` | Zero load position | `ok\r` | — |
| `s r0x32 0\r` | Zero motor position | `ok\r` | — |
| `r\r` | Reset amplifier | (no response) | — |

### Important Variable IDs

| Variable | ID | Bank | Description | Units |
|----------|-----|------|-------------|-------|
| Motor position | 0x32 | R | Actual motor position | counts |
| Actual current | 0x0c | R | Measured output current | 0.01 A |
| Desired state | 0x24 | R F | Operating mode (3 = PWM current) | — |
| Baud rate | 0x90 | R | Serial baud rate | baud |
| Peak current limit | 0x21 | R F | Maximum current | 0.01 A |
| Programmed current | 0x02 | R F | Commanded current | 0.01 A |

---

## Baud Rate Strategy

### Problem
The Copley amplifier defaults to **9600 baud** on every power-up or reset. At 9600
baud, a single get command round-trip (send ~8 bytes + receive ~10 bytes = ~18 bytes)
takes approximately 18 ms. Two queries per amplifier (position + current) = ~36 ms,
capping effective poll rate at ~28 Hz per amplifier.

### Solution
After reset, immediately set baud to **115200** on each amplifier. At 115200 baud,
the same round-trip takes ~1.5 ms, allowing poll rates of 300+ Hz per amplifier.

### Baud Rate Change Procedure (per amplifier)
1. At 9600 baud, send: `s r0x90 115200\r`
2. The amplifier responds `ok\r` at the **new** baud rate (so the Teensy won't see it)
3. Wait **100 ms** minimum (Copley requirement)
4. Switch the Teensy's HWSerial port to 115200: `HWSerialX.begin(115200)`
5. Wait another 50 ms before sending commands
6. Verify communication by sending `g r0x90\r` and checking for a response

### Critical Warning from Copley Manual
After sending the baud change command, do NOT send any additional characters at the
old baud rate. Some serial libraries append a line feed (`\n`) after `\r` — make sure
to use `.print()` not `.println()`, and send only the `\r` as the terminator. Extra
characters at 9600 may be misinterpreted as a break command, resetting the baud back
to 9600.

---

## Initialization Sequence

```
1. Power up → amplifiers default to 9600 baud
2. Teensy starts all three HWSerial ports at 9600
3. Reset amplifiers (enable pin: HIGH → LOW → wait 500ms → HIGH)
4. Wait 500 ms for amplifiers to boot
5. For each amplifier (A, B, C):
   a. Send: s r0x90 115200\r          (at 9600 baud)
   b. Wait 150 ms
   c. Switch Teensy HWSerial to 115200
   d. Verify: g r0x90\r → expect v <close to 115200>\r
      (NOTE: Copley sets as close as possible to requested baud,
       so returned value may differ slightly — that's normal)
6. For each amplifier:
   a. Send: s r0x24 3\r               (enable PWM current mode)
   b. Wait for ok\r
7. Enable amplifiers via digital pins
8. Zero motor positions: s r0x32 0\r on each
```

---

## Polling Strategy

### Goal
Read motor position (int32) and actual current (int16) from all three amplifiers
as fast as possible, then send all 6 values to the PC.

### Architecture: Parallel Asynchronous Polling

Since each amplifier has its own serial port, queries to all three run simultaneously.
Use a per-amplifier state machine rather than a queue of strings:

```
Per-amplifier states:
  IDLE          → nothing pending
  SENT_POS      → "g r0x32\r" sent, awaiting response
  RECEIVED_POS  → position parsed, now send current query
  SENT_CUR      → "g r0x0c\r" sent, awaiting response
  RECEIVED_CUR  → current parsed, cycle complete → back to IDLE
```

Each poll cycle:
1. Trigger all three amplifiers simultaneously by setting each to SENT_POS
   and sending `g r0x32\r` on all three ports
2. As each response arrives, parse it, immediately send `g r0x0c\r` (SENT_CUR)
3. As each current response arrives, parse it → IDLE
4. When all three are IDLE, the cycle is complete — all 6 values are fresh

This keeps the serial lines maximally busy. At 115200 baud, a full cycle
(position + current for all 3 motors in parallel) completes in ~3 ms,
enabling 300+ Hz effective polling.

### Timing Configuration
- **HWSerial poll trigger rate**: 200-500 Hz (IntervalTimer, sets flag for main loop)
- **Amplifier PWM write rate**: 1000 Hz (IntervalTimer, direct in ISR)
- **Teensy → PC send rate**: 200-500 Hz (IntervalTimer, sets flag for main loop)

### Response Parsing — Avoid Arduino String

The existing code uses Arduino `String` for building and parsing responses. At high
poll rates this causes heap fragmentation. Replace with fixed char buffers:

```cpp
// Fixed-buffer response parsing
struct AmpQueryState {
    enum State { IDLE, SENT_POS, SENT_CUR };
    State state = IDLE;
    char  responseBuf[16];
    uint8_t responseIdx = 0;

    // Parsed values
    int32_t motorPosition = 0;
    int16_t actualCurrent = 0;
};

// Parse "v 12345\r" into integer
int32_t parseValueResponse(const char* buf) {
    // buf contains "v 12345" (without \r, which triggered the parse)
    if (buf[0] == 'v' && buf[1] == ' ') {
        return strtol(buf + 2, nullptr, 10);
    }
    return 0; // or error flag
}
```

### Reading Responses

In the main loop, check each serial port for available bytes and accumulate
into the fixed buffer. When `\r` is received, parse and advance state:

```cpp
void pollAmplifier(HardwareSerial& port, AmpQueryState& amp) {
    while (port.available()) {
        char c = port.read();
        if (c == '\r') {
            amp.responseBuf[amp.responseIdx] = '\0';

            if (amp.state == AmpQueryState::SENT_POS) {
                amp.motorPosition = parseValueResponse(amp.responseBuf);
                // Immediately request current
                port.print("g r0x0c\r");
                amp.state = AmpQueryState::SENT_CUR;
            }
            else if (amp.state == AmpQueryState::SENT_CUR) {
                amp.actualCurrent = (int16_t)parseValueResponse(amp.responseBuf);
                amp.state = AmpQueryState::IDLE;
            }

            amp.responseIdx = 0;
        }
        else if (amp.responseIdx < sizeof(amp.responseBuf) - 1) {
            amp.responseBuf[amp.responseIdx++] = c;
        }
    }
}
```

### Triggering a New Poll Cycle

```cpp
void startPollCycle(HardwareSerial& portA, HardwareSerial& portB,
                    HardwareSerial& portC,
                    AmpQueryState& ampA, AmpQueryState& ampB,
                    AmpQueryState& ampC) {
    if (ampA.state == AmpQueryState::IDLE &&
        ampB.state == AmpQueryState::IDLE &&
        ampC.state == AmpQueryState::IDLE) {

        portA.print("g r0x32\r");
        ampA.state = AmpQueryState::SENT_POS;
        ampA.responseIdx = 0;

        portB.print("g r0x32\r");
        ampB.state = AmpQueryState::SENT_POS;
        ampB.responseIdx = 0;

        portC.print("g r0x32\r");
        ampC.state = AmpQueryState::SENT_POS;
        ampC.responseIdx = 0;
    }
}
```

---

## Teensy → PC Communication

### Protocol
Use the same binary packet framing as the existing code:
```
[0xAA] [length] [checksum] [payload bytes...] [0x55]
```

### Packet Payload
The outgoing packet must include at minimum:
- Packet type (uint8_t)
- Packet counter (uint16_t)
- Motor position A, B, C (3 × int32_t = 12 bytes)
- Actual current A, B, C (3 × int16_t = 6 bytes)
- Commanded PWM A, B, C (3 × uint16_t = 6 bytes)
- System state / flags (uint8_t)

Total payload: ~28 bytes. At 1 MHz USB serial and 500 Hz, this is negligible.

### PC → Teensy Incoming Packet
Must include:
- Packet type / command (uint8_t)
- Packet counter (uint16_t)
- Commanded PWM A, B, C (3 × uint16_t)
- System state command (uint8_t)

---

## Summary of Changes from Old Code

| Aspect | Old | New |
|--------|-----|-----|
| Baud rate | 9600 (default, never changed) | 115200 (set during init) |
| Poll rate | 20 Hz | 200-500 Hz |
| Query method | String-based queue, sequential per amp | State machine, parallel, fixed buffers |
| Queries per cycle | 6 serial (2 per amp, sequential within queue) | 6 serial (2 per amp, chained within state machine) |
| Round-trip time | ~36 ms per amp | ~3 ms all three in parallel |
| String handling | Arduino String (heap allocation) | Fixed char buffers with strtol |
| Teensy → PC rate | 200 Hz | 200-500 Hz (match poll rate) |

---

## Important Notes

- The amplifier's `g r0x32` returns motor position in **encoder counts** (int32).
  The Maxon ENX 22 MILE encoder has 1024 counts per turn. This is the absolute
  encoder position used for the spool-corrected virtual fingertip mapping.
- The amplifier's `g r0x0c` returns actual measured current in **units of 0.01 A**
  (int16). So a value of 150 means 1.50 A. This is the measured current used
  for the stiffness calibration (force = I × K_t / r_eff).
- PWM output is written via `analogWrite` at 12-bit resolution (0-4095),
  independent of the serial communication. The 1000 Hz PWM write rate does
  not depend on serial poll speed.
- The serial polling and PWM writing are decoupled — PWM runs in an ISR at
  1000 Hz, while serial polling runs in the main loop triggered by a flag
  from an IntervalTimer.
