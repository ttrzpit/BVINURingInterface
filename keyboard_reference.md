# NURing Keyboard Command Reference

All commands are typed into the **operator display window** (click it to give it focus),
then confirmed with **Enter**. The current input is shown in the **Input** row of the
System Information panel. The result appears in the **Output** row.

---

## System State Commands

| Command | Result |
|---------|--------|
| `cal`  | Enter **CALIBRATING** state — shows full ArUco marker grid on touchscreen |
| `cal3` | Enter **CAL3** state — Stage 3 offset calibration (10 finger touches) |
| `idle` | Return to **IDLE** state — hides the touchscreen grid |
| `fitts`| Enter **FITTS** task state — shows full grid on touchscreen; use `r` to select a target |

---

## Fitts Task Commands

| Command | Active In | Result |
|---------|-----------|--------|
| `r` | FITTS only | Randomly select a new target marker (1–45) — only that marker is shown |

---

## ArUco Marker Tracking

| Command | Result |
|---------|--------|
| `a01` – `a45` | Set the **active tag** to the given marker ID — a green outline appears around it in the operator display |
| `a00` | Clear the active tag — removes the green outline |

---

## System

| Key | Result |
|-----|--------|
| `ESC` | Quit the program cleanly |
| `Backspace` | Delete the last character while typing a command |

---

## System States

| State | Touchscreen | Telemetry Colour |
|-------|-------------|-----------------|
| `IDLE`        | Blank                                   | Gray   |
| `CALIBRATING` | Full ArUco grid                         | Yellow |
| `CAL3`        | Full ArUco grid — touch anywhere 10×    | Orange |
| `FITTS`       | Single randomly selected target marker  | Green  |

## CAL3 Flow

1. Type `cal3` + Enter
2. Touch fingertip firmly to the touchscreen and **hold ~200 ms**
3. Release — the telemetry Output row confirms the sample and shows the computed offset
4. Wait 2 seconds (cooldown), then repeat from step 2
5. After 10 samples the final averaged `offset_cam_to_fingertip` is printed to the terminal
6. Type `idle` to return to normal operation
