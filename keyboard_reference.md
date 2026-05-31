# NURing Keyboard Command Reference

All commands are typed into the **operator display window** (click it to give it focus),
then confirmed with **Enter**. The current input is shown in the **Input** row of the
System Information panel. The result appears in the **Output** row.

---

## System State Commands

| Command | Result |
|---------|--------|
| `cal` | Enter **CALIBRATING** state — shows full ArUco marker grid on touchscreen |
| `idle` | Return to **IDLE** state — hides the touchscreen grid |
| `fitts` | Enter **FITTS** task state — shows full grid on touchscreen; use `r` to select a target |

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
| `IDLE` | Blank | Gray |
| `CALIBRATING` | Full 9×5 ArUco grid | Yellow |
| `FITTS` | Single randomly selected target marker | Green |
