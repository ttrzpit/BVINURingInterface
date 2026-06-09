# NURing Keyboard Command Reference

All commands are typed into the **operator display window** (click it to give it focus),
then confirmed with **Enter**. The current input is shown in the **Input** row of the
System Information panel. The result appears in the **Output** row.


---


## System State Commands

| Command               | Result                                                                                                  |
|-----------------------|---------------------------------------------------------------------------------------------------------|
| `cal`                 | Enter **CALIBRATING** state — shows full ArUco marker grid on touchscreen                               |
| `cal1`                | Enter **CAL1** state — Stage 1, active range of motion capture                                          |
| `cal2`                | Enter **CAL2** state — Stage 2, stiffness measurement                                                   |
| `cal3`                | Enter **CAL3** state — Stage 3, fingertip to camera offset calibration (10 finger touches)              |
| `idle`                | Return to **IDLE** state — hides the touchscreen grid                                                   |
| `study1`   `fitts`    | Enter **FITTS** task state — shows full grid on touchscreen; use `r` to select a target                 |


---


## Study 1 (Fitts) Task Commands

| Command               | Result                                                                                                  |
|-----------------------|---------------------------------------------------------------------------------------------------------|
| `r`                   |  Randomly select a new target marker (1–45) — only that marker is shown                                 |
| `r00`                 |  Set a new target marker (1-45), only that marker is shown                                              |


---


## Amplifier Commands

| Command               | Result                                                                                                  |
|-----------------------|---------------------------------------------------------------------------------------------------------|
| `testAnnnn`           | Send a test PWM signal to amplifier A with PWM = nnnn for 1 second                                      |
| `testBnnnn`           | Send a test PWM signal to amplifier B with PWM = nnnn for 1 second                                      |
| `testCnnnn`           | Send a test PWM signal to amplifier C with PWM = nnnn for 1 second                                      |


---




## ArUco Marker Tracking

| Command               | Result                                                                                                  |
|-----------------------|---------------------------------------------------------------------------------------------------------|
| `a01` – `a45`         | Set the **active tag** to the given marker ID                                                           |
| `a00`                 | Clear the active tag — removes the green outline                                                        |


---


## Serial (Teensy)

| Command               | Result                                                                                                  |
|-----------------------|---------------------------------------------------------------------------------------------------------|
| `con`    `connect`    | Open `/dev/ttyACM0` and connect to the Teensy                                                           |
| `dis`    `disconnect` | Close the serial connection gracefully                                                                  |

The port is **not** opened at startup — type `con` or `connect` when the Teensy is ready. The program runs normally without it.


---


## System

| Command               | Result                                                                                                  |
|-----------------------|---------------------------------------------------------------------------------------------------------|
| `ESC`                 | Quit the program cleanly                                                                                |
| `Backspace`           | Delete the last character while typing a command                                                        |


---


## System States

| State              | Touchscreen Display                        | Telemetry Colour |
|--------------------|--------------------------------------------|------------------|
| `IDLE`             | Blank                                      | Gray             |
| `CALIBRATING`      | Full ArUco grid                            | Yellow           |
| `CAL3`             | Full ArUco grid — touch anywhere 10×       | Orange           |
| `FITTS`            | Single randomly selected target marker     | Green            |


## CAL3 Flow

1. Type `cal3` + Enter
2. Touch fingertip firmly to the touchscreen and **hold ~200 ms**
3. Release — the telemetry Output row confirms the sample and shows the computed offset
4. Wait 2 seconds (cooldown), then repeat from step 2
5. After 10 samples the final averaged `offset_cam_to_fingertip` is printed to the terminal
6. Type `idle` to return to normal operation
