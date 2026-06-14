# === NURing Keyboard Command Reference =====================================================================
## All commands are typed into the **operator display window**, with each command processed immediately. This  
## means that "Enter" does not have to be pressed on they keyboard to confirm input, with the exception of 
## numerical inputs, such as those formatted as `n.n` or `nn` or `nnnnn` below. Those need "Enter" pressed 
## to confirm the numerical input. The "Keyboard Inputs" in the DisplayHandler should display the inputs in 
## the following manner: 
## - Text being typed before being processed (essentially the buffer) should be displayed in [INPUT_BUFFER] 
## - The last input should be displayed in the [LAST_INPUT] box
## - The "Display Text" from below should be displayed in the [DISPLAY_TEXT] box 

### Description of input table elements
Command                 Keyboard input command
KeyID                   ASCII-based DEC value corresponding to keyboard input
Description             Text-based input description 
Required Input State    Required previous input state for valid keyboard input
New Input State         New input state for next input command
Display Text            Text to display after command entered


## SYSTEM TOP-LEVEL INPUTS ["SYSTEM"]
 Command   | KeyID    | Description                             | Required Input State       | New Input State         | Display Text            
-----------|----------|-----------------------------------------|----------------------------|-------------------------|-------------------------------------------------------------------------------
 `ESC`     | 27       | "Quit the program cleanly"              | ANY                        | NONE                    | "Exiting.""
 `SPACE`   | 32       | "Clear / cancel input"                  | ANY                        | IDLE                    | "Input cleared."
 `grave`   | 96       | "Exit task, return system to idle"      | ANY                        | IDLE                    | "System cleared, returning to IDLE state."
 `e`       | 101      | "Disable READY/GUIDING, return to IDLE" | ANY                        | (same as previous)      | "Returning to IDLE — guidance/ready disabled."
 `E`       | 69       | "Enable READY (if tensioning complete)" | ANY                        | (same as previous)      | "READY enabled — preload tension active." or "Cannot enter READY: pretensioning not complete."
 `k`       | 107      | "Toggle K(theta) stiffness gain"        | ANY                        | (same as previous)      | "Stiffness gain enabled." or "Stiffness gain disabled." (see Note3)
**Note1** They command `grave` refers to the "grave" character on the "tilde" key
**Note2** `e`/`E` control the RobotState ladder (DISCONNECTED/IDLE/READY/GUIDING, shown in the telemetry panel's "Teensy" status cell) and do not change the InputState/menu — they work the same regardless of what else is on screen. While READY/GUIDING, the PC sends `PcState::READY` ('R') to the Teensy, which echoes back `TeensyState::READY` ('R') in the "Teensy State" telemetry cell; while DISCONNECTED/IDLE it sends `PcState::IDLE` ('I').
**Note3** `k` toggles whether ControllerHandler's Stage 1 PID uses the per-heading stiffness profile K(theta) (from stiffness calibration, see CALIBRATION Note2) in place of the fixed `gain_kP`, for performance comparisons. Works regardless of the current InputState/menu. If no K(theta) profile has been computed yet, the toggle still flips `stiffnessGainEnabled_` but has no effect on the controller until one exists.



## CALIBRATION INPUTS ["CALIBRATION"]
 Command   | KeyID    | Description                             | Required Input State       | New Input State         | Display Text
-----------|----------|-----------------------------------------|----------------------------|-------------------------|-------------------------------------------------------------------------------
 `C`       | 67       | "Enter calibration mode"                | IDLE                       | CAL_SEL                 | "Select calibration mode: [a] ARoM, [s] Stiffness, [o] Offset..."
 `a`       | 97       | "Start ARoM calibration"                | CAL_SEL   	               | CAL_ROM		 	           | (see Note1 — status text comes from Cal1Handler)
 `s`       | 115      | "Start stiffness calibration"           | CAL_SEL		                 | CAL_STI               	 | (see Note2 — status text comes from Cal2Handler)
 `o`       | 111      | "Start fingertip offset calibration"    | CAL_SEL		                 | CAL_OFF		   	         | "Running fingertip offset calibration."
 `grave`   | 96       | "Exit ARoM calibration, return to IDLE" | CAL_ROM                    | IDLE                    | "System cleared, returning to IDLE state."
 `grave`   | 96       | "Exit stiffness calibration, return to IDLE" | CAL_STI               | IDLE                    | "System cleared, returning to IDLE state."
**Note1** While in CAL_ROM, ControllerHandler PWM output is enabled (preload tension held, no active target) and `Cal1Handler` records the virtual fingertip position for 10 s while the participant traces circles at the edge of comfortable reach. The [DISPLAY_TEXT] box shows `Cal1Handler::GetStatus()`: a countdown ("AROM: Trace circles with your finger -- N.Ns remaining.") followed by "AROM calibration complete (<N> samples)." once the 95th-percentile/periodic-cubic-spline boundary has been computed. Output is disabled again on exiting CAL_ROM.
**Note2** While in CAL_STI, ControllerHandler PWM output is enabled and put into calibration force mode (Stage 1 PID bypassed; `Cal2Handler` drives an open-loop force command). For each of the 12 `CONSTANT_CALIBRATION_ANGLES_DEG` headings, the commanded force ramps up at `cal2.force_ramp_rate` [N/s] along that heading until either the AROM boundary (from CAL_ROM) is reached or `force_max` is hit, holds for `cal2.hold_secs` — fitting a stiffness K(theta) = dF/dphi from the ramp-up samples — then releases immediately (no ramp-down, since friction prevents the finger returning to center during a slow release) and waits `cal2.release_wait_secs` before starting the next heading. The [DISPLAY_TEXT] box shows `Cal2Handler::GetStatus()`, e.g. "Stiffness 3/12 (60.0 deg): holding -- 1.5 N" or "Stiffness 3/12 (60.0 deg): released -- waiting". Requires CAL_ROM to have been completed first (`Cal2Handler::Reset()` reports "Stiffness: run ARoM calibration first." otherwise). Once all 12 headings are done, the resulting K(theta) profile is pushed into ControllerHandler (`SetStiffnessProfile()`) and shown in the controller panel; output is disabled again on exiting CAL_STI. See SYSTEM Note3 for the `k` key that toggles whether this profile is applied.



## PRETENSIONING / TENSION INPUTS ["PRETENSION" / "TENSION_ADJUST"]
 Command   | KeyID    | Description                                  | Required Input State       | New Input State         | Display Text            
-----------|----------|-----------------------------------------------|----------------------------|-------------------------|-------------------------------------------------------------------------------
 `T`       | 84       | "Open tensioning menu"                         | ANY                        | TEN_MENU                | "Tensioning: [p] guided pretensioning sequence, or [a/b/c/d] adjust tension directly."

### Guided pretensioning sequence (TEN_MENU -> PRE_TENSION -> ... -> DONE)
 `p`       | 112      | "Start guided pretensioning sequence"          | TEN_MENU                   | PRE_TENSION             | (see Note1 — status text comes from PretensionHandler)
 `Enter`   | 13, 10   | "Advance the guided pretension step"           | PRE_TENSION                | (same as previous)      | (see Note1)
 `a`       | 97, 185  | "Select motor A to adjust preload tension"     | PRE_TENSION or TEN_SEL_[B/C/ALL] | TEN_SEL_A          | (see Note1)
 `b`       | 98, 183  | "Select motor B to adjust preload tension"     | PRE_TENSION or TEN_SEL_[A/C/ALL] | TEN_SEL_B          | (see Note1)
 `c`       | 99, 178  | "Select motor C to adjust preload tension"     | PRE_TENSION or TEN_SEL_[A/B/ALL] | TEN_SEL_C          | (see Note1)
 `d`       | 100, 181 | "Select all motors to adjust preload tension"  | PRE_TENSION or TEN_SEL_[A/B/C]   | TEN_SEL_ALL        | (see Note1)
 `+`       | 61, 171  | "Increase [MOTOR] preload tension by 0.1 N"    | TEN_SEL_[MOTOR]            | (same as previous)      | (see Note1)
 `-`       | 45, 173  | "Decrease [MOTOR] preload tension by 0.1 N"    | TEN_SEL_[MOTOR]            | (same as previous)      | (see Note1)
 `n.n`     | [NUM]    | "Buffer an absolute preload tension value"     | TEN_SEL_[MOTOR]            | (same as previous)      | (see Note2)
 `Enter`   | 13, 10   | "Confirm buffered n.n -> [MOTOR] tension [N]"  | TEN_SEL_[MOTOR], buffer full | (same as previous)    | (see Note1)
 `Enter`   | 13, 10   | "Advance step 3/4 -> 4/4 (record home pose)"   | TEN_SEL_[MOTOR], buffer empty | (same as previous)   | (see Note1)

### Standalone tension adjustment (TEN_MENU -> TEN_ADJ_*, no guided sequence)
 `a`       | 97, 185  | "Adjust motor A tension directly"              | TEN_MENU or TEN_ADJ_[B/C/ALL]    | TEN_ADJ_A          | (see Note6)
 `b`       | 98, 183  | "Adjust motor B tension directly"              | TEN_MENU or TEN_ADJ_[A/C/ALL]    | TEN_ADJ_B          | (see Note6)
 `c`       | 99, 178  | "Adjust motor C tension directly"              | TEN_MENU or TEN_ADJ_[A/B/ALL]    | TEN_ADJ_C          | (see Note6)
 `d`       | 100, 181 | "Adjust all motors' tension directly"          | TEN_MENU or TEN_ADJ_[A/B/C]      | TEN_ADJ_ALL        | (see Note6)
 `+`       | 61, 171  | "Increase [MOTOR] tension by 0.1 N"            | TEN_ADJ_[MOTOR]            | (same as previous)      | (see Note6)
 `-`       | 45, 173  | "Decrease [MOTOR] tension by 0.1 N"            | TEN_ADJ_[MOTOR]            | (same as previous)      | (see Note6)
 `n.n`     | [NUM]    | "Buffer an absolute tension value"             | TEN_ADJ_[MOTOR]            | (same as previous)      | (see Note2)
 `Enter`   | 13, 10   | "Confirm buffered n.n -> [MOTOR] tension [N]"  | TEN_ADJ_[MOTOR], buffer full | (same as previous)    | (see Note6)
 `grave`   | 96       | "Exit tension adjustment, return to IDLE"      | TEN_ADJ_[MOTOR]            | IDLE                    | "System cleared, returning to IDLE state."
 `T`       | 84       | "Exit tension adjustment, reopen tensioning menu" | TEN_ADJ_[MOTOR]         | TEN_MENU                | "Tensioning: [p] guided pretensioning sequence, or [a/b/c/d] adjust tension directly."

**Note1** The [DISPLAY_TEXT] box shows PretensionHandler::GetStatus(), which steps through 4 phases:
  - 1/4 "Tension 1/4: Unspool all tendons fully, then press Enter."
  - 2/4 "Tension 2/4: Zeroing motor encoders..."
  - 3/4 "Tension 3/4: Select motor [a,b,c,d], [+/-], [n.n]; press Enter to save." (shown while ControllerHandler is in manual tension mode)
  - 4/4 "Tension 4/4: Home position recorded."
**Note2** For command `n.n`, this represents a value from 0.0 N to 10.0 N, always entered with one digit, a period, and one digit (e.g., 0.3, 1.0, 7.5). Digits are buffered in [INPUT_BUFFER] until `Enter` confirms.
**Note3** Steps 1/4 -> 2/4 are advanced by pressing `Enter`; step 2/4 -> 3/4 happens automatically after a short settle delay (`Enter` is ignored during step 2/4). During step 3/4, `[a]/[b]/[c]/[d]` select which motor(s) the `+`/`-`/`n.n` commands act on ('d' = all three motors together); a "bare" `Enter` (nothing typed) advances 3/4 -> 4/4.
**Note4** During step 2/4 the PC sends `PcState::ZERO_ENC` so the Teensy zeroes its motor encoders. During step 3/4, ControllerHandler PWM output is enabled and put into manual tension mode: tension setpoints [N] come directly from the `[a]/[b]/[c]/[d]` + `+`/`-`/`n.n` commands above (seeded to `tension_min` on entry) and are converted to PWM via the normal tension->current->PWM pipeline, visible live in the controller display's Tendon/Motor State table. On entering step 4/4, those per-motor tensions are captured as the new preload (`ControllerHandler::SetPreloadTensions()`), manual tension mode is turned off, and PWM output stays enabled — the RobotState ladder automatically requests READY (as if `E` were pressed), so the captured preload tensions continue to be held once PRETENSION is exited (e.g. via `SPACE`). If `grave` aborts the sequence before step 4/4, output and manual tension mode are disabled immediately and no READY request is made (preload remains whatever it was previously).
**Note5** For commands with multiple KeyIDs, either key ID is a valid option for that command. The keyID [NUM] means any acceptable value entered via keyboard number row or numpad.
**Note6** While in TEN_ADJ_[MOTOR], the [DISPLAY_TEXT] box shows PretensionHandler::GetTensionAdjustStatus(), updated live every frame: "Tension adjust: A=x.xxN  B=x.xxN  C=x.xxN -- [a/b/c/d] select motor, +/- = +/-0.1N, n.n+Enter = set value. Press [grave] or [T] to exit." On first entering TEN_ADJ_* from TEN_MENU, ControllerHandler PWM output is enabled and put into manual tension mode (seeded to `tension_min` on all three motors); both are disabled again on exit — independent of the guided pretensioning sequence above (no unspool/zero/home-recording steps).



## GAIN TUNING INPUTS ["GAIN_ALL" / "GAIN_A" / "GAIN_B" / "GAIN_C"]
 Command   | KeyID    | Description                                     | Required Input State       | New Input State         | Display Text            
-----------|----------|-------------------------------------------------|----------------------------|-------------------------|-------------------------------------------------------------------------------
 `G`       | 71       | "Open gain tuning (all motors)"                  | ANY                        | GAIN_ALL                | "Gain tuning: [+/-] adjust all motors, or [a/b/c/d] select a motor."
 `a`       | 97, 185  | "Tune motor A's direction-dependent gain boost"  | GAIN_ALL or GAIN_[B/C]     | GAIN_A             | (see Note1)
 `b`       | 98, 183  | "Tune motor B's direction-dependent gain boost"  | GAIN_ALL or GAIN_[A/C]     | GAIN_B             | (see Note1)
 `c`       | 99, 178  | "Tune motor C's direction-dependent gain boost"  | GAIN_ALL or GAIN_[A/B]     | GAIN_C             | (see Note1)
 `d`       | 100, 181 | "Tune all motors' gain boost together"           | GAIN_[A/B/C]               | GAIN_ALL           | (see Note1)
 `+`       | 61, 171  | "Increase [MOTOR] gain boost by 0.1"             | GAIN_ALL or GAIN_[A/B/C]   | (same as previous)      | (see Note1)
 `-`       | 45, 173  | "Decrease [MOTOR] gain boost by 0.1"             | GAIN_ALL or GAIN_[A/B/C]   | (same as previous)      | (see Note1)
 `grave`   | 96       | "Exit gain tuning, return to IDLE"               | GAIN_ALL or GAIN_[A/B/C]   | IDLE                    | "System cleared, returning to IDLE state."
**Note1** The [DISPLAY_TEXT] box shows `ControllerHandler::GetGainTuneStatus()`, updated live every frame: "Gain tune (added to kP_effective): A=x.xx  B=x.xx  C=x.xx -- [a/b/c/d] select motor, +/- = +/-0.1. Press [grave] to exit." Each of `gainTune_A/B/C` (default 0.0, clamped to [0.0, 5.0]) is an extra K(theta)-style boost centered on that motor's direction (35/145/270 deg) and periodically interpolated the same way as the stiffness profile K(theta) (see CALIBRATION Note2); the interpolated result is ADDED to `kP_effective` in Stage 1 — on top of K(theta) (if enabled) or `gain_kP` — before `force_x_`/`force_y_` are computed. 'd'/GAIN_ALL adjusts all three motors' boosts together. Does not enable/disable PWM output or manual tension mode — normal operation (and any active FITTS task) continues while tuning. The per-motor values are also shown live in the controller panel's "Gain kP" row.



## LOGGING INPUTS ["LOGGING"]
 Command   | KeyID    | Description                             | Required Input State       | New Input State         | Display Text            
-----------|----------|-----------------------------------------|----------------------------|-------------------------|-------------------------------------------------------------------------------
 `L`       | 76       | "Enter logging mode"                    | ANY                        | LOG                     | "Select logging option: [u] Set User ID..."
 `u`       | 117      | "Set user ID"                           | LOG                        | LOG_UID                 | "Enter ID for user (000-999)..."
 `nnn`     | [NUM]    | "Set user ID value (000-999)"           | LOG_UID 	                  | LOG                 | "User ID set to [VAL]."
**Note1** For command `nnn`, this represents a 3-digit value from 000 to 999, always entered with three digits (e.g., 001, 104, 204)



## MOTOR PWM TEST INPUTS ["PWM_TEST"]
 Command   | KeyID    | Description                             | Required Input State       | New Input State         | Display Text            
-----------|----------|-----------------------------------------|----------------------------|-------------------------|-------------------------------------------------------------------------------
 `M`       | 77       | "Enter motor test mode"                 | ANY                        | MOT_PWM                 | "Select motor to test: [a] Motor A, [b] Motor B, [c] Motor C, [d] All Motors..."
 `a`       | 97, 185  | "Send PWM value to motor A for 1s"      | MOT_PWM                    | MOT_PWM_A               | "Enter PWM for Motor A (0-2047)..."
 `b`       | 98, 183  | "Send PWM value to motor B for 1s"      | MOT_PWM                    | MOT_PWM_B               | "Enter PWM for Motor B (0-2047)..."
 `c`       | 99, 178  | "Send PWM value to motor C for 1s"      | MOT_PWM                    | MOT_PWM_C               | "Enter PWM for Motor C (0-2047)..."
 `d`       | 100, 181 | "Send PWM value to all motors for 1s"   | MOT_PWM                    | MOT_PWM_ALL             | "Enter PWM for Motor A, B, C (0-2047)..."
 `nnnn`    | [NUM]    | "Set PWM value (0 to 2047) to [MOTOR]"  | MOT_PWM_[MOTOR]	         | MOT_PWM                 | "Sending test pulse to motor [MOTOR]."
**Note1** For command `nnnn`, this represents a 4-digit value from 0000 to 2047, always entered with four digits (e.g., 0001, 0104, 2040)
**Note2** The index [MOTOR] refers to motor A, B, C, or all motors, depending on previous "Required State"
**Note2** The keyID [NUM] means any acceptable value entered via keyboard number row or numpad
**Note4** For commands with multiple KeyIDs, either key ID is a valid option for that command



## SERIAL INPUTS ["SERIAL"]
 Command   | KeyID    | Description                             | Required Input State       | New Input State         | Display Text            
-----------|----------|-----------------------------------------|----------------------------|-------------------------|-------------------------------------------------------------------------------
 `S`       | 83       | "Toggle teensy serial connection"       | ANY                        | (same as previous)      | "PC-->Teensy connection established." or "PC-->Teensy connection disabled."
**Note1** This key can be entered at any time, toggling the Serial connection on or off; does not change state
**Note2** If serial is being toggled off, PC must send a "disable output" packet and wait for confirmation from the Teensy 
**Note3** Display text will depend on whether Teensy successfully acknowledges connection made



## TASK 1 - FITTS-STYLE ACCURACY TASK INPUTS ["ACCURACY"]
 Command   | KeyID    | Description                             | Required Input State       | New Input State         | Display Text            
-----------|----------|-----------------------------------------|----------------------------|-------------------------|-------------------------------------------------------------------------------
 `F`       | 70       | "Enter guidance accuracy mode"          | IDLE                       | FIT_SEL                 | "Select marker mode: [r] Random, [m] Manual..."
 `m`       | 109      | "Manually set active marker (1 to 45)"  | FIT_SEL or FIT_RUN         | FIT_ACT                 | "Which marker (1-45)..."
 `r`       | 114      | "Randomly set active marker (1 to 45)"  | FIT_SEL                    | FIT_RUN                 | "Active marker set to [MARKER_ID]."
 `nn`      | [NUM]    | NONE                                    | FIT_ACT                    | FIT_RUN                 | "Active marker set to [MARKER_ID]."
**Note1** For command `nn`, this represents a 2-digit value from 00 to 45, always entered with two digits (e.g., 01, 04, 45) 
**Note2** The keyID [NUM] means any acceptable value entered via keyboard number row or numpad
