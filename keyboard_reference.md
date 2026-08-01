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
 `ESC`     | 27       | "Quit the program cleanly"              | ANY                        | NONE                    | "Exiting."
 `SPACE`   | 32       | "E-stop toggle (tension PWM only / full output)" | ANY               | (same as previous)      | "E-STOP engaged - tension PWM only." or "E-STOP released - full guidance output resumed." (see Note4)
 `Delete`  | 255      | "Cancel input / task, return to idle"   | ANY                        | IDLE                    | "Input cleared."
 `grave`   | 96       | "Exit task, return system to idle"      | ANY                        | IDLE                    | "System cleared, returning to IDLE state."
 `e`       | 101      | "Disable guidance output (tension only)"| ANY                        | (same as previous)      | "Amplifier output: tension only (guidance disabled)."
 `E`       | 69       | "Enable guidance output"                | ANY                        | (same as previous)      | "Amplifier output: guidance enabled."
 `k`       | 107      | "Toggle K(theta) stiffness gain"        | ANY                        | (same as previous)      | "Stiffness gain enabled." or "Stiffness gain disabled." (see Note3)
**Note1** They command `grave` refers to the "grave" character on the "tilde" key
**Note2** `e`/`E` toggle whether ControllerHandler's Stage 2 includes the guidance force (force_x_/force_y_) or zero (`ControllerHandler::SetGuidanceOutputEnabled()`), i.e. whether PWM output is tension-only or tension+guidance. They do not change the InputState/menu or the RobotState ladder directly - RobotState (DISCONNECTED/IDLE/READY/GUIDING, shown in the telemetry panel's "Teensy" status cell) is derived from serial connection + `pretension.IsComplete()` + whether guidance is actively contributing to output. While READY/GUIDING, the PC sends `PcState::READY` ('R') to the Teensy, which echoes back `TeensyState::READY` ('R') in the "Teensy State" telemetry cell; while DISCONNECTED/IDLE it sends `PcState::IDLE` ('I').
**Note3** `k` toggles whether ControllerHandler's Stage 1 PID uses the per-heading stiffness profile K(theta) (from stiffness calibration, see CALIBRATION Note2) in place of the fixed `gain_kP`, for performance comparisons. Works regardless of the current InputState/menu. If no K(theta) profile has been computed yet, the toggle still flips `stiffnessGainEnabled_` but has no effect on the controller until one exists.
**Note4** `SPACE` is the e-stop: a single-key toggle of the same guidance-output path as `e`/`E` (`ControllerHandler::SetGuidanceOutputEnabled()`). Engaged, the guidance force is zeroed so the amplifier holds tension PWM only; pressing it again resumes full guidance output. It does not change the InputState/menu, so it can be hit at any time, including mid numeric-entry. The previous SPACE behaviour (clear input -> IDLE) now lives on `Delete` (key code 255 from OpenCV), which is itself a global escape hatch so it cancels back to IDLE from any state, including mid numeric-entry and at the end of the guided pretensioning sequence. `Backspace` (8/127) is separate and edits the numeric input buffer (delete one buffered character), like backspace in a text field.



## CALIBRATION INPUTS ["CALIBRATION"]
 Command   | KeyID    | Description                             | Required Input State       | New Input State         | Display Text
-----------|----------|-----------------------------------------|----------------------------|-------------------------|-------------------------------------------------------------------------------
 `C`       | 67       | "Enter calibration mode"                | IDLE                       | CAL_SEL                 | "Select calibration mode: [a] ARoM, [s] Stiffness, [o] Offset..."
 `a`       | 97       | "Start ARoM calibration"                | CAL_SEL   	               | CAL_ROM		 	           | (see Note1 - status text comes from Cal1Handler)
 `s`       | 115      | "Start stiffness calibration"           | CAL_SEL		                 | CAL_STI               	 | (see Note2 - status text comes from Cal2Handler)
 `o`       | 111      | "Start fingertip offset calibration"    | CAL_SEL		                 | CAL_OFF		   	         | "Running fingertip offset calibration."
 `grave`   | 96       | "Exit ARoM calibration, return to IDLE" | CAL_ROM                    | IDLE                    | "System cleared, returning to IDLE state."
 `grave`   | 96       | "Exit stiffness calibration, return to IDLE" | CAL_STI               | IDLE                    | "System cleared, returning to IDLE state."
**Note1** While in CAL_ROM, ControllerHandler PWM output is enabled (preload tension held, no active target) and `Cal1Handler` records the virtual fingertip position for 10 s while the participant traces circles at the edge of comfortable reach. The [DISPLAY_TEXT] box shows `Cal1Handler::GetStatus()`: a countdown ("AROM: Trace circles with your finger -- N.Ns remaining.") followed by "AROM calibration complete (<N> samples)." once the 95th-percentile/periodic-cubic-spline boundary has been computed. Output is disabled again on exiting CAL_ROM.
**Note2** While in CAL_STI, ControllerHandler PWM output is enabled and put into calibration force mode (Stage 1 PID bypassed; `Cal2Handler` drives an open-loop force command). For each of the 12 `CONSTANT_CALIBRATION_ANGLES_DEG` headings, the commanded force ramps up at `cal2.force_ramp_rate` [N/s] along that heading until either the AROM boundary (from CAL_ROM) is reached or `force_max` is hit, holds for `cal2.hold_secs` - fitting a stiffness K(theta) = dF/dphi from the ramp-up samples - then releases immediately (no ramp-down, since friction prevents the finger returning to center during a slow release) and waits `cal2.release_wait_secs` before starting the next heading. The [DISPLAY_TEXT] box shows `Cal2Handler::GetStatus()`, e.g. "Stiffness 3/12 (60.0 deg): holding -- 1.5 N" or "Stiffness 3/12 (60.0 deg): released -- waiting". Requires CAL_ROM to have been completed first (`Cal2Handler::Reset()` reports "Stiffness: run ARoM calibration first." otherwise). Once all 12 headings are done, the resulting K(theta) profile is pushed into ControllerHandler (`SetStiffnessProfile()`) and shown in the controller panel; output is disabled again on exiting CAL_STI. See SYSTEM Note3 for the `k` key that toggles whether this profile is applied.



## PRETENSIONING / TENSION INPUTS ["PRETENSION" / "TENSION_ADJUST"]
 Command   | KeyID    | Description                                  | Required Input State       | New Input State         | Display Text            
-----------|----------|-----------------------------------------------|----------------------------|-------------------------|-------------------------------------------------------------------------------
 `T`       | 84       | "Open tensioning menu"                         | ANY                        | TEN_MENU                | "Tensioning: [p] guided pretensioning sequence, or [a/b/c/d] adjust tension directly."

### Guided pretensioning sequence (TEN_MENU -> PRE_TENSION -> ... -> DONE)
 `p`       | 112      | "Start guided pretensioning sequence"          | TEN_MENU                   | PRE_TENSION             | (see Note1 - status text comes from PretensionHandler)
 `Enter`   | 13, 10   | "Advance the guided pretension step"           | PRE_TENSION                | (same as previous)      | (see Note1)
 `a`       | 97, 185  | "Select motor A to adjust preload tension"     | PRE_TENSION or TEN_SEL_[B/C/ALL] | TEN_SEL_A          | (see Note1)
 `b`       | 98, 183  | "Select motor B to adjust preload tension"     | PRE_TENSION or TEN_SEL_[A/C/ALL] | TEN_SEL_B          | (see Note1)
 `c`       | 99, 178  | "Select motor C to adjust preload tension"     | PRE_TENSION or TEN_SEL_[A/B/ALL] | TEN_SEL_C          | (see Note1)
 `d`       | 100, 181 | "Select all motors to adjust preload tension"  | PRE_TENSION or TEN_SEL_[A/B/C]   | TEN_SEL_ALL        | (see Note1)
 `+`       | 61, 171  | "Increase [MOTOR] preload tension by 0.1 N"    | TEN_SEL_[MOTOR]            | (same as previous)      | (see Note1)
 `-`       | 45, 173  | "Decrease [MOTOR] preload tension by 0.1 N"    | TEN_SEL_[MOTOR]            | (same as previous)      | (see Note1)
 `n.n`     | [NUM]    | "Buffer an absolute preload tension value"     | TEN_SEL_[MOTOR]            | (same as previous)      | (see Note2)
 `Enter`   | 13, 10   | "Confirm buffered n.n -> [MOTOR] tension [N]"  | TEN_SEL_[MOTOR], buffer full | (same as previous)    | (see Note1)
 `Enter`   | 13, 10   | "Advance step 2/3 -> 3/3 (record home pose)"   | TEN_SEL_[MOTOR], buffer empty | (same as previous)   | (see Note1)

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

**Note1** The [DISPLAY_TEXT] box shows PretensionHandler::GetStatus(), which steps through 3 phases:
  - 1/3 "Tension 1/3: Unspool all tendons fully, then press Enter."
  - 2/3 "Tension 2/3: Select motor [a,b,c,d], [+/-], [n.n]; press Enter to save." (shown while ControllerHandler is in manual tension mode)
  - 3/3 "Tension 3/3: Home position recorded."
**Note2** For command `n.n`, this represents a value from 0.0 N to 10.0 N, always entered with one digit, a period, and one digit (e.g., 0.3, 1.0, 7.5). Digits are buffered in [INPUT_BUFFER] until `Enter` confirms.
**Note3** Step 1/3 -> 2/3 is advanced by pressing `Enter`. During step 2/3, `[a]/[b]/[c]/[d]` select which motor(s) the `+`/`-`/`n.n` commands act on ('d' = all three motors together); a "bare" `Enter` (nothing typed) advances 2/3 -> 3/3.
**Note4** Motor encoders are zeroed once by the Teensy at boot (T_AmplifierClass::Begin()), not by this sequence - re-zeroing at runtime would misalign the amplifier's internal cogging-compensation table from the motor's commutation reference. During step 2/3, ControllerHandler PWM output is enabled and put into manual tension mode: tension setpoints [N] come directly from the `[a]/[b]/[c]/[d]` + `+`/`-`/`n.n` commands above (seeded to `tension_preload_min` on entry) and are converted to PWM via the normal tension->current->PWM pipeline, visible live in the controller display's Tendon/Motor State table. On entering step 3/3, those per-motor tensions are captured as the new preload (`ControllerHandler::SetPreloadTensions()`), manual tension mode is turned off, and PWM output stays enabled - the RobotState ladder automatically requests READY (as if `E` were pressed), so the captured preload tensions continue to be held once PRETENSION is exited (e.g. via `grave` or `Delete`). If `grave` aborts the sequence before step 3/3, output and manual tension mode are disabled immediately and no READY request is made (preload remains whatever it was previously).
**Note5** For commands with multiple KeyIDs, either key ID is a valid option for that command. The keyID [NUM] means any acceptable value entered via keyboard number row or numpad.
**Note6** While in TEN_ADJ_[MOTOR], the [DISPLAY_TEXT] box shows PretensionHandler::GetTensionAdjustStatus(), updated live every frame: "Tension adjust: A=x.xxN  B=x.xxN  C=x.xxN -- [a/b/c/d] select motor, +/- = +/-0.1N, n.n+Enter = set value. Press [grave] or [T] to exit." On first entering TEN_ADJ_* from TEN_MENU, ControllerHandler PWM output is enabled and put into manual tension mode (seeded to `tension_preload_min` on all three motors); both are disabled again on exit - independent of the guided pretensioning sequence above (no unspool/zero/home-recording steps).



## GAIN TUNING INPUTS ["GAIN_ALL" / "GAIN_A" / "GAIN_B" / "GAIN_C" / "IGAIN_ALL" / "IGAIN_A" / "IGAIN_B" / "IGAIN_C"]
Two parallel overlays: `P` tunes the per-motor proportional gain (`gainTune`), `I` tunes the per-motor integral gain (`iGainTune`). Both use the identical input pattern below ('a/b/c/d' select a motor's direction, '+/-' nudge, 'P'/'I'/Enter exit).
 Command   | KeyID    | Description                                     | Required Input State       | New Input State         | Display Text            
-----------|----------|-------------------------------------------------|----------------------------|-------------------------|-------------------------------------------------------------------------------
 `P`       | 80       | "Open proportional gain tuning (all motors)"     | ANY (not already tuning)   | GAIN_ALL                | "Proportional gain tuning: [+/-] adjust all motors, or [a/b/c/d] select a motor. [Enter] exits."
 `P`       | 80       | "Close proportional gain tuning, resume task"    | GAIN_ALL or GAIN_[A/B/C]   | (state before 'P')      | "Proportional gain tuning closed." (see Note2)
 `Enter`   | 13, 10   | "Close proportional gain tuning, resume task"    | GAIN_ALL or GAIN_[A/B/C]   | (state before 'P')      | "Proportional gain tuning closed." (see Note2)
 `I`       | 73       | "Open integral gain tuning (all motors)"         | ANY (not already tuning)   | IGAIN_ALL               | "Integral gain tuning: [+/-] adjust all motors, or [a/b/c/d] select a motor. [Enter] exits."
 `I`       | 73       | "Close integral gain tuning, resume task"        | IGAIN_ALL or IGAIN_[A/B/C] | (state before 'I')      | "Integral gain tuning closed." (see Note2)
 `Enter`   | 13, 10   | "Close integral gain tuning, resume task"        | IGAIN_ALL or IGAIN_[A/B/C] | (state before 'I')      | "Integral gain tuning closed." (see Note2)
 `a`       | 97, 185  | "Tune motor A's gain (P or I, per current mode)"  | (I)GAIN_ALL or (I)GAIN_[B/C] | (I)GAIN_A             | (see Note1)
 `b`       | 98, 183  | "Tune motor B's gain (P or I, per current mode)"  | (I)GAIN_ALL or (I)GAIN_[A/C] | (I)GAIN_B             | (see Note1)
 `c`       | 99, 178  | "Tune motor C's gain (P or I, per current mode)"  | (I)GAIN_ALL or (I)GAIN_[A/B] | (I)GAIN_C             | (see Note1)
 `d`       | 100, 181 | "Tune all motors' gain together"                 | (I)GAIN_[A/B/C]            | (I)GAIN_ALL        | (see Note1)
 `+`       | 61, 171  | "Increase [MOTOR] gain (P:+0.01, I:+0.005)"      | (I)GAIN_ALL or (I)GAIN_[A/B/C] | (same as previous)  | (see Note1)
 `-`       | 45, 173  | "Decrease [MOTOR] gain (P:-0.01, I:-0.005)"      | (I)GAIN_ALL or (I)GAIN_[A/B/C] | (same as previous)  | (see Note1)
 `grave`   | 96       | "Abandon gain tuning AND the active task, return to IDLE" | (I)GAIN_ALL or (I)GAIN_[A/B/C] | IDLE   | "System cleared, returning to IDLE state."
**Note1** The [DISPLAY_TEXT] box shows `ControllerHandler::GetGainTuneStatus()` ('P' mode) or `GetIGainTuneStatus()` ('I' mode), updated live every frame, e.g. "Proportional gain tune (custom kP): A=x.xx  B=x.xx  C=x.xx -- [a/b/c/d] select motor, +/- = +/-0.01. Press [Enter], [P], or [grave] to exit." Each of `gainTune_A/B/C` (default `gain_kP`, clamped [0.0, 5.0]) / `iGainTune_A/B/C` (default `gain_kI`, clamped [0.0, 2.0]) is centered on that motor's direction (35/145/270 deg) and periodically interpolated the same way as the stiffness profile K(theta) (see CALIBRATION Note2). The proportional result is ADDED to `kP_effective`; the integral result is `kI_effective` used by the Stage 1 gated "endgame" integrator. 'd'/(I)GAIN_ALL adjusts all three together. Does not enable/disable PWM output or manual tension mode - normal operation (and any active task) continues while tuning. The per-motor values are shown live in the controller panel's "Gain kP" / "Gain kI" rows.
**Note2** 'P' and 'I' are toggles (`KeyboardHandler::IsGainTuneInputState()` / `KeyAction::EXIT_GAIN_MODE`), and `Enter` also exits. Opening either overlay remembers whichever InputState was active (e.g. FIT_RUN, CAL_STI) and freezes `systemState` so the active task - FITTS marker tracking, stiffness calibration, etc. - keeps running underneath the overlay. Switching directly between 'P' and 'I' preserves that remembered task (both are gain-tuning overlay states). Exiting restores that InputState exactly, without re-triggering its entry logic (e.g. CAL_STI's `cal2.Reset()` does not re-fire) - so you can tune gains mid-FITTS and immediately press 'r' for a new target. `grave`/`Delete` still perform a full reset to IDLE from within gain tuning, abandoning the active task as usual.



## LOGGING INPUTS ["LOGGING"]
 Command   | KeyID    | Description                             | Required Input State       | New Input State         | Display Text            
-----------|----------|-----------------------------------------|----------------------------|-------------------------|-------------------------------------------------------------------------------
 `L`       | 76       | "Toggle trial logging (prime / disarm)" | ANY                        | (same as previous, or LOG_UID if no user ID) | "Trial logging primed." / "Trial logging off." (see Note2)
 `l`       | 108      | "Start / stop operator-view video recording" | ANY                    | (same as previous, or LOG_UID if no user ID) | "Video logging started - [FILE]." / "Video logging stopped - saved [FILE]." (see Note2b)
 `U`       | 85       | "Set user ID"                           | ANY                        | LOG_UID                 | "Enter ID for user (000-999)..."
 `nnn`     | [NUM]    | "Set user ID value (000-999)"           | LOG_UID 	                  | LOG                 | "User ID set to [VAL]."
 `y`       | 121/89   | "Load stored participant calibration"   | LOG_CONFIRM                | IDLE                    | "Loading participant configuration..."
 `n`       | 110/78   | "Ignore stored participant calibration" | LOG_CONFIRM                | IDLE                    | "Ignoring stored configuration - recalibrate to overwrite."
**Note1** For command `nnn`, this represents a 3-digit value from 000 to 999, always entered with three digits (e.g., 001, 104, 204)
**Note2** `L` is a system-level toggle that works in any state, so logging can be armed once at the start of a session and left alone. If no user ID has been entered yet (`activeUserId < 0`), the first `L` instead jumps straight to the `LOG_UID` prompt ("Enter user ID (000-999) to start logging..."); once a valid ID is entered, logging is primed automatically ("User ID set - trial logging primed."). While primed, the next Fitts target start (`r`/`m`) begins a capture; pressing `L` mid-capture cancels and discards it. The operator panel's "Trial Logging" cell shows OFF / PRIMED / REC.
**Note2b** `l` (lowercase) records the composited "NURing Operator" camera view - every overlay included - to `logging/<UUU>/<UUU>-mmddyyyy-hhmmss.mp4`, named from the recording START time using the same convention as the trial CSVs. Like `L` it works in any state, and if no user ID has been entered yet it jumps to the `LOG_UID` prompt first ("Enter user ID (000-999) to start recording...") and begins recording once a valid ID is set. It is fully independent of `L` - either can run without the other. The operator panel's "Video Logging" cell shows ON (green) / OFF, and while recording an elapsed-time stamp (`12.3456 s`, matching the accuracy CSVs' `t_secs` format) is drawn in the bottom left of the camera view and captured into the video. The view is sampled at 10 fps and encoded to H.264 by an `ffmpeg` child process on a background thread, so recording adds no measurable work to the main loop; if the encoder ever falls behind, video frames are dropped (and the count reported on stop) rather than the main loop being stalled. A recording left running is finalised automatically on exit.

**Note3** Per-participant calibration config: when a user ID is set, main.cpp checks for `logging/<UUU>/config<UUU>.yaml`. If it exists with stored calibrations, the system diverts to the `LOG_CONFIRM` prompt ("User <UUU> configuration found, load (y/n)?"). `y` loads the stored Cal1 (AROM) / Cal2 (stiffness) / Cal3 (fingertip offset) values and marks them complete; `n` ignores them. The file is created/updated automatically as each calibration completes (partial-aware - each of the three sections is saved independently). Pressing `n` then re-running a calibration overwrites that section. Only real participant IDs (>= 001) are persisted; `000` is the non-logging ID.



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
 `F`       | 70       | "Enter accuracy mode (cals incomplete)" | IDLE                       | FIT_WARN                | "Calibrations not complete, (p)roceed or (r)eturn"
 `p`       | 112      | "Proceed into accuracy mode anyway"     | FIT_WARN                   | FIT_SEL                 | "Select marker mode: [r] Random, [m] Manual..."
 `r`       | 114      | "Return to idle to finish calibrations" | FIT_WARN                   | IDLE                    | "Returning to idle - complete calibrations, then press F."
 `m`       | 109      | "Manually set active marker (1 to 45)"  | FIT_SEL or FIT_RUN         | FIT_ACT                 | "Which marker (1-45)..."
 `r`       | 114      | "Randomly set active marker (1 to 45)"  | FIT_SEL                    | FIT_RUN                 | "Active marker set to [MARKER_ID]."
 `nn`      | [NUM]    | NONE                                    | FIT_ACT                    | FIT_RUN                 | "Active marker set to [MARKER_ID]."
 `b`       | 98       | "Start a study block"                   | FIT_SEL or FIT_RUN         | FIT_BLK                 | "Which block? [0] - [9]..."
 `0`-`9`   | 48-57    | "Draw and arm that block (0-9)"         | FIT_BLK                    | FIT_RUN                 | (set by AccuracyBlockHandler)
 `n`       | 110      | "Present the next block target"         | FIT_RUN                    | FIT_RUN                 | (set by AccuracyBlockHandler)
**Note1** For command `nn`, this represents a 2-digit value from 00 to 45, always entered with two digits (e.g., 01, 04, 45) 
**Note3** `F` from IDLE only goes straight to FIT_SEL when all three calibrations (AROM, stiffness, fingertip offset) are complete. Otherwise it diverts to the FIT_WARN confirmation handled in `KeyboardHandler::ProcessKey` (gated by `SetCalibrationsComplete()`): `p` proceeds into the task with whatever calibration data exists, `r` returns to IDLE. 
**Note2** The keyID [NUM] means any acceptable value entered via keyboard number row or numpad
**Note4** Study blocks (`AccuracyBlockHandler`): `b` then a single digit 0-9 (no Enter - FIT_BLK is not a numeric-entry state; number row or numpad) draws one marker from each configured `accuracy_trials.target_set_NN`, shuffles them, and prints the block to the terminal as `Block 1 target set: b01_108, b11_302, ...` (`bNN` = the set the marker came from). Starting a block clears the active target, so guidance stays off until the first `n`. Each `n` presents the next target and, when logging is primed, opens a trial capture exactly as `r` does. `n` REFUSES to advance until the participant's touchscreen contact ends the current trial ("Trial 4/12 (b03_120) not complete - waiting for touch."); after the last trial it reports the block complete and disarms. Blocks are drawn independently, so a marker may recur across blocks. `r`/`m` still work mid-block as an operator override - they present an off-sequence target, and the next `n` resumes the block where it left off.



## TASK 2 - OBJECT GUIDANCE INPUTS ["OBJECTS"]
 Command   | KeyID    | Description                                | Required Input State       | New Input State         | Display Text            
-----------|----------|--------------------------------------------|----------------------------|-------------------------|-------------------------------------------------------------------------------
 `O`       | 79       | "Enter object guidance (scan phase)"       | IDLE                       | OBJ_SCAN                | "Scanning objects - aim at each object with a world marker, [t] to train, [Enter] to finish."
 `O`       | 79       | "Enter object mode (cals incomplete)"      | IDLE                       | OBJ_WARN                | "Calibrations not complete, (p)roceed or (r)eturn"
 `p`       | 112      | "Proceed into object mode anyway"          | OBJ_WARN                   | OBJ_SCAN                | "Scanning objects - aim at each object with a world marker, [t] to train, [Enter] to finish."
 `r`       | 114      | "Return to idle to finish calibrations"    | OBJ_WARN                   | IDLE                    | "Returning to idle - complete calibrations, then press O."
 `t`       | 116      | "Train visible objects (burst-average)"    | OBJ_SCAN, OBJ_SEL, OBJ_RUN | (same as previous)      | "Training visible objects - hold the camera steady..." then "Trained N object(s) (M total)." (see Note1)
 `u`       | 117      | "Untrain (forget) all trained objects"     | OBJ_SCAN, OBJ_SEL, OBJ_RUN | (same as previous)      | "Cleared all trained objects." (see Note2)
 `D`       | 68       | "Corner-jitter probe (terminal output)"    | OBJ_SCAN, OBJ_SEL, OBJ_RUN | (same as previous)      | "Corner-jitter probe running - hold the camera rigidly still (result in terminal)." (see Note4)
 `Enter`   | 13, 10   | "Finish the scan phase"                    | OBJ_SCAN                   | OBJ_SEL                 | "Select object mode: [r] Random, [m] Manual..."
 `m`       | 109      | "Manually set active object marker"        | OBJ_SEL or OBJ_RUN         | OBJ_ACT                 | "Which object marker ID (00-99)..."
 `r`       | 114      | "Randomly pick a TRAINED object from pool" | OBJ_SEL or OBJ_RUN         | OBJ_RUN                 | "Re-scanning objects..." then "Object marker set to [OBJECT_ID] (N present)." (see Note3)
 `nn`      | [NUM]    | "Set active object marker (00-99)"         | OBJ_ACT                    | OBJ_RUN                 | "Object marker set to [OBJECT_ID]."
**Note1** `t` starts a training burst (`object_world.train_frames` detection frames, ~0.25 s): every burst frame with a valid world-board pose contributes one marker->world sample per visible object; at the end each sampled object's pose is averaged (translation mean, rotation SVD-orthonormalized) into a LOCKED world anchor. Training is the ONLY way an object is mapped - there is no automatic mapping during the scan phase. A trained object is rendered AND guided from anchor + current world pose (its marker being visible only recolors the wireframe green), so it stays put under an occluding hand and its jitter reduces to world-pose jitter. It never re-maps automatically: after physically moving an object, press `t` again with it in view. The countdown shows in the status line as "[TRAINING N]".
**Note2** `u` forgets every trained anchor. Objects revert to live "(preview)" wireframes (drawn only while their marker is visible) and produce NO guidance target until re-trained.
**Note3** `r` first runs a PRESENCE RE-SCAN (`object_world.presence_scan_frames` detection frames, ~0.4 s): each TRAINED object's marker must be re-detected on at least a few frames to count as still present, so a physically removed object is never randomly selected again. The pick then draws from pool ∩ trained ∩ present, with no repeats until that subset is exhausted (then the pool refills, still presence-filtered). An untrained object has no world anchor, so it never resolves a guidance target - train it first. A manual `m` selection made while the re-scan is running cancels the pending random pick.
**Note4** `D` runs the corner-jitter diagnostic: with the camera held rigidly still, it accumulates the RAW detected corner positions of world markers 1, 5, 9, 19, 27, 37, 45 over 300 detection frames, then prints one copy/paste-friendly row to the terminal - per-marker corner standard deviation [px], computed as sqrt(mean over the 4 corners of (var_x + var_y)). "nan" = that marker was seen fewer than 2 frames. Use it to A/B jitter fixes (`camera.detect_on_clahe`, `aruco_detector.object_corner_refinement_method`, gain/exposure/lighting changes).



## SUMMARY - ALL COMMANDS (alphabetical)
Every command from the sections above, in one list. **State** is the section the command is documented under, so a key bound in several sections gets one row per section - look up the key, then read across to see which contexts it is live in. Refer back to that section for the required/new InputState, key IDs, and notes.

 Command   | Description                                             | State
-----------|---------------------------------------------------------|-------------------------
 `+`       | Increase [MOTOR] gain (P: +0.01, I: +0.005)             | GAIN TUNING
 `+`       | Increase [MOTOR] preload tension by 0.1 N               | PRETENSION
 `+`       | Increase [MOTOR] tension by 0.1 N                       | TENSION_ADJUST
 `-`       | Decrease [MOTOR] gain (P: -0.01, I: -0.005)             | GAIN TUNING
 `-`       | Decrease [MOTOR] preload tension by 0.1 N               | PRETENSION
 `-`       | Decrease [MOTOR] tension by 0.1 N                       | TENSION_ADJUST
 `0`-`9`   | Draw and arm that study block (one target per set)      | ACCURACY
 `a`       | Start ARoM calibration                                  | CALIBRATION
 `a`       | Tune motor A's gain (P or I, per current mode)          | GAIN TUNING
 `a`       | Select motor A to adjust preload tension                | PRETENSION
 `a`       | Send PWM value to motor A for 1 s                       | PWM_TEST
 `a`       | Adjust motor A tension directly                         | TENSION_ADJUST
 `b`       | Start a study block (then a digit 0-9)                  | ACCURACY
 `b`       | Tune motor B's gain (P or I, per current mode)          | GAIN TUNING
 `b`       | Select motor B to adjust preload tension                | PRETENSION
 `b`       | Send PWM value to motor B for 1 s                       | PWM_TEST
 `b`       | Adjust motor B tension directly                         | TENSION_ADJUST
 `c`       | Tune motor C's gain (P or I, per current mode)          | GAIN TUNING
 `c`       | Select motor C to adjust preload tension                | PRETENSION
 `c`       | Send PWM value to motor C for 1 s                       | PWM_TEST
 `c`       | Adjust motor C tension directly                         | TENSION_ADJUST
 `C`       | Enter calibration mode                                  | CALIBRATION
 `d`       | Tune all motors' gain together                          | GAIN TUNING
 `d`       | Select all motors to adjust preload tension             | PRETENSION
 `d`       | Send PWM value to all motors for 1 s                    | PWM_TEST
 `d`       | Adjust all motors' tension directly                     | TENSION_ADJUST
 `D`       | Corner-jitter probe (result printed to terminal)        | OBJECTS
 `Delete`  | Cancel input / task, return to idle                     | SYSTEM
 `e`       | Disable guidance output (tension only)                  | SYSTEM
 `E`       | Enable guidance output                                  | SYSTEM
 `Enter`   | Close proportional / integral gain tuning, resume task  | GAIN TUNING
 `Enter`   | Finish the scan phase                                   | OBJECTS
 `Enter`   | Advance the guided pretension step                      | PRETENSION
 `Enter`   | Confirm buffered n.n -> [MOTOR] preload tension [N]     | PRETENSION
 `Enter`   | Advance step 2/3 -> 3/3 (record home pose)              | PRETENSION
 `Enter`   | Confirm buffered n.n -> [MOTOR] tension [N]             | TENSION_ADJUST
 `ESC`     | Quit the program cleanly                                | SYSTEM
 `F`       | Enter guidance accuracy mode                            | ACCURACY
 `F`       | Enter accuracy mode (cals incomplete - proceed/return)  | ACCURACY
 `grave`   | Exit ARoM / stiffness calibration, return to IDLE       | CALIBRATION
 `grave`   | Abandon gain tuning AND the active task, return to IDLE | GAIN TUNING
 `grave`   | Exit task, return system to idle                        | SYSTEM
 `grave`   | Exit tension adjustment, return to IDLE                 | TENSION_ADJUST
 `I`       | Open integral gain tuning (all motors)                  | GAIN TUNING
 `I`       | Close integral gain tuning, resume task                 | GAIN TUNING
 `k`       | Toggle K(theta) stiffness gain                          | SYSTEM
 `L`       | Toggle trial logging (prime / disarm)                   | LOGGING
 `l`       | Start / stop operator-view video recording              | LOGGING
 `m`       | Manually set active marker                              | ACCURACY
 `m`       | Manually set active object marker                       | OBJECTS
 `M`       | Enter motor test mode                                   | PWM_TEST
 `n`       | Present the next study-block target                     | ACCURACY
 `n`       | Ignore stored participant calibration                   | LOGGING
 `n.n`     | Buffer an absolute preload tension value (0.0-10.0 N)   | PRETENSION
 `n.n`     | Buffer an absolute tension value (0.0-10.0 N)           | TENSION_ADJUST
 `nn`      | Set active marker (manual entry)                        | ACCURACY
 `nn`      | Set active object marker (00-99)                        | OBJECTS
 `nnn`     | Set user ID value (000-999)                             | LOGGING
 `nnnn`    | Set PWM value (0000-2047) for [MOTOR]                   | PWM_TEST
 `o`       | Start fingertip offset calibration                      | CALIBRATION
 `O`       | Enter object guidance (scan phase)                      | OBJECTS
 `O`       | Enter object mode (cals incomplete - proceed/return)    | OBJECTS
 `p`       | Proceed into accuracy mode anyway                       | ACCURACY
 `p`       | Proceed into object mode anyway                         | OBJECTS
 `p`       | Start guided pretensioning sequence                     | PRETENSION
 `P`       | Open proportional gain tuning (all motors)              | GAIN TUNING
 `P`       | Close proportional gain tuning, resume task             | GAIN TUNING
 `r`       | Return to idle to finish calibrations                   | ACCURACY
 `r`       | Randomly set active marker                              | ACCURACY
 `r`       | Return to idle to finish calibrations                   | OBJECTS
 `r`       | Randomly pick a TRAINED object from the pool            | OBJECTS
 `R`       | One-time rig alignment capture (screen <-> world board) | RIG_ALIGN (see Note2)
 `s`       | Start stiffness calibration                             | CALIBRATION
 `S`       | Toggle Teensy serial connection                         | SERIAL
 `SPACE`   | E-stop toggle (tension PWM only / full output)          | SYSTEM
 `t`       | Train visible objects (burst-average into anchors)      | OBJECTS
 `T`       | Open tensioning menu                                    | PRETENSION / TENSION_ADJUST
 `T`       | Exit tension adjustment, reopen tensioning menu         | TENSION_ADJUST
 `u`       | Untrain (forget) all trained objects                    | OBJECTS
 `U`       | Set user ID                                             | LOGGING
 `y`       | Load stored participant calibration                     | LOGGING
 `Z`       | Record the current encoder pose as home position        | SYSTEM (see Note2)
**Note1** Sorted case-insensitively by command, lowercase before uppercase where a letter is bound to both (e.g. `c` = select motor C, `C` = enter calibration mode). Rows for the same command are then ordered by State. "GAIN TUNING" covers the GAIN_ALL / GAIN_[A/B/C] / IGAIN_ALL / IGAIN_[A/B/C] overlay states.
**Note2** `R` and `Z` are in `kKeyCommandTable` but have no section of their own above: `R` (IDLE -> RIG_CAP) runs the one-time screen<->world-board rotation capture that writes `rig_alignment.yaml` - only needed if the touchscreen or world board physically moves; `Z` (any state) records the current encoder pose as the home position.
