# NURing Outstanding Tasks

Use this document as a reference to tasks that need to still be completed. If the task has been finished, update it's status.

Rank 1 = Highest priority
Rank 2 = Middle priority
Rank 3 = Lowest priority


## PC Side 

 Status  | Rank | Source             | Task
---------|------|--------------------|-----------------------------------------------------------------------------------------------------------------------------
[X]      | 1    | Cal1Handler        | Implement calibration 1 handler (measuring finger active range of motion)
[X]      | 1    | Cal1Handler        | Show virtual points being captured on the little virtualXY window
[X]      | 1    | Cal1Handler        | Check if r_pulley_effective can drop below 2.5; ensure that when tendon is fully extended, r_pulley_eff is set at 2.5
[ ]      | 3    | Cal1Handler        | Implement "soft centering": if fingertip has stationary for X seconds, set that as new virtual fingertip zero
---------|------|--------------------|-----------------------------------------------------------------------------------------------------------------------------
[X]      | 1    | Cal2Handler        | Implement calibration 2 handler (finger deflection stiffness)
[X]      | 1    | Cal2Handler        | Display polygon of stiffness based on normalized stiffness value
[ ]      | 2    | Cal2Handler        | Implement camera-based ground truth for virtual XY estimation
---------|------|--------------------|-----------------------------------------------------------------------------------------------------------------------------
[X]      | 1    | Cal3Handler        | Implement fingertip-to-camera offset calibration measurement
[ ]      | 2    | Cal3Handler        | Confirm calibration 3 is accurate
[ ]      | 2    | Cal3Handler        | Display calibration status
[ ]      | 2    | Cal3Handler        | Display calibrated fingertip position
[ ]      | 1    | Cal3Handler        | Allow for toggle of fingertip-to-camera offset
---------|------|--------------------|-----------------------------------------------------------------------------------------------------------------------------
[ ]      | 3    | CameraHandler      | Test "auto-calibration" tool to find optimal camera settings for ArUco detection
[ ]      | 3    | CameraHandler      | Test "auto-calibration" tool to find optimal detector settings for ArUco detectionand 
---------|------|--------------------|-----------------------------------------------------------------------------------------------------------------------------
[X]      | 1    | ControllerHandler  | Add input that enables/disables application of stiffness gain to allow for performance comparisons
[ ]      | 3    | ControllerHandler  | Attempt auto-tensioning, where the tendons will spool up on their own until they detect no more rotation per that motor's encoder
[ ]      | 1    | ControllerHandler  | Enable PI controller, where we allow for either a single proportional gain across the 3 tendons or the user stiffness gains
[ ]      | 1    | ControllerHandler  | Scale error by distance to prevent large deflections during more precise guidance during homing phase
[ ]      | 1    | ControllerHandler  | Find way for dealing with lost markers, if a marker goes off the FOV, then trigger "reverse" cue
[ ]      | 1    | ControllerHandler  | Set amplitude, frequency, and pattern for reverse cue
[ ]      | 1    | ControllerHandler  | Check default values for controller (gains, max tension, etc)
[ ]      | 1    | ControllerHandler  | Implement vibrotactile feedback for "on target"
[ ]      | 1    | ControllerHandler  | Implement vibrotactile feedback for "about to contact"
---------|------|--------------------|-----------------------------------------------------------------------------------------------------------------------------
[ ]      | 1    | DisplayHandler     | Flag for UserID in the status panel
[ ]      | 1    | DisplayHandler     | Fix controller panel controller values
[ ]      | 1    | DisplayHandler     | Fix status panel on main panel
---------|------|--------------------|-----------------------------------------------------------------------------------------------------------------------------
[ ]      | 2    | FittsTaskHandler   | Project fingertip position onto ArUco tag plane
[ ]      | 1    | FittsTaskHandler   | Enable logging when task begins
[ ]      | 1    | FittsTaskHandler   | Add baseline study with audio cues (tones or voice)
[ ]      | 1    | FittsTaskHandler   | Add cognitive loading task
[ ]      | 1    | FittsTaskHandler   | Add toggles for various calibrations
[ ]      | 2    | FittsTaskHandler   | Check projected fingertip endpoint on touchscreen
---------|------|--------------------|-----------------------------------------------------------------------------------------------------------------------------
[X]      | 1    | GestureHandler     | Implement up/down gesture recognition
[X]      | 1    | GestureHandler     | Implement circle gesture recognition
[ ]      | 3    | GestureHandler     | Test left/right flick for confirm/reject
[ ]      | 3    | GestureHandler     | Test optical flow for gesture detection
[ ]      | 2    | GestureHandler     | Test closed fist for "stop" gesture
---------|------|--------------------|-----------------------------------------------------------------------------------------------------------------------------
[ ]      | 1    | LoggingHandler     | Save and output participant gains info
[ ]      | 1    | LoggingHandler     | Enable logging of appropriate data; potentially have a section in the config.yaml where I can flag on/off data to save?
[ ]      | 1    | LoggingHandler     | Allow dummy userID that prevents data from being saved (e.g., U000 means no-save, U-1 means user ID not set yet)
[ ]      | 1    | LoggingHandler     | Find optimal way to save logging data (e.g., log in real time, save data to vector, rolling vector save, etc.)
[ ]      | 1    | LoggingHandler     | Check that userID is entered before starting calibration or tasks
[ ]      | 1    | LoggingHandler     | Set up folder and file for logging once valid user ID is entered
[ ]      | 1    | LoggingHandler     | Output configuration file for each participant
---------|------|--------------------|-----------------------------------------------------------------------------------------------------------------------------
[ ]      | 1    | SerialHandler      | Add large whole-screen text for when safety switch is released (from Teensy serial state)
[ ]      | 1    | SerialHandler      | Add large whole-screen text for when Teensy is disconnected during runtime



## Teensy Side
 Status  | Rank |Source              | Task
---------|------|--------------------|-----------------------------------------------------------------------------------------------------------------------------
[X]      | 1    | T_SerialClass      | Implement hardware serial interfaces 
[X]      | 1    | T_AmplifierClass   | Implement intelligent serial probing for encoder position and measured current 
[X]      | 1    | T_AmplifierClass   | Send encoder and current values to PC
[X]      | 1    | T_SerialClass      | Add LED for verifying PC serial connection
[X]      | 1    | T_SerialClass      | Implement so that blue LEDs go one once higher baud rate has been achieved
[X]      | 1    | T_AmplifierClass   | Implement 'SAFETY_OFF" state if participant releases safety button, and send state to PC



---

**Logging Implementation notes**
- Implement logging as comma-delimited text files with file names based on task, userID, and block number
- Keep everything in lower case

**Example accuracy task logging output**
file header:
   task:            [taskName]
   userID:          [userID]
   blockNumber:     [block]
   trialOrder:      [csv list of target markers or poses]
data:
   trialNumber:     [trialNumber]
   targetMarkerId:  [markerID]
   timestamp    targetXMM   targetYMM   errorXMM    errorYMM    errorZMM    virtualX    virtualY    vibroCue
   0.00  
   0.01
   0.02
   ...
Potential additions: current, encoder counts, virtual fingertipXY, 

**Example user settings file**
header:
  userID:       [userID]
  offset:       [point3f]   Offset vector from the camera to fingertip
data:
  angle,    limit,   stiffness
  0.0,
  1.0
  2.0
  ...




**ASCII commands for reference**
Command         Response    Type    Units       Use
g r0x0c         v 000       INT16   0.01 A      Get actual current (Q-axis) output from the amplifier. 
g r0x90         v 000       INT32   bits/s      Get actual baud rate.
g r0x17         v 000       INT32   count       Actual motor position.
g f0x92         v "sss"     STRING  -           Name of amplifier.
s r0x90 115237  ok          INT32   bits/s      Set baud rate to 115237.
s r0x24 3       ok          INT16   -           Set command mode to current loop driven by PWM
g r0x00         e 00        INT16   -           Failed to set value, error number 00
s r0x17 0       ok          INT32   count       Set motor position (encoder counts)



---

