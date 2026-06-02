# NURing PC program outstanding tasks

Use this document as a reference to tasks that need to still be completed. If the task has been finished, update it's status.



---



## Overall list of outstanding tasks
**PC Side**
-   [CALIBRATION1]  Implement calibration 1 handler (finger active range of motion)
-   [CALIBRATION2]  Implement calibration 2 handler (finger deflection stiffness)
-   [CALIBRATION2]  Implement camera-based ground truth for virtual XY estimation
-   [CALIBRATION3]  Confirm calibration 3 is complete
-   [CALIBRATION3]  Allow for toggle of fingertip-to-camera offset
-   [CAMERA]        Test "auto-calibration" tool to find optimal camera settings for ArUco detection
-   [CAMERA]        Test "auto-calibration" tool to find optimal detector settings for ArUco detectionand 
-   [CONTROLLER]    Add input that enables/disables application of stiffness gain to allow for performance comparisons
-   [CONTROLLER]    Attempt auto-tensioning, where the tendons will spool up on their own until they detect no more rotation per that motor's encoder
-   [CONTROLLER]    Enable PI controller, where we allow for either a single proportional gain across the 3 tendons or the user stiffness gains
-   [CONTROLLER]    Scale error by distance to prevent large deflections during more precise guidance during homing phase
-   [CONTROLLER]    Find way for dealing with lost markers, if a marker goes off the FOV, then trigger "reverse" cue
-   [CONTROLLER]    Set amplitude, frequency, and pattern for reverse cue
-   [DISPLAY]       Flag for UserID in the status panel
-   [LOGGING]       Save and output participant gains info
-   [LOGGING]       Enable logging of appropriate data; potentially have a section in the config.yaml where I can flag on/off data to save?
-   [LOGGING]       Allow dummy userID that prevents data from being saved (e.g., U000 means no-save, U-1 means user ID not set yet)
-   [LOGGING]       Find optimal way to save logging data (e.g., log in real time, save data to vector, rolling vector save, etc.)
-   [SERIAL]        Add large whole-screen text for when safety switch is released (from Teensy serial state)
-   [STUDY]         Add overall task handler that accepts user ID
-   [STUDY]         Check that userID is entered before starting calibration or tasks
-   [STUDY]         Set up folder and file for logging once valid user ID is entered
-   [STUDY]         Output configuration file for each participant
-   [TASKS_ACC]     Create / move all fitts-related stuff into a Tasks_Accuracy handler
-   [TASKS_ACC]     Enable logging when task begins
-   [TASKS_ACC]     Add baseline study with audio cues (tones or voice)

**Teensy Side**
-   [AMPLIFIER]     Implement hardware serial interfaces 
-   [AMPLIFIER]     Implement intelligent serial probing for encoder position and measured current 
-   [AMPLIFIER]     Send encoder and current values to PC
-   [LED]           Add LED for verifying PC serial connection
-   [LED]           Implement so that blue LEDs go one once higher baud rate has been achieved
-   [SERIAL]        Implement 'SAFETY_OFF" state if participant releases safety button, and send state to PC



---



## Detailed list of PC tasks






### CALIBRATION1 (Active range of motion)
-   [INCOMPLETE]    Implement calibration 1 handler (finger active range of motion)



---



### CALIBRATION2 (Stiffness calculation)
-   [INCOMPLETE]    Implement calibration 2 handler (finger deflection stiffness)
-   [INCOMPLETE]    Implement camera-based ground truth for virtual XY estimation



---



### CALIBRATION3 (Fingertip-to-camera offset)
-   [INCOMPLETE]    Confirm calibration 3 is complete
-   [INCOMPLETE]    Allow for toggle of fingertip-to-camera offset



---



### CAMERA
-   [INCOMPLETE]    Test "auto-calibration" tool to find optimal camera settings for ArUco detection
-   [INCOMPLETE]    Test "auto-calibration" tool to find optimal detector settings for ArUco detectionand 



---



### CONTROLLER 
-   [INCOMPLETE]    Add input that enables/disables application of stiffness gain to allow for performance comparisons
-   [INCOMPLETE]    Attempt auto-tensioning, where the tendons will spool up on their own until they detect no more rotation per that motor's encoder
-   [INCOMPLETE]    Enable PI controller, where we allow for either a single proportional gain across the 3 tendons or the user stiffness gains
-   [INCOMPLETE]    Scale error by distance to prevent large deflections during more precise guidance during homing phase
-   [INCOMPLETE]    Find way for dealing with lost markers, if a marker goes off the FOV, then trigger "reverse" cue
-   [INCOMPLETE]    Set amplitude, frequency, and pattern for reverse cue



---



### DISPLAY 
-   [INCOMPLETE]    Flag for UserID in the status panel
  


---



### LOGGING
-   [INCOMPLETE]    Save and output participant gains info
-   [INCOMPLETE]    Enable logging of appropriate data; potentially have a section in the config.yaml where I can flag on/off data to save?
-   [INCOMPLETE]    Allow dummy userID that prevents data from being saved (e.g., U000 means no-save, U-1 means user ID not set yet)

**Implementation notes**
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



---



### SERIAL
-   [INCOMPLETE]    Add large whole-screen text for when safety switch is released (from Teensy serial state)



---



### STUDY
-   [INCOMPLETE]    Add overall task handler that accepts user ID
-   [INCOMPLETE]    Check that userID is entered before starting calibration or tasks
-   [INCOMPLETE]    Set up folder and file for logging once valid user ID is entered
-   [INCOMPLETE]    Output configuration file for each participant



---



### TASKS_ACC
-   [INCOMPLETE]    Create / move all fitts-related stuff into a Tasks_Accuracy handler
-   [INCOMPLETE]    Enable logging when task begins
-   [INCOMPLETE]    Add baseline study with audio cues (tones or voice)



---



## Detailed list of Teensy tasks



### AMPLIFIER
-   [INCOMPLETE]    Implement hardware serial interfaces 
-   [INCOMPLETE]    Implement hardware serial probing for encoder position and measured current 
-   [INCOMPLETE]    Send encoder and current values to PC

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



### Hardware
-   [INCOMPLETE]    Add LED for verifying PC serial connection
-   [INCOMPLETE]    Implement so that blue LEDs go one once higher baud rate has been achieved
-   [INCOMPLETE]    Implement 'SAFETY_OFF" state if participant releases safety button, and send state to PC

