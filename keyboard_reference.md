# NURing Keyboard Command Reference

All commands are typed into the **operator display window** (click it to give it focus),
then confirmed with **Enter**. The current input is shown in the **Input** row of the
System Information panel. The result appears in the **Output** row.



## STATE COMMANDS
 Command             | Required State          | New State               | Result                                                     
---------------------|-------------------------|-------------------------|------------------------------------------------------------
 `ESC`               | ANY                     | NONE                    | Quit the program cleanly
 `Backspace`         | ANY					   | PREVIOUS                | Returns back to previous state
 `~` / `apostrophe`  | ANY                     | IDLE                    | Enter / return to IDLE state, stopping all other logging and tasks
 `c`                 | IDLE                    | CALIBRATE_SELECT        | Enter CALIBRATE_SELECT mode for calibration 
 `g`                 | IDLE                    | GUIDANCE_TASK_SELECT    | Enter GUIDANCE_TASK_SELECT mode for study tasks
` m`                 | IDLE                    | MOTOR_SELECT            | Enter MOTOR_SELECT mode
 `s`                 | ANY                     | SERIAL_SELECT           | Enter SERIAL_SELECT mode
 `t`                 | IDLE                    | TENSION_SELECT          | Enter TENSION_SELECT mode

 

## CALIBRATION INPUTS
 Command             | Required State          | New State               | Result                                                     
---------------------|-------------------------|-------------------------|------------------------------------------------------------
 `c`                 | IDLE                    | CALIBRATE_SELECT        | Enter calibration select mode
 `1`                 | CALIBRATE_SELECT		   | CAL_1_ROM		  	     | Enter cal1, active range of motion capture                    
 `2`                 | CALIBRATE_SELECT		   | CAL_2_STIFFNESS		 | Enter cal2, stiffness measurement
 `3`                 | CALIBRATE_SELECT		   | CAL_3_OFFSET			 | Enter cal3, fingertip to camera offset calibration


## GUIDANCE TASK INPUTS
 Command             | Required State          | New State               | Result                                                     
---------------------|-------------------------|-------------------------|------------------------------------------------------------
 `g`                 | IDLE                    | TASK_SELECT             | Enter guidance task select mode
 `1`                 | TASK_SELECT             | TASK_1_FITTS            | Enter fitts task 
 `r`                 | TASK_1_FITTS            | TASK_1_FITTS            | Randomly select a new target marker (1–45) — only show that marker
 `a`                 | TASK_1_FITTS            | TASK_1_FITTS_SELECT     | Enter state to select new target marker, only show that marker
 `00`                | TASK_1_FITTS_SELECT     | TASK_1_FITTS            | Selects active target to 00


## MOTOR INPUTS
 Command             | Required State          | New State               | Result                                                     
---------------------|-------------------------|-------------------------|------------------------------------------------------------
 `m`                 | IDLE                    | MOTOR_SELECT_ALL        | Enter motor all select mode
 `YYYY`              | MOTOR_SELECT_ALL		   | MOTOR_SELECT_ALL        | Send PWM value of YYYY to all motors for 1 second
 `a` / `b` / `c`     | MOTOR_SELECT_ALL        | MOTOR_SELECT_X          | Enter motor X select mode
 `YYYY`              | MOTOR_SELECT_X		   | MOTOR_SELECT_X          | Send PWM value of YYYY to motor X for 1 second
 
 
## SERIAL INPUTS
 Command             | Required State          | New State               | Result                                                     
---------------------|-------------------------|-------------------------|------------------------------------------------------------
 `s`                 | ANY                     | SERIAL_COMMAND          | Enter SERIAL_COMMAND mode
 `c`       		     | SERIAL_COMMAND          | IDLE			         | Open `/dev/ttyACM0` and connect to the Teensy
 `d`       		     | SERIAL_COMMAND          | IDLE			         | Close the serial connection gracefully
 

## TENSION INPUTS
 Command             | Required State          | New State               | Result                                                     
---------------------|-------------------------|-------------------------|------------------------------------------------------------
 `t`                 | ANY                     | TENSION_SELECT_ALL      | Enter tension select mode
 `+` / `-`           | TENSION_SELECT_ALL	   | TENSION_SELECT_ALL      | Increase / decrease tension to all motors
 `a` / `b` / `c`     | TENSION_SELECT_ALL      | TENSION_SELECT_X        | Select motor X for tension adjustment
 `+` / `-`           | TENSION_SELECT_X	       | TENSION_SELECT_X        | Increase / decrease tension to motor X


### KEYPAD MAPPING
 Raw Input  | Mapped Key | Result                                                     
------------|------------|------------------------------------------------------------------------------------------------------------
 185        | `k9`       | Selects motor A (same as 'a')
 183        | `k7`       | Selects motor B (same as 'b')
 178        | `k2`       | Selects motor C (same as 'd')
 181        | `k5`       | Deselects motors (goes back to MOTOR_SELECT_ALL from MOTOR_SELECT_X or goes back to TENSION_SELECT ALL from TENSION_SELECT_X)




