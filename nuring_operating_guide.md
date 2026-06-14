# NURing Operating Guide

## Prechecks
0. Confirm connections 
   a. Ring camera USB cable
   b. PC to Teensy USB serial connection cable

## Startup
1. Reset NURing wearable
   a. Align BLDC motors with home index markings
2. Turn on NURing power station
   a. Turn on power strip
   b. Turn on 24V BLDC motor power 
   c. Turn on 12V amplifier power
   d. Turn on 5V Teensy power
   e. Verify successful hardware startup
      - Three green vertical LEDs = amplifiers successfully enabled
      - Three blue vertical LEDs = amplifier hardware serial speed boosted to 115200 (from 9600)
      - Teensy LED flashing at 1 Hz = waiting for PC serial connection
      - Motors should be free of cogging
3. Launch NURing software
   a. Establish serial connection with [S]

## Personalization
4. Configure preload and tendon length measurements
   a. Enter Tension mode with [T]
   b. Run auto-tension sequence with [p]
      Step 1: Extend ring / tendons all the way out
              - Press "Enter" when extended
      Step 2: Zero out motor encoders
              - Happens automatically at transition from Step 1 to Step 2
      Step 3: Set motor tension
              - Use [+] and [-] keys to increase / decrease tension
              - To adjust an individual motor, select it with [a], [b], or [c], or use [d] to adjust all
              - Press "Enter" when complete
      Step 4: Set virtual fingertip XY 0
              - Happens automatically
    c. Exit Tension mode by pressing [Spacebar]
5. Calibrate NURing device
   a. Enter Calibration mode with [C]
      Step 1: Measuring Active Range of Motion (ARoM)
            - Press [a] to start measurement           
            - Move fingertip about the MCP joint for 10 seconds
            - Verify limit set in virtual fingertip display
      Step 2: Measuring stiffness profile 
            - Press [s] to start fingertip deflection ramps
            - Allow finger to be deflected without curling
            - Stiffness profile is measured automatically
            - Verify stiffness profile in virtual fingertip display
      Step 3: Measuring fingertip to camera offset
            - Press [o] to start offset calibration
            - Verify touchscreen has ArUco field loaded
            - Touch index fingertip to touchscreen to record position
            - Repeat for 10 total touches
            - Verify offset saved in controller panel

## Guidance Accuracy Task (Fitt's proxy)
6. Performing the Guidance Accuracy task
   a. Enter Guidance Accuracy Task mode with [F]
   b. Set target randomly with [r], or 
      Set target manually with [m] followed by 2-digit ArUco ID (e.g., [03])


## Shutdown
9. To shut down the software safely, press [Esc]
   a. Sends a final {IDLE} command to the Teensy to disable output
   b. Safely closes all openCV windows
   c. Releases the camera and related components

      