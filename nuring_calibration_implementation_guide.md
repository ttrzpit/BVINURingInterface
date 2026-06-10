# NURing Calibration & Controller Pipeline — Implementation Reference

This document describes the three-stage per-participant calibration procedure and
the corrected controller pipeline for the NURing device. Use this as the
authoritative reference for implementation. See `nuring_variable_conventions.md`
for variable naming.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Hardware Context](#hardware-context)
3. [Pretensioning & Encoder Zeroing](#pretensioning--encoder-zeroing)
4. [Virtual Fingertip Mapping (Spool-Corrected)](#virtual-fingertip-mapping-spool-corrected)
5. [Calibration Stage 1: Finger Active Range of Motion](#calibration-stage-1-finger-active-range-of-motion)
6. [Calibration Stage 2: Finger Deflection Stiffness](#calibration-stage-2-finger-deflection-stiffness)
7. [Calibration Stage 3: Camera-to-Fingertip Offset](#calibration-stage-3-camera-to-fingertip-offset)
8. [Controller Pipeline (Corrected)](#controller-pipeline-corrected)
9. [Depth-Dependent Gain Scheduling](#depth-dependent-gain-scheduling)
10. [Bugs Fixed from Previous Code](#bugs-fixed-from-previous-code)
11. [Implementation Priority](#implementation-priority)

---

## System Overview

The NURing is a tendon-driven wearable ring that deflects the index finger about
the MCP joint to provide directional guidance cues. Three BLDC motors (A, B, C)
pull tendons routed to an actuation ring on the middle phalanx. A ring-mounted
camera provides visual feedback for closed-loop guidance toward ArUco-tagged targets.

The system runs on two compute platforms:
- **Teensy 4.1 microcontroller**: runs the control loop at 1000 Hz, sends PWM
  commands to motor amplifiers, reads motor encoder counts from amplifiers at 100 Hz
- **PC (Linux/Ubuntu)**: runs the camera pipeline (OpenCV, ArUco detection,
  solvePnP), calibration analysis, and high-level task logic

The controller pipeline on the Teensy is:
```
Desired Position → PID Force → Tension Solver → Current → PWM
```

The camera pipeline on the PC provides target position information that feeds
into the desired position for the controller.

---

## Hardware Context

### Motor Layout (standard math convention: 0° = right, 90° = up)
- **Motor A**: 35° (dorsal-lateral, up-right)
- **Motor B**: 145° (dorsal-medial, up-left)
- **Motor C**: 270° (volar, straight down / pure flexion)

### Motor Constants
- Bare pulley radius: r_p = 2.5 mm (0.0025 m)
  - NOTE: the old code has this wrong as 0.003 m (3 mm) with comment "6mm diameter"
  - Must be corrected to 0.0025 m
- Tendon thickness: t = 0.3 mm (0.0003 m)
- Pulley groove width: 5 mm (tendon is 4.5 mm, so neat stacked layers)
- Torque constant: K_t = 0.0144 N·m/A
- Max current: 1.89 A
- PWM range: 24 (max current) to 2047 (off), inverted scale
- PWM zero-torque: 2024

### Tendon Direction Unit Vectors
- w_A = [cos(35°), sin(35°)]
- w_B = [cos(145°), sin(145°)]
- w_C = [cos(270°), sin(270°)] = [0, -1]

### Camera
- Wide-angle global shutter, 120° FOV
- 1600×1200 pixels at 60 Hz
- Mounted on top of actuation ring (dorsal side of middle phalanx)
- ArUco detection + solvePnP via OpenCV on PC

### Touchscreen
- Melfas LGDisplay Incell Touch (xinput device id=9)
- Reports touch contact coordinates
- Used to display ArUco tag grids during calibration and for the target
  acquisition task

---

## Pretensioning & Encoder Zeroing

Before calibration or guidance, the system must establish absolute encoder
references for accurate spool-corrected radius computation.

### Procedure
1. Command all motors to fully release tendons (unspool completely)
2. Zero all three motor encoders → this is the absolute zero (bare pulley, no tendon wound)
3. Command pretensioning to take up tendon slack and seat the ring
4. Once the participant's finger is in the neutral home pose (index finger extended),
   record each motor's absolute encoder position as `q_home_i`

### Why This Matters
During pretensioning, each motor spools approximately 5-7 revolutions of tendon.
This increases the effective pulley radius from 2.5 mm to approximately 4.0-4.6 mm.
Each motor may spool a different amount depending on tendon routing and anatomy.
The absolute encoder positions allow the system to compute the correct effective
radius at any point during guidance.

---

## Virtual Fingertip Mapping (Spool-Corrected)

### Purpose
Estimate the 2D fingertip deflection position from motor encoder readings without
requiring a sensor on the fingertip.

### Equations

**Effective radius** at absolute encoder position q_i:
```
r_eff_i(q_i) = r_p + (t / 2π) × q_i
```

**Tendon length change** from home position:
```
Δℓ_i = r_p × (q_i − q_home_i) + (t / 4π) × (q_i² − q_home_i²)
```

This is the integral of r_eff over the encoder displacement, accounting for the
continuously increasing winding radius.

**Virtual fingertip position** via pseudoinverse projection:
```
p_v = (W × W^T)^{-1} × W × Δℓ
```

where W is the 2×3 tendon direction matrix and Δℓ = [Δℓ_A, Δℓ_B, Δℓ_C]^T.
The matrix (W × W^T)^{-1} × W is constant (depends only on motor angles) and
should be precomputed.

### Change from Old Code
The old code used a constant bare pulley radius for Δℓ = q_i × r_p, ignoring
spool-up entirely. After ~7 revolutions of pretensioning, this underestimates
tendon displacement by roughly a factor of 2. The controller still worked because
it drives error to zero in whatever units the virtual space uses, but the
reported "millimeters" did not correspond to physical millimeters.

---

## Calibration Stage 1: Finger Active Range of Motion

### Purpose
Map the safe limits of fingertip deflection per participant to prevent
over-deflection during guidance and stiffness probing.

### Procedure
1. Participant rests forearm on arm rest
2. Holding tension is applied to stabilize the ring
3. Participant is instructed to trace circles with their index finger, pivoting
   about the MCP joint, for 10 seconds
4. Motor encoder positions are recorded and mapped to virtual fingertip positions
   at 100 Hz, producing several hundred data points over multiple revolutions

### Analysis
1. Convert all points to polar coordinates (angle θ, radius r from origin)
2. Bin by angular heading, centered on 10 calibration angles:
   - Motor angles: 35°, 145°, 270°
   - Cardinal angles: 0°, 90°, 180°
   - Ventral intermediates: 210°, 240°, 300°, 330°
3. Compute the 95th percentile of radial extent within each bin
4. Interpolate between the 10 boundary samples with a periodic cubic spline
   to define a smooth, direction-dependent AROM boundary

### Runtime Use
When the controller commands a fingertip deflection, if the commanded position
exceeds the AROM boundary at that heading, it is projected back onto the boundary.
Forces pushing outward past the boundary are attenuated; forces pulling inward
are not attenuated.

### Replaces
The ellipse-fit boundary from Study 2. The direction-dependent boundary captures
the actual workspace shape (which is not elliptical due to the asymmetric motor
layout and finger biomechanics).

---

## Calibration Stage 2: Finger Deflection Stiffness

### Purpose
Measure the stiffness of the finger in each calibration direction to enable
direction-dependent controller gains, and validate the virtual fingertip mapping
against camera ground truth.

### Setup
- Participant's forearm on arm rest, hand 30 cm from touchscreen
- Touchscreen displays a grid of ArUco tags (e.g., 5×7 grid of 20 mm tags)
- Camera pipeline running on PC, tracking multiple tags via solvePnP

### Procedure (for each of the 10 calibration angles)
1. System commands a slow linear force ramp along the calibration heading at
   +0.5 N/s, using the existing torque allocation solver to distribute force
   across motors
2. Ramp continues until the AROM boundary for that heading is reached
3. Hold at peak force for 0.5 s (steady-state measurement)
4. Ramp back down at -0.5 N/s until the finger returns to neutral

### Data Recorded at Each Timestep
- **Applied force**: computed from measured motor current (read from amplifiers),
  converted via: F = I × K_t / r_eff_i. Use MEASURED current, not commanded.
- **Virtual fingertip position**: from motor encoders via the spool-corrected
  pseudoinverse mapping
- **Camera pose**: rotation R and translation t from multi-tag solvePnP (pooling
  all visible ArUco tag corners into a single solvePnP call against the known
  grid geometry)

### Post-Processing

**Decompose camera motion:**
- The rotation component R represents finger deflection about the MCP joint
- The translation component t represents hand/wrist drift
- These fall out directly from solvePnP — no separate decomposition step needed

**Compute stiffness K(θ):**
- For each calibration angle θ, apply a least-squares linear fit to the
  force-vs-camera-measured-deflection curve during the ramp-up phase
- K(θ) = ΔF / Δφ, where F is force magnitude along heading θ and φ is the
  camera-measured angular deflection
- Retain ramp-down data to assess hysteresis

**Compute virtual mapping correction C(θ):**
- At each calibration angle, compute the ratio between camera-measured deflection
  and virtual-mapping-estimated deflection
- C(θ) = φ_camera / φ_virtual
- Computed as the mean ratio across the ramp
- At runtime, the corrected virtual position is: p_corrected = p_v / C(θ),
  where θ is the heading angle of p_v

### Runtime Use
- **K(θ)** replaces the fixed proportional gain K_p. The controller scales force
  commands by K(θ) at the current error heading to produce perceptually uniform
  deflections across directions. This is the principled replacement for the
  Fy suppression heuristic in the old code.
- **C(θ)** is applied to the virtual position estimate before the PID controller
  sees it, correcting for tendon slack, routing friction, and anatomy differences.

### Timing
~10 headings × ~4-6 seconds each = under 60 seconds total.

---

## Calibration Stage 3: Camera-to-Fingertip Offset

### Purpose
Measure the fixed translational offset between the camera (on the dorsal side of
the middle phalanx) and the fingertip contact point (volar tip of distal phalanx).
This offset ensures the guidance controller drives the fingertip to the target,
not the camera.

### Setup
- Same as stiffness calibration: participant 30 cm from touchscreen with ArUco
  tag grid displayed
- Touchscreen detecting touch input

### Procedure
1. Participant is asked to press their fingertip firmly against the touchscreen
   10 times at various positions (they can touch anywhere — no visual targeting
   needed, important for BVI participants)
2. At each stable contact (sustained touch for 200-300 ms), the system
   simultaneously records:
   - Fingertip touch coordinates from the touchscreen (in screen coordinates)
   - Camera pose relative to the ArUco tag grid from solvePnP
3. Since tag positions on the screen are known, the camera position can be
   expressed in screen coordinates
4. The vector from camera position to touch point is the offset

### Post-Processing
- Average the offset vector across all 10 touches → stored as **d** in the
  finger's local coordinate frame
- Record the hand's roll angle at calibration as the reference: ψ_roll_ref
- The offset has two components:
  - Vertical (dorsal-to-volar): camera is on top of finger, fingertip pad is below
  - Longitudinal (proximal-to-distal): camera is on middle phalanx, fingertip is
    at end of distal phalanx

### Runtime Use
During guidance, the fingertip position is estimated as:
```
p_f = p_c + R(ψ_roll - ψ_roll_ref) × d
```

where:
- p_c is the camera position from solvePnP
- ψ_roll is the current hand roll angle (extracted from solvePnP rotation matrix)
- ψ_roll_ref is the reference roll from calibration
- d is the calibrated offset in the finger's local frame
- R() rotates the offset by the roll difference

The guidance error is then:
```
e = p_t − p_f
```

The roll correction is necessary because the hand's roll can vary over a ~70°
range during reaching. At 20 mm offset and 35° roll, uncompensated lateral error
would be ~11 mm — comparable to the guidance accuracy from prior studies.

### Cross-Validation
The touchscreen contact point can be compared against the camera-predicted
fingertip position (p_c + offset) to verify the calibration accuracy.
1
### Timing
~10 touches × ~3-4 seconds each = under 40 seconds total.

---

## Controller Pipeline (Corrected)

The full corrected pipeline, running at 1000 Hz on the Teensy:

### Stage 0: Telemetry Update
1. Read absolute encoder positions q_abs_i from amplifiers
2. Compute spool-corrected effective radius: r_eff_i = r_p + (t/2π) × q_abs_i
3. Compute tendon length changes: Δℓ_i = r_p·(q_i − q_home_i) + (t/4π)·(q_i² − q_home_i²)
4. Compute virtual fingertip position via pseudoinverse: p_v = (WW^T)^{-1} W Δℓ
5. Apply mapping correction: p_corrected = p_v / C(θ) [if calibration available]
6. Low-pass filter and compute velocity

### Stage 1: PID Controller (Position → Force)
1. Compute error: e = p_t − p_corrected
2. Check deadband: if |e| < position_tolerance, return zero force, decay integrator
3. Update integrator with anti-windup: clamp to ±(0.5 × F_max / K_i)
4. Compute PID force:
   - F_x = K(θ) × e_x − K_d × v_x + K_i × ∫e_x dt
   - F_y = K(θ) × e_y − K_d × v_y + K_i × ∫e_y dt
   - NOTE: D-term uses v_y for F_y (old code had bug using v_x for both)
5. Apply AROM boundary wall: if force pushes outward past boundary, attenuate;
   inward forces are not attenuated
6. Saturate force magnitude to F_max

### Stage 2: Tension Solver (Force → Tension)
Projected gradient descent solving: min_T 0.5‖WT − F‖² s.t. T_min ≤ T_i ≤ T_max

1. If force is near-zero, return preload tensions
2. Initialize from previous cycle's solution (do NOT add preload — it's already
   included from prior convergence). First-cycle fallback: initialize to preload.
3. For 5 iterations:
   - Compute residual: r = W×T − F
   - Compute gradient: g = W^T × r
   - Projected step: T_i = clamp(T_i − α × g_i, T_min, T_max)
4. Step size α = 1/L where L = 2.5 (Lipschitz bound)
5. T_min = preload tension, T_max = maximum safe tension

### Stage 3: Tension → Current (Spool-Corrected)
```
I_i = T_i × r_eff_i / K_t
```
Uses per-motor spool-corrected radius, NOT the bare radius constant.
Clamp each to amplifier max current (1.89 A).

### Stage 4: Current → PWM
Linear inverted mapping: I=0 → PWM=2047 (off), I=max → PWM=24 (max torque).
No changes from original code.

---

## Depth-Dependent Gain Scheduling

### Purpose
Reduce guidance force as the finger approaches the target to prevent overshoot
at contact and transition control back to the participant.

### Implementation [NOT YET IMPLEMENTED]
```
G(z) = clamp((z - z_min) / (z_taper - z_min), G_floor, 1.0)
```

where:
- z = depth from camera to target (from solvePnP translation Z component)
- z_min = ~40 mm (closest reliable ArUco tracking distance)
- z_taper = ~150 mm (distance where tapering begins)
- G_floor = ~0.2 (minimum gain, not zero — maintain some guidance)

Applied to the error before the PID controller:
```
e = G(z) × (p_t − p_f)
```

### Design Rationale
- At 40 cm with 20 mm lateral offset: full deflection cue is appropriate
- At 2 cm with 20 mm lateral offset: strong lateral cue would overshoot
- Connects to DG3 (user agency): system guides during approach, yields near contact
- Can be paired with audible proximity cue at close range for multimodal feedback

---

## Bugs Fixed from Previous Code

| # | Location | Bug | Fix | Impact on Prior Studies |
|---|----------|-----|-----|----------------------|
| 1 | Controller.cpp line 508 | D-term uses measuredVel2f_.x for both Fx and Fy | Use .y for Fy | None — K_d was zero |
| 2 | Controller.cpp line 629 | Fy zeroed when |Fx| > 1.5×|Fy| (±34° dead band) | Remove; replace with K(θ) from stiffness calibration | Active during studies — affected force at near-horizontal headings |
| 3 | Controller.cpp line 636 | Preload added to previous tensions every cycle (double-counting) | Initialize from prior solution; preload as lower bound only | Masked by fMax clamping — biased solver starting point |
| 4 | Controller.cpp line 690 | Tension→current uses constant bare pulley radius | Use per-motor spool-corrected r_eff_i | Commanded ~half the needed current; controller compensated but telemetry values are wrong |
| 5 | MapPositionToForce | No force magnitude saturation before solver | Add clamp to F_max | Solver could be asked for infeasible forces, biasing output direction |
| 6 | MapPositionToForce | Integrator clamp at ±2×fMax (arbitrary) | Clamp based on K_i contribution budget | Minor — could allow integrator windup |
| 7 | Globals.h line 72 | Pulley radius = 0.003 m (3 mm), comment says "6mm diameter" | Correct to 0.0025 m (2.5 mm), "5mm diameter" | Scale error in virtual mapping (absorbed by proportional controller) |
| 8 | Virtual mapping | Constant pulley radius ignores spool-up | Integral formula with per-motor q_home | ~2× scale error after pretensioning (absorbed by proportional controller) |

---

## Implementation Priority

### Phase 1: Foundation (implement first)
1. Add `CONSTANT_TENDON_THICKNESS` to Globals.h
2. Fix `CONSTANT_MOTOR_PULLEY_RADIUS` to 0.0025 m
3. Implement pretensioning encoder zeroing procedure (unspool → zero → tension → record q_home)
4. Implement spool-corrected virtual fingertip mapping
5. Fix D-term bug (vel.y for Fy)
6. Fix preload double-counting in tension solver
7. Add force magnitude saturation
8. Implement spool-corrected tension-to-current conversion
9. Remove Fy suppression heuristic (leave as commented-out code with note)

### Phase 2: AROM Calibration
1. Implement data collection (10-second circular trace, record virtual positions)
2. Implement polar binning and 95th percentile extraction at 10 calibration angles
3. Implement periodic cubic spline interpolation for boundary
4. Replace ellipse boundary enforcement with AROM boundary enforcement

### Phase 3: Stiffness Calibration
1. Implement force ramp procedure (ramp up → hold → ramp down per heading)
2. PC-side: implement multi-tag solvePnP for ground-truth finger deflection
3. PC-side: compute K(θ) from force-vs-deflection curves
4. PC-side: compute C(θ) from camera vs virtual mapping comparison
5. Implement direction-dependent K_p in controller
6. Implement C(θ) correction on virtual position

### Phase 4: Camera-to-Fingertip Offset
1. Implement touchscreen contact detection (stable touch trigger)
2. Simultaneous recording of touch coordinates + camera pose
3. Compute offset vector d in finger local frame
4. Record reference roll angle
5. Implement runtime roll-corrected offset compensation
6. Implement roll extraction from solvePnP rotation matrix

### Phase 5: Depth Gain Scheduling
1. Extract Z from solvePnP translation
2. Implement G(z) gain function
3. Apply to error before PID controller
4. Tune z_taper, z_min, G_floor parameters empirically
