# NURing Variable Naming Convention Reference

Use this document as the authoritative reference for variable naming throughout
the NURing codebase. All new code should follow these conventions. Variables
marked [NOT YET IMPLEMENTED] should be defined as stubs (declared with comments
but not yet functional) to reserve the names.

---

## Motor / Actuator Space

| Symbol | Code name | Type | Units | Description |
|--------|-----------|------|-------|-------------|
| q_i | `q_abs_A`, `q_abs_B`, `q_abs_C` | float | rad | Absolute motor encoder angle from unspooled zero (i ∈ {A, B, C}) |
| q_home_i | `q_home_A`, `q_home_B`, `q_home_C` | float | rad | Absolute encoder position at neutral home pose after pretensioning |
| r_p | `CONSTANT_MOTOR_PULLEY_RADIUS` | float | m | Bare pulley radius (2.5 mm = 0.0025 m) |
| t | `CONSTANT_TENDON_THICKNESS` | float | m | Tendon ribbon thickness (0.3 mm = 0.0003 m) |
| r_eff_i | `r_eff_A`, `r_eff_B`, `r_eff_C` | float | m | Spool-corrected effective pulley radius: r_p + (t / 2π) × q_abs_i |
| Δℓ_i | `dL_A`, `dL_B`, `dL_C` | float | m | Spool-corrected tendon length change from home: r_p·(q_i − q_home_i) + (t/4π)·(q_i² − q_home_i²) |
| T_i | `tension_A`, `tension_B`, `tension_C` | float | N | Motor tendon tension |
| τ_i | `torque_A`, `torque_B`, `torque_C` | float | N·m | Motor torque: T_i × r_eff_i |
| I_i | `current_A`, `current_B`, `current_C` | float | A | Motor current: τ_i / K_t |
| K_t | `CONSTANT_MOTOR_TORQUE_CONSTANT` | float | N·m/A | Motor torque constant (0.0144) |

---

## Tendon Geometry

| Symbol | Code name | Type | Units | Description |
|--------|-----------|------|-------|-------------|
| W | `W_matrix` | float[2][3] | unitless | Tendon direction matrix, columns are unit vectors at motor angles |
| w_A | `CONSTANT_UNIT_VECTOR_A` | Point2f | unitless | [cos 35°, sin 35°] |
| w_B | `CONSTANT_UNIT_VECTOR_B` | Point2f | unitless | [cos 145°, sin 145°] |
| w_C | `CONSTANT_UNIT_VECTOR_C` | Point2f | unitless | [cos 270°, sin 270°] = [0, −1] |

---

## Virtual Task Space (2D)

| Symbol | Code name | Type | Units | Description |
|--------|-----------|------|-------|-------------|
| p_v | `pos_virtual` | Point2f | mm | Virtual fingertip position from encoder-based pseudoinverse mapping |
| p_t | `pos_target` | Point2f | mm | Target position (desired deflection or goal) |
| e | `error` | Point2f | mm | Fingertip-to-target error: p_t − p_f |
| θ | `theta` | float | rad or deg | Heading angle in the virtual XY task space (direction of deflection, error heading, calibration angle) |

---

## Camera / 3D Space

| Symbol | Code name | Type | Units | Description |
|--------|-----------|------|-------|-------------|
| p_c | `pos_camera` | Point3f | mm | Camera position relative to target (from solvePnP translation) |
| p_f | `pos_fingertip` | Point3f | mm | Estimated fingertip contact point: p_c + R(ψ_roll) · d |
| p_t | `pos_target_3d` | Point3f | mm | Target position in 3D (from ArUco tag pose) |
| d | `offset_cam_to_fingertip` | Point3f | mm | Fixed translational offset from camera origin to fingertip in finger's local frame (from calibration step 3) |
| ψ_roll | `roll_current` | float | rad | Current hand roll angle extracted from solvePnP rotation matrix |
| ψ_roll_ref | `roll_reference` | float | rad | Reference hand roll angle captured during offset calibration |
| R(ψ) | `R_roll` | Mat3f | unitless | Rotation matrix for roll correction of the offset vector |
| z | `depth_to_target` | float | mm | Depth (Z component) from camera to target |

---

## Controller (PID)

| Symbol | Code name | Type | Units | Description |
|--------|-----------|------|-------|-------------|
| K_p | `gain_kP` | float | N/mm | Proportional gain (may become direction-dependent via K(θ)) |
| K_d | `gain_kD` | float | N·s/mm | Derivative gain |
| K_i | `gain_kI` | float | N/(mm·s) | Integral gain |
| F_x, F_y | `force_x`, `force_y` | float | N | Commanded force in virtual task space |
| F_max | `force_max` | float | N | Maximum allowable force magnitude |
| T_min | `tension_min` | float | N | Minimum tension (preload) per motor |
| T_max | `tension_max` | float | N | Maximum tension per motor |
| G(z) | `gain_depth_schedule` | float | unitless | [NOT YET IMPLEMENTED] Depth-dependent gain scaling: clamp((z − z_min) / (z_taper − z_min), G_floor, 1.0) |
| z_min | `DEPTH_MIN` | float | mm | [NOT YET IMPLEMENTED] Closest reliable tracking distance (~40 mm) |
| z_taper | `DEPTH_TAPER` | float | mm | [NOT YET IMPLEMENTED] Distance at which gain tapering begins (~150 mm) |
| G_floor | `DEPTH_GAIN_FLOOR` | float | unitless | [NOT YET IMPLEMENTED] Minimum gain at close range (~0.2) |

---

## Per-Participant Calibration

| Symbol | Code name | Type | Units | Description |
|--------|-----------|------|-------|-------------|
| AROM boundary | `arom_boundary` | float[] | mm | [NOT YET IMPLEMENTED] Direction-dependent active range of motion boundary, sampled at calibration angles, interpolated with periodic spline |
| K(θ) | `stiffness_profile` | float[] | N/rad | [NOT YET IMPLEMENTED] Direction-dependent finger deflection stiffness from calibration ramps: K(θ) = ΔF / Δφ |
| C(θ) | `mapping_correction` | float[] | unitless | [NOT YET IMPLEMENTED] Virtual mapping correction factor: C(θ) = φ_camera / φ_virtual |
| φ | `phi_camera` | float | rad | Camera-measured angular deflection of finger (ground-truth from solvePnP rotation during stiffness calibration) |
| φ_virtual | `phi_virtual` | float | rad | Virtual-mapping-estimated angular deflection (from encoder pseudoinverse, for comparison against φ) |

---

## Calibration Angles

The following angles are used for AROM boundary sampling and stiffness probing.
0° = right (abduction), 90° = up (extension), standard math convention.

| Angle | Category | Description |
|-------|----------|-------------|
| 0°   | Cardinal | Pure abduction (right) |
| 35°  | Motor    | Motor A axis (dorsal-lateral) |
| 90°  | Cardinal | Pure extension (up, A+B combined) |
| 145° | Motor    | Motor B axis (dorsal-medial) |
| 180° | Cardinal | Pure adduction (left) |
| 210° | Ventral intermediate | Between adduction and flexion |
| 240° | Ventral intermediate | Between adduction and flexion |
| 270° | Motor    | Motor C axis (pure flexion, down) |
| 300° | Ventral intermediate | Between flexion and abduction |
| 330° | Ventral intermediate | Between flexion and abduction |

---

## Naming Rules

1. **Positions** are lowercase bold **p** with subscript: `pos_virtual`, `pos_camera`, `pos_fingertip`, `pos_target`
2. **Motor/actuator quantities** use `q` for encoder angles, `T` for tension, `τ` for torque, `I` for current
3. **Angles in task space** use `theta` (heading/direction)
4. **Camera-measured deflection** uses `phi` (ground-truth finger deflection magnitude)
5. **Motor encoder angles** use `q` with subscript (q_abs_A, q_home_A, etc.)
6. **Error** is `error` or `e` = p_t − p_f
7. **Offsets** use `d` (camera-to-fingertip offset vector)
8. **Roll angle** uses `psi` / `roll_current`, `roll_reference`
9. **Gains and stiffness** use `K` with descriptive subscript or `gain_` prefix
10. **Calibration profiles** stored as arrays indexed by calibration angle: `stiffness_profile`, `mapping_correction`, `arom_boundary`
11. **Constants** use `CONSTANT_` or `UPPERCASE` prefix
12. **[NOT YET IMPLEMENTED]** items should be declared as stubs with comments noting they are placeholders for future implementation
