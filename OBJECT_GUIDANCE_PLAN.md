# Object-Guidance Mode (`'O'`) — Implementation Plan

**Status:** ALL PHASES (1–6) implemented on branch `object-guidance-mode` and verified
off-rig; ON-RIG VERIFICATION STILL PENDING (see below). The build compiles/links clean.
What was verified off-rig, per phase:
- P1 config parse: 36 board positions, 6 objects (types/params/target points correct).
- P2 detector: a DICT_6X6_100 detector decodes markers across the world+object ID range.
- P3 `WorldObjectHandler`: synthetic-scene pose/anchor math to sub-micron error, incl.
  anchored persistence when the object marker is occluded but the board is visible.
- P4 overlays: render to coherent, correctly-coloured objects (green live / yellow
  anchored, wireframe, gizmo, target dot); gated off by default.
- P5 state/keyboard: OBJECTS FSM transition test passes, incl. a FITTS regression.
- P6 integration: end-to-end selection -> Update -> guidance-target chain (real
  KeyboardHandler + WorldObjectHandler through main's per-frame sequence) resolves the
  Y-up target to sub-micron error and persists it via the anchor when occluded; detector
  toggles correctly on OBJECTS entry/exit.

The FITTS/Cal hot paths are byte-for-byte unchanged (all OBJECTS work is gated behind
`systemState == OBJECTS`; `activeTagId` stays 0 so the Fitts guidance/circle paths are
dormant). NOT yet committed.

LOCALISATION (final design - "clean global PnP", no filtering). After investigating
several approaches in simulation (relative/constellation anchoring; EMA smoothing;
corner-PnP + bundle-adjust), the data settled it:
  - Config-independent anchoring (learn geometry from single-marker IPPE) carries an
    8-22 mm noise-induced BIAS - single squares are depth-biased and it bakes into the
    map. Rejected.
  - EMA smoothing removes jitter but adds visible LAG (occluded/moved object floats into
    place). Removed entirely per the requirement of minimal delay.
  - Sim showed CONFIG ACCURACY is the whole game: with ~1 mm world positions a plain
    global PnP is 0.08 mm jitter / 0.8 mm bias; with +/-5 mm config it is ~6 mm jitter.
    Beating a rough config from the sweep alone needs a full joint bundle-adjust (a big,
    delicate feature); accurate config makes it unnecessary.
So the implemented model (WorldObjectHandler) is:
  * World->camera: one multi-marker solvePnPRansac each frame over the detected world
    markers' CENTRES <-> their config `marker_positions` (orientation-agnostic, RANSAC
    rejects a misdetected marker). Accuracy of `marker_positions` sets everything -
    measure to ~1 mm.
  * Object VISIBLE: use the object marker's OWN pose directly - tracks instantly, NO
    averaging/smoothing (fixes the float/lag). Each visible frame refreshes a
    single-frame anchor (object pose in the world frame).
  * Object OCCLUDED: reconstruct from the held anchor via the current (steady) world
    pose, so it stays put under the reaching hand.
There is NO temporal filtering anywhere. Verified off-rig: a 270 mm object jump tracks
in ONE frame (zero lag); occluded + moving-camera + noise holds to 0.56 mm mean / 0.12 mm
jitter with accurate config. Operator HUD shows world-marker count, pose OK/--, and the
active object's LIVE / ANCHORED / not-seen / lost state; world markers are outlined faint
blue. `object_world.world_marker_size_mm` is currently unused (world pose uses centres).

REMAINING (on-rig, §7): with the physical 6x6 world board + tagged objects, confirm
detection at 90 Hz (`GetDetectionHz`), overlay orientation, and — critically — the Y-up
guidance sign (a wrong sign drives the finger the wrong way): TEST WITH GUIDANCE OUTPUT
OFF FIRST. Trial logging for the object task is still out of scope (guide continuously,
stop via spacebar).

Self-contained handoff so a fresh Claude Code instance opened in this repo can execute
it without prior conversation.

**Goal:** add a new in-app mode that guides the NURing to **real physical objects**
tagged with ArUco markers, mirroring how the Fitts task (`'F'`) works. Prototyped
separately in `~/Code/ArUcoTest` (world board + object wireframes + a per-object
guidance target); this folds that capability in as a first-class `SystemState`.

**Priorities (do not compromise):** program stability, 90 Hz loop-frequency
stability, ArUco detector stability. This runs participant studies.

---

## 1. Core decisions (settled)

- **In-app mode, NOT a separate program.** It needs the existing controller/serial/
  robot stack to guide, and a separate binary would duplicate that. It's added as a
  new `SystemState::OBJECTS`, entered with `'O'` from `IDLE`, exactly like `'F'`
  enters `FITTS`.
- **Single detection thread only.** `main.cpp` sets `cv::setNumThreads(1)` after a
  past intermittent heap-corruption/double-free in `cv::findContours` from concurrent
  OpenCV `parallel_for` across camera + detection threads. **Never** add a second
  detection thread or a second `VideoCapture`. OBJECTS mode reuses the one
  `ArucoHandler` detection thread, swapping to a new dictionary (see Phase 2).
- **Gate everything.** All OBJECTS per-frame work lives behind
  `if (kb.systemState == SystemState::OBJECTS)`. The Fitts/Cal hot paths must be
  byte-for-byte unchanged when not in OBJECTS mode.
- **Finite/NaN guards are mandatory** on every pose and projected point (see §6).

### The four operator decisions
1. **Target selection = mirror Fitts.** `'O'` → `OBJ_SEL`, then:
   - `[m]` → numeric entry of the object marker ID → active target.
   - `[r]` → random pick from a new `object_marker_pool` config list (no repeat until
     exhausted, like the Fitts random pool).
   Also apply the same calibration-incomplete `[p]roceed / [r]eturn` gate `'F'` uses.
2. **Reuse the Cal3 fingertip offset** — the object target flows through the same
   `controller.SetTarget(…, cal3.IsComplete(), cal3.GetFinalOffset(), cal3.GetRollRef(), …)`
   path as Fitts, so the fingerpad lands on `target_point_mm`.
3. **Stop guidance = spacebar (ALREADY WIRED, no new code).** Space is bound to
   `KeyAction::TOGGLE_ESTOP` (`KeyCommandTable.h`), and `main.cpp` flips
   `controller.SetGuidanceOutputEnabled()` on it in any state. Pressing space when the
   participant reaches the object zeros guidance (tension only); press again to re-arm.
   (Later enhancement: auto-re-arm on next object select.)
4. **Physical world board present; use `marker_positions` a priori.** Knowing the world
   marker layout lets all visible world markers solve **one** camera→world pose (PnP),
   which (a) keeps an object correctly localized when the reaching hand occludes the
   object's own marker, and (b) is steadier than any single marker. When the object
   marker IS visible, guidance uses it directly; the world board is the fallback.

---

## 2. Config additions

Add a new section to this repo's `config.yaml` (OpenCV FileStorage format, same as the
existing file). You (Tom) will populate the values — port `marker_positions` and
`target_objects` from `~/Code/ArUcoTest/config.yaml`.

```yaml
object_world:
   # World board marker centres (id -> XYZ mm) in the rig/world frame.
   # Ported from ArUcoTest. Establishes the world frame for occlusion-robust
   # object persistence.
   marker_positions:
      - { id: 1, x: -365.0, y: 357.5, z: 0.0 }
      # ... (36 entries)

   # Object marker IDs to randomize among for the 'r' selection.
   object_marker_pool: [ 60, 61, 62, 70, 71, 72 ]

   # Virtual objects tagged by object markers. Two shapes:
   #   type: "box"      -> 8 explicit corner points (marker-frame mm).
   #   type: "cylinder" -> base_origin_mm + base_normal_mm + cylinder_radius_mm +
   #                       cylinder_height_mm.
   # target_point_mm = the guidance target the ring drives to.
   #   BOX:      target_point_mm is relative to the MARKER origin.
   #   CYLINDER: target_point_mm is relative to base_origin_mm.
   target_objects:
      - id: 60
        name: "pen"
        type: "cylinder"
        marker_size_mm: 40
        cylinder_radius_mm: 7
        cylinder_height_mm: 138
        base_origin_mm:  { x: -65, y: 41, z: 5 }
        base_normal_mm:  { x: 1,  y: 0,  z: 0 }
        target_point_mm: { x: 95, y: 0,  z: 0 }
      # ... (box example)
      - id: 70
        name: "cellphone"
        type: "box"
        marker_size_mm: 40
        points:
           - { x: -32.0, y:  32.0, z:   0.0 }   # 1 top UL … 8 bottom BR
           # ... (8 entries, order: top UL/UR/BL/BR then bottom UL/UR/BL/BR)
        target_point_mm: { x: 46, y: 0, z: 0 }
```

Marker geometry conventions (marker frame, mm): origin = marker centre, **+X right,
+Y up, +Z out of the marker face** (toward camera). Object body extends toward −Z.

**Object marker dictionary = `DICT_6X6_100`** (world board 1–36 + objects 50–90 all in
it). This differs from the Fitts/Cal `DICT_4X4_*` dictionaries — that's fine, OBJECTS
mode swaps detectors (Phase 2).

---

## 3. Source to port from (`~/Code/ArUcoTest/main.cpp`)

The pose/geometry math is already written and debugged there. Lift these:
- `struct Config` object structs + `readPoint3f()` + `loadMarkerPositions()` +
  `loadObjects()` — config parsing.
- `struct TargetObject`, `ObjType`, `objectEdges()` — box/cylinder wireframe edges
  (incl. general cylinder: base circle + top circle + 2 side lines along `base_normal`).
- `markerWorldCorners()` — world marker square from centre.
- `projectMarkerPt()` and `finite3()` — **the NaN-guarded projection helpers.**
- The main-loop object block: world PnP (`SOLVEPNP_SQPNP` over
  `marker_positions ↔ detected centres`), the persistent `ObjectAnchor`
  (`marker→world = (world→camera)⁻¹ · (marker→camera)`), the "prefer live pose, fall
  back to anchor" selection, and the overlay draw (green/yellow outline, wireframe,
  base gizmo, magenta target dot).

**Frame-convention gotcha:** ArUcoTest works in OpenCV camera frame (**Y-down**).
BVINURing's `DetectedMarker::positionMm` and `controller.SetTarget()` use camera frame
**Y-up** (see `ArucoHandler.cpp` where `tvec.y` is negated). When handing the resolved
object target to the controller, **negate Y** to match BVINURing's convention.

---

## 4. Six-phase build (in this order)

### Phase 1 — Config (`include/Config.h`, `src/Config.cpp`)
- Add `ObjectWorldConfig` (mirrors the ArUcoTest structs: world positions map, object
  list, marker pool) as a new `Config` member `objectWorld`.
- Parse it in `Config::load()` following the existing per-section pattern (the file
  already uses `cv::FileStorage`; reuse ArUcoTest's readers).
- **Verify:** builds; the objects/positions/pool parse (print counts at startup).

### Phase 2 — Detection sub-mode (`include/ArucoHandler.h`, `src/ArucoHandler.cpp`)
- Add `objDetector_` built from `DICT_6X6_100` (mirror how `calDetector_` /
  `calGridDictionary_` are set up in `initDetector`/ctor).
- Add `SetObjectDetection(bool)` (mirror `SetCalibrationDetection` @ ~L515 /
  `SetFittsBoardDetection` @ ~L565): on true, select `objDetector_` and set
  `activeValidIdMin_/Max_` to cover the world+object ID range (e.g. 1..99); on false,
  restore defaults.
- In `RunDetection()` extend the detector selection (currently
  `useCalDetector_ ? calDetector_ : detector_`) to pick `objDetector_` when the object
  flag is set.
- **No other detection changes.** `RunDetection` already returns `cornersPx` for every
  in-range marker; the new handler (Phase 3) does all PnP on the main thread from those
  corners. The "pose only for active target" fast path stays as-is.
- **Verify on rig:** enter OBJECTS mode, confirm 6×6 markers are detected and their
  outlines draw.

### Phase 3 — `WorldObjectHandler` (`include/WorldObjectHandler.h`, `src/WorldObjectHandler.cpp`)
`file(GLOB src/*.cpp)` auto-adds the .cpp. Mirror `FittsTaskHandler`'s shape:
- ctor takes `const ObjectWorldConfig&` + `const CameraConfig&` (for intrinsics).
- `Reset()`, `OnNewTarget(int objectMarkerId)`, `Update(const std::vector<DetectedMarker>& markers)`.
- Owns: world camera pose (SQPNP), persistent per-object anchors, and resolution of the
  active object's `target_point` into a **camera-frame Y-up** position + roll.
- Getters: `bool HasTarget()`, `cv::Point3f GetTargetPosMm()` (Y-up), `float GetTargetRoll()`
  for guidance; plus overlay accessors (outlines, wireframe edges, gizmo, target dot,
  per-object visible/anchored state) for the display.
- **Port the finite/NaN guards** — reject non-finite `solvePnP` output before use, and
  guard projected points before drawing.
- **Verify:** target position tracks the object; anchored persistence works when the
  object marker is covered but world markers are visible.

### Phase 4 — Display overlays (`include/DisplayHandler.h`, `src/DisplayHandler.cpp`)
- Add setters + draw for the OBJECTS overlays on the operator view (green outline when
  the object marker is visible, yellow when anchored/occluded; magenta wireframe / green
  or yellow per visibility as in ArUcoTest; base-origin XYZ gizmo; magenta target dot).
- Reuse the existing displayed-frame ↔ detection pairing.
- Touchscreen shows nothing new (object markers are physical) — the OBJECTS entry block
  leaves the grid/board hidden.
- Note: `DisplayHandler.cpp` is pinned to `-O2` in CMake (GCC ICE workaround) — keep it.

### Phase 5 — State + keyboard wiring
- `include/KeyboardHandler.h`: add `SystemState::OBJECTS`; add `InputState` values
  `OBJ_SEL`, `OBJ_RUN`, `OBJ_ACT` (mirror `FIT_SEL/FIT_RUN/FIT_ACT`); add
  `KeyAction::SET_OBJECT_TARGET` (+ random variant if not reusing the Fitts one).
- `src/KeyboardHandler.cpp`: extend `DeriveSystemState()` to map `OBJ_*` → `OBJECTS`
  (mirror `FIT_*`→`FITTS`); handle the new KeyAction in `ExecuteAction()` (set the active
  object id; random path picks from `object_marker_pool`).
- `include/KeyCommandTable.h`: add rows mirroring the `'F'` block —
  `{ {'O'}, IDLE, OBJ_SEL, "Select object: [m] by ID, [r] random...", NONE }`, the
  `[p]/[r]` calibration-gate rows, `[m]`→`OBJ_ACT`, `[r]`→`OBJ_RUN` (random action);
  add a `kNumericEntryTable` row for `OBJ_ACT` (object marker ID range).
- Track an active object id in `KeyboardState` (a dedicated `activeObjectId` is cleaner
  than overloading `fittsTargetId`).

### Phase 6 — Guidance hookup (`main.cpp`)
- Add an `OBJECTS` branch in the `SystemState` transition block (~L301): on enter,
  `aruco.SetObjectDetection(true)` + `worldObj.Reset()`; on exit,
  `aruco.SetObjectDetection(false)`. Structurally identical to the `FITTS` block (~L320).
- Construct `WorldObjectHandler worldObj(cfg.objectWorld, cfg.camera)` with the other
  handlers (~L114).
- Per-frame, gated by `kb.systemState == SystemState::OBJECTS`:
  `worldObj.Update(markers);` then feed its target into the **existing**
  `controller.SetTarget(worldObj.HasTarget(), <Y-up target>, worldObj.GetTargetRoll(),
  cal3.IsComplete(), cal3.GetFinalOffset(), cal3.GetRollRef(), cfg.target.offsetDefaultMm,
  !guidanceSuppressed)`. Reset the guidance ramp on object change (mirror the
  `activeTagId != prevActiveTagId` handler ~L657, incl. `controller.ResetRamp(nowSecs)`).
- Pass `worldObj`'s overlay data to `display`.
- Existing RobotState ladder + spacebar e-stop handle output enable/disable — no changes.
- **Verify on rig:** select object → ring guides to `target_point_mm` → spacebar stops.

---

## 5. Recommended sequencing / testing
1. Phase 1 + 2 → confirm 6×6 detection on the rig (no guidance yet).
2. Phase 3 + 4 → confirm overlays + target tracking + anchored persistence.
3. Phase 5 + 6 → confirm live guidance + spacebar stop.
Work on a branch (e.g. `object-guidance-mode`); keep `main` clean.

## 6. Stability guardrails (must hold)
- One detection thread, one camera. No new threads, no `cv::setNumThreads` change.
- All OBJECTS per-frame work gated behind the mode.
- `finite3(rvec) && finite3(tvec)` on every solve; `projectMarkerPt` rejects
  non-finite before drawing. (This is the same crash class the repo already fought in
  `cv::findContours`.)
- Detector cost for ~36 world + a few object markers ≤ the 450-marker Fitts board, so
  90 Hz is safe — but confirm with the existing `GetDetectionHz()` readout.

## 7. Open items to confirm on the rig / later
- Object marker mounting: `WorldObjectHandler` assumes marker +Y = object's up etc.
  (per ArUcoTest conventions). Verify wireframes render in the right orientation.
- Verify the Y-up sign conversion produces guidance in the correct direction (a wrong
  sign sends the finger the wrong way — test with guidance output OFF first).
- Trial logging for the object task is out of scope for v1 (guide continuously; stop
  via spacebar). Add an end condition + logging later.
