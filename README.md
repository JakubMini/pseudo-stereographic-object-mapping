# pseudo-stereographic object mapping

Hold a button, walk around an object with an Android phone, let go, and get a STEP file you can
open in CAD.

> **Status: built, not yet proven on a real phone.** The engine reconstructs synthetic scans within
> its test targets and its STEP files open in FreeCAD; the app runs end to end on an arm64 emulator
> with a recorded scan. Scanning with ARCore on a real phone, and accuracy against calipers, have
> not been done yet. Until they have, treat it as unproven.

## What it is

- An **Android app** (Kotlin, Jetpack Compose, ARCore) that records a scan: keyframes, ARCore
  camera poses, raw depth where the phone has it, feature points and IMU data, saved as it goes.
- A **C++ engine** that turns a recorded scan into a mesh and then a STEP file. The same code runs
  as a desktop command-line tool and inside the app.

## What the STEP is, and isn't

The export is **faceted geometry**: every triangle of the scanned mesh becomes a flat CAD face. It
is valid CAD exchange data that keeps the scanned shape. It is **not** a feature-based, parametric
model: there are no planes, cylinders or holes to edit, and it holds up to 20,000 faces. Fitted
surfaces ("Mode B") are a later stage.

The export is a **solid** only when the mesh is a validated closed shell. If parts of the object
were never seen, the file is an **open shell** and the app says so. Missing surfaces are never
invented; the only closures are the base on the observed table and holes under about 1 cm, and
both are reported.

## How a scan works

1. Put the object on a table and point the camera at it.
2. Press and **hold** the scan button and move slowly around it. A ring shows which directions you
   have covered; a banner tells you when tracking is lost.
3. Let go. The scan is already saved, so a crash or a failed reconstruction never means rescanning.
4. Review the mesh. Adjust the crop box if the table or background got in, and tap two points a known
   distance apart to set the scale in millimetres. Then export, share or save the STEP.

Retry, re-crop and recalibrate all reuse the saved scan.

## How the engine works

```
session (images, ARCore poses, raw depth)
  → depth per keyframe: ARCore raw depth, or multi-view stereo (plane sweep with occlusion-aware
    matching, multi-view consistency checks, per-pixel uncertainty)
  → support plane + crop box → sparse TSDF fusion within a memory budget → mesh
  → base cap on the table, small holes, manifold-safe decimation, topology checks
  → faceted B-rep (Open CASCADE) → STEP, read back and checked before it counts
```

Each surface point keeps three confidence signals: how often it was observed, from how many
directions, and the fused depth uncertainty. The viewer can colour by each. None of them is a
measured accuracy.

## Accuracy

ARCore's poses are not metrology. The two-point calibration corrects scale, not local distortion.

| Test | Target | Result |
|---|---|---|
| Synthetic box 100 × 60 × 40 mm, raw depth | ±2 mm | 101.2 × 61.5 × 40.3 mm, closed solid |
| Synthetic box, images only (MVS) | ±4 mm | 102.4 × 62.3 × 40.5 mm, closed solid |
| Synthetic cylinder Ø50 × 80 mm, raw depth | ±2 mm | 50.5 × 50.5 × 80.2 mm, closed solid |
| Synthetic cylinder, images only (MVS) | ±4 mm | 50.7 × 50.6 × 80.5 mm, closed solid |
| Real phone, caliper-measured objects | — | **not yet recorded** ([how](docs/real-sessions.md)) |

The synthetic rows are rendered scenes with perfect poses. They are test gates, not a promise about
real scans. The real-phone row is the one that matters.

## Building it

Engine and tests (macOS):

```bash
brew install cmake ninja opencv opencascade
cmake -S engine -B engine/build -G Ninja && cmake --build engine/build
ctest --test-dir engine/build
```

Try it on a synthetic scan:

```bash
engine/build/psom synth /tmp/box --dims 100,60,40
engine/build/psom reconstruct /tmp/box
engine/build/psom export /tmp/box      # /tmp/box/recon/model.step
```

App (needs the Android SDK, NDK 27.1 and an ARCore phone, arm64 only):

```bash
engine/scripts/build-android-deps.sh   # Open CASCADE and OpenCV for Android, once
cd android && ./gradlew installDebug
```

## Layout

```
docs/plan.md            the plan and its requirements
docs/session-format.md  what the app records and the engine reads
docs/real-sessions.md   recording and measuring the real-device test scans
engine/                 C++ engine, the psom CLI and its tests
android/                the app
```

## Roadmap

- [x] Capture: hold-to-scan, ARCore poses and raw depth, sessions saved as they are recorded
- [x] Reconstruction: raw depth or multi-view stereo, fusion, isolation, meshing
- [x] CAD conversion: repair, topology checks, scale calibration, faceted STEP that opens in FreeCAD
- [x] Usability: coverage ring, tracking warnings, quality notes, 3D review, crop, share and save
- [ ] **On a real phone:** scan, recover an interrupted scan, export, open in FreeCAD
- [ ] **Measured accuracy** on real objects, published above
- Later: fitted CAD surfaces (Mode B), live meshing while scanning

## License

MIT
