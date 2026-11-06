# pseudo-stereographic object mapping

Hold a button, walk around an object with an Android phone, let go, and get a STEP file you can
open in CAD.

> **Status: built, not yet proven on a real phone.** The first real-phone scans (Galaxy A56)
> failed because ARCore's tracking broke down within seconds; Google's own Hello AR sample lost
> track the same way. So the app no longer uses ARCore: it takes photos, and the engine works out
> where each one was taken. Run on those same phone photos with ARCore's poses removed, it places
> every photo in three of the four scans (52, 79 and 127 photos) and rebuilds a watch from one; the
> fourth, taken while an early build froze the camera at each shot, breaks after 31 of 105. A full
> scan-to-STEP on the phone with this version, and accuracy against calipers, are still to come.
> Until then, treat it as unproven.

## What it is

- An **Android app** (Kotlin, Jetpack Compose, CameraX) that records a scan: photos, the phone's
  gravity reading for each, and its motion sensors, saved as it goes. Any Android 8 phone, no ARCore.
- A **C++ engine** that turns a recorded scan into a mesh and then a STEP file. The same code runs
  as a desktop command-line tool and inside the app.

## What the STEP is, and isn't

The export is **faceted geometry**: every triangle of the scanned mesh becomes a flat CAD face. It
is valid CAD exchange data that keeps the scanned shape. It is **not** a feature-based, parametric
model: there are no planes, cylinders or holes to edit, and it holds up to 20,000 faces. Fitted
surfaces ("Mode B") are a later stage.

The export is a **solid** only when the mesh is a validated closed shell. Parts of the object the
photos never showed are **guessed**: the measured surface is carried smoothly across each gap, like
a membrane stretched over it. A guess is a guess. The viewer paints it yellow, the report gives the
share of the surface that was guessed, and the STEP header says so too. With the guess switched off
(`--no-guess`), unseen parts stay open and the file is an **open shell**.

## How a scan works

1. Put the object on a table and point the camera at it.
2. Press and **hold** the scan button and move slowly around it. A ring shows which directions you
   have covered; it says so when you turn too fast to stay sharp.
3. Let go. The scan is already saved, so a crash or a failed reconstruction never means rescanning.
4. Review the mesh. Adjust the crop box if the table or background got in, and tap two points a known
   distance apart to set the scale: photos alone don't say how big something is, so export waits
   for this. Then export, share or save the STEP.

Retry, re-crop and recalibrate all reuse the saved scan.

## How the engine works

```
session (photos, gravity)
  → camera poses from the photos: RootSIFT features, matches checked against epipolar geometry,
    photos placed one by one and refined by bundle adjustment (focal length included), levelled
    with gravity, given a working scale until calibrated
  → depth per photo: multi-view stereo (plane sweep with occlusion-aware matching, multi-view
    consistency checks, per-pixel uncertainty); ARCore raw depth for older scans that have it
  → support plane + crop box → sparse TSDF fusion within a memory budget → mesh
  → base cap on the table, small holes, manifold-safe decimation, topology checks
  → faceted B-rep (Open CASCADE) → STEP, read back and checked before it counts
```

Each surface point keeps three confidence signals: how often it was observed, from how many
directions, and the fused depth uncertainty. The viewer can colour by each. None of them is a
measured accuracy.

## Accuracy

Poses from photos are not metrology. The scale comes only from your two-point calibration, which
corrects scale, not local distortion.

| Test | Target | Result |
|---|---|---|
| Synthetic box 100 × 60 × 40 mm, raw depth | ±2 mm | 101.2 × 61.5 × 40.3 mm, closed solid |
| Synthetic box, images only (MVS) | ±4 mm | 102.4 × 62.3 × 40.5 mm, closed solid |
| Synthetic cylinder Ø50 × 80 mm, raw depth | ±2 mm | 50.5 × 50.5 × 80.2 mm, closed solid |
| Synthetic cylinder, images only (MVS) | ±4 mm | 50.7 × 50.6 × 80.5 mm, closed solid |
| Synthetic box, photos only (no poses), scaled on its 100 mm side | ±4 mm | 100 × 60.9 × 39.4 mm, closed solid |
| Real phone, caliper-measured objects | — | **not yet** ([how to record](docs/real-sessions.md)) |

The synthetic rows are rendered scenes; all but the last have perfect poses. They are test gates, not a promise about
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

App (needs the Android SDK, NDK 27.1 and an arm64 phone):

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

- [x] Capture: hold-to-scan photos with gravity, sessions saved as they are recorded
- [x] Reconstruction: poses from the photos, multi-view stereo, fusion, isolation, meshing
- [x] CAD conversion: repair, topology checks, scale calibration, faceted STEP that opens in FreeCAD
- [x] Usability: coverage ring, tracking warnings, quality notes, 3D review, crop, share and save
- [ ] **On a real phone:** scan, recover an interrupted scan, export, open in FreeCAD
- [ ] **Measured accuracy** on real objects, published above
- Later: fitted CAD surfaces (Mode B), live meshing while scanning

## License

MIT. The bundled Geist and Geist Mono fonts are under the SIL Open Font License ([android/fonts-OFL.txt](android/fonts-OFL.txt)).
