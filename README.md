# pseudo-stereographic object mapping

Hold a button, walk around an object with an Android phone, let go, and get a STEP file you can
open in CAD.

> **Status: in development. Not yet verified on a real phone.** Until a real scan-to-STEP has
> worked and its errors have been measured, treat everything here as unproven.

## What it is

- An **Android app** (Kotlin, Jetpack Compose, ARCore) that records a scan: keyframes, ARCore
  camera poses, raw depth where the phone has it, and IMU data.
- A **C++ engine** that turns a recorded scan into a mesh and then a STEP file: depth (ARCore raw
  depth, or multi-view stereo from the images), sparse TSDF fusion, meshing, repair, and
  faceted B-rep export through Open CASCADE. The same engine runs as a desktop CLI and inside the app.

## What the STEP is, and isn't

The export is **faceted geometry**: every triangle of the scanned mesh becomes a flat CAD face. It is
valid CAD exchange data that keeps the scanned shape. It is **not** a feature-based, parametric model:
no planes, cylinders or holes you can edit, and it can hold tens of thousands of faces. Fitted
surfaces ("Mode B") are a later stage, not part of this MVP.

The export is a **solid** only when the mesh is a valid closed shell. If parts of the object were never
seen, the file is an **open shell** and the app says so. Missing surfaces are never invented.

## How a scan works

1. Point the camera at an object on a table.
2. Press and hold the scan button and move slowly around the object. A coverage ring shows which sides still need views; a banner shows when tracking is lost.
3. Release. The scan is saved as you go, so a crash or failed reconstruction never means rescanning.
4. Review the mesh, adjust the crop box if the table or background got in, tap two points a known distance apart to set the scale in millimetres, then export.

## Accuracy

ARCore poses are not metrology. Scale can be corrected with the two-point calibration; local
distortion cannot.

| Test | Target | Measured |
|---|---|---|
| Synthetic box / cylinder, depth path | ±2 mm | not yet run |
| Synthetic box / cylinder, MVS path | ±4 mm | not yet run |
| Real phone, measured objects | — | not yet run |

The synthetic numbers are test gates on rendered data, not a promise about real scans.

## Layout

```
docs/plan.md            the plan and its requirements
docs/session-format.md  what the app records and the engine reads
engine/                 C++ engine and the `psom` CLI
android/                the app
```

## Roadmap

1. Capture prototype: hold-to-scan, ARCore poses, saved sessions.
2. Reconstruction: depth, fusion, isolation, mesh.
3. CAD conversion: repair, topology checks, scale, faceted STEP.
4. Usability and validation: coverage, warnings, viewer, sharing, real measured scans.

## License

MIT
