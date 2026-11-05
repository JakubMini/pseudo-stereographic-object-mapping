# Development plan

A single-camera, IMU-assisted 3D scanner for Android that exports STEP files. Hold a button to
scan, move around the object, release to finish.

The hard part is not capturing 3D geometry. It is turning the reconstruction into a useful CAD
model: a STEP file is a boundary representation, not the triangle mesh photogrammetry produces.

## 1. User experience

1. Open the app and point the camera at an object.
2. Press and **hold** the scan button (not a toggle).
3. Move slowly around the object while the app records images, camera motion and IMU data.
4. Watch a live coverage indicator showing which sides still need views.
5. Release to stop capture and start reconstruction.
6. Review the model, set or confirm its scale, export a `.step`.

The scan session is saved as it is captured, so a failed reconstruction never means rescanning.

## 2. Architecture

```
Android app (Kotlin + Compose): preview, hold button, feedback, viewer
  ├─ ARCore: camera frames, visual-inertial poses, raw depth where supported
  ├─ SensorManager: gyroscope + accelerometer
  └─ JNI → C++ engine
        keyframes → depth (raw depth or multi-view stereo) → fusion → mesh
        → repair → scale → faceted B-rep → STEP (Open CASCADE)
```

Open CASCADE (OCCT) writes the STEP. The capture app and the reconstruction engine are separate,
so the engine is developed and tested on a desktop first and the same code is built into the app.

## 3. Stack

| Component | Technology |
|---|---|
| UI | Kotlin + Jetpack Compose |
| Camera and tracking | ARCore |
| IMU | SensorManager (recorded) |
| Image processing | OpenCV (C++) |
| Depth | ARCore Raw Depth, multi-view stereo fallback |
| Reconstruction | C++ (sparse TSDF, marching cubes) |
| CAD export | Open CASCADE |
| Integration | Android NDK + JNI |
| Storage | Local, offline projects |

## 4. Two levels of STEP quality

- **Mode A, faceted STEP (this MVP):** triangle mesh → repaired → each triangle a planar CAD face.
  Valid CAD exchange data that preserves the scanned shape, but can hold thousands of faces and is
  **not** a feature-based, editable parametric model.
- **Mode B, fitted CAD (later):** recognise planes, cylinders, holes and edges and fit
  mathematical surfaces. Much more useful for mechanical parts, much more work.

## 5. Roadmap

1. **Capture prototype:** preview, press-and-hold, ARCore poses, frame capture, session storage,
   motion feedback. Deliverable: a replayable session of images and poses.
2. **Reconstruction:** keyframes, depth, multi-view fusion, object isolation, meshing.
   Deliverable: a recognisable mesh of a stationary object.
3. **CAD conversion:** repair, watertightness, units, manual scale calibration, faceted STEP.
   Deliverable: a STEP that opens in FreeCAD.
4. **Usability and validation:** live coverage, progress, quality warnings, 3D preview, sharing,
   dimensions compared against calipers. Deliverable: an MVP ready for real-world testing.

## 6. Engineering requirements

- **Capture:** state machine `READY → SCANNING → PROCESSING → REVIEW → EXPORTED`. Press starts,
  lift ends, even if the finger slid off the button. An interrupted scan is recoverable.
- **Frames:** keyframes chosen by translation, rotation, sharpness, tracking quality and overlap,
  with synchronised intrinsics and poses. Not every frame at 30 fps.
- **Accuracy:** ARCore poses are useful, not metrology. Two-point calibration against a known
  distance corrects scale, not local distortion.
- **Reconstruction:** depth fusion where supported, multi-view stereo as fallback. Keep confidence
  and do not invent geometry for unseen regions.
- **Processing:** post-capture first; incremental live meshing later.
- **Validation:** a textured box, a bracket and a cylinder; compare dimensions, check completeness,
  open the STEP in FreeCAD.

## 7. Implementation addendum

1. **MVS:** reference-camera plane sweep with calibrated projection, overlap-aware neighbour
   selection, occlusion handling, multi-view consistency checks and depth uncertainty. Unreliable
   depth is rejected before fusion.
2. **Confidence:** observation weight, viewing-angle diversity and geometric uncertainty are stored
   separately. TSDF weight alone is never presented as accuracy.
3. **TSDF:** sparse voxel blocks with an enforced memory budget; fail gracefully when exceeded.
4. **Isolation:** automatic support plane plus a user-adjustable 3D crop box in review. A
   successful ARCore hit test is never assumed.
5. **STEP:** validate manifoldness, edge sharing, orientation and topology. A solid only from a
   valid closed shell; otherwise an open shell with an explicit warning. Never invent surfaces.
6. **Capture metadata:** camera, pose and depth timestamps, image orientation, intrinsics,
   coordinate conventions and depth-to-image alignment are recorded and validated.
7. **Real-device tests:** recorded ARCore sessions of measured objects, including tracking loss,
   blur and low texture. Synthetic tests are necessary but not sufficient.
8. **Accuracy:** ±2 mm (depth) and ±4 mm (MVS) are synthetic test targets, not real-world claims.
   Real-device errors are measured and reported.
9. **Completion:** green CI, a STEP round trip and synthetic reconstruction are not a finished MVP.
   That takes a successful real-phone scan-to-STEP.
10. **Processing:** the captured session is kept independent of results. Retry, re-crop and
    recalibrate without rescanning.
11. **Documentation:** Mode A is faceted CAD geometry, not a parametric model.
12. **Failure handling:** on any failure keep the project, report it and allow retry. Never report
    success just because a file was written.

Out of scope for the MVP: Mode B, custom SLAM, cloud processing, live incremental meshing.

## 8. MVP acceptance

- [ ] Press-and-hold scanning starts and stops reliably
- [ ] Camera images and calibrated poses are saved
- [ ] Stationary objects reconstruct into a recognisable mesh
- [ ] Tracking loss and insufficient coverage are reported
- [ ] Model scale can be calibrated in millimetres
- [ ] Faceted STEP exports and opens in FreeCAD
- [ ] Scan projects can be reopened after closing the app
- [ ] All of the above on a real phone, with measured errors published

## Decision, 2026-11-05: poses from the photos, not ARCore

On the Galaxy A56, ARCore lost track within seconds in every scan, and Google's own Hello AR sample
did the same. Jakub chose to drop ARCore and work the poses out from the photos after capture
(structure from motion, in the engine). This is not live SLAM: nothing tracks while scanning, so the
"no custom SLAM" line above still holds. Photos carry no size, so the two-point calibration is now
required before export.
