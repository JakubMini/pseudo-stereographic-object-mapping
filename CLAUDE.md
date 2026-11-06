# pseudo-stereographic object mapping: working agreement

Rules for anyone (human or Claude) changing this repo. Read before touching code. The same
agreement as Jakub's other projects, adapted to an Android app and a C++ engine.

## Branches, commits, pull requests

- **Never commit to main.** Every piece of work, however small, gets its own branch cut from an
  up-to-date main and lands through a pull request.
  - Exception, asked for by Jakub on 2026-11-05: the initial build is pushed straight to main in
    small commits. The PR rule applies again once the MVP has been verified on a phone.
- Branch names say what the branch is for: `feat/<thing>`, `fix/<thing>`, `chore/<thing>`, `docs/<thing>`, `test/<thing>`.
- Start every task with: `git switch main && git pull --ff-only && git switch -c <branch>`.
- One PR, one purpose. Commit subjects are imperative and under 72 characters.
- Before pushing: in `engine/`, `cmake --build build && ctest --test-dir build`; in `android/`,
  `./gradlew :app:testDebugUnitTest :app:assembleDebug`. CI runs the same.
- The PR body says what changed, how it was verified (on a phone, if it touches capture or the
  app), and whether the README needed updating.

## The README is for humans

- Written for someone deciding in two minutes whether this project is worth their attention.
  Keep it true. Any PR that changes the architecture, the session format, the export, the setup or
  the roadmap updates the README in the same PR. Do not inflate. Accuracy numbers in it are
  measured, and synthetic ones are labelled as synthetic.

## Rules the code keeps

- **The session is immutable.** The app writes `session/` during capture only. The engine never
  writes there; everything derived goes in `recon/`. Retry, re-crop and recalibrate must never need a rescan.
- **A guessed surface is always marked.** What wasn't seen is filled by diffusion (`fill_unseen`),
  and every guessed vertex carries weight 0 so the report (`guess.fraction`), the viewer (yellow) and
  the STEP header can say how much was guessed. Never show a guess as measured. A STEP solid only
  from a validated closed shell; otherwise an open shell and a warning.
- **A written file is not success.** Every stage returns `{ok, stage, error, warnings}`; the STEP is
  read back and checked before it replaces the old one; `project.json` status moves only on `ok`.
- **Confidence channels stay separate:** observation weight, view diversity, geometric
  uncertainty. Never call any of them "accuracy".
- **A photo scan has no size until calibrated.** Its dimensions are a guess and export refuses
  until the user sets the scale; never show them as millimetres before that.
- **Respect the memory budget.** The TSDF counts every block it allocates and fails cleanly when the
  budget runs out.
- `docs/session-format.md` is the contract. Change it and the loader, the writer and the tests together.

## Where things live

- `engine/` (C++20, CMake; OpenCV 4.x or 5.x (5 renamed features2d/calib3d to features/geometry), Open CASCADE, nlohmann/json):
  - `src/session.cpp`: loads and validates a session; the only place the GL→CV pose conversion happens.
  - `src/sfm.cpp`: camera poses from photos (features, matching, placement, bundle adjustment), then
    levelled with gravity and given a working scale. Cached in `recon/sfm.json`.
  - `src/depth.cpp`: raw-depth frames → metric depth + σ; the `recon/depth` cache.
  - `src/mvs.cpp`: plane-sweep multi-view stereo, consistency checks, uncertainty.
  - `src/isolate.cpp`: support plane, region of interest, crop box.
  - `src/tsdf.cpp`: sparse voxel-block TSDF with the memory budget; marching tetrahedra (welded, manifold).
  - `src/mesh.cpp`: the mesh type and its PLY file.
  - `src/repair.cpp`: fragments, the base cap, small-hole fill, quadric decimation that keeps the
    mesh manifold (link condition), topology validation.
  - `src/step.cpp`: faceted B-rep with exactly shared edges, STEP write and read-back.
  - `src/pipeline.cpp`: the stages in order, the report and `project.json`.
  - `src/synth.cpp`: renders synthetic sessions for tests.
  - `cli/main.cpp`: `psom synth | reconstruct | poses | export | evaluate`.
  - `test/`: GoogleTest unit and end-to-end tests.
  - `scripts/`: Android deps build, FreeCAD STEP check, real-session fetch and evaluation
    (`docs/real-sessions.md`).
- `android/` (Kotlin, Compose, CameraX, NDK): `scan/` capture, `gl/` rendering, `engine/` JNI bridge,
  `ui/` screens. `app/src/main/cpp/` builds the engine into `libpsom.so`.

## Setup

- macOS: `brew install cmake opencv opencascade`, then
  `cmake -S engine -B engine/build -G Ninja && cmake --build engine/build && ctest --test-dir engine/build`.
- Android: `engine/scripts/build-android-deps.sh` once (OCCT + OpenCV for arm64-v8a, into
  `third_party/android/`), then `./gradlew assembleDebug` in `android/`.
- Pull a project off the phone (debug build):
  `adb exec-out run-as com.jakubmini.psom tar c files/projects/<id> > <id>.tar`.
