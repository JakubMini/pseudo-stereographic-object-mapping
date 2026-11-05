# Project and session format (`psom-session/1`)

The contract between the capture app and the engine. The app writes `session/` while scanning and
never touches it again. The engine only reads `session/`; everything it makes goes in `recon/`, which
can be deleted and rebuilt at any time.

```
<project>/
  project.json          mutable: status, crop, scale, last error, runs
  session/              immutable once capture ends
    session.json
    frames.jsonl
    events.jsonl
    points.csv
    imu.csv
    frames/000000.jpg …
    depth/000000.png …  (only if raw depth was available for that frame)
    conf/000000.png  …
  recon/                derived
    depth/000000.bin …  cached per-keyframe depth + uncertainty
    mesh.ply
    report.json
    model.step
```

## Conventions

- **World:** ARCore world, metres, +Y up. The engine uses the same world.
- **Pose:** `T_world_cam`, row-major 4×4, in ARCore's camera convention (OpenGL: +X right, +Y up,
  looking down −Z), declared as `"pose": "arcore_gl_world_from_camera"`. The engine converts to the
  vision convention (+X right, +Y down, +Z forward) in one place, `T_cv = T_gl · diag(1,−1,−1,1)`.
- **Images** are stored in sensor orientation, exactly as ARCore's CPU image comes out, never
  rotated for display. `sensor_orientation_deg` and `display_rotation_deg` are recorded.
- **Intrinsics** `K = [fx, fy, cx, cy]` in pixels of the stored image (ARCore `imageIntrinsics`).
- **Depth** is 16-bit PNG in millimetres (0 = no depth), confidence 8-bit (0–255), both in sensor
  orientation. ARCore raw depth covers the same field of view as the CPU image, so depth
  intrinsics are `K` scaled by `depth.width / image.width` and `depth.height / image.height`. The
  engine rejects depth when the two aspect ratios differ by more than 1 %.
- **Timestamps:** `t_ns` is `Frame.timestamp`, `depth_t_ns` the raw depth `Image.timestamp`, both on
  ARCore's clock. IMU rows use the `SensorEvent` clock; `clocks.sensor_minus_arcore_ns` is sampled
  once at scan start. The engine drops a frame's depth when `|depth_t_ns − t_ns|` exceeds
  `depth.max_skew_ns`.

## `session.json`

```json
{
  "format": "psom-session/1",
  "created_utc": "2026-11-05T10:00:00Z",
  "device": { "manufacturer": "Google", "model": "Pixel 8", "arcore": "1.46.0" },
  "conventions": { "pose": "arcore_gl_world_from_camera", "world_up": "+y",
                   "depth_units": "mm", "image_orientation": "sensor" },
  "camera": { "sensor_orientation_deg": 90, "display_rotation_deg": 0 },
  "image": { "width": 1920, "height": 1080 },
  "depth": { "supported": true, "width": 160, "height": 90, "max_skew_ns": 16666666 },
  "clocks": { "sensor_minus_arcore_ns": 0 },
  "target": [0.01, -0.20, -0.35]
}
```

`target` is the world point of a hit test at the screen centre when the scan started, or `null`. It is a hint only.

## `frames.jsonl`

One keyframe per line. Each line is written and flushed as it happens, so an app that dies mid-scan leaves
a valid prefix. The loader drops a truncated last line.

```json
{"i":0,"t_ns":123456789000,"pose":[1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1],"K":[1450.2,1450.2,960.1,540.3],
 "tracking":"tracking","sharpness":182.5,"depth":true,"depth_t_ns":123456789000}
```

`i` is the file stem (`frames/000000.jpg`). `t_ns` strictly increases. `pose` has a finite, orthonormal
rotation. The principal point lies inside the image.

## `events.jsonl`

```json
{"t_ns":…,"event":"capture_start"}
{"t_ns":…,"event":"tracking_lost","reason":"EXCESSIVE_MOTION"}
{"t_ns":…,"event":"tracking_resumed"}
{"t_ns":…,"event":"capture_end","rejected":{"blur":12,"motion":0,"tracking":40,"rate":310}}
```

No `capture_end` means the capture was interrupted; the engine still uses what is there and says so.

## `points.csv` and `imu.csv`

```
t_ns,id,x,y,z,confidence        ARCore feature points (world), written with each keyframe
t_ns,sensor,x,y,z               sensor = g (rad/s) or a (m/s²), SensorEvent clock
```

## `project.json`

```json
{
  "format": "psom-project/1",
  "status": "captured",
  "last_error": null,
  "crop": { "source": "auto", "center": [0,0,0], "axes": [[1,0,0],[0,0,-1],[0,1,0]], "half": [0.1,0.1,0.1] },
  "scale": { "factor": 1.0, "a": null, "b": null, "known_mm": null },
  "runs": [ { "started_utc": "…", "ok": true, "stage": "done", "error": null } ]
}
```

- `status`: `capturing` → `captured` → `reconstructed` → `exported`. It only moves forward after the
  stage's output has been validated. A failure leaves `status` where it was and sets `last_error`
  to `{ "stage": …, "error": … }`.
- `crop`: an oriented box in world metres. `axes` are the box's unit x, y, z (z is the support-plane
  normal); `half` are half-extents. `source` is `auto` (engine) or `user` (edited in review).
- `scale.factor` multiplies world metres before conversion to millimetres for the STEP. `a`, `b` are the two
  picked mesh points, and `known_mm` is the distance the user entered.

## `recon/depth/*.bin`

Little-endian: `u32 magic "PSDP"`, `u32 version (1)`, `u32 w`, `u32 h`, `f32 fx, fy, cx, cy`,
then `w·h` float32 depth in metres (0 = rejected) and `w·h` float32 σ in metres.
