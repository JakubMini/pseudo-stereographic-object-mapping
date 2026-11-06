# Real-device sessions

Synthetic scans prove the pipeline is consistent. Only real scans of measured objects say how
accurate it is. The MVP is not done until these exist and their errors are published in the README.

## What to record

| Session | Object | What it tests |
|---|---|---|
| `box` | a textured box (a printed or patterned carton) | the main case |
| `bracket` | an L-bracket or similar part | thin walls, concave corner |
| `cylinder` | a can or a turned part | curved surface |
| `tracking-loss` | the box, covering the lens for a second mid-scan | tracking-loss reporting |
| `blur` | the box, moving fast on purpose | blur rejection |
| `low-texture` | a plain white object | the guessed share is reported; with `--no-guess`, holes left open |

Scan on a table, walk all the way around, and include some views from above.

## Measuring

Measure each object with calipers before scanning: length and width (longest first) and height off
the table, in millimetres. Write them next to the session as `truth.json`:

```json
{ "name": "textured box", "case": "measured", "dims_mm": [120.4, 80.1, 45.2], "measured_with": "calipers" }
```

For the failure cases, `case` names the case and `expect_warning` is text the engine must report:

```json
{ "name": "box, lens covered", "case": "tracking_loss", "dims_mm": [120.4, 80.1, 45.2], "expect_warning": "tracking was lost" }
```

The expected warnings are `tracking was lost`, `rejected` (for blur, from the keyframe counts) and
`not seen or not measurable` (for low texture).

## Getting a session off the phone

The debug build lets `adb` read the app's files:

```bash
adb exec-out run-as com.jakubmini.psom tar c files/projects/<id> > <id>.tar
```

Rename the folder to the session's name, add `truth.json`, and pack it as `<name>.tar`. The sessions
are too big for git, so they live as assets of the GitHub release `real-sessions`:

```bash
gh release create real-sessions --title "Real-device sessions" --notes "Recorded scans with caliper truth" || true
gh release upload real-sessions box.tar --clobber
```

## Evaluating

```bash
engine/scripts/fetch-real-sessions.sh
python3 engine/scripts/evaluate_real.py
```

The script reconstructs and exports a copy of each session (the recordings are never modified) and
prints a Markdown table of measured against true dimensions. CI runs the same.
