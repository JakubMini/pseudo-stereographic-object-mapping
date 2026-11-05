# Opens STEP files the way a CAD user would, in FreeCAD, and prints what it sees as JSON lines.
# Run: PSOM_STEP_FILES=a.step:b.step freecadcmd engine/scripts/freecad_check.py
# (freecadcmd would try to open file arguments itself, so the list comes from the environment.)
import json
import os
import sys

import Part

files = [f for f in os.environ.get("PSOM_STEP_FILES", "").split(":") if f]
files += [a for a in sys.argv[1:] if a.lower().endswith((".step", ".stp"))]
for path in files:
    shape = Part.Shape()
    try:
        shape.read(path)
    except Exception as e:  # noqa: BLE001 - report every failure the same way
        print(json.dumps({"file": path, "ok": False, "error": str(e)}))
        continue
    bb = shape.BoundBox
    print(json.dumps({
        "file": path,
        "ok": True,
        "valid": shape.isValid(),
        "solids": len(shape.Solids),
        "shells": len(shape.Shells),
        "faces": len(shape.Faces),
        "closed": all(s.isClosed() for s in shape.Shells) if shape.Shells else False,
        "volume_mm3": shape.Volume if shape.Solids else 0.0,
        "bbox_mm": [bb.XLength, bb.YLength, bb.ZLength],
    }))
