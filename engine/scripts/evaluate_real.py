"""Reconstructs every real session in testdata/real/ and compares it with its truth.json.

Prints a Markdown table for the README. Accuracy is reported, not gated: a real scan's error is
what it is. The script fails only if the engine crashes, or a failure-case session does not
report the warning it must (docs/real-sessions.md).
"""
import json
import pathlib
import shutil
import subprocess
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[2]
PSOM = ROOT / "engine" / "build" / "psom"
REAL = ROOT / "testdata" / "real"


def run(*args):
    p = subprocess.run([str(PSOM), *map(str, args)], capture_output=True, text=True)
    try:
        return json.loads(p.stdout)
    except json.JSONDecodeError:
        return {"ok": False, "stage": "crash", "error": (p.stderr or p.stdout)[-300:], "warnings": []}


def main():
    sessions = sorted(d for d in REAL.glob("*") if (d / "truth.json").exists()) if REAL.exists() else []
    if not sessions:
        print("no real sessions in testdata/real: see docs/real-sessions.md")
        return 0
    rows, problems = [], []
    for src in sessions:
        truth = json.loads((src / "truth.json").read_text())
        with tempfile.TemporaryDirectory() as tmp:
            p = pathlib.Path(tmp) / src.name
            shutil.copytree(src, p)  # the recording is never touched
            rec = run("reconstruct", p)
            exp = run("export", p) if rec["ok"] else None
            ev = run("evaluate", p, src / "truth.json") if rec["ok"] else None
            report = json.loads((p / "recon" / "report.json").read_text()) if rec["ok"] else {}
        warnings = rec["warnings"] + (exp["warnings"] if exp else [])
        if rec["stage"] == "crash" or (exp and exp["stage"] == "crash"):
            problems.append(f"{src.name}: engine crashed")
        want = truth.get("expect_warning")
        if want and not any(want in w for w in warnings) and not (want == "rejected" and report.get("session", {}).get("rejected_by_app")):
            problems.append(f"{src.name}: expected a warning containing '{want}'")
        if ev and ev["ok"]:
            r = ev["result"]
            m, t, e = r["measured_mm"], r["truth_mm"], r["error_mm"]
            dims = " × ".join(f"{x:.1f}" for x in m)
            true = " × ".join(f"{x:.1f}" for x in t)
            err = " / ".join(f"{x:+.1f}" for x in e)
        else:
            dims, true, err = "—", " × ".join(f"{x:.1f}" for x in truth["dims_mm"]), f"failed at {rec['stage']}: {rec['error']}"
        solid = "solid" if exp and exp["ok"] and report and report.get("topology", {}).get("closed_valid") else "open shell" if exp and exp["ok"] else "—"
        rows.append(f"| {truth.get('name', src.name)} | {truth.get('case', 'measured')} | {true} | {dims} | {err} | {solid} |")
    print("| Session | Case | Calipers (mm) | Scan (mm) | Error (mm) | STEP |")
    print("|---|---|---|---|---|---|")
    print("\n".join(rows))
    for p in problems:
        print(f"PROBLEM: {p}", file=sys.stderr)
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
