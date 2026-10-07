"""Test the packaged executable without relying on the development environment."""

from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import tempfile


def main():
    project = Path(__file__).resolve().parent
    executable = project / "dist/MotionViewer.exe"
    report = project / "artifacts" / datetime.now().strftime("build-smoke-%Y%m%d-%H%M%S-%f.json")
    environment = os.environ.copy()
    windows = Path(environment.get("SYSTEMROOT", "C:/Windows"))
    environment["PATH"] = os.pathsep.join(map(str, (windows / "System32", windows)))
    for name in ("PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV"):
        environment.pop(name, None)
    try:
        with tempfile.TemporaryDirectory(prefix="motion-viewer-check-") as directory:
            result = subprocess.run(
                [str(executable), "--smoke-test", str(report)],
                cwd=directory, env=environment, timeout=90,
            )
        if result.returncode:
            raise RuntimeError(f"Executable exited with code {result.returncode}; report: {report}")
        data = json.loads(report.read_text(encoding="utf-8"))
        if not (data.get("ok") is True and data.get("frozen") is True):
            raise RuntimeError(f"Packaged self-test failed: {data}")
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as exc:
        print(f"Build verification failed: {exc}")
        return 1
    print(f"Packaged self-test passed: {report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
