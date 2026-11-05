import os, subprocess, sys
from pathlib import Path
import pytest

ROOT = Path(__file__).resolve().parents[1]

@pytest.mark.parametrize("name", ["test_tracking", "test_planner", "test_smooth", "test_astar"])
def test_script(name):
    env = {**os.environ, "PYTHONIOENCODING": "utf-8"}
    r = subprocess.run([sys.executable, str(ROOT / "tests" / f"{name}.py")], capture_output=True,
                       text=True, encoding="utf-8", cwd=ROOT, env=env)
    out = r.stdout + r.stderr
    bad = [l for l in out.splitlines() if l.lstrip().startswith("FAIL") or "CÓ LỖI" in l]
    assert r.returncode == 0 and not bad, "\n".join(bad) or out[-2000:]