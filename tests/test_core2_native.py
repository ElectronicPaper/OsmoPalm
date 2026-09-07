"""Execute production C++ control/feedback modules, not Python reimplementations."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class TestCore2Native(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = os.environ.get("CXX") or shutil.which("g++")
        if not compiler and Path("C:/msys64/mingw64/bin/g++.exe").is_file():
            compiler = "C:/msys64/mingw64/bin/g++.exe"
        if not compiler:
            raise unittest.SkipTest("native C++17 compiler required")
        cls.temp = tempfile.TemporaryDirectory(prefix="core2-native-")
        cls.addClassCleanup(cls.temp.cleanup)
        cls.exe = Path(cls.temp.name) / ("core2-test.exe" if os.name == "nt" else "core2-test")
        cmd = [compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-O2"]
        if os.name == "nt":
            cmd += ["-static"]
        cmd += ["-I", str(ROOT / "firmware/core2_panel/src"),
                str(ROOT / "tests/native/core2_math_test.cpp"), "-o", str(cls.exe)]
        env = dict(os.environ)
        env["PATH"] = str(Path(compiler).parent) + os.pathsep + env.get("PATH", "")
        result = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=90)
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr)

    def run_case(self, name):
        result = subprocess.run([str(self.exe), name], capture_output=True,
                                text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


def case(name):
    def test(self):
        self.run_case(name)
    return test


for _name in ("relative_pose", "shaping", "precision", "curve", "wire_limits",
              "manual_ease", "playback_actions", "motion_delete", "soft_haptic", "haptic_profiles", "haptic_cancel", "haptic_deadline"):
    setattr(TestCore2Native, "test_" + _name, case(_name))
