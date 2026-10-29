"""master_link — the host's link to the robot firmware.

Importable library: the wire ``protocol``, the generated ``motor_config_gen``
constants, and ``session_logger`` for data logging. Runnable programs live in
``host/apps``; offline log tools in ``host/analysis``.
"""
import os
import subprocess
import sys


def _ensure_generated():
    """motor_config_gen.py is an untracked per-bench output (see configs/README.md);
    materialize it from the active setup on first import after a fresh clone.
    Staleness against configs/active is still check_config_fresh()'s job."""
    here = os.path.dirname(os.path.abspath(__file__))
    if os.path.isfile(os.path.join(here, "motor_config_gen.py")):
        return
    script = os.path.abspath(os.path.join(here, "..", "..", "scripts",
                                          "gen_motor_config.py"))
    print("master_link: motor_config_gen.py missing — regenerating from the active setup",
          file=sys.stderr)
    r = subprocess.run([sys.executable, script, "--system"])
    if r.returncode != 0 or not os.path.isfile(os.path.join(here, "motor_config_gen.py")):
        raise ImportError(
            "could not generate motor_config_gen.py — set the active setup "
            "(e.g. `echo 1s_5m > configs/active` or SOCCER_SETUP=<name>), then run "
            "python3 scripts/gen_motor_config.py --system")


_ensure_generated()
del _ensure_generated
