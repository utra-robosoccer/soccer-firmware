"""Active-config metadata: name, hash, YAML contents, git info, staleness check.

Used by the runner to (a) warn loudly if the generated motor_config_gen.py is stale
or mismatched relative to configs/active, and (b) fill the binary-log header so a
session is reproducible. The hash matches scripts/gen_motor_config.py's
config_hash(): sha256 over the setup's slave YAMLs, sorted by filename.

The staleness check treats the **configs/active file** as the source of truth (that
is what "should" be running). A $SOCCER_SETUP env override steers the *generator*
toolchain, so if it diverges from configs/active the check flags it loudly — an
export can otherwise make the check look "up to date" against the wrong setup.
"""
import glob
import hashlib
import os
import subprocess

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_CONFIGS = os.path.join(_REPO_ROOT, "configs")


def env_override() -> str | None:
    """$SOCCER_SETUP, stripped, or None."""
    env = os.environ.get("SOCCER_SETUP")
    return env.strip() if env and env.strip() else None


def active_file_name() -> str | None:
    """The setup named by the configs/active pointer FILE (ignores $SOCCER_SETUP)."""
    ptr = os.path.join(_CONFIGS, "active")
    if os.path.isfile(ptr):
        with open(ptr) as fh:
            return fh.read().strip() or None
    return None


def active_config_name() -> str | None:
    """What the generator toolchain uses: $SOCCER_SETUP overrides the active file."""
    return env_override() or active_file_name()


def _yaml_paths(name: str) -> list[str]:
    return sorted(glob.glob(os.path.join(_CONFIGS, name, "slave*.yaml")),
                  key=os.path.basename)


def config_yaml(name: str | None = None) -> dict[str, str]:
    """{filename: contents} for a setup's slave YAMLs (for the log header)."""
    name = name or active_config_name()
    if not name:
        return {}
    out = {}
    for p in _yaml_paths(name):
        with open(p) as fh:
            out[os.path.basename(p)] = fh.read()
    return out


def config_hash(name: str | None = None) -> str | None:
    """sha256 over a setup's slave YAMLs, sorted by filename (matches the generator)."""
    name = name or active_config_name()
    if not name:
        return None
    paths = _yaml_paths(name)
    if not paths:
        return None
    h = hashlib.sha256()
    for p in paths:
        with open(p, "rb") as fh:
            h.update(fh.read())
    return h.hexdigest()


# Back-compat aliases (older callers / tests).
active_config_yaml = config_yaml
active_config_hash = config_hash


def generated_config() -> tuple[str | None, str | None]:
    """(CONFIG_NAME, CONFIG_HASH) baked into the generated motor_config_gen.py."""
    try:
        from . import motor_config_gen as g
        return getattr(g, "CONFIG_NAME", None), getattr(g, "CONFIG_HASH", None)
    except Exception:
        return None, None


def check_config_fresh() -> tuple[bool, str]:
    """(ok, message). Compares the configs/active FILE (name + YAML hash) against the
    generated host module; ok=False on any mismatch/staleness OR a divergent
    $SOCCER_SETUP. The caller should warn loudly (it may still proceed)."""
    gen_name, gen_hash = generated_config()
    if gen_name is None or gen_hash is None:
        return False, ("motor_config_gen.py has no CONFIG_NAME/CONFIG_HASH — regenerate: "
                       "python3 scripts/gen_motor_config.py --system")

    file_name = active_file_name()
    if not file_name:
        return False, ("configs/active is empty or missing — set it, e.g. "
                       "`echo 1s_5m > configs/active`")

    live_hash = config_hash(file_name)
    if live_hash is None:
        return False, (f"configs/active names {file_name!r} but configs/{file_name}/ "
                       "has no slave*.yaml")

    env = env_override()
    env_note = ""
    if env and env != file_name:
        env_note = (f"  [WARNING: SOCCER_SETUP={env!r} is overriding the generator away "
                    f"from configs/active={file_name!r}; unset it to use the file]")

    if gen_name != file_name:
        return False, (f"generated host config is {gen_name!r} but configs/active is "
                       f"{file_name!r} — regenerate: "
                       f"python3 scripts/gen_motor_config.py --system" + env_note)
    if gen_hash != live_hash:
        return False, (f"generated motor_config_gen.py is STALE vs configs/{file_name}/ "
                       f"(hash {gen_hash[:8]} != {live_hash[:8]}) — regenerate: "
                       f"python3 scripts/gen_motor_config.py --system" + env_note)
    if env_note:
        return False, (f"config {file_name!r} matches the generated module, but" + env_note)
    return True, f"config {file_name!r} up to date ({live_hash[:8]})"


def git_info() -> dict:
    """{'commit': <sha or None>, 'dirty': <bool or None>}."""
    def _run(args):
        return subprocess.run(["git", "-C", _REPO_ROOT, *args],
                              capture_output=True, text=True, timeout=5)
    info = {"commit": None, "dirty": None}
    try:
        r = _run(["rev-parse", "HEAD"])
        if r.returncode == 0:
            info["commit"] = r.stdout.strip()
        s = _run(["status", "--porcelain"])
        if s.returncode == 0:
            info["dirty"] = bool(s.stdout.strip())
    except Exception:
        pass
    return info


def build_log_meta(producer: str) -> dict:
    """Binary-log header metadata. Records the config the RUNTIME actually uses (the
    generated module) plus the configs/active file and any $SOCCER_SETUP override, so
    a divergence is captured in the log itself."""
    gi = git_info()
    file_name = active_file_name()
    gen_name, gen_hash = generated_config()
    return {
        "producer": producer,
        "git_commit": gi["commit"],
        "git_dirty": gi["dirty"],
        "config_name": gen_name,          # what the host code actually loaded
        "config_hash": gen_hash,
        "configs_active": file_name,      # the persistent pointer
        "soccer_setup": env_override(),   # env override, if any
        "config_yaml": config_yaml(file_name),
    }
