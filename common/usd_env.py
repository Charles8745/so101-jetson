#!/usr/bin/env python3
"""Getting `pxr` on whatever machine this is. Measured, not assumed.

`pxr` is USD's python binding. Anything that reads a USD file needs it, and
where it comes from is different on every machine we use:

    Mac, x86_64 box, container   pip install usd-core. Plain `import pxr`.
    Spark (dgx702, Grace ARM)    ** neither of the obvious answers works. **

On Spark, measured 2026-09-10:

    pip install usd-core     PyPI has no Linux aarch64 wheel -- x86_64, macOS
                             and win_amd64 only -- so pip falls through to
                             building all of USD from source.
    python.sh on its own     there is no `pxr` directory anywhere in the Isaac
                             release tree, and sourcing setup_python_env.sh
                             does not add one.
    python.sh, Kit started   works. Kit extends sys.path when SimulationApp
                             comes up. About 17 s.

That is why this lives in one module instead of being rediscovered by each
tool. Two tools already got it wrong in different ways.

One more thing that bites before any of the above: `python.sh` refuses to use
its own interpreter while a conda env is active. It warns, falls back to the
system python, and the error you finally see names `pxr` -- not conda.

    conda deactivate
    "$ISAACSIM/python.sh" tools/whatever.py
"""
import sys

_KIT_APP = None


def ensure_pxr(via="auto", quiet=False):
    """Make `pxr` importable. Returns (route, where).

    `where` is a list of paths, not one path: under Kit, `pxr` is a NAMESPACE
    package, so its `__file__` is None and only `__path__` says anything. The
    first version of this printed `__file__` and told us "None", which is true
    and useless.

    via: "auto" (direct, then Kit), "direct" (never start Kit), "kit".
    """
    global _KIT_APP

    if via in ("auto", "direct"):
        try:
            import pxr
            return "direct", list(getattr(pxr, "__path__", []) or
                                  [getattr(pxr, "__file__", "?")])
        except ImportError:
            if via == "direct":
                raise RuntimeError(
                    "no pxr, and --via direct forbids starting Kit. Install "
                    "usd-core (x86_64 / macOS only -- there is no Linux "
                    "aarch64 wheel), or use --via kit.")

    try:
        from isaacsim import SimulationApp
    except ImportError:
        raise RuntimeError(
            "no pxr and no isaacsim either. On Spark run this with "
            "IsaacSim/_build/linux-aarch64/release/python.sh, and "
            "`conda deactivate` first or python.sh refuses to use its own "
            "interpreter. Elsewhere, pip install usd-core.")

    if not quiet:
        print("starting a headless SimulationApp purely to get pxr on "
              "sys.path (about 17 s) ...", flush=True)
    _KIT_APP = SimulationApp({"headless": True})
    import pxr
    return "simulation_app", list(getattr(pxr, "__path__", []) or
                                  [getattr(pxr, "__file__", "?")])


def close_pxr():
    """Shut Kit down, if we started it. Flush first: Kit writes its own
    shutdown banner to the same stream and will interleave with our last
    line otherwise -- which it did, on the first run that used this."""
    global _KIT_APP
    sys.stdout.flush()
    sys.stderr.flush()
    if _KIT_APP is not None:
        try:
            _KIT_APP.close()
        except Exception:
            pass
        _KIT_APP = None


def flat(msg):
    """pxr raises multi-line exceptions with embedded paths and tabs. A
    failure message that scrolls is a failure message nobody reads."""
    return " ".join(str(msg).split())
