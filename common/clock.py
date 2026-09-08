"""Single shared time base for the arm process and the camera process.

The two programs run as INDEPENDENT OS processes, but every record they emit is
stamped with the SAME clock so their streams can be aligned offline afterwards
(e.g. to reconstruct one synchronized demonstration).

Why this works: time.monotonic() on Linux is CLOCK_MONOTONIC, which counts from
boot and is system-wide, so it is directly comparable across processes on the
same machine. We also stamp time.time() (wall clock) for human-readable
correlation. (This is the D103 lesson: never let each stream use its own clock.)
"""
import time


def stamp():
    """Put this in every emitted record."""
    return {"t_mono": time.monotonic(), "t_unix": time.time()}


def epoch():
    """Snapshot the monotonic<->wall relationship ONCE at startup.

    Written to each output so a reader can convert either way even across
    machines or days. Do not recompute per-frame.
    """
    return {"t_mono": time.monotonic(), "t_unix": time.time()}
