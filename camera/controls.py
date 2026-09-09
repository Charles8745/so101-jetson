"""Query and lock V4L2 camera controls.

Why this matters for a robot dataset (not just for pretty pictures):

  * Auto-exposure makes every episode a different exam paper. Worse, when the
    arm enters frame the exposure compensates and the BACKGROUND BRIGHTNESS
    becomes correlated with the arm's position -- a policy can read arm pose off
    the background. That is a spurious cue that will not survive deployment.
  * Auto-exposure also varies the exposure TIME, so the frame interval jitters
    and timestamps stop matching the true sampling instants.
  * Mains flicker (Taiwan is 60 Hz) produces rolling bands unless
    power_line_frequency is set correctly.

Control names were renamed in newer kernels (exposure_auto -> auto_exposure,
white_balance_temperature_auto -> white_balance_automatic), so nothing here
hard-codes a name: we read what the device actually offers and match aliases.
"""
import re
import shutil
import subprocess

# first match wins; covers both the old and the new kernel naming
ALIASES = {
    "auto_exposure": ("auto_exposure", "exposure_auto"),
    "exposure_time": ("exposure_time_absolute", "exposure_absolute"),
    "auto_white_balance": ("white_balance_automatic",
                           "white_balance_temperature_auto"),
    "white_balance": ("white_balance_temperature",),
    "power_line_frequency": ("power_line_frequency",),
    "gain": ("gain",),
    "brightness": ("brightness",),
}

# v4l2 menu value for manual exposure is 1 in both the old and new control
MANUAL_EXPOSURE = 1
# UVC's menu is 0 auto / 1 manual / 2 shutter priority / 3 aperture priority,
# and most webcams expose "auto" as 3 rather than 0. But we do not assume that:
# restoring auto reads the control's OWN default and writes that back. This is
# only the fallback for a device that reports no default.
AUTO_EXPOSURE_FALLBACK = 3

_LINE = re.compile(r"^\s*(\w+)\s+0x[0-9a-fA-F]+\s+\((\w+)\)\s*:\s*(.*)$")


def parse_ctrls(text):
    """Parse `v4l2-ctl --list-ctrls` output into {name: {type, value, ...}}."""
    out = {}
    for line in text.splitlines():
        m = _LINE.match(line)
        if not m:
            continue
        name, ctype, rest = m.group(1), m.group(2), m.group(3)
        info = {"type": ctype}
        for kv in rest.split():
            if "=" in kv:
                k, v = kv.split("=", 1)
                try:
                    info[k] = int(v)
                except ValueError:
                    info[k] = v
        out[name] = info
    return out


def resolve(ctrls, logical):
    """Map a logical name ('auto_exposure') to whatever this device calls it."""
    for cand in ALIASES.get(logical, ()):
        if cand in ctrls:
            return cand
    return None


def have_v4l2ctl():
    return shutil.which("v4l2-ctl") is not None


def list_controls(device):
    """Returns (ctrls dict, error or None)."""
    if not have_v4l2ctl():
        return {}, "v4l2-ctl not installed (apt install v4l-utils)"
    try:
        r = subprocess.run(["v4l2-ctl", "-d", device, "--list-ctrls"],
                           capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.SubprocessError) as e:
        return {}, f"{type(e).__name__}: {e}"
    if r.returncode != 0:
        return {}, (r.stderr or r.stdout).strip()[:200]
    return parse_ctrls(r.stdout), None


def set_ctrl(device, name, value):
    try:
        r = subprocess.run(["v4l2-ctl", "-d", device, "-c", f"{name}={value}"],
                           capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.SubprocessError) as e:
        return False, f"{type(e).__name__}: {e}"
    if r.returncode != 0:
        return False, (r.stderr or r.stdout).strip()[:200]
    return True, ""


def set_power_line(device, power_line_hz):
    """Mains frequency only. NOTHING else.

    This used to be a parameter of lock_exposure(), which meant asking for
    anti-flicker silently locked the exposure and the white balance too --
    there was no way to have one without the other. They are unrelated
    settings: flicker is about the light source, exposure is about the sensor.
    """
    ctrls, err = list_controls(device)
    if err:
        return [{"step": "list_controls", "ok": False, "detail": err}]
    n = resolve(ctrls, "power_line_frequency")
    if not n:
        return [{"step": "power_line_frequency", "ok": False,
                 "detail": "control not offered by this camera"}]
    v = {50: 1, 60: 2, 0: 0}.get(power_line_hz)
    if v is None:
        return [{"step": "power_line_frequency", "ok": False,
                 "detail": f"unsupported {power_line_hz} Hz"}]
    ok, detail = set_ctrl(device, n, v)
    return [{"step": f"{n}={v} ({power_line_hz} Hz)", "ok": ok, "detail": detail}]


def unlock_exposure(device):
    """Put auto exposure and auto white balance back.

    ** V4L2 controls are DEVICE state, not process state. ** A run that locked
    the exposure leaves the camera locked -- the next run inherits it, and every
    run after that, until someone unplugs the camera. Nothing in a later log
    would say so, which is exactly how two days of episodes end up with
    different exposure settings and no record of it. So "auto" is something we
    SET, not something we assume by not asking.

    The value written is the control's own reported DEFAULT, because what "auto"
    means in the menu differs between cameras.
    """
    steps = []
    ctrls, err = list_controls(device)
    if err:
        return [{"step": "list_controls", "ok": False, "detail": err}]
    for logical, fallback in (("auto_exposure", AUTO_EXPOSURE_FALLBACK),
                              ("auto_white_balance", 1)):
        n = resolve(ctrls, logical)
        if not n:
            steps.append({"step": logical, "ok": False,
                          "detail": "control not offered by this camera"})
            continue
        dflt = ctrls[n].get("default")
        v = int(dflt) if dflt is not None else fallback
        ok, detail = set_ctrl(device, n, v)
        steps.append({"step": f"{n}={v} (auto"
                             + ("" if dflt is not None else ", assumed") + ")",
                      "ok": ok, "detail": detail})
    return steps


def lock_exposure(device, exposure_time=None, white_balance=None):
    """Turn OFF auto exposure / auto white balance, then optionally pin values.

    Order matters: the manual controls are flagged inactive while auto is on,
    so auto must be disabled first.

    Mains frequency is NOT set here -- see set_power_line().
    Returns a list of {step, ok, detail} for the event log.
    """
    steps = []
    ctrls, err = list_controls(device)
    if err:
        return [{"step": "list_controls", "ok": False, "detail": err}]

    ae = resolve(ctrls, "auto_exposure")
    if ae:
        ok, detail = set_ctrl(device, ae, MANUAL_EXPOSURE)
        steps.append({"step": f"{ae}={MANUAL_EXPOSURE} (manual)",
                      "ok": ok, "detail": detail})
    else:
        steps.append({"step": "auto_exposure", "ok": False,
                      "detail": "control not offered by this camera"})

    awb = resolve(ctrls, "auto_white_balance")
    if awb:
        ok, detail = set_ctrl(device, awb, 0)
        steps.append({"step": f"{awb}=0 (manual)", "ok": ok, "detail": detail})

    # re-read: the manual controls only become settable after auto is off
    ctrls, _ = list_controls(device)

    if exposure_time is not None:
        n = resolve(ctrls, "exposure_time")
        if n:
            ok, detail = set_ctrl(device, n, exposure_time)
            steps.append({"step": f"{n}={exposure_time}", "ok": ok,
                          "detail": detail})
    if white_balance is not None:
        n = resolve(ctrls, "white_balance")
        if n:
            ok, detail = set_ctrl(device, n, white_balance)
            steps.append({"step": f"{n}={white_balance}", "ok": ok,
                          "detail": detail})
    return steps


def snapshot(device):
    """Current values of the controls we care about, for the event log."""
    ctrls, err = list_controls(device)
    if err:
        return {"error": err}
    out = {}
    for logical in ALIASES:
        n = resolve(ctrls, logical)
        if n:
            out[logical] = {"control": n, "value": ctrls[n].get("value"),
                            "default": ctrls[n].get("default")}
    return out
