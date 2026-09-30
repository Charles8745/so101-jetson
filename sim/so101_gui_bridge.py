"""在 Isaac Sim GUI 內部執行的 P3 橋接程式。

用途：`python.bat` 啟動的視窗算不出畫面，但一般的 Isaac Sim GUI 正常。
這段程式讓 GUI 自己扮演 sim/receiver.py 的角色，直接驅動畫面上的手臂，
Jetson 那邊的 p3 指令完全不用改。

用法
----
1. 開 Isaac Sim GUI，File → Open 開啟 new.usda，確認看得到手臂。
2. Window → Script Editor，把這整個檔案的內容貼進去。
3. 確認最上面的 REPO 路徑正確。
4. 按 Run（或 Ctrl+Enter）。
5. 看到 "[gui] listening on 0.0.0.0:9871" 就可以在 Jetson 執行 p3。

要停止：在 Script Editor 貼上 `stop_so101()` 再按 Run。

注意：物理由 GUI 的時間軸推進（這段程式會自動按 Play），
不是由 receiver 的 tick 推進，所以 ack 裡的 sim_time 只是概略值。
方向驗證與錄製都不受影響。
"""
import os
import sys
import builtins
import threading
import time

# ----------------------------------------------------------------- 設定
REPO = r"C:\Users\702A\so101-jetson"          # so101-jetson 的位置
MAP = os.path.join(REPO, "configs", "simmap_leader.json")
PORT = 9871                                    # 要跟 p3 的 --cmd-port 一致
FPS = 30.0                                     # receiver 的 tick 速率

if REPO not in sys.path:
    sys.path.insert(0, REPO)

import numpy as np                                                  # noqa: E402
import omni.kit.app                                                 # noqa: E402
import omni.usd                                                     # noqa: E402
import omni.timeline                                                # noqa: E402
from pxr import UsdPhysics                                          # noqa: E402
from isaacsim.core.prims import Articulation                        # noqa: E402

from arm.sim_mapping import SimMap                                  # noqa: E402
from sim.backend import SimBackend                                  # noqa: E402
from sim.receiver import Receiver                                   # noqa: E402

# Script Editor 每次執行都是新的 globals，所以狀態掛在 builtins 上，
# 重跑時才找得到上一次的 receiver 並把它關掉。
_G = getattr(builtins, "_SO101_GUI", None)
if _G is None:
    _G = {}
    builtins._SO101_GUI = _G


def stop_so101():
    """停掉 receiver 與更新回呼。重跑這個檔案前會自動先呼叫一次。"""
    g = builtins._SO101_GUI
    rcv = g.pop("receiver", None)
    if rcv is not None:
        rcv.stop = True
    th = g.pop("thread", None)
    if th is not None:
        th.join(timeout=5.0)
    sub = g.pop("sub", None)
    if sub is not None:
        try:
            sub.unsubscribe()
        except Exception:
            pass
    g.clear()
    print("[gui] stopped")


builtins.stop_so101 = stop_so101
if _G:
    stop_so101()


# ------------------------------------------------------------- backend
class GuiBackend(SimBackend):
    """把 receiver 的指令交給主執行緒的更新回呼去套用。

    receiver 跑在背景執行緒，而 PhysX 只能在主執行緒讀寫，所以這裡只做
    「存下最新目標」與「回報最近一次量到的角度」，實際的讀寫都在
    _on_update 裡。回報值會落後約一個影格，遠小於 5.7 度的容忍值。
    """

    name = "isaac-gui"
    readback = True                 # 回報的是量測值，不是我們自己的輸入

    def __init__(self, dof_names):
        self.dof_names = dict(dof_names)
        self.our = list(self.dof_names.keys())
        self.sim = [self.dof_names[o] for o in self.our]
        self.lock = threading.Lock()
        self.pending = None
        self.measured = {o: 0.0 for o in self.our}
        self.limits = None
        self._t = 0.0

    def joint_names(self):
        return list(self.our)

    def apply(self, joints_rad):
        applied, clipped = {}, []
        for o in self.our:
            if o not in joints_rad:
                continue
            v = float(joints_rad[o])
            if self.limits and o in self.limits:
                lo, hi = self.limits[o]
                if v < lo:
                    v, _ = lo, clipped.append(o)
                elif v > hi:
                    v, _ = hi, clipped.append(o)
            applied[o] = v
        with self.lock:
            self.pending = dict(applied)
        return applied, clipped

    def step(self):
        # 物理由 GUI 的時間軸推進，這裡只讓出 CPU。
        time.sleep(0.001)

    def read(self):
        with self.lock:
            return dict(self.measured)

    def sim_time(self):
        return self._t


# --------------------------------------------------- 找出 articulation
stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("沒有開啟中的 stage。先 File → Open 開啟 new.usda。")

art_path = None
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        art_path = prim.GetPath().pathString
        break
if art_path is None:
    raise RuntimeError(
        "這個 stage 裡找不到 articulation root。確認開的是 new.usda，"
        "而且 Stage 面板看得到手臂。")
print(f"[gui] articulation root: {art_path}")

mp = SimMap.from_file(MAP)
sim_target = mp.doc.get("sim_target") or {}
dof_names = sim_target.get("dof_names")
if not dof_names:
    raise RuntimeError(f"{MAP} 裡沒有 dof_names，無法對應關節名稱。")
print(f"[gui] map {os.path.basename(MAP)}  sha {mp.sha256()[:12]}  "
      f"verified {mp.is_verified()}")

backend = GuiBackend(dof_names)
art = Articulation(prim_paths_expr=art_path, name="so101_gui")

omni.timeline.get_timeline_interface().play()     # 沒有 Play 就沒有物理


# ------------------------------------------------- 主執行緒的更新回呼
def _read_limits(a, order):
    try:
        lim = a.get_dof_limits()
        try:
            lim = lim.numpy()
        except Exception:
            pass
        lim = np.asarray(lim)
        while lim.ndim > 2:
            lim = lim[0]
        idx = a.get_dof_index
        return {o: (float(lim[idx(t)][0]), float(lim[idx(t)][1]))
                for o, t in order}
    except Exception as e:
        print(f"[gui] 讀不到關節極限（不影響操作）: {type(e).__name__}: {e}")
        return None


def _on_update(e):
    g = builtins._SO101_GUI
    if not g.get("ready"):
        try:
            art.initialize()
            backend.limits = _read_limits(art, list(zip(backend.our,
                                                        backend.sim)))
            g["ready"] = True
            names = list(getattr(art, "dof_names", None)
                         or getattr(art, "joint_names"))
            print(f"[gui] articulation 已就緒，關節: {names}")
        except Exception:
            return                      # 物理還沒起來，下一個影格再試

    with backend.lock:
        pending = backend.pending
        backend.pending = None

    if pending:
        names = [backend.dof_names[o] for o in pending]
        vals = [pending[o] for o in pending]
        try:
            art.set_joint_position_targets(
                np.asarray([vals], dtype=np.float32), joint_names=names)
        except Exception as ex:
            print(f"[gui] 寫入目標失敗: {type(ex).__name__}: {ex}")

    try:
        pos = art.get_joint_positions(joint_names=backend.sim)
        try:
            pos = pos.numpy()
        except Exception:
            pass
        arr = np.asarray(pos)
        while arr.ndim > 1:
            arr = arr[0]
        with backend.lock:
            backend.measured = {o: float(arr[i])
                                for i, o in enumerate(backend.our)}
    except Exception:
        pass

    try:
        backend._t += float(e.payload.get("dt", 1.0 / 60.0))
    except Exception:
        backend._t += 1.0 / 60.0


sub = (omni.kit.app.get_app().get_update_event_stream()
       .create_subscription_to_pop(_on_update, name="so101_gui_bridge"))

# ------------------------------------------------------ 啟動 receiver
rcv = Receiver(backend, port=PORT, fps=FPS, expect_map_sha=mp.sha256(),
               out=os.path.join(REPO, "logs", "p3_sim_gui"))
th = threading.Thread(target=rcv.run, name="so101_receiver", daemon=True)
th.start()

_G.update({"receiver": rcv, "thread": th, "sub": sub, "art": art,
           "backend": backend})

print(f"[gui] listening on 0.0.0.0:{PORT}  backend=isaac-gui readback=True")
print(f"[gui] logs: {rcv.run_dir}")
print("[gui] 現在可以在 Jetson 執行 p3。要停止就執行 stop_so101()")
