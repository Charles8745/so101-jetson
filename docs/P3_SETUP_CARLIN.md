# P3 在 Jetson + Windows 上的實機設定

這份文件記錄 P3（真實 leader 手臂遙控 Isaac Sim 裡的虛擬 SO-101）在本實驗室這組機器上的完整設定與實測結果。原始的 `docs/SIM_BRIDGE.md` 假設模擬器跑在 Spark（Linux aarch64），這裡改成 Windows + Isaac Sim 6.0.1。

最後更新：2026-09-23

---

## 1. 機器與網路

| 角色 | 機器 | IP | 說明 |
|---|---|---|---|
| 手臂端 | Jetson（Ubuntu, aarch64, R39） | 192.168.50.108 | 接 leader 手臂，執行 `p3` |
| 模擬端 | Windows 11 + RTX 4090 | 192.168.50.219 | Isaac Sim 6.0.1，執行 receiver |

- 通訊：UDP，指令 9871（Jetson → Windows），ack 9872（Windows → Jetson）
- Windows 防火牆要放行 UDP 9871：
  ```powershell
  New-NetFirewallRule -DisplayName "so101 p3" -Direction Inbound -Protocol UDP -LocalPort 9871 -Action Allow
  ```
  （需要以系統管理員身分執行 PowerShell）
- Jetson 的 leader 序號 `5B79050417`，校正檔 `my_leader.json` 放在
  `~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/`
- P3 不需要 follower 手臂。

---

## 2. USD 資產的製作與修正

### 2.1 來源

USD 由 TheRobotStudio 官方 URDF 匯入：

```
https://github.com/TheRobotStudio/SO-ARM100
  Simulation/SO101/so101_new_calib.urdf   ← 匯入來源
  Simulation/SO101/so101_new_calib.xml    ← MuJoCo 模型，增益來源
```

用 `so101_new_calib`（零點在各關節行程中點），不要用 `so101_old_calib`。

### 2.2 匯入設定（Isaac Sim 6.0 的新版 URDF Importer）

| 選項 | 設定 |
|---|---|
| Base type | **Fixed** |
| Collision from visuals | 不勾 |
| Allow self-collision | 不勾 |
| Merge mesh | 不勾 |

新版匯入器沒有 Joint Drive Type 選項，它會自動建立 drive，但所有增益都是 0。

### 2.3 修正驅動器增益（必做）

匯入後的資產 `stiffness = 0`、`damping = 0`、`maxForce = 10`。stiffness 為 0 的關節無法跟隨目標，會自由擺動；`maxForce = 10` 是真實 STS3215（3.35 N·m）的三倍，模擬手臂會舉起真實手臂舉不動的東西。

```powershell
conda deactivate
cd C:\Users\702A\so101-jetson
$ISAAC = "C:\isaacsim\python.bat"
$USD   = "C:\so101_usd\so101_new_calib\so101_new_calib.usda"
$MJCF  = "C:\SO-ARM100\Simulation\SO101\so101_new_calib.xml"

& $ISAAC tools\usd_apply_mjcf_gains.py --usd $USD --mjcf $MJCF --dry-run
& $ISAAC tools\usd_apply_mjcf_gains.py --usd $USD --mjcf $MJCF --out C:\so101_usd\so101_new_calib\new.usda
& $ISAAC tools\usd_joint_report.py --via kit --usd C:\so101_usd\so101_new_calib\new.usda --mjcf $MJCF
```

第三行必須印出 `asset matches the MJCF`（修正前是 30 個 mismatch）。

修正後的數值：

| 參數 | 值 | 備註 |
|---|---|---|
| stiffness | 17.4222 N·m/deg | = MuJoCo 的 kp 998.22 N·m/rad ÷ 57.29578 |
| damping | 0.0581369 N·m·s/deg | kv 2.731 + joint damping 0.6 |
| maxForce | 3.35 N·m | 真實 STS3215 的出力 |
| armature | 0.028 kg·m² | |
| frictionloss | 0.052 N·m | |

**USD 的角度驅動欄位是「每度」，MuJoCo 是「每弧度」，兩者差 57.29578 倍。** 直接把 MuJoCo 的值寫進 USD 是 57 倍的靜默錯誤，工具會自動換算。

之後所有指令都用 `new.usda`，不要用原本匯入的那份。

---

## 3. 關節極限與映射檔

### 3.1 讀出模型極限（Windows）

```powershell
& $ISAAC sim\probe_isaac.py --stage limits --usd C:\so101_usd\so101_new_calib\new.usda --joints identity --out sim_limits.json
scp sim_limits.json ictalab@192.168.50.108:~/sim_limits_win.json
```

`--joints identity` 表示 USD 的關節名稱與 lerobot 完全相同，這份資產確實如此（`shoulder_pan`…`gripper`）。

讀到的極限（度）：

| 關節 | 下限 | 上限 |
|---|---|---|
| shoulder_pan | -110.0 | +110.0 |
| shoulder_lift | -100.0 | +100.0 |
| elbow_flex | -96.83 | +96.83 |
| wrist_flex | -95.0 | +95.0 |
| wrist_roll | -157.21 | +162.79 |
| gripper | -10.0 | +100.0 |

這組數值與前一位同學在 Spark 上做的 USD 完全一致，確認是同一份 URDF 轉出來的。

### 3.2 擬合與驗證（Jetson）

```bash
cd ~/so101-jetson
so101 simmap fit --arm-role leader --arm-id my_leader \
    --sim-limits ~/sim_limits_win.json --out configs/simmap_leader.json
so101 simmap show --map configs/simmap_leader.json
```

方向檢查的結果：**6 個關節全部同向，不需要 `--flip`。**

```bash
so101 simmap verify --map configs/simmap_leader.json \
    --by "Carlin" --method visual \
    --note "2026-09-23 逐一移動 6 關節，虛擬手臂方向與幅度一致（Isaac GUI 橋接）"
```

驗證後的 map sha：`96be3c561f96`（驗證會改變 sha）。

> **verify 之後一定要把映射檔重新複製到 Windows。** 接收端會比對 sha，對不上會回 `map_sha_refused`，p3 送出的指令全部無效。

span ratio 有四個關節大於 1，代表真實手臂的行程比模型大，極端姿勢會被截斷，這是 identity 擬合的正常現象：

| 關節 | 真實行程 | 模型行程 | 比值 |
|---|---|---|---|
| shoulder_pan | 237.3° | 220.0° | 1.079 |
| shoulder_lift | 207.6° | 200.0° | 1.038 |
| elbow_flex | 193.6° | 193.7° | 1.000 |
| wrist_flex | 204.0° | 190.0° | 1.074 |
| wrist_roll | 360.0° | 320.0° | 1.125 |

---

## 4. 執行方式

### 4.1 目前使用的方式：Isaac GUI 內橋接

**背景：** `python.bat` 啟動的 Isaac 視窗在這台 Windows 上 viewport 完全畫不出東西（連 Cube 都看不到），但物理模擬正常。同一份 `new.usda` 用一般的 Isaac Sim GUI 開啟則顯示正常。因為關節方向驗證必須用眼睛看，所以改用 `sim/so101_gui_bridge.py`，讓 GUI 本身扮演 receiver。

**Windows：**

1. 開 Isaac Sim GUI（`isaac-sim.bat`）→ File → Open → `C:\so101_usd\so101_new_calib\new.usda`
2. Window → Script Editor，貼上 `sim/so101_gui_bridge.py` 的內容，確認最上面的 `REPO` 路徑正確
3. 按 Run，等出現：
   ```
   [gui] articulation root: /so101_new_calib/Geometry
   [gui] map simmap_leader.json  sha 96be3c561f96  verified True
   [gui] listening on 0.0.0.0:9871  backend=isaac-gui readback=True
   ```

**Jetson：**

```bash
cd ~/so101-jetson
p3 --map configs/simmap_leader.json --sim 192.168.50.219 --fps 20
```

**停止：** Script Editor 執行 `stop_so101()`，Jetson 按 `q`，關掉主手臂電源。

橋接程式的運作方式：receiver 跑在背景執行緒，關節目標的寫入與量測值的讀取都在 Kit 的 update 回呼（主執行緒）裡進行，因為 PhysX 不能跨執行緒操作。物理由 GUI 的時間軸推進，所以 ack 裡的 `sim_time` 只是概略值，方向驗證與錄製都不受影響。

### 4.2 正規方式：standalone receiver

viewport 問題解決之後可以改回這條路，時間戳比較精確：

```powershell
& $ISAAC sim\receiver.py --backend isaac --map configs\simmap_leader.json --usd C:\so101_usd\so101_new_calib\new.usda
```

`sim/isaac_adapter.py` 已針對 Isaac 6.0.1 修正，見下一節。

---

## 5. `sim/isaac_adapter.py` 的修改

這個檔案在原 repo 裡標註為 written, never run。在 Isaac Sim 6.0.1 上首次實際執行，改了三處：

| 修改 | 原因 |
|---|---|
| `_read_limits()` 先去掉 batch 維度 | 6.0.1 的 `get_dof_limits()` 回傳 (1, K, 2)，原程式直接用 DOF index 索引第 0 軸，會 `IndexError: index 2 is out of bounds for axis 0 with size 1` |
| 新增 `_add_light_and_camera()` | 原本的場景沒有任何燈光，RTX 畫面必然全黑；同時把視角對準手臂。失敗只印警告，不影響物理 |
| `SO101_KIT_EXP` 環境變數 | 可指定 Kit experience 檔（例如 `isaacsim.exp.full.kit`），不設定時行為與原本相同 |

API 方面 6.0.1 與原本針對的 6.0.0-rc.22 相容，`isaacsim.core.prims.Articulation`、`isaacsim.core.api.World`、`isaacsim.core.utils.stage.add_reference_to_stage` 都沒有變動。

---

## 6. 實測數據（2026-09-23）

兩次錄製，差別只在 `--fps`：

| 指標 | 30 Hz | 20 Hz |
|---|---|---|
| sent / acked | 1561 / 1474 | **1644 / 1644** |
| lost | 0 | 0 |
| superseded | 87（5.6%） | **0** |
| mismatched | 74 | **17（1%）** |
| 最大偏差 | 19.2°（gripper） | 12.9°（gripper） |
| 回合內截斷 | 761 / 1081（70%） | **202 / 1058（19%）** |

30 Hz 時模擬器跟不上，超過 5% 的指令被略過；20 Hz 下每一筆都確實套用。**建議固定用 `--fps 20`。**

延遲分解（30 Hz 那次，`tools/analyze_latency.py`）：

```
rtt total                      : 19.34 / 31.08   (median / p95, ms)
  ... waiting for the sim's tick : 16.00 / 32.00
  ... the sim step itself        :  0.00 /  0.00
  ... network + our own handling :  0.78 /  8.79

SIM-FOLLOW latency by joint (cross-correlation)
  MEDIAN : 33.8 ms   (一個取樣週期，各關節 r ≥ 0.99)
```

主要延遲來自等待模擬器的 tick，不是網路。

---

## 7. 已知問題

| 問題 | 狀態 |
|---|---|
| `python.bat` 啟動的 Isaac 視窗 viewport 全黑 | **未解決。** 連 Cube 都畫不出來；一般 Isaac GUI 正常。清 `%LOCALAPPDATA%\ov\cache` 的 shaders / DerivedDataCache / texturecache、改用完整版 experience 都無效。啟動時持續出現 `carb.ujitsoagent TRANSIENT_FAILURE` 與 `UJITSO : Preload data failed for 'MDLToHlsl : DebugWhite'`。目前以 GUI 橋接繞過。未嘗試：更新顯示卡驅動 |
| gripper 偏差 12–19° | 只出現在快速開合的瞬間（1644 步中 17 步）。目標範圍 -10–46°、實際 -10–45.9°，沒有被卡住，是驅動追隨落後。動作放慢即可 |
| `shoulder_lift` 截斷 | 手臂收合時約 -102.7°，超出模型的 -100°。錄資料時先把大臂抬起來，截斷從 70% 降到 19% |

---

## 8. 錄資料的規則

1. 按 `s` 之前先把大臂抬到大約水平，明顯離開收合位置。
2. 按 `e` 結束之後才把手臂收回去，不要邊錄邊收。
3. 動作放慢，特別是夾爪開合。
4. 記得轉一下 `wrist_roll`，否則延遲分析會 skip 這個關節。
5. 用 `--fps 20`。
6. 操作鍵：`s` 開始、`e` 結束並保留、`d` 丟棄、`q` 離開、Enter 看狀態。

資料寫在 `logs/p3/<時間戳>/rows.jsonl`，被丟棄的回合也會標記在 rows 裡，載入資料集時要依 `so101.episode.v1` 的結果過濾。

---

## 9. 本 fork 新增與修改的檔案

| 檔案 | 說明 |
|---|---|
| `sim/so101_gui_bridge.py` | 新增。在 Isaac Sim GUI 的 Script Editor 內執行，讓 GUI 當 receiver |
| `sim/isaac_adapter.py` | 修改。Isaac 6.0.1 的 dof limits 修正、自動燈光與鏡頭、`SO101_KIT_EXP` |
| `configs/simmap_leader.json` | 新增。已驗證的 leader 映射檔，sha `96be3c561f96`，對應 `new.usda` |
| `.gitignore` | 新增 `logs/` |

沒有納入版本控制的：`devices.env`（本機硬體設定）、`logs/`（執行紀錄）、`new.usda` 與網格檔（體積過大，依第 2 節自行產生）。
