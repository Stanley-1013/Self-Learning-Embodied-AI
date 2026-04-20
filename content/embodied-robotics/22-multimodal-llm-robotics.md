---
title: "多模態大模型與具身感知決策"
prerequisites: ["20-imitation-learning-dagger", "21-sim-to-real-transfer"]
estimated_time: 50
difficulty: 4
tags: ["vla", "world-model", "foundation-model", "embodied-ai", "llm", "grounding"]
sidebar_position: 22
---

# 多模態大模型與具身感知決策

## 你將學到

- 能精確講出 VLA、World Model、Foundation Model、End-to-End、Grounding Problem、Action Tokenization 六個概念的定義與差異，面試時不含糊
- 遇到「機器人要理解模糊自然語言指令並執行操作」時，知道該用分層架構（VLM 高階規劃 + MPC 底層控制），並理解為什麼 LLM 不能直接輸出關節力矩
- 判斷什麼場景用 Foundation Model 的零樣本泛化（新物體、新指令），什麼場景仍需 fine-tune 或傳統控制

## 核心概念

### 六個精確定義

1. **VLA (Vision-Language-Action) Model**：接收視覺觀測（RGB/depth）和自然語言指令，直接輸出機器人動作（末端位姿、關節角度或 action token）的 end-to-end 模型。代表：RT-2、Octo、OpenVLA。本質是把 VLM 的輸出從「文字 token」擴展到「動作 token」。和純 RL 的根本差異：VLA 汲取了互聯網規模的視覺-語言預訓練知識，而非從零學起。

2. **World Model**：學習環境動力學的預測模型 — 給定當前狀態和動作，預測未來的觀測序列。可在 latent space 裡「做白日夢」— 不需要真正執行就能模擬行動後果，用於 model-based planning。代表：DreamerV3、UniSim、Genie。物理意義：機器人在腦中預演「如果我這樣做，世界會怎樣？」。

3. **Foundation Model for Robotics**：在大規模多任務數據上預訓練的通用模型，具備零樣本或少樣本泛化到新任務/新物體/新環境的能力。核心價值是把網路規模的語義知識（「杯子是拿來喝水的」）帶進機器人決策迴路。和 task-specific 模型的差異：一個模型服務多個任務，而非每個任務一個模型。

4. **End-to-End Learning**：從原始感測器（影像、力感測器）直接到控制輸出（關節力矩、末端速度），省去中間的手工特徵和模組化 pipeline。優勢：省去工程瓶頸，能學到人類設計不出的中間表徵。代價：黑箱、重度數據依賴、debug 困難、安全保證弱。

5. **Grounding Problem**：LLM 從文字學到的「物理知識」是統計相關性，不是真正的物理理解。它知道「杯子裝水」，但不知道翻轉杯子水會灑出來的力學原因。在機器人場景，grounding 失敗 = 幻覺 — LLM 自信地生成物理上不可行的計劃（「穿過牆壁」、「用無限長手臂」）。語言幻覺只是講錯話，物理幻覺會撞壞機器人或傷人。

6. **Action Tokenization**：把連續的機器人動作（7-DoF 末端位姿）離散化為 token，讓 Transformer 用自迴歸方式預測動作序列 — 和預測下一個文字 token 完全相同的架構。RT-2 每維用 256 bins 離散化，並**重用 VLM vocabulary 中使用頻率最低的 256 個 token 作為 action token**（不新增 token、直接復用 Transformer head）— 這是 VLA 能無縫吃 VLM 預訓練權重的關鍵 trick。trade-off：bins 太少 → 精度差，bins 太多 → vocabulary 爆炸。

### 閉環定位

**在感知 → 規劃 → 控制閉環的位置**：
- **輸入**：RGB/depth 影像、自然語言指令、（可選）力/觸覺感測器數據
- **輸出**：取決於架構 — task plan（子任務序列）、target pose（目標位姿）、或直接的 action token
- **下游**：底層控制器（MPC / PID / RL policy）執行動作 → 感測器回饋 → 下一幀觀測
- **閉環節點**：坐落在閉環的**高階認知中樞**，統一了感知和決策。但幾乎不直接觸碰底層控制 — LLM/VLM 推理延遲 200-2000ms，控制迴路需要 1-10ms，頻率差 100 倍以上，分層是必然的

### 三種架構模式

| 架構 | 代表 | 高階頻率 | 優勢 | 劣勢 |
|------|------|---------|------|------|
| LLM as Task Planner | SayCan, Code as Policies | 0.1-1 Hz | 利用 LLM 語義知識、skill library 可複用 | 需要預定義 skill、grounding 問題 |
| VLA End-to-End | RT-2, Octo, OpenVLA | 1-10 Hz | 省去中間模組、能學隱式表徵 | 需大量機器人數據、黑箱 |
| VLM + Low-level Controller | VLM+MPC 分層 | VLM 1-5 Hz + MPC 100+ Hz | 語義理解 + 即時反應兼顧 | 系統複雜度高 |

### 最少夠用的數學 / API

1. **VLA 的 action tokenization**：

$$
a_t = \text{Detokenize}\left( \arg\max_{a \in \mathcal{V}_a} P_\theta(a \mid I_t, l, a_{<t}) \right)
$$

**物理意義**：把 7-DoF 末端位姿的每個維度離散化為 256 個 bin → 轉成 token → 和語言 token 共享 vocabulary。模型輸出「下一個 token」，只是這個 token 被解碼為動作而非文字。自迴歸預測天然支持 action sequence generation。

2. **分層架構的頻率分離**：

$$
g_t = \pi_{\text{VLM}}(I_t, l) \quad \text{@ 1-5 Hz}
$$

$$
u_t = \pi_{\text{MPC}}(x_t, g_t) \quad \text{@ 100-1000 Hz}
$$

**物理意義**：VLM 思考慢但理解語義（「把杯子放到盤子右邊」→ 目標位姿 $g_t$），MPC 反應快但不懂語義（追蹤 $g_t$、處理力矩約束和碰撞迴避）。兩者頻率差 100 倍以上。這不是暫時的工程妥協 — 是認知和反射在計算架構上的必然分離。

3. **SayCan 的 Affordance Grounding**：

$$
\pi(a \mid l, s) = P_{\text{LLM}}(a \mid l) \cdot P_{\text{affordance}}(a \mid s)
$$

**物理意義**：LLM 提出語義上合理的候選動作（$P_{\text{LLM}}$），affordance model 判斷物理上是否可行（$P_{\text{affordance}}$）。兩者相乘排序 — LLM 負責「想得到」，affordance 負責「做得到」。解決 grounding problem 的經典範式。

4. **World Model 的預測損失**：

$$
\mathcal{L}_{\text{world}} = \mathbb{E}\left[ \sum_{t=1}^{T} \| \hat{o}_{t} - o_t \|^2 + \beta \cdot D_{\text{KL}}(q(z_t \mid o_{\le t}) \| p(z_t \mid z_{t-1}, a_{t-1})) \right]
$$

**物理意義**：第一項是觀測重建誤差（預測的未來影像和真實的差距），第二項是 latent dynamics 的 KL 正則化（讓 latent 動力學模型保持穩定）。World model 在 latent space 學環境動力學 — 能在「想像」中模擬行動後果，不需要真正執行。DreamerV3 用此框架做 model-based RL。

5. **Action Tokenization 的離散化**：

$$
\text{token}_d = \text{clip}\left(\left\lfloor \frac{a_d - a_{d,\min}}{a_{d,\max} - a_{d,\min}} \times (N_{\text{bins}} - 1) \right\rfloor, 0, N_{\text{bins}}-1\right)
$$

**物理意義**：把動作空間第 $d$ 維的連續值均勻切成 $N_{\text{bins}}$ 個 bin。RT-2 用 256 bins → 每維精度約 $(a_{max} - a_{min})/256$。7-DoF 動作 = 7 個 token → 可以和語言 token 串在一起做自迴歸預測。

6. **VLA Scaling 的經驗觀察（非正式法則）**：

RT-2 / OpenVLA 等論文的實驗顯示兩條定性趨勢，**但目前沒有已驗證的閉式 scaling law**（不像 LLM 有 Kaplan/Chinchilla 那種曲線擬合）：

- **模型 scaling**：同系列下更大的 VLM backbone（例如 RT-2-PaLI-X 55B vs 較小模型）在 unseen 物體 / 指令上的 emergent 能力明顯提升，但論文只給離散比較點，不是連續曲線
- **數據 scaling**：從數萬到 ~130K episodes，in-distribution 任務成功率顯著上升；但泛化能力更多來自 VLM backbone 的網路規模預訓練，而非機器人資料本身

**工程直覺**：機器人資料獲取速度是線性的（每台機器人每天約 100–200 個 episodes），而 LLM 能從網路無限爬文字。這個資料稀缺性是 VLA 的結構性瓶頸，不是靠堆更大模型能解的。

<details>
<summary>深入：VLA 訓練數據瓶頸與 Scaling — 為什麼數據比模型更關鍵</summary>

### 數據規模差距

| 領域 | 典型數據量 | 來源 |
|------|----------|------|
| LLM (GPT-4) | ~13T tokens | 網路文本，可無限爬取 |
| VLM (LLaVA) | ~1.2M 影像-文字對 | 網路影像 + 標註 |
| VLA (RT-2) | ~130K episodes | 真機操作（Google 內部） |
| VLA (Open X-Embodiment) | ~1M episodes | 22 個機器人、多機構聯合 |

差距 3-4 個數量級。這就是為什麼 VLA 必須從預訓練 VLM 做 fine-tune，而不能從頭訓練。

### RT-2 / OpenVLA 的 scaling 觀察

- **模型 scaling**：RT-2 論文比較了不同規模的 VLM backbone（PaLM-E 12B、PaLI-X 5B、PaLI-X 55B 等），更大的模型在 emergent / unseen 任務上明顯更好。但原文用的是個別任務的絕對百分比（例如某類 unseen 物體 +20–30 個百分點），**不是一個乾淨的「3×」倍率**，也不是連續 scaling 曲線
- **資料 scaling**：從數萬到 ~130K 真機 episodes，in-distribution 任務成功率明顯上升；但 unseen 的泛化能力主要來自 VLM 預訓練的語義知識，而非多收機器人資料
- **資料獲取速度的結構性瓶頸**：每台機器人每天約收集 100–200 episodes，線性成長；Open X-Embodiment 靠 22 個機構聯合才湊到 ~1M — 這個量級在 LLM 眼裡仍是極小

### 數據效率的四條路

1. **Sim-to-Real + Domain Randomization**：sim 裡大量生成操作數據，用 DR 增加多樣性。數據量可以 10x-100x 提升，但 sim-to-real gap 仍是問題
2. **Video Pre-training**：用 YouTube 操作影片做 video prediction pre-training → fine-tune 到 action prediction。利用了網路規模的影片數據
3. **Cross-Embodiment Transfer**：Open X-Embodiment 證明不同機器人的數據可以互相幫助 — 共享語義理解（「抓杯子」的概念跨機器人通用）
4. **Human-in-the-Loop**：DAgger + VR teleoperation 高效收集高品質 demo。比自主探索的 RL 數據效率高一個數量級

### 為什麼不能從頭訓練 VLA

130K episodes 的機器人數據，如果不借用預訓練 VLM 的語義知識，模型只能學到：
- 低級視覺特徵 → 動作的映射
- 無法理解語言指令
- 無法泛化到新物體

預訓練 VLM 帶來的：
- 豐富的物體語義（「杯子」、「碗」、「螺絲起子」的概念）
- 空間關係理解（「左邊」、「上面」、「旁邊」）
- 語言-視覺對齊（指令和影像的對應）

</details>

<details>
<summary>深入：LLM Grounding Problem — 為什麼 LLM 的物理推理常出錯以及工程解法</summary>

### 什麼是 Grounding Problem

LLM 從文字學到的「物理知識」是統計相關性，不是因果理解：
- LLM 知道「杯子裝水」，但不知道杯子翻轉時水灑出來的力學原因
- LLM 能回答「物體從高處落下會加速」，但不能準確預測 2.5 秒後的位置
- LLM 對空間推理特別弱：「A 在 B 左邊，B 在 C 左邊，A 在 C 的哪邊？」正確率遠低於 100%

### 五類典型失敗

1. **空間推理錯誤**：LLM 無法可靠判斷「這個物體能不能疊在另一個上面」（需要質心、摩擦、接觸幾何）
2. **幻覺計劃**：LLM 自信地輸出「聽起來合理但物理上不可行」的計劃（「用右手同時拿兩個碗」）
3. **力學直覺缺失**：不理解摩擦力、力矩平衡、柔性物體形變
4. **尺度感錯誤**：不知道機器人的 workspace 邊界、關節角度限制
5. **時序動力學**：不理解慣性、延遲、力的累積效應

### 四層工程應對

1. **VLM 替代純 LLM**：加入視覺輸入讓模型「看到」實際場景。GPT-4V 比 GPT-4 在物理推理上好很多，但仍不完美

2. **Affordance-based Filtering（SayCan）**：
   - LLM 提出候選動作列表
   - 每個 skill 有一個 success predictor（affordance model）
   - $\text{score}(a) = P_{\text{LLM}}(a \mid l) \times P_{\text{affordance}}(a \mid s)$
   - 語義合理 × 物理可行 = 最終排序

3. **Safety Filter**：在 LLM 輸出和真機執行之間插一層：
   - 碰撞檢測（MoveIt collision checking）
   - 力矩限制（joint torque limits）
   - Workspace boundary check
   - 不確定性閾值（uncertainty > threshold → fallback 到安全狀態）
   - Control Barrier Function (CBF) 做硬約束安全保證

4. **Closed-loop Replanning**：不把 LLM 的一次性計劃當真 — 每步執行後重新觀測、重新規劃。短 horizon 規劃 + 頻繁重規劃 > 長 horizon 一次性規劃

</details>

<details>
<summary>深入：完整 VLA Fine-tuning Pipeline（Python + Octo）</summary>

```python
"""
VLA Fine-tuning Pipeline — 以 Octo 為例
1. 準備 demo 數據（teleoperation 收集）
2. 載入預訓練 VLA
3. Fine-tune 到你的機器人 + 任務
4. 部署推理
"""
import numpy as np

# ============ Step 1: 數據格式 ============
# Octo 使用 RLDS (Robot Learning Data Sets) 格式
# 每個 episode:
#   observation: {'image_primary': (T, H, W, 3), 'state': (T, state_dim)}
#   action: (T, action_dim)  # 7-DoF delta pose + gripper
#   language_instruction: str

def create_dataset_from_demos(demo_dir: str):
    """teleoperation demo → RLDS 格式"""
    episodes = []
    for demo_path in sorted(glob(f"{demo_dir}/*.hdf5")):
        with h5py.File(demo_path) as f:
            episodes.append({
                'observation': {
                    'image_primary': f['images'][()],
                    'state': f['joint_positions'][()],
                },
                'action': f['actions'][()],
                'language_instruction': f.attrs['instruction'],
            })
    return episodes

# ============ Step 2: Fine-tune ============
def finetune_octo(pretrained_path: str, dataset, num_steps: int = 50_000):
    """
    Fine-tune Octo — 凍結 vision encoder，只 tune action head + 部分 transformer
    """
    from octo.model.octo_model import OctoModel
    model = OctoModel.load_pretrained(pretrained_path)
    config = {
        'optimizer': {'lr': 3e-5, 'warmup_steps': 1000},
        'frozen_keys': ['vision_encoder'],  # 凍結視覺骨幹
        'action_head': {'type': 'diffusion', 'num_steps': 20},
    }
    for step in range(num_steps):
        batch = sample_batch(dataset, batch_size=128)
        loss = model.train_step(batch, config)
    return model

# ============ Step 3: 部署推理 ============
class OctoInference:
    """部署 fine-tuned Octo + action chunking"""
    def __init__(self, model_path: str):
        self.model = OctoModel.load_pretrained(model_path)
        self.action_buffer = []

    def predict_action(self, image: np.ndarray, instruction: str, state: np.ndarray):
        if len(self.action_buffer) == 0:
            obs = {'image_primary': image, 'state': state}
            actions = self.model.sample_actions(obs, instruction, num_steps=4)
            self.action_buffer = list(actions)
        return self.action_buffer.pop(0)
```

**Fine-tune 關鍵決策**：
1. **凍結 vision encoder**：預訓練視覺骨幹已足夠好，全 tune 在小數據上容易 overfit
2. **Diffusion action head**：比 regression head 更能表達多模態動作分佈
3. **數據量**：桌面操作 200-1000 demo；長序列任務需更多
4. **Action chunking**：一次預測多步 + 逐步執行 → 更平滑的動作軌跡

</details>

## 直覺理解

**類比：能看能聽有常識的大腦 vs 快速反射的脊髓**

傳統機器人 pipeline 像是分開的眼睛（感知）、大腦（規劃）、手（控制），彼此只通過窄介面溝通。VLA/LLM-based 架構把眼睛和大腦統一成「能看能聽有常識的高階認知中樞」— 但它想事情很慢（1-5 Hz）。底層仍需要「脊髓」（MPC/RL，100+ Hz）負責快速反應。

你的大腦說「把杯子拿起來」，但手碰到杯子時的力量調節是脊髓反射在做，不是大腦。LLM 是大腦，MPC 是脊髓。

**World Model = 做白日夢**：DreamerV3 在 latent space 裡模擬「如果我往左推，物體會滑到哪裡？」。不需要真正推 — 在腦中預演多種方案，選最好的執行。就像棋手在腦中走幾步棋，預判對手反應。

**模擬器觀察**：在 Isaac Sim 或 SAPIEN 裡：
1. **純 LLM 規劃**：GPT-4V + 桌面影像 + 「把紅色方塊放碗裡」→ 合理 task plan，但無法直接變成馬達指令
2. **加 grounding**：每步 LLM 動作 + affordance model 檢查可行性 → 過濾不可行選項
3. **分層控制**：VLM 1 Hz 輸出 target pose → MPC 100 Hz 追蹤 → 平滑抓取
4. **直接用 LLM 控制**：讓 LLM 輸出關節角度 → 延遲 >500ms，動作斷續，碰擾動無法即時反應

## 實作連結

**四個典型工程場景**：

1. **LLM Task Planner + Skill Library**：GPT-4 把「清理桌面」拆成 detect_objects → pick(cup) → place(cup, shelf) → pick(plate) → ...，每個子任務呼叫預訓練好的 skill。SayCan、Code as Policies 架構。

2. **VLA 端到端操作**：Octo / OpenVLA fine-tune — 輸入 RGB + 語言指令 → 直接輸出末端 delta pose。需要 200-1000 個 demo 做 fine-tune。適合桌面操作（抓取、推動、擺放）。

3. **VLM + MPC 分層架構**：VLM（GPT-4V / Gemini）1 Hz 看影像 + 語言 → 輸出 target pose；MPC 100+ Hz 追蹤 + 力矩約束 + 碰撞迴避。適合需要語義理解 + 即時反應的場景（家用機器人、倉儲操作）。

4. **邊緣部署（Edge Deployment）**：大 VLA 模型放雲端，只下發 target pose / action chunk → 機器人端用輕量 MPC 追蹤。或者用量化 + 蒸餾得到的小模型跑在機載 GPU（Jetson Orin）。雲邊協同：雲端做語義規劃（1 Hz），邊端做即時控制（100+ Hz）。

**Code 骨架**（Python，分層 VLM + MPC）：

```python
class VLMPlanner:
    """高階語義規劃 — 1-5 Hz"""
    def __init__(self, model_name: str = "gpt-4o"):
        self.client = create_vlm_client(model_name)

    def get_target(self, image, instruction: str) -> dict:
        """影像 + 語言 → 目標位姿或子任務"""
        # prompt = format_prompt(image, instruction)
        # response = self.client.generate(prompt)
        # return parse_target_pose(response)
        pass

class LowLevelController:
    """底層即時控制 — 100+ Hz"""
    def __init__(self, robot_interface):
        self.robot = robot_interface
        self.mpc = create_mpc_controller()

    def track_target(self, target_pose, duration: float = 2.0):
        """MPC 追蹤 VLM 的目標位姿"""
        # while not reached(target_pose):
        #     state = self.robot.get_state()
        #     action = self.mpc.solve(state, target_pose)
        #     self.robot.apply(action)
        pass

class HierarchicalAgent:
    """分層代理：VLM 想、MPC 做"""
    def __init__(self, planner, controller):
        self.planner = planner
        self.controller = controller

    def execute(self, instruction: str):
        while not task_complete:
            image = self.get_observation()
            target = self.planner.get_target(image, instruction)
            if self.safety_check(target):  # 安全過濾層
                self.controller.track_target(target)
            else:
                self.replan(instruction)  # fallback
```

## 常見誤解

1. **「LLM 可以直接輸出馬達控制指令」** — LLM 推理延遲 200-2000ms，控制迴路需要 1-10ms。即使延遲不是問題，LLM 從文字訓練的「物理直覺」也不足以做精確力矩計算。**正確理解**：LLM/VLM 做高階語義規劃（1-5 Hz），底層必須有 MPC/RL/PID 做即時控制（100+ Hz）。分層是必然的，不是可選的。

2. **「Foundation Model 不會幻覺，可以直接信任規劃結果」** — LLM 會自信地輸出物理上不可行的計劃（「把球放進比球小的杯子」）。物理幻覺比語言幻覺更危險 — 語言幻覺只是講錯話，物理幻覺會撞壞機器人或傷人。**正確理解**：LLM 輸出和真機執行之間**必須有安全過濾層** — affordance checking + 碰撞檢測 + workspace boundary + CBF 硬約束。

3. **「不需要安全兜底，模型夠大就行」** — 即使是 55B 參數的 RT-2，在 unseen 場景仍有 25% 失敗率。模型不確定性高的時候，如果沒有 safety monitor，機器人會繼續執行高風險動作。**正確理解**：模型越大越好，但安全保證不能靠模型本身 — CBF、碰撞檢測、不確定性閾值是必須的工程層。

4. **「大模型萬能，PID 已經過時」** — 在精確力控（裝配、拋光）、高頻追蹤（CNC 加工）、確定性保證（醫療手術）場景，PID/MPC 比任何 learned policy 都更可靠和可驗證。**正確理解**：大模型解決語義理解和泛化問題，傳統控制解決精確、可靠、可驗證問題。兩者互補，不是替代。

5. **「VLA 是 end-to-end，不需要系統工程」** — VLA 統一了感知和決策，但你仍需要：相機標定和影像前處理、action space 設計（delta pose vs joint angle）、safety constraints（力矩限制、碰撞邊界）、failure recovery（不確定性高時 fallback）、部署基礎設施（模型服務、延遲優化、硬體驅動）。**正確理解**：End-to-end 簡化了部分 pipeline，但沒有消除系統工程。基礎設施工作量可能比模型本身更大。

## 練習題

<details>
<summary>Q1：服務機器人收到「把桌上的髒碗都收到洗碗機裡」，但桌上碗盤杯混在一起。怎麼設計從語言理解到執行的架構？</summary>

**完整推理鏈**：

1. **語義理解層**：VLM（GPT-4V）看桌面影像 + 指令 → 辨識「髒碗」（和盤子、杯子區分）。VLM 的語義知識能理解「碗 = 深的圓形容器」
2. **Task Planning**：LLM 拆解 → detect_dirty_bowls → for each bowl: pick(bowl) → open_dishwasher → place(bowl, rack) → close_dishwasher
3. **Grounding**：每個 pick 前，affordance model 確認抓取位姿可行、手臂到得了
4. **Low-level 控制**：MPC 追蹤目標位姿，力控模式拿碗（碗是易碎的，需要 impedance control）
5. **Closed-loop**：每次 pick 完重新拍照，確認碗已拿起，處理遮擋和物體位移

**關鍵設計決策**：VLM 做語義（什麼是碗）+ LLM 做規劃（先拿哪個）+ MPC 做控制（怎麼穩穩拿起來）。不要試圖用一個 VLA 端到端做完整個長序列。

</details>

<details>
<summary>Q2：VLA 推理延遲 800ms，動作很卡但準確度不錯。怎麼在不損失太多精度的前提下降低延遲？</summary>

**完整推理鏈**：

1. **Action Chunking（最簡單有效）**：一次推理預測未來 4-8 步 → 有效推理頻率從 1.25 Hz 提升到 5-10 Hz。連續動作有相關性，預測品質損失可控
2. **模型量化**：INT8 量化 → 推理速度 2-3x 提升，精度損失通常 < 2%
3. **分層 fallback**：VLA 以 1-2 Hz 輸出 target waypoint → 中間用插值 + impedance control 做 100 Hz 平滑追蹤
4. **模型蒸餾**：把 7B VLA 蒸餾成 300M 小模型，在特定任務上保留 90%+ 精度
5. **避開陷阱**：不要降輸入影像解析度太多 — 抓取需要精確空間資訊，影像太糊精度大幅下降

**結論**：Action chunking 是最簡單有效的。長期方案是分層架構（VLA 高階 + lightweight controller 底層）。

</details>

<details>
<summary>Q3：LLM 做 task planner，偶爾生成不可行計劃（「右手同時拿兩個碗」）。怎麼系統性解決？</summary>

**完整推理鏈**：

1. **根因**：LLM 沒有 grounding — 不知道機器人的物理能力和環境狀態
2. **Affordance-based filtering（SayCan）**：
   - LLM 提出所有語義合理的候選動作
   - Affordance model 評估每個動作在當前狀態的成功率
   - $\text{score}(a) = P_{\text{LLM}}(a \mid l) \times P_{\text{affordance}}(a \mid s)$
   - 選 score 最高的執行
3. **PDDL/行為樹約束**：用形式化 precondition/effect 定義每個 skill → LLM 計劃後用 PDDL planner 驗證邏輯可行性
4. **Closed-loop replanning**：每步執行後重新觀測、重新規劃。短 horizon 多次規劃 > 長 horizon 一次性規劃
5. **避開陷阱**：不要只靠 prompt engineering（「你只有一隻手」）— 複雜場景會失效。Grounding 必須靠工程系統保證

**結論**：Affordance filtering + closed-loop replanning 是工業級解法。LLM 是「候選生成器」而非「最終決策者」。

</details>

<details>
<summary>Q4：機器人需要處理從未見過的物體。Foundation Model 的零樣本泛化能幫到什麼程度？什麼時候不夠用？</summary>

**完整推理鏈**：

1. **Foundation Model 能做的**：
   - 語義辨識：VLM 能認出「這是螺絲起子」即使訓練數據沒這把特定的（網路影像知識遷移）
   - 粗略抓取：預訓練含足夠多樣物體時，能泛化到類似形狀
   - 功能推理：LLM 知道「螺絲起子轉螺絲」→ 能規劃合理使用步驟
2. **Foundation Model 不夠的**：
   - 精確抓取位姿：新物體幾何可能偏離訓練分佈太遠
   - 力控參數：不同材質/重量需不同力控設定，zero-shot 無法知道
   - 工具精確操作：知道「螺絲起子轉螺絲」和「精確對準螺絲頭施力」是兩回事
3. **互補策略**：zero-shot 做粗略規劃和抓取 → few-shot fine-tune（3-5 demo）精修 → 力控交給底層 impedance controller
4. **避開陷阱**：不要高估 zero-shot — 安全關鍵場景必須有 fallback 和 human-in-the-loop

**結論**：Foundation Model 提供強大語義 prior，精確操作仍需 few-shot adaptation + 傳統控制。zero-shot 是起點不是終點。

</details>

## 面試角度

1. **分層架構是必然的** — 最重要的架構洞察。帶出：「LLM/VLM 推理延遲百毫秒級，控制需要毫秒級。頻率差 100 倍以上，必須分層 — VLM 1-5 Hz 做語義規劃，MPC/RL 100+ Hz 做即時控制。這不是暫時的工程妥協，是認知和反射在計算架構上的必然分離。就像人的大腦想『拿杯子』，但手碰到杯子時的力量調節是脊髓反射在做。」

2. **Grounding 是 LLM 做機器人規劃的核心問題** — 區分懂理論和懂工程。帶出：「LLM 從文字學的物理知識是統計相關性，不是因果理解。它會自信地生成不可行計劃。解法是 affordance-based filtering — LLM 負責語義（想得到），affordance model 負責物理可行性（做得到），兩者相乘排序。SayCan 就是這個範式的代表。」

3. **Action Tokenization 的設計取捨** — 展現工程細節理解。帶出：「VLA 把連續動作離散化為 token，和語言 token 共享 vocabulary，用同一個 Transformer 自迴歸預測。RT-2 用 256 bins 每維 — bins 少精度差，bins 多 vocabulary 爆炸。7-DoF 動作 = 7 個 token。這個離散化是 VLA 能用語言模型架構做動作預測的關鍵 trick。」

4. **World Model 是 Sim-to-Real 的新路徑** — 展現前沿視野。帶出：「DreamerV3 在 latent space 學環境動力學，能在『想像』中模擬行動後果。這比傳統 sim 更靈活 — 不需要精確的物理引擎參數，直接從數據學動力學。未來 world model + VLA 的結合可能讓機器人在部署前做大量虛擬預演。」

5. **安全兜底不是可選的** — 展現工程成熟度。帶出：「任何 LLM/VLA 輸出到真機前，必須經過安全過濾 — 碰撞檢測、力矩限制、workspace boundary、CBF 硬約束、不確定性閾值。物理世界的幻覺不是講錯話，是撞壞東西或傷人。我會在 VLM 和 controller 之間插 safety monitor，不確定性高時 fallback 到安全狀態或請求人類介入。」

## 延伸閱讀

- **Brohan et al.,《RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control》(2023)** — VLA 里程碑，證明 VLM fine-tune 成 action predictor 可行，55B 模型 zero-shot 泛化驚人
- **Ahn et al.,《Do As I Can, Not As I Say: Grounding Language in Robotic Affordances (SayCan)》(2022)** — LLM + affordance grounding 經典架構，解決 LLM 規劃不可行的核心問題
- **Octo (UC Berkeley, 2024)** — 開源 VLA 模型，支援 fine-tune，學 VLA 工程的最佳起手式
- **OpenVLA (Stanford + UC Berkeley + TRI + MIT + Google DeepMind, 2024)** — Kim et al. 領銜的多機構協作，基於 Llama 2 7B 的開源 VLA，展示用開源 LLM 做 VLA 的可行性
- **Hafner et al.,《DreamerV3》(2023)** — World model SOTA，latent space model-based RL，sim 和 real 上均有展示
- **Liang et al.,《Code as Policies》(2023)** — LLM 生成 Python 程式碼作為機器人策略，靈活但需嚴格 sandbox
- **Open X-Embodiment Collaboration (2024)** — 22 機構聯合數據集，證明 cross-embodiment transfer 可行，VLA scaling 基礎
- **Driess et al.,《PaLM-E: An Embodied Multimodal Language Model》(2023)** — 562B 參數的 embodied LLM，展示了 scale 對 embodied reasoning 的影響
