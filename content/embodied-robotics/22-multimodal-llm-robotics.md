---
title: "多模態大模型與具身感知決策"
prerequisites: ["20-imitation-learning-dagger", "21-sim-to-real-transfer"]
estimated_time: 45
difficulty: 4
tags: ["vla", "world-model", "foundation-model", "embodied-ai", "llm"]
sidebar_position: 22
---

# 多模態大模型與具身感知決策

## 你將學到

- 能精確講出 VLA（Vision-Language-Action）模型和傳統感知-規劃-控制 pipeline 的本質差異：VLA 把感知和決策統一在一個 end-to-end 模型裡，而不是三個獨立模組串接
- 遇到「機器人要理解模糊自然語言指令並執行操作」時，知道該用 LLM 做高階語義規劃、底層仍需 MPC/RL 做即時控制，以及為什麼不能讓 LLM 直接輸出關節力矩
- 判斷什麼場景用 Foundation Model 的零樣本泛化能力（新物體、新指令），什麼場景仍需要 fine-tune 或傳統控制

## 核心概念

**精確定義**：

- **VLA (Vision-Language-Action) Model**：接收視覺觀測（RGB/depth）和自然語言指令，直接輸出機器人動作（末端位姿、關節角度或 action token）的 end-to-end 模型。代表架構：RT-2、Octo、OpenVLA。本質是把 VLM（Vision-Language Model）的輸出從「文字 token」擴展到「動作 token」
- **World Model**：學習環境動力學的預測模型 — 給定當前狀態和動作，預測未來的觀測序列。可用於 model-based planning（在想像中模擬後果再行動）。代表：DreamerV3、UniSim、Genie
- **Foundation Model for Robotics**：在大規模多任務數據上預訓練的通用模型，具備零樣本或少樣本泛化到新任務/新物體/新環境的能力。核心價值是把網路規模的語義知識（「杯子是拿來喝水的」）帶進機器人的決策迴路

**三種架構模式**：

1. **LLM as Task Planner**：LLM 做高階語義規劃（把「幫我泡咖啡」拆成子任務序列），底層用現有的 pick-and-place / navigation 技能執行。SayCan、Code as Policies
2. **VLA End-to-End**：一個模型直接從影像 + 語言 → 動作。省去中間表徵，但需要大量機器人操作數據。RT-2、Octo
3. **VLM + Low-level Controller**：VLM 以 1-5 Hz 產生高階目標（目標位姿、waypoint），MPC/RL 以 100+ Hz 做即時追蹤控制。分層架構，兼顧語義理解和即時反應

**在感知 → 規劃 → 控制閉環的位置**：
- **輸入**：RGB/depth 影像、自然語言指令、（可選）力/觸覺感測器數據
- **輸出**：取決於架構 — task plan（子任務序列）、target pose（目標位姿）、或直接的 action token
- **下游**：底層控制器（MPC / PID / RL policy）執行動作 → 感測器回饋 → 下一幀觀測
- **閉環節點**：坐落在閉環的**高階認知中樞**，統一了感知和決策，但幾乎不直接觸碰底層控制（因為太慢）

**最少夠用的數學 / API**：

1. **VLA 的 action tokenization**：

$$
a_t = \text{Detokenize}\left( \arg\max_{a \in \mathcal{V}_a} P_\theta(a \mid I_t, l, a_{<t}) \right)
$$

**物理意義**：把連續的機器人動作（7-DoF 末端位姿）離散化為 token，和語言 token 共享同一個 vocabulary。模型的輸出就是「下一個 token」，只是這個 token 被解碼為動作而非文字。RT-2 用 256 bins 離散化每個維度。

2. **分層架構的頻率分離**：

$$
\text{High-level (VLM)}: \quad g_t = \pi_{\text{VLM}}(I_t, l) \quad \text{@ 1-5 Hz}
$$

$$
\text{Low-level (MPC)}: \quad u_t = \pi_{\text{MPC}}(x_t, g_t) \quad \text{@ 100-1000 Hz}
$$

**物理意義**：VLM 思考慢但理解語義（「把杯子放到盤子右邊」→ 目標位姿 $g_t$），MPC 反應快但不懂語義（追蹤 $g_t$、處理力矩約束和碰撞迴避）。兩者頻率差 100 倍以上，必須分層。

3. **World Model 的預測損失**：

$$
\mathcal{L}_{\text{world}} = \mathbb{E}\left[ \sum_{t=1}^{T} \| \hat{o}_{t} - o_t \|^2 + \beta \cdot D_{\text{KL}}(q(z_t \mid o_{\le t}) \| p(z_t \mid z_{t-1}, a_{t-1})) \right]
$$

**物理意義**：第一項是觀測預測的重建誤差，第二項是 latent dynamics 的 KL 正則化。World model 在 latent space 裡學習環境動力學，能在「想像」中模擬行動後果，不需要真正執行。DreamerV3 用這個框架在 sim 中做 model-based RL。

<details>
<summary>深入：VLA 訓練數據與 scaling law — 為什麼數據是最大瓶頸</summary>

### 機器人數據 vs 語言/視覺數據的規模差距

| 領域 | 典型訓練數據量 | 數據來源 |
|------|-------------|---------|
| LLM (GPT-4) | ~13T tokens | 網路文本 |
| VLM (LLaVA) | ~1.2M 影像-文字對 | 網路影像 + 標註 |
| VLA (RT-2) | ~130K 機器人 episodes | 真機操作（Google 內部） |
| VLA (Open X-Embodiment) | ~1M episodes | 22 個機器人、多機構聯合 |

差距是 3-4 個數量級。這就是為什麼 VLA 必須從預訓練 VLM 做 fine-tune，而不能從頭訓練。

### Scaling 趨勢

Google RT-2 的實驗顯示：
- 模型從 5B → 55B 參數：zero-shot 新物體泛化提升 ~3x
- 數據從 10K → 130K episodes：成功率從 40% → 75%
- 但數據獲取速度（真機操作）是線性的，不像文本可以從網路爬

### 數據效率的解法

1. **Sim-to-Real + DR**：在 sim 裡大量生成操作數據（Ch21），用 DR 增加多樣性
2. **Video Pre-training**：用 YouTube 操作影片做 video prediction pre-training，再 fine-tune 到 action prediction
3. **Cross-Embodiment Transfer**：Open X-Embodiment 證明不同機器人的數據可以互相幫助（共享語義理解）
4. **Human-in-the-Loop**：DAgger (Ch20) + VR teleoperation 高效收集高品質 demo

</details>

<details>
<summary>深入：LLM Grounding 問題 — 為什麼 LLM 的物理推理常出錯</summary>

### 什麼是 Grounding Problem

LLM 從文字學到的「物理知識」是統計相關性，不是真正的物理理解：
- LLM 知道「杯子裝水」，但不知道杯子翻轉時水會灑出來的力學原因
- LLM 能回答「物體從高處落下會加速」，但不能準確預測 2.5 秒後的位置

### 典型失敗案例

1. **空間推理錯誤**：LLM 被問「A 在 B 左邊，B 在 C 左邊，A 在 C 的哪邊？」正確率遠低於 100%
2. **物理直覺缺失**：LLM 不能可靠判斷「這個物體能不能疊在另一個上面」（需要質心、摩擦、接觸幾何）
3. **幻覺問題**：LLM 會自信地輸出一個「聽起來合理但物理上不可行」的計劃

### 工程上的應對

1. **VLM 替代純 LLM**：加入視覺輸入讓模型能「看到」實際場景，減少幻覺
2. **Grounding function**：SayCan 的核心思路 — LLM 提出候選動作，affordance model 判斷「這個動作在當前狀態下可行嗎？」，兩者相乘做排序

$$
\pi(a \mid l, s) = P_{\text{LLM}}(a \mid l) \cdot P_{\text{affordance}}(a \mid s)
$$

3. **Safety filter**：在 LLM 輸出和執行之間加一層物理可行性檢查（碰撞檢測、力矩限制、workspace boundary）
4. **Closed-loop replanning**：不要把 LLM 的一次性計劃當真 — 每步執行後重新觀測、重新規劃

</details>

## 直覺理解

**類比：能看能聽有常識的大腦 vs 快速反射的脊髓**。傳統機器人 pipeline 像是分開的眼睛（感知模組）、大腦（規劃器）、手（控制器），彼此只通過窄介面溝通。VLA/LLM-based 架構像是把眼睛和大腦統一成一個「能看能聽還有常識的高階認知中樞」，但它「想事情很慢」（1-5 Hz）。底層仍需要「脊髓」（MPC/RL，100+ Hz）負責快速反應 — 你的大腦說「把杯子拿起來」，但手碰到桙子時的力量調節是脊髓反射在做，不是大腦。

**模擬器觀察**：在 Isaac Sim 或 SAPIEN 裡搭建一個分層架構的實驗：

1. **純 LLM 規劃**：給 GPT-4V 一張桌面影像 + 「把紅色方塊放到碗裡」→ 它能輸出合理的 task plan，但沒有辦法直接變成馬達指令
2. **加 grounding**：每一步 LLM 提出的動作（pick red_block）配合 affordance model 檢查「手臂能不能到達 red_block？」→ 過濾掉不可行的選項
3. **分層控制**：VLM 1 Hz 輸出 target pose → MPC 100 Hz 追蹤 → 觀察到平滑的抓取動作
4. **直接用 LLM 輸出控制**：讓 LLM 嘗試輸出關節角度 → 延遲太大（>500ms），動作斷斷續續，碰到擾動無法即時反應

## 實作連結

**三個典型工程場景**：

1. **LLM Task Planner + Skill Library**：用 GPT-4 把自然語言指令（「清理桌面」）拆解成子任務序列（detect_objects → pick(cup) → place(cup, shelf) → pick(plate) → ...），每個子任務呼叫預先訓練好的 skill。SayCan、Code as Policies 架構。

2. **VLA 端到端操作**：用 Octo 或 OpenVLA 做 fine-tune — 輸入 RGB + 語言指令，直接輸出末端 delta pose。適合桌面操作（抓取、推動、擺放），需要幾百到幾千個 demo 做 fine-tune。

3. **VLM + MPC 分層架構**：VLM（GPT-4V / Gemini）以 1 Hz 看影像 + 理解語言 → 輸出 target pose；MPC 以 100+ Hz 追蹤 target pose + 處理力矩約束 + 碰撞迴避。適合需要語義理解又需要即時反應的場景（家用服務機器人、倉儲操作）。

**Code 骨架**（Python，分層 VLM + MPC 架構）：

```python
# 分層架構：VLM 高階規劃 + MPC 底層控制
import time

class VLMPlanner:
    """高階語義規劃 — 1-5 Hz"""
    def __init__(self, model_name: str = "gpt-4o"):
        self.client = create_vlm_client(model_name)

    def get_target(self, image, language_instruction: str) -> dict:
        """輸入影像 + 語言 → 輸出目標位姿或子任務"""
        # prompt = format_prompt(image, language_instruction)
        # response = self.client.generate(prompt)
        # return parse_target_pose(response)
        pass

class LowLevelController:
    """底層即時控制 — 100+ Hz"""
    def __init__(self, robot_interface):
        self.robot = robot_interface
        self.mpc = create_mpc_controller()

    def track_target(self, target_pose, duration: float = 2.0):
        """MPC 追蹤 VLM 輸出的目標位姿"""
        # while not reached(target_pose):
        #     state = self.robot.get_state()
        #     action = self.mpc.solve(state, target_pose)
        #     self.robot.apply(action)
        pass

class HierarchicalAgent:
    """分層代理：VLM 想、MPC 做"""
    def __init__(self):
        self.planner = VLMPlanner()
        self.controller = LowLevelController(robot)

    def execute(self, instruction: str):
        while not task_complete:
            image = self.get_observation()
            target = self.planner.get_target(image, instruction)
            # 安全檢查：target 在 workspace 內？不碰撞？
            if self.safety_check(target):
                self.controller.track_target(target)
            else:
                self.replan(instruction)
```

<details>
<summary>深入：完整 VLA Fine-tuning Pipeline（Python + Octo / OpenVLA）</summary>

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
# 每個 episode 包含:
#   - observation: {'image_primary': (T, H, W, 3), 'state': (T, state_dim)}
#   - action: (T, action_dim)  # 通常是 7-DoF delta pose + gripper
#   - language_instruction: str

def create_dataset_from_demos(demo_dir: str):
    """把 teleoperation demo 轉成 RLDS 格式"""
    episodes = []
    for demo_path in sorted(glob(f"{demo_dir}/*.hdf5")):
        with h5py.File(demo_path) as f:
            episode = {
                'observation': {
                    'image_primary': f['images'][()],  # (T, 256, 256, 3)
                    'state': f['joint_positions'][()],   # (T, 7)
                },
                'action': f['actions'][()],          # (T, 7) delta EE pose
                'language_instruction': f.attrs['instruction'],
            }
            episodes.append(episode)
    return episodes

# ============ Step 2: Fine-tune ============
def finetune_octo(
    pretrained_path: str,
    dataset,
    num_steps: int = 50_000,
    batch_size: int = 128,
    learning_rate: float = 3e-5,
):
    """
    Fine-tune Octo to your robot + task.
    關鍵：凍結 vision encoder，只 tune action head + 部分 transformer
    """
    from octo.model.octo_model import OctoModel

    model = OctoModel.load_pretrained(pretrained_path)

    # 關鍵配置
    config = {
        'optimizer': {'lr': learning_rate, 'warmup_steps': 1000},
        'frozen_keys': ['vision_encoder'],  # 凍結視覺骨幹
        'action_head': {'type': 'diffusion', 'num_steps': 20},
    }

    # 訓練循環
    for step in range(num_steps):
        batch = sample_batch(dataset, batch_size)
        loss = model.train_step(batch, config)
        if step % 1000 == 0:
            print(f"Step {step}: loss = {loss:.4f}")

    model.save(f"octo_finetuned_{num_steps}")
    return model

# ============ Step 3: 部署推理 ============
class OctoInference:
    """部署 fine-tuned Octo 到真機"""
    def __init__(self, model_path: str):
        self.model = OctoModel.load_pretrained(model_path)
        self.action_buffer = []  # action chunking buffer

    def predict_action(
        self, image: np.ndarray, instruction: str, state: np.ndarray
    ) -> np.ndarray:
        """
        預測下一步動作.
        action chunking: 一次預測多步，逐步執行，提升平滑度
        """
        if len(self.action_buffer) == 0:
            # 預測 action chunk (e.g., 未來 4 步)
            obs = {'image_primary': image, 'state': state}
            actions = self.model.sample_actions(
                obs, instruction, num_steps=4
            )
            self.action_buffer = list(actions)

        return self.action_buffer.pop(0)
```

**Fine-tune 的關鍵決策**：
1. **凍結 vision encoder**：預訓練的視覺骨幹已經很好了，全 tune 容易 overfit 小數據集
2. **Action chunking**：一次預測多步並逐步執行，比每步獨立預測更平滑
3. **數據量**：桌面操作任務通常需要 200-1000 個 demo；複雜長序列任務需要更多
4. **Diffusion action head**：比 regression head 更能表達多模態動作分佈（同一個觀測可能有多種合理動作）

</details>

## 常見誤解

1. **「LLM 可以直接輸出馬達控制指令」** — LLM 的推理延遲是 200-2000ms，而機器人控制迴路需要 1-10ms。即使延遲不是問題，LLM 從文字訓練來的「物理直覺」也不足以做精確的力矩計算。**正確理解**：LLM/VLM 做高階語義規劃（1-5 Hz），底層必須有 MPC/RL/PID 做即時控制（100+ Hz）。分層架構是必然的。

2. **「Foundation Model 不會幻覺，可以直接信任其規劃結果」** — LLM 會自信地輸出物理上不可行的計劃（「把球放進比球小的杯子裡」）。物理推理的幻覺比語言幻覺更危險 — 語言幻覺只是講錯話，物理幻覺會撞壞機器人或傷人。**正確理解**：LLM 輸出和執行之間**必須有安全過濾層** — affordance checking + 碰撞檢測 + workspace boundary check。

3. **「VLA 是 end-to-end，不需要系統工程」** — VLA 雖然把感知和決策統一了，但你仍需要：(a) 相機標定和影像前處理、(b) action space 設計（delta pose vs joint angle）、(c) safety constraints（力矩限制、碰撞邊界）、(d) failure recovery（模型預測不確定性高時 fallback 到安全狀態）。End-to-end 模型簡化了部分 pipeline，但沒有消除系統工程。

## 練習題

<details>
<summary>Q1：你的服務機器人收到指令「把桌上的髒碗都收到洗碗機裡」，但桌上有碗、盤子、杯子混在一起。你怎麼設計從語言理解到執行的架構？</summary>

**分析推理**：

1. **語義理解層**：用 VLM（GPT-4V）看桌面影像 + 指令 → 辨識「髒碗」（和盤子、杯子區分）。VLM 的語義知識能理解「碗 = 深的圓形容器」
2. **Task Planning**：LLM 拆解 → detect_dirty_bowls → for each bowl: pick(bowl) → open_dishwasher → place(bowl, rack) → close_dishwasher
3. **Grounding**：每個 pick 動作前，affordance model 確認「這個碗的抓取位姿可行嗎？手臂到得了嗎？」
4. **Low-level 控制**：每個 pick/place 用 MPC 追蹤目標位姿，力控模式拿碗（碗是易碎的）
5. **Closed-loop**：每次 pick 完重新拍照，確認碗已被拿起，處理遮擋和物體位移

**關鍵設計決策**：分層架構 — VLM 做語義（什麼是碗）+ LLM 做規劃（先拿哪個碗）+ MPC 做控制（怎麼穩穩拿起來）。不要試圖用一個 VLA 從頭到尾做完整個長序列任務。

</details>

<details>
<summary>Q2：你部署了一個 VLA 模型做桌面抓取，發現推理延遲 800ms，機器人動作很卡。但模型準確度不錯。怎麼在不損失太多精度的前提下降低延遲？</summary>

**分析推理**：

1. **Action Chunking**：一次推理預測未來 4-8 步動作，按順序執行。推理頻率從 1/0.8s 降到 1/(0.8*4)s = 有效 5 Hz，同時因為連續動作有相關性，預測品質不會差太多
2. **模型量化**：INT8 量化可以把推理速度提升 2-3x，精度損失通常 < 2%
3. **分層 fallback**：VLA 以 1-2 Hz 輸出 target waypoint，中間用簡單的插值 + impedance control 做 100 Hz 的平滑追蹤
4. **模型蒸餾**：把大 VLA（7B）蒸餾成小模型（300M），在特定任務上保留 90%+ 精度
5. **避開陷阱**：不要為了降延遲就降輸入影像解析度太多 — 抓取需要精確的空間資訊，影像太糊抓取精度會大幅下降

**結論**：Action chunking 是最簡單有效的方法。長期方案是分層架構（VLA 做高階 + lightweight controller 做底層）。

</details>

<details>
<summary>Q3：老闆要你用 LLM 做機器人的 task planner。你用 GPT-4 測試了幾個指令，發現它偶爾會生成不可行的計劃（例如「用右手同時拿兩個碗」）。怎麼系統性地解決？</summary>

**分析推理**：

1. **根因**：LLM 沒有 grounding — 它不知道機器人的物理能力和當前環境狀態
2. **Affordance-based filtering（SayCan 方法）**：
   - LLM 提出候選動作列表（語義上合理的所有選項）
   - Affordance model（每個 skill 的 success predictor）評估「在當前狀態下，這個動作的成功率」
   - $\text{score}(a) = P_{\text{LLM}}(a \mid \text{instruction}) \times P_{\text{affordance}}(a \mid \text{state})$
   - 選 score 最高的動作執行
3. **PDDL/行為樹約束**：用形式化的 precondition/effect 定義每個 skill。LLM 生成計劃後，用 PDDL planner 驗證計劃的邏輯可行性
4. **Closed-loop replanning**：每步執行後重新觀測、重新生成計劃。一次性的計劃容易累積誤差
5. **避開陷阱**：不要只靠 prompt engineering 解決 grounding — 「你是一個機器人，只有一隻手」這種 prompt 在複雜場景會失效

**結論**：Affordance filtering + closed-loop replanning 是工業級解法。把 LLM 當「候選生成器」而非「最終決策者」。

</details>

<details>
<summary>Q4：你的機器人需要處理從未見過的物體（例如訓練時沒有的奇形怪狀工具）。Foundation Model 的零樣本泛化能幫到什麼程度？什麼時候不夠用？</summary>

**分析推理**：

1. **Foundation Model 能做的**：
   - 語義理解：VLM 能辨識「這是一把螺絲起子」即使訓練數據裡沒有這把特定的螺絲起子（從網路影像的知識遷移）
   - 抓取策略：如果預訓練數據包含足夠多樣的物體，VLA 能泛化到類似形狀的新物體
   - 功能推理：LLM 知道「螺絲起子是用來轉螺絲的」，能規劃合理的使用步驟
2. **Foundation Model 不夠的**：
   - 精確的抓取位姿：新物體的幾何可能和訓練分佈差很遠，抓取位姿可能不準
   - 力控參數：不同材質/重量的物體需要不同的力控設定，zero-shot 無法知道
   - 工具使用的精確操作：知道「螺絲起子轉螺絲」和「精確地對準螺絲頭施力」是兩回事
3. **互補策略**：zero-shot 做粗略規劃和抓取 → few-shot fine-tune（3-5 個 demo）精修 → 力控交給底層 impedance controller
4. **避開陷阱**：不要高估 zero-shot 能力 — 在安全關鍵場景，必須有 fallback 和 human-in-the-loop

**結論**：Foundation Model 提供強大的語義 prior，但精確操作仍需 few-shot adaptation + 傳統控制。

</details>

## 面試角度

1. **語義規劃 + 底層控制的分層架構是必然的** — 最重要的架構洞察。面試時帶出：「LLM/VLM 的推理延遲是百毫秒級，機器人控制需要毫秒級反應。兩者頻率差 100 倍以上。所以必須分層 — VLM 以 1-5 Hz 做高階語義規劃，MPC/RL 以 100+ Hz 做即時控制。這不是暫時的工程妥協，而是認知和反射在計算架構上的必然分離。」

2. **Grounding 是 LLM 做機器人規劃的核心問題** — 區分懂理論和懂工程。面試時帶出：「LLM 從文字學的物理知識是統計相關性，不是真正的物理理解。它會自信地生成不可行的計劃。解法是 affordance-based filtering — LLM 負責語義，affordance model 負責物理可行性，兩者相乘排序。SayCan 就是這個範式的代表。」

3. **VLA 的數據瓶頸比模型架構更關鍵** — 展現對領域的深刻理解。面試時帶出：「VLA 模型架構已經很成熟（RT-2、Octo），但真正的瓶頸是機器人操作數據。網路有 13T tokens 的文字，但全世界的機器人操作數據加起來可能不到 1M episodes。Open X-Embodiment 是數據聯盟的嘗試，sim-to-real + teleoperation 是擴充數據的兩條路。」

4. **安全過濾層不是可選的** — 展現工程成熟度。面試時帶出：「任何 LLM/VLA 輸出到真機執行之前，必須經過安全過濾 — 碰撞檢測、力矩限制、workspace boundary、不確定性閾值。物理世界的幻覺不是講錯話，是撞壞東西或傷人。我會在 VLM 和 controller 之間插一層 safety monitor，不確定性高時 fallback 到安全狀態或請求人類介入。」

## 延伸閱讀

- **Brohan et al., "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control" (2023)** — VLA 的里程碑論文，證明把 VLM fine-tune 成 action predictor 是可行的，55B 模型的 zero-shot 泛化能力驚人
- **Ahn et al., "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances (SayCan)" (2022)** — LLM + affordance grounding 的經典架構，解決了 LLM 規劃不可行的核心問題
- **Octo (UC Berkeley, 2024)** — 開源的 VLA 模型，支援 fine-tune 到你的機器人，是學 VLA 工程的最佳起手式
- **OpenVLA (Stanford, 2024)** — 基於 Llama 的開源 VLA，7B 參數，展示了用開源 LLM 做 VLA 的可行性
- **Hafner et al., "DreamerV3" (2023)** — World model 的 SOTA，在 latent space 做 model-based RL，展示了 world model 在 sim 和 real 上的潛力
- **Liang et al., "Code as Policies" (2023)** — 用 LLM 生成 Python 程式碼作為機器人策略，省去 skill 預定義，靈活性極高但需要嚴格的 sandbox
- **Open X-Embodiment Collaboration (2024)** — 22 個機構、多種機器人的聯合數據集，證明 cross-embodiment transfer 是可行的，為 VLA 的 scaling 打下基礎
