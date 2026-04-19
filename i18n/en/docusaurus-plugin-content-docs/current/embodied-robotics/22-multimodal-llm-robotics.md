---
title: "Multimodal LLMs for Embodied Perception and Decision"
prerequisites: ["20-imitation-learning-dagger", "21-sim-to-real-transfer"]
estimated_time: 45
difficulty: 4
tags: ["vla", "world-model", "foundation-model", "embodied-ai", "llm"]
sidebar_position: 22
---

# Multimodal LLMs for Embodied Perception and Decision

## You Will Learn

- Articulate the fundamental difference between VLA (Vision-Language-Action) models and traditional sense-plan-control pipelines: VLA unifies perception and decision in a single end-to-end model rather than three separate modules connected in series
- When a robot must understand ambiguous natural language instructions and execute manipulation tasks, know that LLMs handle high-level semantic planning while the low level still requires MPC/RL for real-time control — and why LLMs cannot directly output joint torques
- Judge when to leverage Foundation Model zero-shot generalization (novel objects, new instructions) and when fine-tuning or traditional control is still needed

## Core Concepts

**Precise Definitions**:

- **VLA (Vision-Language-Action) Model**: receives visual observations (RGB/depth) and natural language instructions, directly outputs robot actions (end-effector pose, joint angles, or action tokens) end-to-end. Representative architectures: RT-2, Octo, OpenVLA. Essentially extends a VLM's (Vision-Language Model) output vocabulary from "text tokens" to "action tokens"
- **World Model**: a predictive model that learns environment dynamics — given the current state and an action, it predicts the future observation sequence. Enables model-based planning (simulate consequences in imagination before acting). Representatives: DreamerV3, UniSim, Genie
- **Foundation Model for Robotics**: a large model pre-trained on massive multi-task data, capable of zero-shot or few-shot generalization to new tasks, objects, and environments. Core value: bringing web-scale semantic knowledge ("cups are for drinking") into the robot's decision loop

**Three architectural patterns**:

1. **LLM as Task Planner**: the LLM performs high-level semantic planning (decompose "make me coffee" into a sub-task sequence), with existing pick-and-place / navigation skills handling execution. SayCan, Code as Policies
2. **VLA End-to-End**: a single model maps images + language → actions directly. Eliminates intermediate representations but requires massive robot manipulation data. RT-2, Octo
3. **VLM + Low-level Controller**: a VLM produces high-level targets (target pose, waypoints) at 1-5 Hz, while MPC/RL runs real-time tracking control at 100+ Hz. Hierarchical architecture balancing semantic understanding and real-time response

**Location in the Sense → Plan → Control Loop**:
- **Input**: RGB/depth images, natural language instructions, (optional) force/tactile sensor data
- **Output**: depends on architecture — task plan (sub-task sequence), target pose, or direct action tokens
- **Downstream**: low-level controller (MPC / PID / RL policy) executes actions → sensor feedback → next observation frame
- **Loop node**: sits at the **high-level cognitive center** of the loop, unifying perception and decision, but almost never directly touches low-level control (too slow)

**Minimum Sufficient Math / API**:

1. **VLA action tokenization**:

$$
a_t = \text{Detokenize}\left( \arg\max_{a \in \mathcal{V}_a} P_\theta(a \mid I_t, l, a_{<t}) \right)
$$

**Physical meaning**: continuous robot actions (7-DoF end-effector pose) are discretized into tokens sharing the same vocabulary as language tokens. The model's output is "the next token," except this token is decoded as an action rather than text. RT-2 uses 256 bins to discretize each dimension.

2. **Frequency separation in hierarchical architecture**:

$$
\text{High-level (VLM)}: \quad g_t = \pi_{\text{VLM}}(I_t, l) \quad \text{@ 1-5 Hz}
$$

$$
\text{Low-level (MPC)}: \quad u_t = \pi_{\text{MPC}}(x_t, g_t) \quad \text{@ 100-1000 Hz}
$$

**Physical meaning**: the VLM thinks slowly but understands semantics ("place the cup to the right of the plate" → target pose $g_t$); MPC reacts fast but does not understand semantics (tracks $g_t$, handles torque constraints and collision avoidance). The frequency gap exceeds 100x, making hierarchical separation essential.

3. **World Model prediction loss**:

$$
\mathcal{L}_{\text{world}} = \mathbb{E}\left[ \sum_{t=1}^{T} \| \hat{o}_{t} - o_t \|^2 + \beta \cdot D_{\text{KL}}(q(z_t \mid o_{\le t}) \| p(z_t \mid z_{t-1}, a_{t-1})) \right]
$$

**Physical meaning**: the first term is observation prediction reconstruction error; the second is KL regularization on latent dynamics. The world model learns environment dynamics in latent space, enabling it to simulate action consequences "in imagination" without real execution. DreamerV3 uses this framework for model-based RL.

<details>
<summary>Deep dive: VLA training data and scaling laws — why data is the biggest bottleneck</summary>

### Robot data vs language/vision data scale gap

| Domain | Typical training data | Source |
|--------|----------------------|--------|
| LLM (GPT-4) | ~13T tokens | Web text |
| VLM (LLaVA) | ~1.2M image-text pairs | Web images + annotations |
| VLA (RT-2) | ~130K robot episodes | Real-robot operation (Google internal) |
| VLA (Open X-Embodiment) | ~1M episodes | 22 robots, multi-institution |

The gap is 3-4 orders of magnitude. This is why VLAs must fine-tune from pre-trained VLMs rather than training from scratch.

### Scaling trends

Google RT-2 experiments show:
- Model scaling from 5B → 55B parameters: zero-shot novel object generalization improves ~3x
- Data scaling from 10K → 130K episodes: success rate rises from 40% → 75%
- But data acquisition speed (real-robot operation) is linear, unlike text which can be crawled from the web

### Solutions for data efficiency

1. **Sim-to-Real + DR**: generate massive manipulation data in simulation (Ch21), use DR for diversity
2. **Video Pre-training**: use YouTube manipulation videos for video prediction pre-training, then fine-tune for action prediction
3. **Cross-Embodiment Transfer**: Open X-Embodiment demonstrates that data from different robots helps each other (shared semantic understanding)
4. **Human-in-the-Loop**: DAgger (Ch20) + VR teleoperation for efficient high-quality demo collection

</details>

<details>
<summary>Deep dive: the LLM Grounding problem — why LLM physics reasoning often fails</summary>

### What is the Grounding Problem

LLM "physical knowledge" learned from text is statistical correlation, not genuine physical understanding:
- An LLM knows "cups hold water" but does not know the fluid dynamics of why water spills when a cup is inverted
- An LLM can answer "objects dropped from height accelerate" but cannot accurately predict position after 2.5 seconds

### Typical failure cases

1. **Spatial reasoning errors**: when asked "A is left of B, B is left of C, which side of C is A?", LLM accuracy is far below 100%
2. **Missing physical intuition**: LLMs cannot reliably judge "can this object be stacked on that one?" (requires center-of-mass, friction, contact geometry reasoning)
3. **Hallucination**: LLMs confidently output plans that "sound reasonable but are physically infeasible"

### Engineering countermeasures

1. **VLM over pure LLM**: adding visual input lets the model "see" the actual scene, reducing hallucination
2. **Grounding function**: the core idea of SayCan — LLM proposes candidate actions, an affordance model judges "is this action feasible in the current state?", scores are multiplied for ranking

$$
\pi(a \mid l, s) = P_{\text{LLM}}(a \mid l) \cdot P_{\text{affordance}}(a \mid s)
$$

3. **Safety filter**: insert a physical feasibility check between LLM output and execution (collision detection, torque limits, workspace boundary)
4. **Closed-loop replanning**: never trust a one-shot LLM plan — re-observe and re-plan after each step

</details>

## Intuition

**Analogy: a brain that can see, hear, and has common sense vs a fast-reflex spinal cord**. Traditional robot pipelines are like separate eyes (perception module), brain (planner), and hands (controller) communicating through narrow interfaces. VLA/LLM-based architectures unify the eyes and brain into a "high-level cognitive center that can see, hear, and has common sense," but it "thinks slowly" (1-5 Hz). The low level still needs a "spinal cord" (MPC/RL, 100+ Hz) for rapid response — your brain says "pick up the cup," but the force modulation when your hand contacts the cup is a spinal reflex, not a conscious brain decision.

**Simulator observation**: build a hierarchical architecture experiment in Isaac Sim or SAPIEN:

1. **Pure LLM planning**: give GPT-4V a tabletop image + "place the red block in the bowl" → it produces a reasonable task plan, but cannot directly become motor commands
2. **Add grounding**: each LLM-proposed action (pick red_block) is checked by an affordance model: "can the arm reach red_block?" → infeasible options filtered out
3. **Hierarchical control**: VLM at 1 Hz outputs target pose → MPC at 100 Hz tracks → observe smooth grasping motion
4. **Direct LLM control**: let the LLM attempt to output joint angles → latency too high (>500ms), jerky motion, cannot react to disturbances in real time

## Implementation Link

**Three representative engineering scenarios**:

1. **LLM Task Planner + Skill Library**: use GPT-4 to decompose natural language instructions ("clean the table") into sub-task sequences (detect_objects → pick(cup) → place(cup, shelf) → pick(plate) → ...), each sub-task calling a pre-trained skill. SayCan, Code as Policies architecture.

2. **VLA end-to-end manipulation**: fine-tune Octo or OpenVLA — input RGB + language instruction, directly output end-effector delta pose. Suited for tabletop manipulation (grasping, pushing, placing); requires hundreds to thousands of demos for fine-tuning.

3. **VLM + MPC hierarchical architecture**: a VLM (GPT-4V / Gemini) at 1 Hz processes images + understands language → outputs target pose; MPC at 100+ Hz tracks the target pose + handles torque constraints + collision avoidance. Suited for scenarios requiring both semantic understanding and real-time response (home service robots, warehouse operations).

**Code skeleton** (Python, hierarchical VLM + MPC architecture):

```python
# Hierarchical architecture: VLM high-level planning + MPC low-level control
import time

class VLMPlanner:
    """High-level semantic planning — 1-5 Hz"""
    def __init__(self, model_name: str = "gpt-4o"):
        self.client = create_vlm_client(model_name)

    def get_target(self, image, language_instruction: str) -> dict:
        """Input image + language → output target pose or sub-task"""
        # prompt = format_prompt(image, language_instruction)
        # response = self.client.generate(prompt)
        # return parse_target_pose(response)
        pass

class LowLevelController:
    """Low-level real-time control — 100+ Hz"""
    def __init__(self, robot_interface):
        self.robot = robot_interface
        self.mpc = create_mpc_controller()

    def track_target(self, target_pose, duration: float = 2.0):
        """MPC tracks the target pose from VLM"""
        # while not reached(target_pose):
        #     state = self.robot.get_state()
        #     action = self.mpc.solve(state, target_pose)
        #     self.robot.apply(action)
        pass

class HierarchicalAgent:
    """Hierarchical agent: VLM thinks, MPC acts"""
    def __init__(self):
        self.planner = VLMPlanner()
        self.controller = LowLevelController(robot)

    def execute(self, instruction: str):
        while not task_complete:
            image = self.get_observation()
            target = self.planner.get_target(image, instruction)
            # Safety check: target within workspace? No collision?
            if self.safety_check(target):
                self.controller.track_target(target)
            else:
                self.replan(instruction)
```

<details>
<summary>Deep dive: complete VLA fine-tuning pipeline (Python + Octo / OpenVLA)</summary>

```python
"""
VLA Fine-tuning Pipeline — Octo example
1. Prepare demo data (collected via teleoperation)
2. Load pre-trained VLA
3. Fine-tune to your robot + task
4. Deploy for inference
"""
import numpy as np

# ============ Step 1: Data format ============
# Octo uses RLDS (Robot Learning Data Sets) format
# Each episode contains:
#   - observation: {'image_primary': (T, H, W, 3), 'state': (T, state_dim)}
#   - action: (T, action_dim)  # typically 7-DoF delta pose + gripper
#   - language_instruction: str

def create_dataset_from_demos(demo_dir: str):
    """Convert teleoperation demos to RLDS format"""
    episodes = []
    for demo_path in sorted(glob(f"{demo_dir}/*.hdf5")):
        with h5py.File(demo_path) as f:
            episode = {
                'observation': {
                    'image_primary': f['images'][()],  # (T, 256, 256, 3)
                    'state': f['joint_positions'][()],  # (T, 7)
                },
                'action': f['actions'][()],             # (T, 7) delta EE pose
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
    Key: freeze vision encoder, only tune action head + partial transformer
    """
    from octo.model.octo_model import OctoModel

    model = OctoModel.load_pretrained(pretrained_path)

    # Key configuration
    config = {
        'optimizer': {'lr': learning_rate, 'warmup_steps': 1000},
        'frozen_keys': ['vision_encoder'],  # freeze visual backbone
        'action_head': {'type': 'diffusion', 'num_steps': 20},
    }

    # Training loop
    for step in range(num_steps):
        batch = sample_batch(dataset, batch_size)
        loss = model.train_step(batch, config)
        if step % 1000 == 0:
            print(f"Step {step}: loss = {loss:.4f}")

    model.save(f"octo_finetuned_{num_steps}")
    return model

# ============ Step 3: Deployment inference ============
class OctoInference:
    """Deploy fine-tuned Octo to real hardware"""
    def __init__(self, model_path: str):
        self.model = OctoModel.load_pretrained(model_path)
        self.action_buffer = []  # action chunking buffer

    def predict_action(
        self, image: np.ndarray, instruction: str, state: np.ndarray
    ) -> np.ndarray:
        """
        Predict next action.
        Action chunking: predict multiple steps at once, execute
        sequentially for smoothness
        """
        if len(self.action_buffer) == 0:
            # Predict action chunk (e.g., next 4 steps)
            obs = {'image_primary': image, 'state': state}
            actions = self.model.sample_actions(
                obs, instruction, num_steps=4
            )
            self.action_buffer = list(actions)

        return self.action_buffer.pop(0)
```

**Key fine-tuning decisions**:
1. **Freeze vision encoder**: the pre-trained visual backbone is already strong; full tuning risks overfitting to a small dataset
2. **Action chunking**: predicting multiple steps and executing sequentially is smoother than independent per-step prediction
3. **Data volume**: tabletop manipulation tasks typically need 200-1000 demos; complex long-horizon tasks need more
4. **Diffusion action head**: better at expressing multimodal action distributions than a regression head (the same observation may admit multiple valid actions)

</details>

## Common Misconceptions

1. **"LLMs can directly output motor control commands"** — LLM inference latency is 200-2000ms, while robot control loops require 1-10ms. Even if latency were not an issue, the "physical intuition" LLMs learn from text is insufficient for precise torque computation. **Correct understanding**: LLMs/VLMs handle high-level semantic planning (1-5 Hz); the low level must use MPC/RL/PID for real-time control (100+ Hz). Hierarchical architecture is inevitable.

2. **"Foundation Models do not hallucinate and their plans can be trusted directly"** — LLMs confidently output physically infeasible plans ("place the ball inside a cup smaller than the ball"). Physics hallucination is more dangerous than language hallucination — language mistakes produce wrong words, physics mistakes crash robots or injure people. **Correct understanding**: a **safety filter layer is mandatory** between LLM output and execution — affordance checking + collision detection + workspace boundary checks.

3. **"VLA is end-to-end, so no systems engineering is needed"** — although VLA unifies perception and decision, you still need: (a) camera calibration and image preprocessing, (b) action space design (delta pose vs joint angles), (c) safety constraints (torque limits, collision boundaries), (d) failure recovery (fallback to a safe state when model prediction uncertainty is high). End-to-end models simplify parts of the pipeline but do not eliminate systems engineering.

## Situational Questions

<details>
<summary>Q1: Your service robot receives the instruction "put all the dirty bowls from the table into the dishwasher," but the table has bowls, plates, and cups mixed together. How do you design the architecture from language understanding to execution?</summary>

**Complete reasoning chain**:

1. **Semantic understanding layer**: use a VLM (GPT-4V) to process the tabletop image + instruction → identify "dirty bowls" (distinguish from plates and cups). The VLM's semantic knowledge understands "bowl = deep, round container"
2. **Task planning**: LLM decomposes → detect_dirty_bowls → for each bowl: pick(bowl) → open_dishwasher → place(bowl, rack) → close_dishwasher
3. **Grounding**: before each pick action, the affordance model confirms "is this bowl's grasp pose feasible? Can the arm reach it?"
4. **Low-level control**: each pick/place uses MPC tracking the target pose in force-control mode (bowls are fragile)
5. **Closed-loop**: re-capture image after each pick, confirm the bowl has been grasped, handle occlusion and object displacement

**Key design decision**: hierarchical architecture — VLM for semantics (what is a bowl) + LLM for planning (which bowl first) + MPC for control (how to pick it up steadily). Do not try to have a single VLA handle the entire long-horizon task end-to-end.

</details>

<details>
<summary>Q2: You deployed a VLA model for tabletop grasping. Inference latency is 800ms, making robot motion jerky. But model accuracy is good. How do you reduce latency without losing too much precision?</summary>

**Complete reasoning chain**:

1. **Action Chunking**: predict the next 4-8 steps in a single inference, execute sequentially. Effective frequency goes from 1/0.8s to ~5 Hz, and since consecutive actions are correlated, prediction quality does not degrade much
2. **Model quantization**: INT8 quantization can boost inference speed 2-3x with typically < 2% accuracy loss
3. **Hierarchical fallback**: VLA outputs target waypoints at 1-2 Hz; smooth 100 Hz tracking between waypoints via simple interpolation + impedance control
4. **Model distillation**: distill the large VLA (7B) into a smaller model (300M), retaining 90%+ accuracy on the specific task
5. **Trap to avoid**: do not reduce input image resolution too aggressively to cut latency — grasping needs precise spatial information, and blurry images significantly degrade grasp accuracy

**Conclusion**: Action chunking is the simplest and most effective approach. Long-term solution is a hierarchical architecture (VLA for high-level + lightweight controller for low-level).

</details>

<details>
<summary>Q3: Your manager wants you to use an LLM as the robot's task planner. You test GPT-4 with several instructions and find it occasionally generates infeasible plans (e.g., "simultaneously pick up two bowls with the right hand"). How do you systematically fix this?</summary>

**Complete reasoning chain**:

1. **Root cause**: the LLM lacks grounding — it does not know the robot's physical capabilities or current environment state
2. **Affordance-based filtering (SayCan approach)**:
   - LLM proposes a candidate action list (all semantically reasonable options)
   - Affordance model (a success predictor for each skill) evaluates "in the current state, what is this action's success probability?"
   - $\text{score}(a) = P_{\text{LLM}}(a \mid \text{instruction}) \times P_{\text{affordance}}(a \mid \text{state})$
   - Execute the highest-scoring action
3. **PDDL / behavior tree constraints**: define preconditions/effects formally for each skill. After LLM generates a plan, use a PDDL planner to verify logical feasibility
4. **Closed-loop replanning**: re-observe and re-plan after each execution step. One-shot plans accumulate errors
5. **Trap to avoid**: do not rely solely on prompt engineering for grounding — "you are a robot with one hand" works for simple cases but fails in complex scenarios

**Conclusion**: affordance filtering + closed-loop replanning is the industrial-grade solution. Treat the LLM as a "candidate generator," not the "final decision maker."

</details>

<details>
<summary>Q4: Your robot needs to handle objects never seen during training (e.g., oddly-shaped tools). How far does Foundation Model zero-shot generalization go, and when is it not enough?</summary>

**Complete reasoning chain**:

1. **What Foundation Models can do**:
   - Semantic understanding: VLM can identify "this is a screwdriver" even if this specific screwdriver was never in training data (knowledge transfer from web images)
   - Grasp strategy: if pre-training data includes sufficiently diverse objects, VLA can generalize to similarly shaped novel objects
   - Functional reasoning: LLM knows "screwdrivers are for turning screws" and can plan reasonable usage steps
2. **Where Foundation Models fall short**:
   - Precise grasp pose: novel object geometry may be far from the training distribution, leading to inaccurate grasp poses
   - Force control parameters: different materials/weights require different force settings; zero-shot cannot know these
   - Precise tool use: knowing "screwdriver turns screws" versus "precisely align with the screw head and apply torque" are different levels
3. **Complementary strategy**: zero-shot for coarse planning and grasping → few-shot fine-tune (3-5 demos) for refinement → force control delegated to a low-level impedance controller
4. **Trap to avoid**: do not overestimate zero-shot capability — in safety-critical scenarios, fallbacks and human-in-the-loop are mandatory

**Conclusion**: Foundation Models provide powerful semantic priors, but precise manipulation still requires few-shot adaptation + traditional control.

</details>

## Interview Angles

1. **Hierarchical architecture with semantic planning + low-level control is inevitable** — the most important architectural insight. Bring out with: "LLM/VLM inference latency is hundreds of milliseconds; robot control requires millisecond-level response. The frequency gap exceeds 100x. So hierarchy is essential — VLM at 1-5 Hz for high-level semantic planning, MPC/RL at 100+ Hz for real-time control. This is not a temporary engineering compromise but the necessary computational separation of cognition and reflex."

2. **Grounding is the core challenge of LLM-based robot planning** — separates theory from engineering. Bring out with: "LLM physical knowledge from text is statistical correlation, not genuine physical understanding. It will confidently generate infeasible plans. The solution is affordance-based filtering — LLM handles semantics, affordance model handles physical feasibility, scores are multiplied for ranking. SayCan is the canonical example of this paradigm."

3. **The VLA data bottleneck matters more than model architecture** — demonstrates deep domain understanding. Bring out with: "VLA model architectures are mature (RT-2, Octo), but the real bottleneck is robot manipulation data. The web has 13T tokens of text, but all robot manipulation data worldwide might not reach 1M episodes. Open X-Embodiment is a data consortium attempt; sim-to-real and teleoperation are the two paths for scaling data."

4. **The safety filter layer is not optional** — demonstrates engineering maturity. Bring out with: "Between any LLM/VLA output and real-robot execution, there must be safety filtering — collision detection, torque limits, workspace boundaries, uncertainty thresholds. Physics hallucination is not saying the wrong word — it is crashing equipment or injuring people. I insert a safety monitor between VLM and controller, falling back to a safe state or requesting human intervention when uncertainty is high."

## Further Reading

- **Brohan et al., "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control" (2023)** — the VLA milestone paper, proving that fine-tuning a VLM into an action predictor is viable; the 55B model's zero-shot generalization is remarkable
- **Ahn et al., "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances (SayCan)" (2022)** — the canonical LLM + affordance grounding architecture, solving the core problem of infeasible LLM plans
- **Octo (UC Berkeley, 2024)** — open-source VLA model supporting fine-tuning to your robot; the best starting point for learning VLA engineering
- **OpenVLA (Stanford, 2024)** — Llama-based open-source VLA, 7B parameters, demonstrating the feasibility of using open-source LLMs for VLA
- **Hafner et al., "DreamerV3" (2023)** — state-of-the-art world model, model-based RL in latent space, demonstrating world model potential in both sim and real
- **Liang et al., "Code as Policies" (2023)** — using LLMs to generate Python code as robot policies, eliminating skill pre-definition; highly flexible but requires strict sandboxing
- **Open X-Embodiment Collaboration (2024)** — 22 institutions, multiple robots, joint dataset proving cross-embodiment transfer is viable; laying the data foundation for VLA scaling
