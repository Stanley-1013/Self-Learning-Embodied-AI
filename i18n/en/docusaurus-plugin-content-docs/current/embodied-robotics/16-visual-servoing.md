---
title: "Visual Servoing and Feature Extraction"
prerequisites: ["07-forward-kinematics-dh", "15-model-predictive-control"]
estimated_time: 45
difficulty: 4
tags: ["visual-servoing", "ibvs", "pbvs", "feature-extraction"]
sidebar_position: 16
---

# Visual Servoing and Feature Extraction

## You Will Learn

- Articulate the fundamental difference between IBVS and PBVS: IBVS controls directly in pixel space, PBVS reconstructs 3D first then controls in Cartesian space — and when each one fails
- When faced with "camera sees the target, robot arm must align precisely," know to first decide eye-in-hand vs eye-to-hand, whether 2D features suffice, and whether the interaction matrix might go singular
- Decide when to use pure IBVS, when PBVS, and when to switch to 2.5D hybrid visual servoing

## Core Concepts

**Precise Definition**: **Visual servoing (VS)** uses visual feedback to directly drive a robot control loop — the error between image features (pixel points, lines, moments) and their desired values is mapped through an **interaction matrix** to camera (or end-effector) velocity commands. It is the **shortest reflex arc from perception to control**, bypassing full mapping and trajectory planning.

**Two paradigms**:
- **IBVS (Image-Based Visual Servoing)**: defines error directly in 2D pixel space $e = s - s^*$ ($s$ is the current feature vector, $s^*$ the target). The interaction matrix converts this to camera velocity. No 3D reconstruction needed, but the Cartesian-space trajectory may be counterintuitive.
- **PBVS (Position-Based Visual Servoing)**: first estimates the target's 3D pose $^cT_o$ from the image, then defines error in Cartesian space $e = (t, \theta u)$ and computes velocity commands. Cartesian trajectory is intuitive and straight, but accuracy depends entirely on the quality of 3D reconstruction (calibration errors propagate directly).

**Interaction Matrix (Image Jacobian)** — the core mathematical tool:

Maps 2D feature velocity $\dot{s}$ to 6-DoF camera velocity $v_c = (v, \omega)$:

$$
\dot{s} = L_s \cdot v_c
$$

**Physical meaning**: $L_s$ tells you "if the camera moves in this direction → the pixel features shift in that direction." Invert the relationship and you know: "to move the features from here to the target, the camera should move like this."

**Eye-in-Hand vs Eye-to-Hand**:
- **Eye-in-Hand**: camera mounted on the end-effector — end-effector moves, camera moves with it. The interaction matrix directly gives end-effector velocity. First choice for precision alignment (PCB insertion, welding).
- **Eye-to-Hand**: camera fixed externally — observes the entire workspace, requires an additional hand-eye transform. Global field of view, suited for grasp planning.

**Location in the Sense → Plan → Control Loop**:
- **Input**: camera image → feature extraction $s$ (from perception), target features $s^*$ (from task definition)
- **Output**: camera/end-effector velocity command $v_c$ (sent to motion controller / converted to joint velocity via robot Jacobian)
- **Downstream**: $v_c$ is converted to $\dot{q}$ through the robot Jacobian $J$, sent to joint servos
- **Loop node**: directly **short-circuits perception and control** — no full mapping/planning in between, the tightest possible visual feedback loop

**Minimum Sufficient Math**:

1. **Single-point Interaction Matrix** (one pixel point $(u, v)$ produces 2 rows $\times$ 6 columns):

$$
L_s = \begin{bmatrix}
-\frac{f}{Z} & 0 & \frac{u}{Z} & \frac{uv}{f} & -(f + \frac{u^2}{f}) & v \\
0 & -\frac{f}{Z} & \frac{v}{Z} & (f + \frac{v^2}{f}) & -\frac{uv}{f} & -u
\end{bmatrix}
$$

**Physical meaning**: $f$ is the focal length, $Z$ is the feature point depth. The first 3 columns correspond to translational velocity (closer objects, smaller $Z$, faster pixel motion); the last 3 to rotational velocity (more off-center the pixel, larger rotational effect). **Unknown $Z$ is the central difficulty of IBVS** — typically resolved with a depth camera or a fixed estimate.

2. **IBVS control law** (the classic proportional form):

$$
v_c = -\lambda \hat{L}_s^+ (s - s^*)
$$

**Physical meaning**: $\hat{L}_s^+$ is the pseudoinverse of the interaction matrix, $(s - s^*)$ is the feature error vector. $\lambda > 0$ is the gain. This "back-projects" pixel error into camera velocity space — intuitively, "however far the features are from the target, move the camera proportionally to close the gap."

3. **PBVS control law**:

$$
v_c = -\lambda \begin{pmatrix} ^ct - ^{c^*}t \\ \theta u \end{pmatrix}
$$

**Physical meaning**: proportional control directly in 3D space — translation error plus rotation error (axis-angle representation). The Cartesian trajectory is a straight line, but accuracy is entirely bound by the quality of $^cT_o$ estimation.

<details>
<summary>Deep dive: full derivation of the Interaction Matrix and singularity analysis</summary>

### From the pinhole model to the Interaction Matrix

Assuming a pinhole camera (no distortion):

$$
u = f \frac{X}{Z}, \quad v = f \frac{Y}{Z}
$$

Differentiating both sides with respect to time using the chain rule:

$$
\dot{u} = f \frac{\dot{X}Z - X\dot{Z}}{Z^2}
$$

Decomposing the 3D point's velocity in the camera frame into camera translational velocity $v = (v_x, v_y, v_z)$ and rotational velocity $\omega = (\omega_x, \omega_y, \omega_z)$:

$$
\dot{P} = -v - \omega \times P
$$

Expanding and rearranging yields the $L_s$ matrix shown above.

### Singularity analysis

The rank of $L_s$ determines system controllability. Common singular configurations:

1. **All feature points coplanar and parallel to the image plane**: all $Z$ values identical, $L_s$ loses depth-direction information, $v_z$ becomes uncontrollable
2. **Feature points collinear**: $L_s$ rank < 6, some degrees of freedom uncontrollable
3. **Too few feature points**: $n$ points produce a $2n \times 6$ $L_s$; at least 4 non-coplanar points are needed for full rank (6)

### Stacking $n$ feature points

$$
L_s = \begin{bmatrix} L_{s_1} \\ L_{s_2} \\ \vdots \\ L_{s_n} \end{bmatrix} \in \mathbb{R}^{2n \times 6}
$$

Solved via pseudoinverse $L_s^+ = (L_s^T L_s)^{-1} L_s^T$ for the overdetermined system. More features improve stability (redundancy) but increase compute — 4-8 points is the practical sweet spot.

### The IBVS trajectory problem

IBVS interpolates linearly in pixel space, but the corresponding Cartesian trajectory can be severely nonlinear. Classic example: when the target requires 180-degree rotation, IBVS produces a retreat-then-advance arc, potentially failing if $Z$ estimation is wrong.

Solution: **2.5D Visual Servoing** — control translation in pixel space (IBVS) and rotation in Cartesian space (PBVS), combining the advantages of both.

</details>

**Common APIs / Toolchain**:

| Layer | Tool | Example interface |
|-------|------|-------------------|
| Feature extraction | OpenCV | `cv2.goodFeaturesToTrack()`, `cv2.ORB_create()` |
| Visual servoing framework | ViSP | `vpServo.setServo(vpServo::EYEINHAND_L_cVe_eJe)` |
| Depth estimation | RealSense / ZED | `rs2::depth_frame.get_distance(u, v)` |
| ROS 2 integration | visp_ros | `vpROSGrabber` + `vpServo` |
| End-to-end VS | Research | CNN maps image → $v_c$ directly (bypasses feature extraction) |

## Intuition

**Analogy: threading a needle**. IBVS is like threading a needle — you watch the needle eye and the thread tip in your field of vision, directly adjusting your fingers to make them align. You do not need to know how many centimeters away the needle is (3D coordinates); you just need the two things to line up in your visual field. PBVS is like first measuring the needle's 3D position with a ruler, planning a straight-line path to it — more "rational" but requires precise measurement.

**Visual metaphor: two ways to park a car**. IBVS = watching the rearview camera image, adjusting the steering based on the car outline relative to the parking lines. PBVS = using ultrasonic sensors to compute the precise 3D distance to the parking spot, planning the shortest path. The former needs no precise ranging but may curve; the latter gives a straight path but any ranging error skews the result.

**Simulator observation**: in Gazebo + ViSP, set up an eye-in-hand scene (camera on UR5 end-effector, viewing an AprilTag):
- IBVS mode: set the four AprilTag corner pixel coordinates as targets — pixel-space convergence is fast but the Cartesian trajectory curves
- PBVS mode: estimate the AprilTag's 3D pose, control in Cartesian space — trajectory is a straight line, but if camera calibration is off by 2 degrees, the final position drifts several millimeters
- Drop camera frame rate from 60 fps to 15 fps: observe delay-induced oscillations (especially when IBVS gain $\lambda$ is high)

## Implementation Link

**Three representative engineering scenarios**:

1. **PCB hole alignment (IBVS)**: camera on end-effector (eye-in-hand) views fiducial holes on a PCB. Hole center pixel coordinates serve as features; the interaction matrix maps to end-effector velocity. Accuracy < 0.1 mm. 2D planar scene + rich features → IBVS is the optimal choice.

2. **Bin picking (PBVS)**: depth camera (eye-to-hand) views scattered parts, point-cloud registration estimates the target's 6-DoF pose $^cT_o$, PBVS controls in Cartesian space. Requires precise hand-eye calibration $^bT_c$.

3. **High-speed moving target tracking (2.5D hybrid)**: tracking objects on a conveyor belt requires both speed (pixel-level feedback) and sane trajectories (Cartesian constraints). 2.5D visual servoing: IBVS for translation, PBVS for rotation.

**Code skeleton** (Python, ViSP-style IBVS):

```python
import visp  # visp-python bindings
import numpy as np

# Initialize servo
servo = visp.vpServo()
servo.setServo(visp.vpServo.EYEINHAND_L_cVe_eJe)
servo.setLambda(0.5)  # proportional gain

# Define 4 point features
for i in range(4):
    s = visp.vpFeaturePoint()      # current feature
    sd = visp.vpFeaturePoint()     # target feature
    s.buildFrom(u_current[i], v_current[i], Z_est[i])  # Z must be estimated
    sd.buildFrom(u_target[i], v_target[i], Z_target[i])
    servo.addFeature(s, sd)

# Control loop
# v_c = servo.computeControlLaw()  # returns (vx,vy,vz,wx,wy,wz)
# dq = J_inv @ v_c                 # convert to joint velocity
# robot.setVelocity(dq)
```

<details>
<summary>Deep dive: complete IBVS + ROS 2 implementation example (Python)</summary>

```python
#!/usr/bin/env python3
"""
IBVS eye-in-hand: use AprilTag corners as features,
control UR5 end-effector to align with target.
Dependencies: visp, opencv, ros2, apriltag
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2

class IBVSNode(Node):
    def __init__(self):
        super().__init__('ibvs_controller')
        self.bridge = CvBridge()
        self.lambda_gain = 0.5
        self.Z_est = 0.3  # estimated depth (m); update in real time with depth camera

        # Camera intrinsics (from CameraInfo)
        self.fx = 600.0
        self.fy = 600.0
        self.cx = 320.0
        self.cy = 240.0

        # Target features (AprilTag corners at desired pose, in pixels)
        self.s_star = np.array([
            [280, 200], [360, 200],
            [360, 280], [280, 280]
        ], dtype=np.float64)

        # ROS 2 interfaces
        self.vel_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.img_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)

    def compute_interaction_matrix(self, points, Z):
        """Compute stacked interaction matrix L_s (2n x 6) for n points."""
        L = []
        for u, v in points:
            # Convert to normalized coordinates
            x = (u - self.cx) / self.fx
            y = (v - self.cy) / self.fy
            L.append([
                -1/Z,  0,    x/Z,  x*y,    -(1+x*x), y,
                0,     -1/Z, y/Z,  (1+y*y), -x*y,    -x,
            ])
        return np.array(L).reshape(-1, 6)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Detect AprilTag (simplified; use apriltag library in practice)
        corners = self.detect_apriltag(frame)
        if corners is None or len(corners) != 4:
            return

        s = corners.astype(np.float64)  # current features (4x2)

        # Feature error
        error = (s - self.s_star).flatten()  # (8,)

        # Interaction matrix (8x6)
        Ls = self.compute_interaction_matrix(s, self.Z_est)

        # Pseudoinverse
        Ls_pinv = np.linalg.pinv(Ls)  # (6x8)

        # IBVS control law: v_c = -lambda * Ls+ * error
        v_c = -self.lambda_gain * Ls_pinv @ error

        # Publish velocity command
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = float(v_c[0])
        twist.twist.linear.y = float(v_c[1])
        twist.twist.linear.z = float(v_c[2])
        twist.twist.angular.x = float(v_c[3])
        twist.twist.angular.y = float(v_c[4])
        twist.twist.angular.z = float(v_c[5])
        self.vel_pub.publish(twist)

    def detect_apriltag(self, frame):
        # Simplified: use pupil_apriltags or apriltag_ros in practice
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # detector = apriltag.Detector()
        # results = detector.detect(gray)
        # return results[0].corners if results else None
        return None  # placeholder

def main():
    rclpy.init()
    node = IBVSNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

**Key implementation details**:
- Using a fixed `Z_est` still allows IBVS to converge, but velocity scale will be imprecise; real-time depth updates are better
- Gain $\lambda$ too high causes oscillations due to camera latency; start at 0.3 and tune with 30+ fps
- In deployment, using `moveit_servo`'s `delta_twist_cmds` interface is most convenient — it includes built-in collision checking and joint limit protection

</details>

<details>
<summary>Deep dive: how hand-eye calibration errors propagate to control accuracy</summary>

### Eye-in-Hand calibration

Camera mounted on end-effector: need the camera-to-flange transform $^eT_c$ (for eye-to-hand: need $^bT_c$).

Calibration equation (Tsai-Lenz 1989):

$$
A_i X = X B_i, \quad i = 1, \dots, n
$$

where $A_i = {^{e_1}T_{e_i}}$ (end-effector motion), $B_i = {^{c_1}T_{c_i}}$ (camera observation), $X = {^eT_c}$ (unknown). Requires at least 2 distinct robot motions ($n \ge 2$); in practice, 10-20 poses yield a good least-squares solution.

### Error propagation analysis

Rotation error $\Delta R$ and translation error $\Delta t$ in hand-eye calibration propagate directly to control results:

- **PBVS is most affected**: target pose $^bT_o = {^bT_c} \cdot {^cT_o}$; error in $^bT_c$ → $^bT_o$ shifts directly → control accuracy is upper-bounded by calibration accuracy
- **IBVS is less affected**: the control law does not directly use $^eT_c$, but $Z$ estimation and Jacobian conversion still depend on it. 1-degree rotation calibration error → ~1-2 mm IBVS error (at 30 cm working distance)
- **2.5D hybrid**: translation via IBVS (less calibration-sensitive), rotation via PBVS (straight trajectory) → best overall trade-off

### Practical steps to reduce calibration error

1. **Multi-pose calibration**: at least 15-20 diverse calibration poses
2. **Nonlinear refinement**: Tsai-Lenz for initialization, then Levenberg-Marquardt minimizing reprojection error
3. **Online self-calibration**: continuously update $^eT_c$ during robot operation to compensate for mechanical wear
4. **IBVS bypass**: if the task allows, pure IBVS is least dependent on calibration accuracy

</details>

## Common Misconceptions

1. **"PBVS is always better than IBVS because 3D control is more precise"** — PBVS accuracy is entirely limited by 3D pose estimation quality. 1-degree camera calibration error or 5% depth estimation error can push PBVS end-effector error to the centimeter level. IBVS does not require precise 3D reconstruction and is more accurate in scenes with clear 2D features (PCB, planar alignment). **Correct understanding**: IBVS suits planar / close-range / high-precision scenes; PBVS suits large workspace / straight-line trajectory / reliable depth scenarios.

2. **"IBVS needs no calibration at all"** — IBVS does not need precise 3D reconstruction, but it still requires: (a) camera intrinsics (focal length $f$, principal point) — otherwise the interaction matrix is wrong; (b) depth estimate $Z$ — otherwise velocity mapping scale is completely off; (c) robot Jacobian $J$ — otherwise $v_c$ cannot be converted to $\dot{q}$. **Correct understanding**: IBVS has low *sensitivity* to calibration, not zero *dependence*.

3. **"More features means more stability"** — more features do increase redundancy (occlusion robustness), but also: (a) increase interaction matrix computation ($2n \times 6$); (b) low-quality features (blurry, mismatched) degrade accuracy; (c) poor spatial distribution (collinear, clustered) pushes $L_s$ toward singularity. **Correct understanding**: 4-8 high-quality, spatially distributed features >> 50 densely packed low-quality features.

4. **"Visual servoing is only for slow precision alignment"** — high-speed visual servoing is an active research area. Event cameras (microsecond latency) combined with high-rate processing have achieved 500+ Hz visual servoing. Traditional cameras at 60 fps with EKF prediction can support 100+ Hz control. **Key insight**: the bottleneck is image processing latency and camera frame rate, not the control law itself.

## Situational Questions

<details>
<summary>Q1 (medium): Automated PCB insertion task. Camera on end-effector views alignment holes on the PCB. Required precision: < 0.05 mm. Do you choose IBVS or PBVS? How do you design it?</summary>

**Complete reasoning chain**:

1. **Choose IBVS**: PCB is a planar scene, alignment holes are high-contrast circular features, working distance is short (~5-10 cm) → IBVS strengths. PBVS depth estimation accuracy at this distance cannot reach 0.05 mm.
2. **Feature selection**: use 2-4 hole centers as pixel coordinates. OpenCV `HoughCircles` or sub-pixel corner detection.
3. **Depth estimation**: working distance is nearly constant (PCB sits on a fixture at known height); a fixed $Z$ suffices. Real-time depth camera updates are a bonus.
4. **Gain tuning**: start $\lambda$ at 0.3 with a 60 fps camera. Switch to a smaller $\lambda$ (0.1) near convergence to avoid oscillation.
5. **Trap to avoid**: verify that lens distortion has been corrected — at image edges, distortion makes the interaction matrix inaccurate, which is lethal at 0.05 mm precision.

**What the interviewer wants to hear**: IBVS's precision advantage in 2D planar scenes, feature selection strategy, and the easily overlooked but critical detail of distortion correction for extreme precision.

</details>

<details>
<summary>Q2 (hard): The target requires 180-degree rotation to align. You observe IBVS producing a bizarre retreat-then-advance trajectory. How do you analyze and fix?</summary>

**Complete reasoning chain**:

1. **Root cause**: IBVS interpolates linearly in pixel space. For 180-degree rotation, the "shortest path" in pixel space does not correspond to the shortest Cartesian path. Intermediate pixel states may correspond to the camera backing away.
2. **Classic phenomenon**: known as the IBVS retreat problem. Feature points cross in pixel space → the $Z$ estimate in the interaction matrix flips sign → velocity reverses.
3. **Fix 1: 2.5D Visual Servoing**: control translation via IBVS (pixel error), rotation via PBVS ($\theta u$ rotation error). Prevents rotation from taking a wrong path in pixel space.
4. **Fix 2: Path Planning + VS**: use PBVS for coarse alignment first (rotate to within ~10 degrees), then switch to IBVS for fine alignment.
5. **Fix 3: Virtual features**: choose features that do not cross under rotation (e.g., image moments $\mu$ instead of corner points), keeping the interaction matrix stable under large rotations.

**What the interviewer wants to hear**: understanding the root cause of the IBVS retreat problem (pixel-space linearity ≠ Cartesian-space linearity) and the 2.5D VS hybrid strategy.

</details>

<details>
<summary>Q3 (medium-hard): Camera runs at 15 fps but the control loop is 100 Hz. The mismatch between image and control frequency — how do you handle it?</summary>

**Complete reasoning chain**:

1. **The problem**: 100 Hz control needs a velocity command every 10 ms, but images arrive every 67 ms. Six control cycles between each image have no new visual data.
2. **Solution: asynchronous interpolation + prediction**:
   - On each new image → update features $s_k$ and interaction matrix $L_s$
   - Between images → predict feature positions with EKF: $\hat{s}_{k+\Delta t} = s_k + L_s \cdot v_c \cdot \Delta t$
   - Or linearly extrapolate from the last two frames
3. **Reduce gain**: effective feedback is only 15 Hz, so $\lambda$ must be lower than at 60 fps (~0.1-0.2); otherwise high-gain control on stale data oscillates.
4. **Hardware upgrade path**: if precision demands it, switch to a high-frame-rate industrial camera (120+ fps) or an event camera. Event cameras have microsecond latency and asynchronous triggering, ideal for visual servoing.
5. **Trap to avoid**: do not simply hold the last $v_c$ during cycles without new images — the object moves but the velocity command does not update, accumulating position error.

**What the interviewer wants to hear**: EKF prediction is the standard approach, gain must match effective feedback frequency, event cameras are the hardware solution.

</details>

## Interview Angles

1. **Physical meaning and singularity avoidance of the Interaction Matrix** — the core interview topic for visual servoing. Bring out with: "The interaction matrix is the image-space Jacobian — it maps 6-DoF camera velocity to 2D feature velocity. Singularity occurs when features are coplanar, collinear, or at zero depth. My approach is to select spatially distributed features and monitor the condition number of $L_s$; when it spikes, I reduce the gain or switch feature sets."

2. **IBVS vs PBVS scene-selection logic** — demonstrates engineering judgment rather than rote learning. Bring out with: "It is not about which is better, but which fits the scene: planar + short range + high precision → IBVS; large workspace + straight trajectory needed + reliable depth → PBVS; large rotation or mixed requirements → 2.5D. Before choosing, I evaluate camera calibration quality and depth estimation accuracy, because those set the ceiling for PBVS."

3. **How hand-eye calibration errors propagate to control accuracy** — separates "used visual servoing" from "truly understands precision limits." Bring out with: "PBVS accuracy is upper-bounded by hand-eye calibration — 1 degree of rotation error at 30 cm working distance causes ~5 mm end-effector error. For high-precision tasks, I either invest in multi-pose calibration + LM refinement, or default to IBVS which is naturally less calibration-sensitive."

4. **Asynchronous delay compensation** — proves real-world deployment experience. Bring out with: "15 fps camera with 100 Hz control is a common scenario. My approach is EKF prediction of feature positions between image frames, while reducing the gain to match the effective feedback frequency. I also model camera delay in the controller; otherwise high gain + delay = oscillation."

5. **From hand-crafted features to end-to-end VS** — shows frontier awareness. Bring out with: "Traditional visual servoing relies on hand-crafted features (ORB, corners) that are fragile under occlusion and lighting variation. The trend is CNNs that regress $v_c$ directly from the image (end-to-end VS), or NeRF/foundation models for more robust feature representations. But the interpretability and safety guarantees of classical methods remain a hard requirement in industrial settings."

## Further Reading

- **Chaumette & Hutchinson, *Visual Servo Control Part I & II* (IEEE RAM, 2006/2007)** — the definitive survey on visual servoing, covering IBVS/PBVS/2.5D theory comprehensively
- **ViSP (Visual Servoing Platform) tutorials and examples** — the most complete open-source VS framework, C++ core with Python bindings, tutorials covering all scenarios
- **Tsai & Lenz, *A New Technique for Fully Autonomous and Efficient 3D Robotics Hand-Eye Calibration* (1989)** — the foundational hand-eye calibration paper, still the standard method
- **Paper: *Event-based Visual Servoing*** — event cameras in VS, microsecond latency breaking through frame-rate bottlenecks
- **Paper: *End-to-End Training of Deep Visuomotor Policies* (Levine et al., 2016)** — pioneering end-to-end VS work, CNN mapping image → motor command directly
- **OpenCV camera calibration tutorial** — camera calibration is a prerequisite for VS; distortion correction underpins all precision work
- ***Embodied AI Algorithm Engineer Interview Questions*, Ch7.1 Visual Servoing Basics, Ch7.4 Hand-Eye Calibration** — high-frequency interview topics: interaction matrix derivation, IBVS vs PBVS selection logic
