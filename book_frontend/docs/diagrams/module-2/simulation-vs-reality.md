# Simulation vs Reality Gap Specification

**Purpose**: Visualize the reality gap between simulation and physical robots to help learners understand what transfers and what requires real-world tuning

**Diagram Type**: comparison

## The Sim-to-Real Transfer Challenge

```text
┌─────────────────────────────────────────────────────────────────┐
│                    SIMULATION WORLD                             │
│                  (Idealized Physics)                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ✓ Perfect Knowledge       ✓ Deterministic        ✓ Fast Reset│
│  ✓ Controllable Conditions ✓ No Hardware Failures ✓ Scalable  │
│  ✓ Instant Sensor Access   ✓ Ground Truth Labels  ✓ Safe      │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
                              ↓
                    REALITY GAP (Transfer Issues)
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                     PHYSICAL WORLD                              │
│               (Messy, Uncertain Reality)                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ✗ Partial Observability   ✗ Stochastic Noise    ✗ Slow Reset │
│  ✗ Uncontrolled Variables  ✗ Sensor Drift        ✗ Expensive  │
│  ✗ Measurement Latency     ✗ No Ground Truth     ✗ Dangerous  │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Transfer Quality by Domain

| Aspect | Sim Quality | Reality Gap | Transfer Strategy |
|--------|-------------|-------------|-------------------|
| **Kinematics** | High (0.1% error) | Minimal | Direct transfer: Forward/inverse kinematics identical |
| **Collision Detection** | Medium (5% false negatives) | Moderate | Add safety margins: +5cm buffer around obstacles |
| **Contact Forces** | Low (30% error) | Large | Domain randomization: Vary friction μ ∈ [0.4, 0.9] |
| **Sensor Noise** | Low (Gaussian only) | Large | Noise injection: Add real-world noise profiles |
| **Motor Dynamics** | Medium (10-20% error) | Moderate | System identification: Tune PID from real data |
| **Visual Appearance** | Low (synthetic textures) | Large | Sim-to-real transfer: Train on diverse sim textures |
| **Timing/Latency** | High (deterministic) | Large | Hardware-in-loop: Test on real control loop |
| **Wear and Tear** | None (no degradation) | Critical | Periodic recalibration: Update model from real robot |

## What Transfers Well (Direct Use)

### 1. Geometric Reasoning
**Sim Validity**: 99%+
```text
Example: Path planning around static obstacles
┌─────────────┐
│  Obstacle   │  ← Known dimensions (0.5m × 0.5m)
└─────────────┘
      ↑
   [Robot]  ← Plan collision-free path
```
**Why it works**: Geometry is deterministic; CAD models match physical dimensions within ±1mm for most robots.

### 2. Kinematic Chains
**Sim Validity**: 98%+
```text
Forward Kinematics: θ₁, θ₂, θ₃ → (x, y, z)
Joint angles → End effector position
```
**Why it works**: Joint transformations are pure math (rotation matrices, DH parameters); no physics uncertainties.

### 3. High-Level Logic
**Sim Validity**: 90%+
```text
State Machine: IDLE → APPROACH → GRASP → LIFT → PLACE
```
**Why it works**: Task sequencing logic independent of physics; only thresholds need tuning (e.g., "grasp detected" force threshold).

## What Requires Tuning (Moderate Gap)

### 4. PID Controller Gains
**Sim Validity**: 60-80%
```text
Simulation: Kp=10, Ki=0.1, Kd=0.5 → Stable
Reality:    Kp=8,  Ki=0.2, Kd=0.3 → Requires tuning
```
**Reality Gap Sources**:
- Motor backlash not modeled (±2° slack)
- Transmission friction higher than sim (40% vs 25%)
- Inertia estimation error (±15%)

**Transfer Strategy**: Use simulation gains as starting point; auto-tune on real robot with Ziegler-Nichols or twiddle algorithm.

### 5. Sensor-Based Reactive Control
**Sim Validity**: 70%
```text
Simulation: LiDAR range ±3cm → Obstacle avoidance works
Reality:    LiDAR range ±5cm + 2cm systematic bias → Collisions
```
**Reality Gap Sources**:
- Sensor calibration drift over time
- Multi-path reflections (glass, mirrors) not modeled
- Temperature-dependent range errors

**Transfer Strategy**: Add 2× noise in sim; validate on real sensor before deployment.

## What Fails Without Adaptation (Large Gap)

### 6. Contact-Rich Manipulation
**Sim Validity**: 30-50%
```text
Simulation: Insert peg into 0.5mm tolerance hole → Success
Reality:    Same trajectory → Peg jams, motor stalls
```
**Reality Gap Sources**:
- Friction coefficients vary by surface contamination (oil, dust)
- Contact stiffness not modeled (compliant surfaces compress)
- Micro-geometry (surface roughness) affects insertion angle

**Transfer Strategy**: Force-torque feedback loops; compliant control (impedance mode); sim-to-real RL with domain randomization.

### 7. Vision-Based Grasping
**Sim Validity**: 40%
```text
Simulation: Detect object in synthetic scene → 95% accuracy
Reality:    Same model on real camera → 60% accuracy
```
**Reality Gap Sources**:
- Lighting conditions vary (shadows, reflections, brightness)
- Camera calibration errors (±5 pixels distortion)
- Object appearance (synthetic textures don't match real materials)

**Transfer Strategy**: Domain randomization (vary lighting, textures); fine-tune on real images; use depth + RGB (not RGB alone).

### 8. Dynamic Walking/Running
**Sim Validity**: 20-40%
```text
Simulation: Humanoid walks at 0.5 m/s → Stable
Reality:    Same controller → Falls in 2 seconds
```
**Reality Gap Sources**:
- Ground friction varies by surface (carpet vs tile vs wet floor)
- Joint compliance not modeled (ankles flex under load)
- IMU drift accumulates orientation error (±5° after 10 steps)

**Transfer Strategy**: Robust control (MPC with uncertainty); hardware-in-loop testing; learned residuals (train RL policy on real robot to correct sim policy).

## Domain Randomization Strategy

**Goal**: Expose sim-trained policies to wide range of physics parameters so they generalize to real world.

```text
Parameter Randomization Ranges:
┌─────────────────────────┬─────────────┬─────────────┐
│ Parameter               │ Sim Default │ Randomized  │
├─────────────────────────┼─────────────┼─────────────┤
│ Friction (μ)            │ 0.8         │ [0.4, 1.2]  │
│ Mass (kg)               │ 50.0        │ [45, 55]    │
│ Motor Gain              │ 1.0         │ [0.85, 1.15]│
│ Sensor Noise (σ)        │ 0.01        │ [0.01, 0.05]│
│ Timestep Jitter (ms)    │ 0           │ [0, 5]      │
│ Link Length Error (%)   │ 0           │ [-2%, +2%]  │
└─────────────────────────┴─────────────┴─────────────┘
```

**Training Protocol**:
1. Each simulation episode: Randomly sample parameters from ranges
2. Train policy on 1000+ randomized environments
3. Policy learns robust strategy that works across parameter variations
4. Deploy to real robot: Reality is "just another" sample from randomized distribution

## Calibration and System Identification

**When sim-to-real gap is large**, update simulation model from real robot data:

```text
1. Collect Real Data
   ├─ Joint torques at known positions
   ├─ End effector forces during contact
   └─ Sensor readings in controlled conditions

2. Fit Physics Parameters
   ├─ Inertia tensor (least-squares on acceleration data)
   ├─ Friction model (fit Coulomb + viscous terms)
   └─ Sensor offset/scale (linear regression)

3. Update Simulation
   ├─ Replace default URDF <inertial> values
   ├─ Tune <friction> coefficients in SDF
   └─ Add sensor <bias> and <noise> models

4. Validate Alignment
   └─ Compare sim trajectory to real trajectory: MSE < 5%
```

**Iterative Refinement**: Reality → Measure Gap → Update Sim → Retrain → Test Reality → Repeat

## Hardware-in-the-Loop (HIL) Testing

**Hybrid approach**: Simulate environment, use real hardware for actuation/sensing.

```text
┌─────────────────────────────────────────────────────────────┐
│  Gazebo Simulation (Virtual Environment)                    │
│  ├─ Obstacles, terrain, lighting                            │
│  └─ Publishes: /virtual_lidar/scan                          │
└────────────────┬────────────────────────────────────────────┘
                 │
                 ↓ ROS 2 Topics
┌─────────────────────────────────────────────────────────────┐
│  Real Robot Hardware                                        │
│  ├─ Physical motors execute commands                        │
│  ├─ Real sensors (camera, IMU) publish data                 │
│  └─ Controller uses virtual LiDAR + real IMU                │
└─────────────────────────────────────────────────────────────┘
```

**Benefits**:
- Test dangerous scenarios safely (edge of cliff, high-speed collision)
- Faster iteration than full physical testing
- Real motor dynamics, latency, sensor noise

**Limitations**:
- Contact interactions still simulated (grasping, pushing)
- Sensor fusion may mismatch (virtual vs real sensor characteristics)

## Usage in Book

- **Referenced in**: Chapter 1 (Core Concept 3: Simulation-to-Reality Gap)
- **Purpose**: Set realistic expectations for learners about what simulation can and cannot predict
- **Learning Goal**: Understand that simulation is a tool for rapid iteration, not a perfect replica of reality; successful deployment requires validation and tuning

## Key Takeaways

1. **Kinematics transfer well** (98%+): Geometry and transformations are deterministic
2. **Dynamics require tuning** (60-80%): Friction, inertia, motor response differ from sim
3. **Contact-rich tasks fail** (30-50%): Manipulation, walking need domain randomization or real-world fine-tuning
4. **Vision has large gap** (40%): Lighting, textures, camera calibration vary; use depth + RGB
5. **Domain randomization**: Train on wide parameter ranges to improve sim-to-real transfer
6. **Iterative refinement**: Measure gap → Update sim → Retrain → Validate on real robot
7. **Hardware-in-loop**: Hybrid approach for safe testing of dangerous scenarios
8. **Expectation setting**: Simulation accelerates development but never eliminates need for real-world validation
