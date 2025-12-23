# Reinforcement Learning Training Loop

**Purpose**: Illustrate the RL training cycle showing how an agent learns through interaction with an environment, receiving rewards, and updating its policy to maximize cumulative reward.

**Context**: RL is the foundation for training robots in simulation (Isaac Sim) before deploying to the real world. Understanding the training loop is essential for sim-to-real transfer.

---

## ASCII Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│          REINFORCEMENT LEARNING TRAINING LOOP                    │
└─────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────┐
│                         AGENT                                 │
│  ┌────────────────────────────────────────────────────────┐  │
│  │            Policy Network π_θ(a|s)                      │  │
│  │  Neural Network: State → Action probabilities          │  │
│  │  Parameters θ updated via policy gradient              │  │
│  └────────────────┬───────────────────────────────────────┘  │
│                   │                                           │
│     ┌─────────────┴────────────┐                             │
│     │  SELECT ACTION           │                             │
│     │  a_t ~ π_θ(a|s_t)        │                             │
│     │  (sample from policy)    │                             │
│     └─────────────┬────────────┘                             │
└───────────────────┼──────────────────────────────────────────┘
                    │ Action a_t
                    │ (e.g., joint velocities, footstep)
                    v
┌──────────────────────────────────────────────────────────────┐
│                     ENVIRONMENT                               │
│  (Isaac Sim: Physics engine + sensor simulation)             │
│  ┌────────────────────────────────────────────────────────┐  │
│  │  1. Apply action a_t to robot actuators               │  │
│  │  2. Step physics simulation (timestep Δt = 1/60s)     │  │
│  │  3. Observe new state s_{t+1} (joint angles, CoM, etc)│  │
│  │  4. Compute reward r_t based on task objective        │  │
│  └────────────────┬───────────────────────────────────────┘  │
└────────────────────┼─────────────────────────────────────────┘
                     │ State s_{t+1}, Reward r_t
                     v
┌──────────────────────────────────────────────────────────────┐
│                    EXPERIENCE BUFFER                          │
│  Store transitions: (s_t, a_t, r_t, s_{t+1}, done_t)        │
│  Capacity: 10K-1M transitions                                │
│  Used for batch training (sample minibatches)                │
└────────────────────┬─────────────────────────────────────────┘
                     │ When buffer full or episode ends
                     v
┌──────────────────────────────────────────────────────────────┐
│                   POLICY UPDATE                               │
│  ┌────────────────────────────────────────────────────────┐  │
│  │  Algorithm: PPO (Proximal Policy Optimization)         │  │
│  │  1. Sample minibatch from experience buffer            │  │
│  │  2. Compute advantage A(s,a) = Q(s,a) - V(s)           │  │
│  │  3. Update θ to maximize: E[min(r_t(θ) A, clip(...)]  │  │
│  │  4. Repeat for K epochs (K=3-10)                       │  │
│  └────────────────┬───────────────────────────────────────┘  │
│                   │ Updated parameters θ'                    │
│                   v                                           │
│  ┌────────────────────────────────────────────────────────┐  │
│  │  Store checkpoint: policy_epoch_N.pt                   │  │
│  │  Evaluate performance every M episodes                 │  │
│  └────────────────────────────────────────────────────────┘  │
└───────────────────┬──────────────────────────────────────────┘
                    │ Loop back to agent with updated policy
                    └──────────> (Repeat for N episodes)


═══════════════════════════════════════════════════════════════════
DETAILED COMPONENTS
═══════════════════════════════════════════════════════════════════

1. STATE REPRESENTATION s_t
━━━━━━━━━━━━━━━━━━━━━━━━━━
Humanoid walking task example:
  - Joint angles: [θ_hip_L, θ_knee_L, θ_ankle_L, ...] (12 DOF)
  - Joint velocities: [ω_hip_L, ...] (12 DOF)
  - Body orientation: [roll, pitch, yaw] (3 DOF)
  - Center of Mass position: [x_CoM, y_CoM, z_CoM] (3 DOF)
  - Goal direction: [x_goal - x_CoM, y_goal - y_CoM] (2 DOF)
  Total state dim: 32

2. ACTION SPACE a_t
━━━━━━━━━━━━━━━━━━━
Two common formulations:
  A. Joint Torques (continuous):
     a_t ∈ ℝ^12: [-100, 100] Nm per joint

  B. Joint Velocity Targets (continuous):
     a_t ∈ ℝ^12: [-2.0, 2.0] rad/s per joint
     Low-level PD controller converts to torques

3. REWARD FUNCTION r_t
━━━━━━━━━━━━━━━━━━━━━━
Design reward to align with task objective:

  r_t = r_forward + r_alive + r_energy - penalties

  r_forward = 10 * (x_{t+1} - x_t)  # Encourage forward motion
  r_alive = 1.0                      # Survive longer
  r_energy = -0.01 * ||τ_t||^2       # Minimize energy (torque)

  Penalties:
    -10 if fell (z_CoM < 0.3m)       # Large penalty for falling
    -1 if head collision               # Avoid hitting obstacles
    -0.1 * |yaw - yaw_target|        # Stay aligned with goal

4. POLICY NETWORK π_θ(a|s)
━━━━━━━━━━━━━━━━━━━━━━━━━━
Architecture (MLP example):
  Input: State s_t (32 dim)
  Hidden: FC(256) → ReLU → FC(256) → ReLU
  Output: Mean μ(s) (12 dim) + Log-Std σ(s) (12 dim)
  Action sampling: a_t ~ N(μ(s_t), exp(2σ(s_t)))

5. TRAINING ALGORITHM (PPO)
━━━━━━━━━━━━━━━━━━━━━━━━━━━
Objective: Maximize expected return J(θ) = E[Σ r_t]

  PPO Clipped Surrogate Loss:
    L(θ) = E[ min( r_t(θ) Â_t,  clip(r_t(θ), 1-ε, 1+ε) Â_t ) ]

  Where:
    r_t(θ) = π_θ(a|s) / π_{θ_old}(a|s)  # Probability ratio
    Â_t = Advantage estimate (how much better action a is vs average)
    ε = 0.2 (clip range, prevents large policy updates)

  Update: θ ← θ + α ∇_θ L(θ)  (Adam optimizer, α=3e-4)


═══════════════════════════════════════════════════════════════════
TRAINING TIMELINE (HUMANOID WALKING)
═══════════════════════════════════════════════════════════════════

Episode 1-100 (Early Training):
  - Robot falls immediately (&lt;5 steps)
  - Reward: ~5 per episode
  - Policy: Random exploration

Episode 100-1000 (Learning to Balance):
  - Robot walks 10-20 steps before falling
  - Reward: ~50 per episode
  - Policy: Discovers ZMP stability heuristics

Episode 1000-5000 (Stable Walking):
  - Robot walks 100+ steps consistently
  - Reward: ~500 per episode
  - Policy: Smooth gait, balanced turns

Episode 5000-10000 (Optimization):
  - Robot walks indefinitely, navigates obstacles
  - Reward: ~1000+ per episode
  - Policy: Efficient energy use, robust to perturbations

Total Training Time:
  - 10,000 episodes × 20s per episode = 200,000s sim time
  - Real-time factor: 10-20x (parallelized simulation)
  - Wall-clock time: 3-5 hours on 8 GPUs (Isaac Sim)


═══════════════════════════════════════════════════════════════════
KEY INSIGHTS
═══════════════════════════════════════════════════════════════════

1. CREDIT ASSIGNMENT PROBLEM
  Challenge: Which actions led to eventual success?
  Solution: Advantage function A(s,a) estimates marginal value of action

2. EXPLORATION VS EXPLOITATION
  Early training: High entropy (random actions) → explore
  Late training: Low entropy (confident actions) → exploit
  PPO automatically anneals exploration via policy updates

3. SAMPLE EFFICIENCY
  PPO: 10M-100M environment steps to learn walking
  Compare to: DQN (100M-1B steps), SAC (10M-50M steps)
  Isaac Sim parallelization: 100x speedup via multi-GPU

4. SIM-TO-REAL GAP
  Simulation physics ≠ Reality → Trained policy may fail
  Solution: Domain Randomization (next concept)
