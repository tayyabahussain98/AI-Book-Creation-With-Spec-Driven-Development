# Reality Gap Mitigation Strategies

**Purpose**: Decision tree for choosing the appropriate strategy to bridge the sim-to-real gap based on failure mode, resources available, and deployment timeline.

**Context**: The reality gap—differences between simulation and reality—is the primary challenge in sim-to-real transfer. Multiple mitigation strategies exist with different trade-offs.

---

## ASCII Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│           REALITY GAP MITIGATION DECISION TREE                   │
└─────────────────────────────────────────────────────────────────┘

START: Policy trained in sim fails in real world
  │
  v
┌───────────────────────────────────────────────────────────┐
│ Q1: What is the failure mode?                            │
└───┬───────────────────────────────────────────────────────┘
    │
    ├─> A: Physics mismatch (objects fall/slide differently)
    │   → SOLUTION 1: System Identification + Sim Tuning
    │     • Measure real robot params (friction, mass, inertia)
    │     • Update Isaac Sim with measured values
    │     • Retrain policy in updated sim
    │     Cost: Medium (1-2 days measurement + retrain)
    │     Success Rate: 80-90%
    │
    ├─> B: Sensor noise (camera jitter, IMU drift)
    │   → SOLUTION 2: Domain Randomization (Sensor)
    │     • Add noise models to sim sensors
    │     • Gaussian noise for IMU: μ=0, σ=0.01
    │     • Motion blur for camera (simulate fast motion)
    │     • Retrain with noisy observations
    │     Cost: Low (hours to add noise + retrain)
    │     Success Rate: 85-95%
    │
    ├─> C: Actuation delays (motor lag, comm latency)
    │   → SOLUTION 3: Add Latency to Simulation
    │     • Introduce 10-50ms action delay in sim
    │     • Train policy to be robust to lag
    │     • Alternative: Predictive control (anticipate future state)
    │     Cost: Low (config change + retrain)
    │     Success Rate: 80-90%
    │
    ├─> D: Unmodeled dynamics (cable drag, backlash)
    │   → SOLUTION 4: Domain Randomization (Physics)
    │     • Randomize mass ±20%, friction ±50%
    │     • Add external forces (wind, cable pull)
    │     • Train on worst-case scenarios
    │     Cost: Low-Medium (1 day + retrain)
    │     Success Rate: 75-90%
    │
    └─> E: All of the above (large reality gap)
        → SOLUTION 5: Fine-Tuning with Real Data
          • Collect 100-1000 real trajectories
          • Continue training policy on real data
          • Use sim for exploration, real for refinement
          Cost: High (weeks of real-world data collection)
          Success Rate: 95-99%


═══════════════════════════════════════════════════════════════════
STRATEGY COMPARISON
═══════════════════════════════════════════════════════════════════

┌─────────────────┬───────────┬──────┬────────────┬───────────┐
│ Strategy        │Time (days)│ Cost │Success Rate│When to Use│
├─────────────────┼───────────┼──────┼────────────┼───────────┤
│Domain Random.   │    1-2    │  $   │   75-90%   │ Default   │
│Sim Tuning       │    2-3    │  $$  │   80-90%   │ Measure OK│
│Safety Margins   │    0.5    │  $   │   70-80%   │ Quick fix │
│Real Fine-Tuning │   7-14    │ $$$$ │   95-99%   │ Critical  │
│Hybrid (DR+FT)   │   5-10    │ $$$ │   90-95%   │ Best both │
└─────────────────┴───────────┴──────┴────────────┴───────────┘

Legend:
  $ = <$100 (compute only)
  $$ = $100-$1K (measurement equipment)
  $$$ = $1K-$10K (robot wear, human labor)
  $$$$ = $10K+ (extensive real-world testing)


═══════════════════════════════════════════════════════════════════
DETAILED STRATEGIES
═══════════════════════════════════════════════════════════════════

1. DOMAIN RANDOMIZATION (Default, Low Cost)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Pros:
  • No real-world data required
  • Parallelizable (train fast in sim)
  • Robust to diverse conditions

Cons:
  • May not cover all real-world variations
  • Over-randomization can hurt sim performance
  • Requires careful tuning of DR ranges

Best For:
  • Early development (no real robot yet)
  • Tasks with well-understood physics
  • When real-world testing is expensive/dangerous


2. SYSTEM IDENTIFICATION + SIM TUNING (Medium Cost)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Workflow:
  1. Measure real robot parameters:
     - Link masses: Weigh each component
     - Joint friction: Run sine-sweep torque test
     - Surface friction: Measure slip angles
  2. Update Isaac Sim URDF/USD with measured values
  3. Validate: Drop object in sim vs real (should match)
  4. Retrain policy in updated sim

Pros:
  • Simulation matches reality more accurately
  • Reduces need for aggressive DR
  • Physics-grounded approach

Cons:
  • Requires measurement equipment (scales, encoders)
  • Time-consuming (1-2 days of experiments)
  • Doesn't capture all nonlinearities

Best For:
  • Precision tasks (manipulation, assembly)
  • When DR alone insufficient
  • Robot hardware already available


3. SAFETY MARGINS (Quick Fix, Conservative)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Approach:
  • Limit real-world actions to 50-75% of trained range
  • Add conservative joint limits (stop 10° before hardware limit)
  • Reduce max velocity by 30% for safety

Pros:
  • Instant deployment (no retraining)
  • Prevents hardware damage
  • Easy to implement (config change)

Cons:
  • Performance degrades (slower, less capable)
  • Doesn't fix root cause
  • May still fail on edge cases

Best For:
  • Initial real-world tests
  • Safety-critical applications
  • Temporary solution during debugging


4. REAL-WORLD FINE-TUNING (Highest Success, Expensive)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Workflow:
  1. Pre-train in sim (10K episodes)
  2. Collect 100-1000 real trajectories (1-2 weeks)
  3. Continue training on real data (low learning rate)
  4. Alternate: Sim exploration, real refinement

Pros:
  • Highest transfer success (95-99%)
  • Directly addresses reality gap
  • Captures all real-world nuances

Cons:
  • Expensive ($10K+ in robot wear, supervision)
  • Time-consuming (weeks of real-world operation)
  • Risk of hardware damage during exploration

Best For:
  • Production deployment (must work reliably)
  • Tasks where sim is fundamentally limited
  • When robot fleet available (amortize cost)


5. HYBRID: DR + FINE-TUNING (Recommended)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Workflow:
  1. Pre-train with aggressive DR in sim
  2. Deploy to real world with safety margins
  3. Collect real trajectories (100-500)
  4. Fine-tune policy on real data (1-2 days)

Pros:
  • Balances cost and success rate
  • DR provides robustness, fine-tuning fixes gaps
  • Reduces real-world data needs (vs pure fine-tuning)

Cons:
  • Still requires some real-world testing
  • More complex workflow

Best For:
  • Most practical applications
  • When both sim and real robot available
  • Iterative deployment (start DR, refine with real data)


═══════════════════════════════════════════════════════════════════
DECISION FLOWCHART
═══════════════════════════════════════════════════════════════════

┌──────────────────────────────────────┐
│ Do you have a real robot?            │
│ NO → Use DR (only option)            │
│ YES → Continue ↓                     │
└────────────┬─────────────────────────┘
             │
             v
┌──────────────────────────────────────┐
│ Is real-world testing expensive?     │
│ YES (>$1K/day) → Use DR + Sim Tuning │
│ NO → Continue ↓                      │
└────────────┬─────────────────────────┘
             │
             v
┌──────────────────────────────────────┐
│ Can you afford 1-2 weeks testing?    │
│ NO → Use DR + Safety Margins         │
│ YES → Use Hybrid (DR + Fine-Tuning)  │
└──────────────────────────────────────┘


═══════════════════════════════════════════════════════════════════
COMMON PITFALLS
═══════════════════════════════════════════════════════════════════

1. Over-Randomization
   Symptom: Policy works in real world but fails in sim validation
   Fix: Reduce DR range by 50%, retrain

2. Under-Randomization
   Symptom: Works in sim, fails immediately in real world
   Fix: Increase DR range by 2x, add more DR categories

3. Ignoring Latency
   Symptom: Policy oscillates or overshoots in real world
   Fix: Add 20-50ms action delay to simulation

4. Premature Deployment
   Symptom: Hardware damage, safety incidents
   Fix: Start with safety margins (50% action limits)

5. Insufficient Real Data for Fine-Tuning
   Symptom: Policy overfits to few real trajectories
   Fix: Collect at least 100 trajectories, use data augmentation
