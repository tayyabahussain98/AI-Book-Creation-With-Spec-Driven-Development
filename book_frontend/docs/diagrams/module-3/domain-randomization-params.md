# Diagram Specification: Domain Randomization Parameters

**Type**: ComparisonTable

**Purpose**: Provide a comprehensive reference for domain randomization parameters used to improve AI model robustness during training in simulation

**Referenced By**: Chapter 2 (Synthetic Data Generation), Chapter 5 (Sim-to-Real Transfer)

---

## Visual Representation

### Domain Randomization Parameter Categories

Domain randomization introduces controlled variations during training to prevent overfitting to specific simulation conditions. Parameters are grouped into four categories:

| **Category** | **Parameter** | **Purpose** | **Typical Range** | **Example Values** |
|--------------|---------------|-------------|-------------------|-------------------|
| **Object** | Object Pose | Vary object positions/orientations | Position: ±0.5m, Rotation: 0-360° | Box at (0.2, 0.1, 0.5) rotated 45° |
| | Object Scale | Vary object sizes | 0.8x to 1.2x original size | Chair scaled 0.9x (smaller) |
| | Object Textures | Randomize surface materials | 10-50 texture variants | Wooden table with oak/pine/metal textures |
| | Object Shapes | Vary geometry within class | Parametric deformation | Bottle with radius 3-5cm, height 15-25cm |
| | Clutter Density | Add/remove background objects | 0-20 objects per scene | 5 random boxes, 3 cylinders |
| **Environmental** | Lighting Intensity | Vary brightness | 100-2000 lux | Living room at 300 lux (dim) or 1500 lux (bright) |
| | Lighting Color Temp | Simulate different light sources | 2700K (warm) to 6500K (cool) | Office: 4000K (neutral white) |
| | Lighting Direction | Change light source position | Azimuth: 0-360°, Elevation: 30-90° | Sunlight at 60° elevation, 120° azimuth |
| | Background Textures | Randomize walls/floors | 20-100 material variants | Floor: wood/tile/carpet, Walls: painted/wallpaper |
| | Weather/Atmospheric | Add fog, rain, dust | Fog density: 0-0.5, Rain: 0-10mm/hr | Outdoor scene with 0.2 fog density |
| **Sensor** | Camera Exposure | Simulate auto-exposure | -2 to +2 EV (exposure value) | Underexposed (-1 EV) for bright scenes |
| | Camera Gain/ISO | Add sensor noise | ISO 100-3200 | High-gain camera (ISO 1600) in low light |
| | Camera Blur | Motion/defocus blur | 0-5 pixels blur radius | 2px motion blur from robot movement |
| | LiDAR Noise | Point cloud noise | Gaussian noise σ = 0-5cm | ±2cm noise on depth measurements |
| | LiDAR Dropouts | Simulate occlusion/absorption | 0-10% point dropouts | 5% missing points on glass surfaces |
| **Physics** | Friction Coefficients | Vary surface interaction | μ = 0.1 (ice) to 1.5 (rubber) | Floor friction μ = 0.6 (polished wood) |
| | Object Mass | Vary inertia properties | 0.8x to 1.2x nominal mass | Grasped object: 0.5kg instead of 0.6kg |
| | Joint Damping | Simulate wear/age | Damping: 0.5x to 1.5x nominal | Robot joint damping increased 1.2x (stiff) |
| | Contact Dynamics | Restitution, stiffness | Restitution: 0 (inelastic) to 0.95 (elastic) | Ball bounces with 0.8 restitution |
| | Gravity Variations | Simulate slopes, acceleration | -10.5 to -9.5 m/s² | Robot on 5° slope: effective gravity varies |

---

## Detailed Explanation

### Why Domain Randomization?

AI models trained exclusively in simulation can **overfit to simulation artifacts**:
- Always expect perfect lighting (no shadows, no glare)
- Assume objects have identical textures (same blue box every time)
- Rely on noise-free sensor data (perfect camera images, no LiDAR dropouts)

When deployed to the real world, these models **fail catastrophically**:
- Cannot detect objects under different lighting
- Confused by texture variations (wood vs plastic)
- Sensitive to sensor noise and occlusion

**Domain Randomization** solves this by:
1. Training on **diverse simulation conditions** that span real-world variations
2. Forcing models to learn **robust features** (shape, geometry) instead of superficial cues (specific texture, lighting)
3. Creating a **reality gap mitigation** strategy that improves sim-to-real transfer

### Object Randomization

**Purpose**: Prevent overfitting to specific object appearances and positions

#### Object Pose Randomization
- **Why**: Real-world objects appear at unpredictable positions/orientations
- **Example**: Training a grasping policy for bottles
  - Simulation: Randomize bottle position ±20cm, rotation 0-360°
  - Result: Policy learns to grasp bottles from any angle, not just "bottle upright at (0,0,0)"

#### Object Texture Randomization
- **Why**: Real objects have diverse materials (wood, plastic, metal)
- **Example**: Training object detection for chairs
  - Simulation: Apply 50 different chair textures (leather, fabric, wood grain)
  - Result: Model learns chair *geometry* instead of memorizing "blue chair texture"

#### Clutter Randomization
- **Why**: Real environments are cluttered (desks, warehouses, homes)
- **Example**: Training navigation in warehouses
  - Simulation: Add 0-20 random boxes, pallets, barrels to scene
  - Result: Robot navigates around unknown obstacles, not just empty warehouse

### Environmental Randomization

**Purpose**: Simulate diverse lighting and background conditions

#### Lighting Randomization
- **Why**: Real-world lighting varies drastically (morning sun, office fluorescent, evening shadows)
- **Example**: Training person detection
  - Simulation: Vary light intensity 100-2000 lux, color temperature 3000-6000K, direction ±90°
  - Result: Model detects people in bright sunlight, dim indoor lighting, and side-lit scenes

#### Background Randomization
- **Why**: Real backgrounds are diverse (walls, floors, outdoor terrain)
- **Example**: Training object segmentation
  - Simulation: Randomize floor (wood/tile/carpet) and walls (painted/brick/glass)
  - Result: Model segments objects regardless of background

### Sensor Randomization

**Purpose**: Simulate real-world sensor imperfections (noise, blur, dropouts)

#### Camera Noise and Blur
- **Why**: Real cameras have noise (low light), motion blur (fast movement), exposure errors
- **Example**: Training lane detection for autonomous vehicles
  - Simulation: Add Gaussian noise (ISO 400-1600), motion blur (1-3px), exposure variation (±1 EV)
  - Result: Model robust to camera artifacts, not just "perfect simulation images"

#### LiDAR Noise and Dropouts
- **Why**: Real LiDAR has ranging errors (±2-5cm), missed returns (glass, water, black surfaces)
- **Example**: Training 3D object detection
  - Simulation: Add ±3cm Gaussian noise to depth, drop 5-10% of points
  - Result: Model handles noisy, incomplete point clouds

### Physics Randomization

**Purpose**: Account for uncertain dynamics (friction, mass, contact)

#### Friction and Mass Randomization
- **Why**: Real-world friction varies (wet floors, worn surfaces), object mass is uncertain (empty vs full bottle)
- **Example**: Training bipedal walking
  - Simulation: Randomize floor friction μ = 0.4-0.9, robot mass ±10%
  - Result: Walking policy adapts to slippery floors and payload variations

#### Contact Dynamics Randomization
- **Why**: Real contacts are complex (soft materials, bouncing, sliding)
- **Example**: Training object pushing
  - Simulation: Vary object restitution (0.1-0.9), contact stiffness (soft to hard)
  - Result: Policy handles diverse contact behaviors (sliding boxes, bouncing balls)

### Practical Implementation Considerations

1. **Start Simple**: Begin with 1-2 categories (e.g., lighting + object pose) before adding all parameters
2. **Validate Ranges**: Randomization ranges should cover *real-world variations* (measure real lighting, friction)
3. **Computational Cost**: More randomization = longer training time (10x variations = 10x training data)
4. **Curriculum Learning**: Start with narrow ranges, gradually increase randomization during training
5. **Domain Randomization ≠ Reality**: Randomization helps, but *validation on real hardware is essential*

### Example: Training a Humanoid to Navigate

**Goal**: Train a humanoid robot to navigate in diverse environments

**Domain Randomization Configuration**:
- **Object**: Randomize obstacle positions (±1m), add 5-15 clutter objects, vary obstacle textures
- **Environmental**: Lighting 200-1500 lux, floor textures (tile/wood/carpet), background walls (painted/brick)
- **Sensor**: Camera noise (ISO 400-800), LiDAR noise (±2cm), 5% point dropouts
- **Physics**: Floor friction μ = 0.5-0.8, robot mass ±5%, joint damping ±10%

**Training**: 10M simulation steps with randomized parameters

**Result**: Policy robust to:
- Different warehouse layouts
- Lighting conditions (morning vs evening)
- Floor surfaces (dry vs slightly wet)
- Sensor noise and occlusions

---

## Usage in Book

This diagram appears in:
- **Chapter 2, Core Concept 3 (Domain Randomization Strategies)**: Reference table when explaining parameter categories and ranges
- **Chapter 5, Core Concept 2 (Domain Randomization for Sim-to-Real)**: Reference when discussing reality gap mitigation techniques

**Context**: Learners need a **practical reference** for implementing domain randomization in Isaac Sim. This table provides concrete parameter ranges and examples, not just abstract concepts.

---

## Related Diagrams

- [synthetic-data-pipeline.md](synthetic-data-pipeline.md): Process workflow showing where domain randomization fits in data generation pipeline
- [reality-gap-mitigation.md](reality-gap-mitigation.md): Decision tree for choosing domain randomization strategies based on task type

---

## Notes for Implementation

- **Comprehensive table** provides actionable parameter ranges (learners often ask "what values should I use?")
- **Four categories** align with Isaac Sim's randomization APIs (object, environment, sensor, physics)
- **Example values** make parameters concrete ("300 lux" vs "dim lighting")
- **Practical considerations** section addresses common pitfalls (too much randomization, computational cost)
- **Real-world example** (humanoid navigation) shows how to combine parameters for a complete training setup
