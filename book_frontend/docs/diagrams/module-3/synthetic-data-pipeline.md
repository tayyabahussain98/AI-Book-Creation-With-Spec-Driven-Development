# Synthetic Data Pipeline

**Purpose**: Illustrate the end-to-end pipeline for generating synthetic perception datasets in Isaac Sim, from scene configuration through domain randomization to labeled output generation.

**Context**: Synthetic data pipelines automate the generation of large-scale labeled datasets for perception model training, eliminating manual annotation costs while enabling controlled variation.

---

## ASCII Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    SYNTHETIC DATA PIPELINE                       │
└─────────────────────────────────────────────────────────────────┘

1. SCENE CONFIGURATION
   ┌──────────────────┐
   │  3D Asset        │
   │  Library         │
   │  • Robots        │
   │  • Objects       │
   │  • Environments  │
   └────────┬─────────┘
            │
            v
   ┌──────────────────┐
   │  Isaac Sim       │
   │  Scene Setup     │
   │  • Load USD      │
   │  • Place objects │
   │  • Add sensors   │
   └────────┬─────────┘
            │
            v

2. DOMAIN RANDOMIZATION
   ┌──────────────────────────────────────────────┐
   │         Randomization Parameters              │
   ├──────────────────────────────────────────────┤
   │ Lighting    │ Camera       │ Textures        │
   │ • Intensity │ • FOV        │ • Albedo        │
   │ • Position  │ • Exposure   │ • Roughness     │
   │ • Color     │ • Lens dist. │ • Patterns      │
   ├──────────────────────────────────────────────┤
   │ Physics     │ Object Pose  │ Clutter         │
   │ • Gravity   │ • Position   │ • Num objects   │
   │ • Friction  │ • Rotation   │ • Placement     │
   └────────┬─────────────────────────────────────┘
            │
            v

3. RENDERING & DATA CAPTURE
   ┌──────────────────────────────────────────────┐
   │      Isaac Sim Replicator (RTX)              │
   ├──────────────────────────────────────────────┤
   │  Render Passes (per frame):                  │
   │  ┌─────────┐  ┌─────────┐  ┌─────────┐      │
   │  │   RGB   │  │ Depth   │  │ Seg.    │      │
   │  │  Image  │  │  Map    │  │  Mask   │      │
   │  └─────────┘  └─────────┘  └─────────┘      │
   │  ┌─────────┐  ┌─────────┐  ┌─────────┐      │
   │  │ Normals │  │ Bbox 2D │  │ Bbox 3D │      │
   │  └─────────┘  └─────────┘  └─────────┘      │
   └────────┬─────────────────────────────────────┘
            │
            v

4. AUTOMATED ANNOTATION
   ┌──────────────────────────────────────────────┐
   │       Ground Truth Extraction                │
   ├──────────────────────────────────────────────┤
   │  • Object IDs → Semantic labels              │
   │  • 3D positions → 2D bounding boxes          │
   │  • Instance masks → Segmentation maps        │
   │  • Pose matrices → 6DoF annotations          │
   └────────┬─────────────────────────────────────┘
            │
            v

5. DATASET OUTPUT
   ┌──────────────────────────────────────────────┐
   │         Structured Dataset                   │
   ├──────────────────────────────────────────────┤
   │  Format: COCO, KITTI, Custom                 │
   │  ┌────────────────────────────────┐          │
   │  │  images/                       │          │
   │  │    ├─ frame_00001.png          │          │
   │  │    └─ frame_00002.png          │          │
   │  │  labels/                       │          │
   │  │    ├─ frame_00001.json         │          │
   │  │    └─ frame_00002.json         │          │
   │  │  metadata.yaml                 │          │
   │  └────────────────────────────────┘          │
   └────────┬─────────────────────────────────────┘
            │
            v
   ┌──────────────────┐
   │  Perception      │
   │  Model Training  │
   │  (YOLOv8, etc.)  │
   └──────────────────┘
```

---

## Component Explanations

### 1. Scene Configuration
**3D Asset Library**: Pre-built USD (Universal Scene Description) assets from NVIDIA's library or custom models. Isaac Sim uses USD as its native format for scene composition.

**Scene Setup**: Programmatic scene construction using Omniverse Kit APIs. Sensors (RGB cameras, depth, lidar) are attached to robot or fixed viewpoints.

### 2. Domain Randomization
**Purpose**: Increase dataset diversity to improve model generalization by varying conditions that don't affect the core task.

**Lighting**: Randomize intensity (500-3000 lumens), position, and color temperature (2700-6500K) to simulate different times of day and indoor/outdoor environments.

**Camera**: Vary field-of-view (60-90°), exposure, and lens distortion to match real-world camera variations.

**Textures**: Randomize object surface properties (albedo 0.3-0.9, roughness 0.1-0.8) to prevent overfitting to specific appearances.

**Physics**: Vary gravity, friction, and object dynamics to simulate different environmental conditions.

**Object Pose & Clutter**: Randomize target object positions/rotations and add distractor objects to increase scene complexity.

### 3. Rendering & Data Capture
**Isaac Sim Replicator**: RTX-accelerated rendering pipeline that generates multiple data modalities per frame:
- **RGB**: Standard color images for training detection/classification models
- **Depth**: Per-pixel distance for 3D reconstruction and geometric reasoning
- **Segmentation**: Instance and semantic masks for precise object boundaries
- **Normals**: Surface orientation for geometry-aware models
- **Bounding Boxes**: 2D/3D boxes for object detection ground truth

**Performance**: RTX GPUs enable real-time rendering at 30-60 FPS, generating thousands of frames per hour.

### 4. Automated Annotation
**Ground Truth Extraction**: Isaac Sim provides perfect labels "for free" by querying the simulation state:
- Object IDs map directly to semantic labels (no manual annotation)
- 3D positions project to pixel-perfect 2D bounding boxes
- Instance masks extracted from render passes
- 6DoF poses (position + orientation) captured from physics engine

**Advantage**: Zero annotation cost, no human labeling errors, 100% label consistency.

### 5. Dataset Output
**Format Support**: Export to standard formats (COCO, KITTI, Pascal VOC) or custom schemas.

**Structure**:
- `images/`: Rendered RGB frames
- `labels/`: Per-frame annotations (JSON, XML, or TXT)
- `metadata.yaml`: Dataset statistics, randomization config, sensor calibration

**Scale**: Pipelines can generate 10K-1M+ labeled frames overnight, enabling large-scale perception training.

---

## Key Insights

1. **Automation is Critical**: Manual annotation of 10K images would take weeks; synthetic pipelines generate this in hours.

2. **Randomization ≠ Realism**: The goal is **diversity**, not photorealism. Models trained on varied synthetic data often outperform those trained on limited real data.

3. **Iteration Speed**: Changing a single parameter (e.g., lighting) and regenerating the dataset takes minutes, enabling rapid experimentation.

4. **Sim-to-Real Gap**: Domain randomization helps, but models may still need real-world fine-tuning (covered in Chapter 5).

---

## Real-World Application

**Use Case**: Training a robotic arm to grasp objects
- Generate 50K images with 100 object types
- Randomize lighting (warehouse → home), backgrounds, object poses
- Train YOLOv8 detection model
- Fine-tune on 500 real images → 95%+ grasp success rate

**Cost**: $0 annotation cost vs. $5K+ for manual labeling
**Time**: 2 days generation + training vs. 2 weeks manual annotation
