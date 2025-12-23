# LiDAR Ray-Casting Specification

**Purpose**: Explain how GPU LiDAR sensors generate point clouds through ray-casting in Gazebo

**Diagram Type**: data_flow

## Ray-Casting Principle

```text
                    LIDAR RAY-CASTING OVERVIEW
                    ══════════════════════════

                         LiDAR Sensor
                              │
                              ▼
          ────────────────────●────────────────────
         ╱   ╱   ╱   ╱   ╱   │   ╲   ╲   ╲   ╲   ╲
        ╱   ╱   ╱   ╱   ╱    │    ╲   ╲   ╲   ╲   ╲
       ╱   ╱   ╱   ╱   ╱     │     ╲   ╲   ╲   ╲   ╲
      ╱   ╱   ╱   ╱   ╱      │      ╲   ╲   ╲   ╲   ╲
     ●   ●   ●   ●   ●       │       ●   ●   ●   ●   ●
    Hit Hit Hit Hit Hit      │      Hit Hit Hit Hit Hit
                             │
                        No obstacle
                        (max range)

    Each ray:
    1. Cast from sensor origin
    2. Travel in computed direction
    3. Stop at first collision OR max range
    4. Return distance + intensity
```

## Horizontal and Vertical Scan Pattern

```text
TOP VIEW (Horizontal Scan):
═══════════════════════════

                    0° (forward)
                        │
                        │
           ╱ ╱ ╱ ╱ ╱ ╱ ╱│╲ ╲ ╲ ╲ ╲ ╲ ╲
          ╱ ╱ ╱ ╱ ╱ ╱ ╱ │ ╲ ╲ ╲ ╲ ╲ ╲ ╲
    -90° ─╱─╱─╱─╱─╱─╱─╱──●──╲─╲─╲─╲─╲─╲─╲─ +90°
         (left)     LiDAR      (right)
          ╲ ╲ ╲ ╲ ╲ ╲ ╲ │ ╱ ╱ ╱ ╱ ╱ ╱ ╱
           ╲ ╲ ╲ ╲ ╲ ╲ ╲│╱ ╱ ╱ ╱ ╱ ╱ ╱
                        │
                    180° (backward)

    360° scan = -π to +π radians
    640 samples = 0.5625° angular resolution


SIDE VIEW (Vertical Scan - 3D LiDAR):
════════════════════════════════════

                   +15° (up)
                      ╱
                     ╱  ← Channel 16
                    ╱
                   ╱   ← Channel 8
         ─────────●─────────── 0° (horizontal)
                   ╲   ← Channel 8
                    ╲
                     ╲  ← Channel 1
                      ╲
                   -15° (down)

    16 vertical channels spanning ±15°
    Total: 640 × 16 = 10,240 rays per scan
```

## Ray-Casting Algorithm

```text
FOR each horizontal angle θ in [-π, +π]:
    FOR each vertical angle φ in [-15°, +15°]:

        1. COMPUTE RAY DIRECTION
           ──────────────────────
           direction.x = cos(φ) × cos(θ)
           direction.y = cos(φ) × sin(θ)
           direction.z = sin(φ)

        2. CAST RAY (GPU-accelerated)
           ──────────────────────────
           hit = RayCast(
               origin = sensor_pose,
               direction = direction,
               max_distance = range_max
           )

        3. COMPUTE DISTANCE
           ─────────────────
           IF hit.collision:
               distance = hit.distance
               intensity = MaterialReflectance(hit.surface)
           ELSE:
               distance = range_max  // or NaN
               intensity = 0

        4. ADD NOISE
           ──────────
           distance += GaussianNoise(mean=0, stddev=0.03)

        5. STORE POINT
           ────────────
           point.x = distance × direction.x
           point.y = distance × direction.y
           point.z = distance × direction.z
           point.intensity = intensity

           point_cloud.push(point)

PUBLISH point_cloud to /lidar topic
```

## Resolution Trade-offs

```text
ANGULAR RESOLUTION vs PERFORMANCE
═════════════════════════════════

Configuration      Rays/Scan    GPU Time    Detail Level
───────────────────────────────────────────────────────
LOW (fast):
  H: 180 × V: 1    180          ~0.5 ms     2D scan only
  Angular: 2°

MEDIUM (balanced):
  H: 360 × V: 8    2,880        ~2 ms       Basic 3D
  Angular: 1°

HIGH (detailed):
  H: 640 × V: 16   10,240       ~5 ms       Velodyne VLP-16
  Angular: 0.56°

ULTRA (dense):
  H: 1024 × V: 64  65,536       ~20 ms      High-end LiDAR
  Angular: 0.35°


POINT CLOUD DENSITY VISUALIZATION
═════════════════════════════════

LOW (180 rays):          HIGH (10,240 rays):
      ●   ●   ●               ●●●●●●●●●●●●●●●●●●●
    ●           ●           ●●●●●●●●●●●●●●●●●●●●●
   ●             ●         ●●●●●●●●●●●●●●●●●●●●●●●
  ●               ●       ●●●●●●●●●●●●●●●●●●●●●●●●●
   ●             ●         ●●●●●●●●●●●●●●●●●●●●●●●
    ●           ●           ●●●●●●●●●●●●●●●●●●●●●
      ●   ●   ●               ●●●●●●●●●●●●●●●●●●●

  Sparse, fast             Dense, slow
  SLAM possible            Object detection reliable
```

## Range and Accuracy

```text
RANGE BEHAVIOR
══════════════

Distance        Accuracy         Notes
───────────────────────────────────────────────────
0 - min_range   NO DATA          Too close, self-collision risk
min - 3m        ±1cm             Highest accuracy zone
3m - 10m        ±3cm             Typical indoor range
10m - 30m       ±5cm             Outdoor applications
30m - max       ±10cm+           Accuracy degrades with distance
> max_range     NO DATA          Beyond sensor capability


BEAM DIVERGENCE
═══════════════

At longer distances, the laser beam spreads:

Near (1m):           Far (30m):
    │                    ╲   ╱
    │                     ╲ ╱
    ● ← 5mm spot          ● ← 15cm spot
    │                     ╱ ╲
    │                    ╱   ╲

Result: Far objects return averaged distance
        across larger surface area.
```

## Point Cloud Message Structure

```text
sensor_msgs/msg/PointCloud2
═══════════════════════════

header:
  stamp: {sec: 1234567890, nanosec: 123456789}
  frame_id: "lidar_link"

height: 16          # Vertical channels (1 for 2D)
width: 640          # Points per channel
is_dense: false     # May contain NaN values

fields:             # Data layout
  - name: "x"
    offset: 0
    datatype: FLOAT32
    count: 1
  - name: "y"
    offset: 4
    datatype: FLOAT32
    count: 1
  - name: "z"
    offset: 8
    datatype: FLOAT32
    count: 1
  - name: "intensity"
    offset: 12
    datatype: FLOAT32
    count: 1

point_step: 16      # Bytes per point (4 floats × 4 bytes)
row_step: 10240     # Bytes per row (640 points × 16 bytes)
data: [...]         # Raw binary point data
```

## Usage in Book

- **Referenced in**: Chapter 4 (Core Concept 1: LiDAR Simulation)
- **Purpose**: Explain the mechanics of LiDAR simulation for students new to 3D sensing
- **Learning Goal**: Understand how ray-casting produces point clouds and configure resolution appropriately

## Key Takeaways

1. **Ray-casting**: Each LiDAR point comes from casting a ray and measuring collision distance
2. **Resolution = rays**: More horizontal/vertical samples = denser point cloud = slower simulation
3. **GPU acceleration**: Modern Gazebo uses GPU for parallel ray-casting (10,000+ rays in milliseconds)
4. **Noise is essential**: Add Gaussian noise (±3cm typical) for realistic training data
5. **Range limits matter**: Set min/max range to match physical sensor specs
6. **Trade-off exists**: Balance point density against simulation speed for your use case
