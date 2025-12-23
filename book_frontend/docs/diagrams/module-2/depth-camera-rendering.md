# Depth Camera Rendering Specification

**Purpose**: Explain how depth cameras generate RGB-D images through GPU rendering in Gazebo

**Diagram Type**: data_flow

## Depth Image Generation Pipeline

```text
┌─────────────────────────────────────────────────────────────────┐
│                    DEPTH CAMERA PIPELINE                        │
└─────────────────────────────────────────────────────────────────┘

Step 1: SCENE RENDERING (GPU)
────────────────────────────────────────────────────────────────────

    Camera Position                    Scene Geometry
          │                                  │
          ▼                                  ▼
    ┌───────────┐                    ┌─────────────┐
    │ View      │                    │ World       │
    │ Matrix    │                    │ Meshes      │
    └─────┬─────┘                    └──────┬──────┘
          │                                 │
          └──────────────┬──────────────────┘
                         │
                         ▼
               ┌─────────────────┐
               │ Vertex Shader   │
               │ (Transform to   │
               │  view space)    │
               └────────┬────────┘
                        │
                        ▼
               ┌─────────────────┐
               │ Rasterizer      │
               │ (Project to     │
               │  image plane)   │
               └────────┬────────┘
                        │
                        ▼
               ┌─────────────────┐
               │ Fragment Shader │
               │ (Compute depth  │
               │  per pixel)     │
               └────────┬────────┘
                        │
                        ▼
    ┌─────────────────────────────────────────────┐
    │              DEPTH BUFFER                    │
    │  (640 × 480 float32 values in meters)       │
    │                                             │
    │  ┌─────────────────────────────────────┐   │
    │  │ 2.5  2.5  2.6  3.0  3.2  4.0  inf  │   │
    │  │ 2.4  2.5  2.5  2.8  3.1  3.8  inf  │   │
    │  │ 2.3  2.4  2.5  2.7  3.0  3.5  5.0  │   │
    │  │ ...                                 │   │
    │  └─────────────────────────────────────┘   │
    │                                             │
    │  Values: Distance from camera to surface   │
    │  Units: Meters (float32)                   │
    │  Invalid: inf or NaN (no hit / out of range)│
    └─────────────────────────────────────────────┘
```

## Camera Intrinsic Parameters

```text
PINHOLE CAMERA MODEL
════════════════════

                    Image Plane
                   ┌───────────────────────┐
                   │                       │
                   │    ●──────────────────┼──► u (pixels)
                   │    │ (cx, cy)         │
                   │    │ principal        │
                   │    │ point            │
                   │    ▼                  │
                   │    v (pixels)         │
                   └───────────────────────┘
                            ╲
                             ╲ focal length f
                              ╲
                               ●  Camera Center (origin)


INTRINSIC MATRIX K:
═══════════════════

    ┌            ┐     ┌         ┐
    │ fx  0  cx  │     │ 554.26  0     320 │
K = │ 0  fy  cy  │  =  │ 0      554.26 240 │  (for 640×480, 60° FOV)
    │ 0   0   1  │     │ 0       0      1  │
    └            ┘     └                   ┘

Where:
  fx, fy = Focal length in pixels
  cx, cy = Principal point (usually image center)

CALCULATING FOCAL LENGTH:
═════════════════════════

  fx = fy = width / (2 × tan(hfov / 2))

  For 640×480 with 60° horizontal FOV:
  fx = 640 / (2 × tan(30°))
  fx = 640 / (2 × 0.577)
  fx = 640 / 1.155
  fx ≈ 554.26 pixels
```

## 3D Point Reconstruction

```text
DEPTH TO 3D POINT CONVERSION
════════════════════════════

Given:
  • Pixel coordinates: (u, v)
  • Depth value: d (meters)
  • Camera intrinsics: fx, fy, cx, cy

Compute 3D point in camera frame:

  X = (u - cx) × d / fx
  Y = (v - cy) × d / fy
  Z = d

Example:
  Pixel (400, 300) with depth 2.5m:

  X = (400 - 320) × 2.5 / 554.26 = 0.361 m
  Y = (300 - 240) × 2.5 / 554.26 = 0.271 m
  Z = 2.5 m

  3D Point: (0.361, 0.271, 2.5) in camera frame


FULL IMAGE TO POINT CLOUD:
══════════════════════════

FOR each pixel (u, v) in depth_image:
    d = depth_image[v, u]

    IF d > 0 AND d < max_range:
        point.x = (u - cx) × d / fx
        point.y = (v - cy) × d / fy
        point.z = d

        IF rgb_image available:
            point.r = rgb_image[v, u].r
            point.g = rgb_image[v, u].g
            point.b = rgb_image[v, u].b

        point_cloud.append(point)
```

## Depth Encoding Formats

```text
DEPTH IMAGE ENCODING COMPARISON
═══════════════════════════════

┌─────────────────┬─────────────┬─────────────┬───────────────────┐
│ Format          │ Bits/Pixel  │ Range       │ Precision         │
├─────────────────┼─────────────┼─────────────┼───────────────────┤
│ 32FC1 (float)   │ 32          │ 0 - ∞       │ ~7 significant    │
│                 │             │             │ digits            │
├─────────────────┼─────────────┼─────────────┼───────────────────┤
│ 16UC1 (uint16)  │ 16          │ 0 - 65.535m │ 1mm resolution    │
│                 │             │ (in mm)     │                   │
├─────────────────┼─────────────┼─────────────┼───────────────────┤
│ MONO16          │ 16          │ 0 - 65.535m │ 1mm resolution    │
│                 │             │ (in mm)     │                   │
└─────────────────┴─────────────┴─────────────┴───────────────────┘

32FC1 (Recommended):
  • depth_image[y, x] = 2.547  (meters, float)
  • NaN for invalid pixels
  • Best precision, standard for ROS 2

16UC1 (Legacy):
  • depth_image[y, x] = 2547  (millimeters, uint16)
  • 0 for invalid pixels
  • Smaller bandwidth, some precision loss
```

## RGB-D Alignment

```text
RGB AND DEPTH ALIGNMENT
═══════════════════════

Real RGB-D cameras have physically separated sensors:

    ┌─────┐     ┌─────┐
    │ RGB │     │Depth│
    │Sensor│     │Sensor│
    └──┬──┘     └──┬──┘
       │  offset   │
       │◄────────►│
       │  ~50mm    │

Result: RGB and Depth images see slightly different views.


ALIGNMENT OPTIONS:
══════════════════

1. UNALIGNED (raw):
   • RGB: 640×480 from RGB camera viewpoint
   • Depth: 640×480 from Depth camera viewpoint
   • Must account for offset in fusion

2. ALIGNED TO DEPTH:
   • RGB warped to depth camera viewpoint
   • 1:1 pixel correspondence
   • Some RGB pixels may be occluded

3. ALIGNED TO RGB:
   • Depth warped to RGB camera viewpoint
   • 1:1 pixel correspondence
   • Some depth pixels may be invalid


GAZEBO SIMULATION:
══════════════════

Gazebo simulates IDEAL alignment by default:
  • RGB and Depth share same virtual camera
  • No offset, no occlusion
  • Perfect 1:1 pixel correspondence

For realistic simulation, add offset:
  <link name="rgb_camera_link">
    <pose relative_to="depth_camera_link">0 0.05 0 0 0 0</pose>
    <!-- 50mm offset in Y -->
  </link>
```

## Noise Characteristics

```text
DEPTH CAMERA NOISE PATTERNS
═══════════════════════════

1. DISTANCE-DEPENDENT NOISE
───────────────────────────

   Noise increases quadratically with distance:

   σ(d) = σ₀ × (d / d₀)²

   Where:
     σ₀ = base noise at reference distance
     d₀ = reference distance (e.g., 1m)
     d  = actual distance

   Example (Intel RealSense D435):
     At 1m: σ ≈ 2mm
     At 3m: σ ≈ 18mm (9× worse)
     At 5m: σ ≈ 50mm (25× worse)


2. EDGE NOISE (Flying Pixels)
─────────────────────────────

   At depth discontinuities, invalid interpolation:

   ████████                    ████████
   ████████                    ███○○███  ← Flying pixels
   ████████      ────►         ██○○○○██    (interpolated)
   ████████                    █○○○○○○█
           ────                        ────
       Object    Background        With Noise


3. SURFACE-DEPENDENT NOISE
──────────────────────────

   Surface Type        Noise Level    Notes
   ─────────────────────────────────────────
   Matte diffuse       LOW            Ideal surface
   Glossy/specular     HIGH           Reflections
   Transparent         INVALID        No return
   Black (absorbing)   HIGH           Weak signal
   Patterned           MEDIUM         Good for stereo


GAZEBO NOISE MODEL:
═══════════════════

<sensor type="depth_camera">
  <camera>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.005</stddev>  <!-- 5mm base noise -->
    </noise>
  </camera>
</sensor>

Note: Gazebo applies uniform Gaussian noise.
For realistic sim, implement custom distance-dependent
noise in a system plugin.
```

## Usage in Book

- **Referenced in**: Chapter 4 (Core Concept 2: Depth Camera Simulation)
- **Purpose**: Explain depth imaging fundamentals for perception pipeline development
- **Learning Goal**: Configure depth cameras correctly and understand their limitations

## Key Takeaways

1. **GPU rendering**: Depth buffer computed via standard graphics pipeline rasterization
2. **Intrinsic matrix**: fx, fy, cx, cy define 2D-to-3D projection relationship
3. **32FC1 encoding**: Float meters recommended; NaN for invalid pixels
4. **Alignment**: Gazebo uses ideal alignment; real sensors have physical offset
5. **Noise increases with distance**: Quadratic degradation requires distance-aware processing
6. **Edge artifacts**: Depth discontinuities produce "flying pixels"—filter in perception pipeline
