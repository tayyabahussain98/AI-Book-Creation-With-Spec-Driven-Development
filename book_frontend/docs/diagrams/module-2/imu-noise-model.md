# IMU Noise Model Specification

**Purpose**: Explain IMU noise characteristics and how to configure realistic noise models in Gazebo simulation

**Diagram Type**: data_flow

## IMU Sensor Components

```text
┌─────────────────────────────────────────────────────────────────┐
│                    IMU SENSOR ARCHITECTURE                      │
└─────────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────┐
│                     IMU PACKAGE                                │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │                    MEMS CHIP                             │  │
│  │                                                          │  │
│  │  ┌────────────────┐    ┌────────────────┐              │  │
│  │  │ ACCELEROMETER  │    │   GYROSCOPE    │              │  │
│  │  │                │    │                │              │  │
│  │  │ Measures:      │    │ Measures:      │              │  │
│  │  │ • X accel      │    │ • X angular vel│              │  │
│  │  │ • Y accel      │    │ • Y angular vel│              │  │
│  │  │ • Z accel      │    │ • Z angular vel│              │  │
│  │  │                │    │                │              │  │
│  │  │ Units: m/s²    │    │ Units: rad/s   │              │  │
│  │  └────────────────┘    └────────────────┘              │  │
│  │                                                          │  │
│  │  ┌────────────────┐    ┌────────────────┐              │  │
│  │  │ MAGNETOMETER   │    │  TEMPERATURE   │              │  │
│  │  │ (optional)     │    │  SENSOR        │              │  │
│  │  │                │    │                │              │  │
│  │  │ Measures:      │    │ Compensates    │              │  │
│  │  │ • Magnetic     │    │ for thermal    │              │  │
│  │  │   field        │    │ drift          │              │  │
│  │  │                │    │                │              │  │
│  │  │ Units: μT      │    │ Units: °C      │              │  │
│  │  └────────────────┘    └────────────────┘              │  │
│  │                                                          │  │
│  └─────────────────────────────────────────────────────────┘  │
│                              │                                 │
│                              ▼                                 │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │              SENSOR FUSION / FILTERING                   │  │
│  │  • Complementary filter                                  │  │
│  │  • Extended Kalman Filter (EKF)                          │  │
│  │  • Mahony/Madgwick filter                                │  │
│  │                                                          │  │
│  │  Output: Orientation (quaternion or Euler angles)        │  │
│  └─────────────────────────────────────────────────────────┘  │
│                              │                                 │
└──────────────────────────────┼─────────────────────────────────┘
                               ▼
                    sensor_msgs/Imu message
```

## Noise Types

```text
IMU NOISE CHARACTERISTICS
═════════════════════════

1. WHITE NOISE (Random Walk)
────────────────────────────

   Each measurement has random Gaussian error:

   measured = true_value + N(0, σ)

   Time series:
   ───●─●──●───●──●─●───●──●───●──●──●───●── True value
      ○ ○  ○   ○  ○ ○   ○  ○   ○  ○  ○   ○   Measured (noisy)

   σ (standard deviation) from datasheet:
   • Accelerometer: 0.01 - 0.1 m/s²
   • Gyroscope: 0.001 - 0.01 rad/s


2. BIAS (Constant Offset)
─────────────────────────

   Systematic error that persists across measurements:

   measured = true_value + bias + N(0, σ)

   ────────────────────────────────────────── True value
                bias offset
   ═══════════════════════════════════════════ Measured (biased)

   Bias sources:
   • Manufacturing imperfection
   • Temperature variation
   • Mechanical stress

   Typical bias:
   • Accelerometer: 0.001 - 0.01 m/s²
   • Gyroscope: 0.0001 - 0.001 rad/s


3. BIAS DRIFT (Random Walk in Bias)
───────────────────────────────────

   Bias changes slowly over time:

   bias(t) = bias(t-1) + N(0, σ_bias)

   Time evolution:
   ───────────────────────────────── True value
   ═══════════════════════════════   Initial bias
        ══════════════════════       Bias drifts up
             ═══════════════════     Continues drifting

   This is why IMU orientation drifts over time!


4. SCALE FACTOR ERROR
─────────────────────

   Gain error in measurement:

   measured = true_value × (1 + scale_error) + bias + noise

   If scale_error = 0.02 (2%):
   • True acceleration: 10 m/s²
   • Measured: 10.2 m/s² (without noise)
```

## Noise Model Mathematics

```text
COMPLETE IMU NOISE MODEL
════════════════════════

For accelerometer axis (same structure for gyro):

┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  a_measured = a_true × (1 + s) + b + n                     │
│                                                             │
│  Where:                                                     │
│    a_true    = True acceleration                           │
│    s         = Scale factor error (constant)               │
│    b         = Bias (slowly varying)                       │
│    n         = White noise ~ N(0, σ)                       │
│                                                             │
│  Bias evolution:                                            │
│    b(t) = b(t-1) + w    where w ~ N(0, σ_bias × √dt)      │
│                                                             │
└─────────────────────────────────────────────────────────────┘


ALLAN VARIANCE PARAMETERS
═════════════════════════

Sensor datasheets specify noise using Allan variance:

┌─────────────────────────────────────────────────────────────┐
│ Parameter              │ Symbol │ Units        │ Meaning    │
├────────────────────────┼────────┼──────────────┼────────────┤
│ Angle Random Walk      │ ARW    │ °/√hr        │ Gyro white │
│                        │        │ (rad/s/√Hz)  │ noise      │
├────────────────────────┼────────┼──────────────┼────────────┤
│ Velocity Random Walk   │ VRW    │ m/s/√hr      │ Accel white│
│                        │        │ (m/s²/√Hz)   │ noise      │
├────────────────────────┼────────┼──────────────┼────────────┤
│ Bias Instability       │ BI     │ °/hr (gyro)  │ Bias drift │
│                        │        │ μg (accel)   │ rate       │
└─────────────────────────────────────────────────────────────┘


CONVERTING DATASHEET TO GAZEBO
══════════════════════════════

Datasheet gives: ARW = 0.3 °/√hr

Convert to rad/s:
  σ_gyro = ARW × (π/180) × (1/60) / √Hz_rate
  σ_gyro = 0.3 × 0.01745 × 0.01667 / √100
  σ_gyro ≈ 0.000087 rad/s

For Gazebo <stddev>:
  Use σ_gyro directly as stddev value
```

## Gazebo IMU Configuration

```xml
<!--
  COMPLETE IMU NOISE CONFIGURATION
  ================================

  This configuration matches a typical consumer-grade IMU
  like the InvenSense MPU-6050 or Bosch BMI088.
-->
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <topic>imu</topic>

  <imu>
    <!-- Accelerometer noise (per-axis) -->
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <!--
            mean: Systematic bias (should be near 0 for calibrated sensor)
            stddev: Random noise standard deviation (m/s²)

            Consumer IMU: stddev ≈ 0.05 - 0.1 m/s²
            Industrial:   stddev ≈ 0.01 - 0.02 m/s²
            Navigation:   stddev ≈ 0.001 - 0.005 m/s²
          -->
          <mean>0.0</mean>
          <stddev>0.05</stddev>

          <!--
            bias_mean: Initial bias offset
            bias_stddev: How much bias drifts per timestep

            These cause long-term drift in position estimation.
          -->
          <bias_mean>0.0001</bias_mean>
          <bias_stddev>0.0001</bias_stddev>
        </noise>
      </x>
      <!-- Repeat for Y and Z axes -->
    </linear_acceleration>

    <!-- Gyroscope noise (per-axis) -->
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <!--
            Consumer IMU: stddev ≈ 0.01 - 0.02 rad/s
            Industrial:   stddev ≈ 0.001 - 0.005 rad/s
            Navigation:   stddev ≈ 0.0001 - 0.0005 rad/s
          -->
          <mean>0.0</mean>
          <stddev>0.01</stddev>

          <!--
            Gyro bias drift is THE dominant error source.
            After integration, small bias becomes large angle error.
          -->
          <bias_mean>0.0001</bias_mean>
          <bias_stddev>0.0001</bias_stddev>
        </noise>
      </x>
      <!-- Repeat for Y and Z axes -->
    </angular_velocity>

    <enable_orientation>true</enable_orientation>
  </imu>
</sensor>
```

## Error Accumulation Over Time

```text
ORIENTATION DRIFT FROM GYRO BIAS
════════════════════════════════

Gyro bias of 0.001 rad/s (0.057 °/s):

Time        Accumulated Error
──────────────────────────────
0 sec       0°
10 sec      0.57°
60 sec      3.4°
5 min       17°
1 hour      206° (completely wrong!)

This is why pure IMU integration fails for long-term navigation.
Must fuse with other sensors (GPS, vision, etc.)


POSITION DRIFT FROM ACCEL BIAS
══════════════════════════════

Accel bias of 0.01 m/s² (1 mg):

Position error = 0.5 × bias × t²

Time        Position Error
──────────────────────────────
1 sec       0.005 m
10 sec      0.5 m
60 sec      18 m
5 min       450 m (!!!)

Double integration amplifies errors dramatically.


GAZEBO vs REAL WORLD
════════════════════

                    Gazebo Simulation    Real IMU
────────────────────────────────────────────────────
White noise         ✓ Modeled           ✓ Present
Bias                ✓ Modeled           ✓ Present
Bias drift          ✓ Modeled           ✓ Present
Temperature effects ✗ Not modeled       ✓ Significant
Vibration effects   ✗ Limited           ✓ Very significant
Cross-axis coupling ✗ Not modeled       ✓ Present
```

## Usage in Book

- **Referenced in**: Chapter 4 (Core Concept 3: IMU Simulation)
- **Purpose**: Explain why IMU measurements drift and how to configure realistic noise
- **Learning Goal**: Configure IMU noise appropriately for training robust state estimation

## Key Takeaways

1. **Three noise components**: White noise (random), bias (constant offset), bias drift (random walk)
2. **Gyro drift dominates**: Small bias → large orientation error over time due to integration
3. **Accel error squared**: Position error grows with t² due to double integration
4. **Use datasheet values**: Convert Allan variance parameters to Gazebo stddev
5. **Fusion is mandatory**: IMU alone drifts; must combine with GPS, vision, or other sensors
6. **Domain randomization**: Vary noise parameters during training for robust algorithms
