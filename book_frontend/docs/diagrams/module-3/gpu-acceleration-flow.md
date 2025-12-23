# GPU Acceleration Flow

**Purpose**: Illustrate the data flow through a GPU-accelerated perception pipeline, from raw sensor data to perception outputs, highlighting where GPU acceleration provides performance benefits.

**Context**: Isaac ROS packages offload compute-intensive perception tasks to NVIDIA GPUs, achieving 5-50x speedups over CPU-only implementations for image processing, DNN inference, and point cloud operations.

---

## ASCII Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│              GPU-ACCELERATED PERCEPTION PIPELINE                 │
└─────────────────────────────────────────────────────────────────┘

1. SENSOR DATA ACQUISITION (CPU)
   ┌──────────────────┐
   │  Camera Driver   │
   │  (ROS 2 Node)    │
   │  • Image capture │
   │  • Timestamp     │
   │  • Metadata      │
   └────────┬─────────┘
            │ sensor_msgs/Image
            │ (CPU → GPU via DMA)
            v
   ┌──────────────────┐
   │  GPU Memory      │
   │  Transfer (DMA)  │
   │  Latency: ~1ms   │
   └────────┬─────────┘
            │
            v

2. GPU PREPROCESSING (isaac_ros_image_proc)
   ┌────────────────────────────────────────────────┐
   │         Image Processing (GPU)                 │
   ├────────────────────────────────────────────────┤
   │  ┌──────────┐  ┌──────────┐  ┌──────────┐     │
   │  │ Resize   │→ │ Normalize│→ │ Convert  │     │
   │  │ 1920x1080│  │ [0,1]    │  │ RGB→BGR  │     │
   │  │ → 640x640│  │ range    │  │ format   │     │
   │  └──────────┘  └──────────┘  └──────────┘     │
   ├────────────────────────────────────────────────┤
   │  CUDA Operations:                              │
   │  • nppiResize (CUDA NPP library)               │
   │  • Tensor operations (cuBLAS)                  │
   │  • Color space conversion (NPP)                │
   ├────────────────────────────────────────────────┤
   │  Performance: ~2-5ms on GPU vs 20-50ms on CPU  │
   │  Speedup: 10x                                  │
   └────────┬───────────────────────────────────────┘
            │ Preprocessed tensor (GPU memory)
            v

3. DNN INFERENCE (isaac_ros_dnn_inference)
   ┌────────────────────────────────────────────────┐
   │       TensorRT Model Inference (GPU)           │
   ├────────────────────────────────────────────────┤
   │  Model: YOLOv8 / ResNet / EfficientNet         │
   │  ┌──────────────────────────────────┐          │
   │  │   TensorRT Engine                │          │
   │  │   • Optimized for RTX Tensor     │          │
   │  │   • FP16/INT8 precision          │          │
   │  │   • Fused operations             │          │
   │  │   • Kernel auto-tuning           │          │
   │  └────────────┬─────────────────────┘          │
   │               │                                 │
   │               v                                 │
   │  ┌──────────────────────────────────┐          │
   │  │   Detection Output (GPU)         │          │
   │  │   • Bounding boxes [N, 4]        │          │
   │  │   • Class scores [N, C]          │          │
   │  │   • Confidence scores [N]        │          │
   │  └──────────────────────────────────┘          │
   ├────────────────────────────────────────────────┤
   │  Tensor Cores: 4x speedup for FP16 matmul      │
   │  Performance: 5-10ms on GPU vs 100-200ms CPU   │
   │  Speedup: 20-40x                               │
   └────────┬───────────────────────────────────────┘
            │ Raw detections (GPU memory)
            v

4. POST-PROCESSING (GPU)
   ┌────────────────────────────────────────────────┐
   │        Detection Post-Processing (GPU)         │
   ├────────────────────────────────────────────────┤
   │  ┌──────────────┐  ┌──────────────┐            │
   │  │ Confidence   │→ │ Non-Maximum  │            │
   │  │ Thresholding │  │ Suppression  │            │
   │  │ (score > τ)  │  │ (NMS, IoU)   │            │
   │  └──────────────┘  └──────────────┘            │
   ├────────────────────────────────────────────────┤
   │  CUDA Operations:                              │
   │  • Parallel filtering (thrust library)         │
   │  • NMS on GPU (cuNMS)                          │
   │  • Coordinate transformations                  │
   ├────────────────────────────────────────────────┤
   │  Performance: ~1-2ms on GPU vs 5-10ms on CPU   │
   │  Speedup: 5x                                   │
   └────────┬───────────────────────────────────────┘
            │ Filtered detections (GPU memory)
            │ (GPU → CPU via DMA)
            v
   ┌──────────────────┐
   │  CPU Memory      │
   │  Transfer (DMA)  │
   │  Latency: ~1ms   │
   └────────┬─────────┘
            │
            v

5. PERCEPTION OUTPUT (CPU)
   ┌──────────────────────────────────────┐
   │   ROS 2 Message Publishing           │
   │   (vision_msgs/Detection2DArray)     │
   ├──────────────────────────────────────┤
   │  For each detection:                 │
   │  • Class ID + label                  │
   │  • Bounding box (x, y, w, h)         │
   │  • Confidence score                  │
   │  • Timestamp                         │
   └────────┬─────────────────────────────┘
            │ /detections topic
            v
   ┌──────────────────┐
   │  Downstream      │
   │  ROS 2 Nodes     │
   │  • Navigation    │
   │  • Manipulation  │
   │  • Visualization │
   └──────────────────┘

═══════════════════════════════════════════════════════════════════
LATENCY BREAKDOWN (640x640 input, YOLOv8s model)
═══════════════════════════════════════════════════════════════════

┌─────────────────────┬──────────┬──────────┬──────────┐
│ Pipeline Stage      │ GPU (ms) │ CPU (ms) │ Speedup  │
├─────────────────────┼──────────┼──────────┼──────────┤
│ CPU→GPU Transfer    │    1     │    -     │    -     │
│ Preprocessing       │    3     │   30     │   10x    │
│ DNN Inference       │    8     │  150     │   19x    │
│ Post-Processing     │    2     │    8     │    4x    │
│ GPU→CPU Transfer    │    1     │    -     │    -     │
├─────────────────────┼──────────┼──────────┼──────────┤
│ TOTAL LATENCY       │   15ms   │  188ms   │  12.5x   │
├─────────────────────┼──────────┼──────────┼──────────┤
│ THROUGHPUT (FPS)    │   67     │    5     │  13.4x   │
└─────────────────────┴──────────┴──────────┴──────────┘

Note: GPU times measured on RTX 4090, CPU times on Intel i9-13900K
```

---

## Component Explanations

### 1. Sensor Data Acquisition
**Camera Driver**: Standard ROS 2 camera driver (e.g., `usb_cam`, `realsense_ros`) publishes raw RGB images to `/camera/image_raw` topic.

**DMA Transfer**: Direct Memory Access (DMA) bypasses the CPU to transfer image data directly from system RAM to GPU VRAM. This is handled by CUDA APIs and incurs minimal latency (~1ms).

### 2. GPU Preprocessing (isaac_ros_image_proc)
**Purpose**: Prepare raw sensor data for DNN inference.

**Operations**:
- **Resize**: Scale images to model input size (e.g., 1920x1080 → 640x640) using CUDA NPP (NVIDIA Performance Primitives) library
- **Normalize**: Convert pixel values from [0, 255] to [0.0, 1.0] range using CUDA kernels
- **Color Conversion**: RGB ↔ BGR format conversion if needed by the model

**Why GPU Matters**: Image resize/normalize are highly parallel operations. A 1920x1080 image has 2 million pixels—GPU processes all pixels in parallel vs CPU's sequential/SIMD processing.

**Performance**: 10x speedup (3ms vs 30ms) for typical preprocessing pipeline.

### 3. DNN Inference (isaac_ros_dnn_inference)
**TensorRT Engine**: NVIDIA's inference optimizer that converts trained models (PyTorch, TensorFlow) into optimized GPU engines.

**Optimizations**:
- **Precision Tuning**: FP16 (half precision) uses Tensor Cores for 4x speedup over FP32
- **Layer Fusion**: Combines multiple operations (e.g., Conv + BatchNorm + ReLU) into single GPU kernel
- **Kernel Auto-Tuning**: Selects fastest CUDA kernel for each layer on the target GPU
- **Dynamic Tensor Memory**: Reuses GPU memory across layers to reduce overhead

**Tensor Cores**: Specialized hardware in RTX GPUs for matrix multiplication (the core operation in DNNs). FP16 matmul runs at 4x the throughput of FP32 on Tensor Cores.

**Performance**: 20-40x speedup for YOLOv8 inference (8ms vs 150ms). Larger models see even bigger gains.

### 4. Post-Processing (GPU)
**Confidence Thresholding**: Filter detections with score < τ (e.g., 0.5) using parallel GPU filtering via Thrust library.

**Non-Maximum Suppression (NMS)**: Remove duplicate detections for the same object. Traditional NMS is sequential on CPU; GPU implementations (cuNMS) parallelize across detection boxes.

**Coordinate Transformation**: Convert normalized bounding box coordinates [0,1] to pixel coordinates [0, width/height].

**Performance**: 5x speedup (2ms vs 8ms) for NMS on 100+ detections.

### 5. Perception Output
**DMA Transfer**: Transfer final detection results (typically &lt;10 KB) from GPU back to CPU memory.

**ROS 2 Publishing**: Publish `vision_msgs/Detection2DArray` with bounding boxes, class labels, and confidence scores for downstream nodes.

---

## Key Insights

### 1. When GPU Acceleration Provides Maximum Benefit
- **High-Resolution Sensors**: 1920x1080+ images → 10x preprocessing speedup
- **Real-Time DNN Inference**: YOLOv8, ResNet, SegFormer → 20-40x speedup
- **High Throughput**: Need to process 30+ FPS for real-time perception
- **Complex Models**: Large DNNs (50M+ parameters) see bigger GPU gains

### 2. When CPU is Sufficient
- **Low-Resolution Sensors**: 320x240 images process fast enough on CPU
- **Infrequent Processing**: 1-5 FPS workloads don't justify GPU overhead
- **Simple Models**: Tiny models (1-5M parameters) may not saturate GPU
- **Non-Latency-Critical**: Offline batch processing where throughput isn't critical

### 3. GPU Memory Management
**Zero-Copy Operations**: Isaac ROS keeps data on GPU throughout the pipeline, avoiding expensive CPU ↔ GPU transfers between stages.

**Pinned Memory**: Pre-allocate GPU-accessible system RAM for faster DMA transfers.

**Memory Reuse**: TensorRT reuses tensor buffers across inference batches to minimize allocation overhead.

### 4. End-to-End Latency vs Throughput
**Latency**: Time from sensor input to perception output (15ms GPU vs 188ms CPU)
- Critical for control loops (grasping, navigation)
- 15ms latency → 67 FPS max throughput

**Throughput**: Frames processed per second (67 FPS GPU vs 5 FPS CPU)
- Critical for data collection, batch processing
- Higher throughput enables multi-camera systems

---

## Real-World Application

**Use Case**: Warehouse robot with 4 cameras (front, back, left, right) detecting pallets

**CPU Baseline**:
- 188ms per camera × 4 cameras = 752ms total
- Throughput: 1.3 FPS (unacceptable for navigation)

**GPU Accelerated** (Sequential):
- 15ms per camera × 4 cameras = 60ms total
- Throughput: 16.7 FPS (acceptable)

**GPU Accelerated** (Parallel):
- Process 4 cameras simultaneously on GPU
- Latency: 15ms (same as single camera)
- Throughput: 67 FPS per camera (enables real-time multi-camera perception)

**Cost-Benefit**:
- GPU cost: +$500-$2000 (RTX 4060-4090)
- Performance gain: 12.5x latency reduction, 13x throughput increase
- ROI: Enables real-time multi-camera perception that's impossible on CPU alone
