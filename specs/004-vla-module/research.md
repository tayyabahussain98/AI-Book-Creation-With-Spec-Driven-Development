# Research: Module 4 - Vision-Language-Action (VLA)

**Feature**: 004-vla-module
**Date**: 2025-12-24
**Purpose**: Resolve technical unknowns and establish best practices for VLA content

---

## 1. VLA Architecture Patterns

### Research Question
What is the standard architecture for Vision-Language-Action systems in robotics?

### Findings

**Industry Standard: Three-Layer Architecture**
1. **Perception Layer**: Vision models, speech recognition, sensor fusion
2. **Planning Layer**: LLM-based task decomposition, action graph generation
3. **Execution Layer**: Motion planning, safety monitoring, robot control

**Key Research References**:
- **RT-1 (Google)**: Robotics Transformer for real-world manipulation
- **PaLM-E (Google)**: Embodied multimodal language model
- **SayCan (Google)**: Grounding language in robotic affordances
- **Code as Policies (Google)**: LLM-generated robot programs
- **VIMA (Stanford)**: Vision-and-language grounded manipulation

### Decision
**Adopt**: Three-layer architecture with clear interfaces between layers
**Rationale**: Maps naturally to ROS 2 node structure, widely understood, teachable

### Alternatives Rejected
- **End-to-end neural**: Less interpretable, harder to teach, requires extensive training data
- **Pure behavior trees**: Limited language integration, less flexible for novel commands

---

## 2. Speech Recognition for Robotics

### Research Question
Which speech recognition approach is best for robotics education?

### Findings

**OpenAI Whisper**:
- Open-source, MIT licensed
- Multiple model sizes (tiny to large)
- Excellent multilingual support
- Can run locally (no cloud required)
- Well-documented Python API

**Performance Comparison**:
| Model | WER (Word Error Rate) | Latency | Local Deployment |
|-------|----------------------|---------|------------------|
| Whisper Large | 2.7% | ~1s | Yes (GPU) |
| Whisper Small | 4.7% | ~0.3s | Yes (CPU) |
| Google Cloud | 3.0% | ~0.5s | No |
| Vosk | 6.0% | ~0.2s | Yes (CPU) |

### Decision
**Primary**: Whisper (open-source, local deployment)
**Mention**: Vosk for low-latency offline use, Google Cloud for production

### Rationale
- Free-tier compatible (constitution principle VI)
- Students can run examples locally
- Good documentation for educational purposes

---

## 3. LLM Task Planning

### Research Question
How should LLM-based task planning be presented for robotics education?

### Findings

**Prompting Patterns for Robotics**:
1. **Chain-of-Thought (CoT)**: Step-by-step reasoning for complex tasks
2. **ReAct**: Reasoning and Acting interleaved
3. **Function Calling**: Structured output for robot actions
4. **Code Generation**: Python/pseudocode for action sequences

**Key Concepts to Cover**:
- Task decomposition strategies
- Grounding in robot capabilities (affordances)
- Error handling and replanning
- Safety constraints in prompts

### Decision
**Focus on**: Principles and patterns that transfer across models
**Avoid**: Tight coupling to specific model (GPT-4, Claude, etc.)

### Rationale
- LLM field evolves rapidly (principle II: accuracy)
- Conceptual understanding more valuable than API specifics
- Open-source alternatives (LLaMA, Mistral) mentioned for free-tier compliance

---

## 4. Multimodal Fusion Strategies

### Research Question
What fusion strategies should be taught for combining vision, language, and motion?

### Findings

**Fusion Approaches**:

| Strategy | Description | Complexity | Use Case |
|----------|-------------|------------|----------|
| Early Fusion | Combine raw features before processing | High | End-to-end models |
| Late Fusion | Process modalities separately, combine decisions | Medium | Interpretable systems |
| Attention-Based | Learn to weight modalities dynamically | High | Adaptive systems |
| Confidence-Weighted | Combine based on per-modality confidence | Low-Medium | Robust systems |

**Recommended for Education**:
1. **Late Fusion with Confidence Weighting** (primary)
   - Easier to debug and explain
   - Each modality can be tested independently
   - Clear decision boundaries

2. **Attention-Based** (advanced topic)
   - Mention for completeness
   - Point to research papers

### Decision
**Primary**: Late fusion with confidence weighting
**Secondary**: Early fusion for comparison, attention-based as advanced topic

### Rationale
- Late fusion is more interpretable (teaching goal)
- Maps to ROS 2 node architecture (separate nodes per modality)
- Easier to implement in simulation

---

## 5. ROS 2 Integration Patterns

### Research Question
How should VLA components integrate with ROS 2?

### Findings

**Standard ROS 2 Patterns for VLA**:

| Component | ROS 2 Interface | Frequency | QoS |
|-----------|-----------------|-----------|-----|
| Speech Input | Topic (audio_msgs) | Streaming | Best Effort |
| Intent Output | Topic (custom msg) | Event-based | Reliable |
| Visual Perception | Topic (sensor_msgs) | 10-30 Hz | Best Effort |
| Action Commands | Action Server | Request-based | Reliable |
| Safety Monitor | Service + Topic | 100 Hz | Reliable |

**Architecture Pattern**:
```
[Microphone] → [Speech Node] → [Intent Node] → [Planner Node] → [Executor Node] → [Robot]
                    ↑                               ↑
              [Camera Node] ─────────────→ [Scene Understanding]
```

### Decision
**Adopt**: Action servers for long-running tasks, topics for streaming, services for queries
**Consistent with**: Modules 1-3 ROS 2 patterns

### Rationale
- Students already familiar with these patterns from Module 1
- Standard industry practice
- Good separation of concerns

---

## 6. Safety Considerations

### Research Question
What safety patterns should be taught for autonomous humanoid systems?

### Findings

**Safety Layers**:
1. **Intent Verification**: Confirm dangerous commands before execution
2. **Collision Avoidance**: Real-time obstacle detection and path adjustment
3. **Force Limiting**: Restrict actuator forces during interaction
4. **Emergency Stop**: Immediate halt capability at all times
5. **Watchdog Timers**: Detect system hangs and trigger safe shutdown

**Industry Standards**:
- ISO 10218 (Industrial robots)
- ISO 15066 (Collaborative robots)
- ISO 13482 (Service robots)

### Decision
**Cover**: All five safety layers with simulation examples
**Focus**: Design patterns over implementation details

### Rationale
- Safety is critical for humanoid robots
- Simulation allows safe experimentation
- Students learn safety-first mindset

---

## Summary of Decisions

| Topic | Decision | Primary Reference |
|-------|----------|-------------------|
| Architecture | Three-layer (Perception→Planning→Execution) | RT-1, SayCan |
| Speech | Whisper (open-source, local) | OpenAI docs |
| LLM Planning | Principles-first, model-agnostic | CoT, ReAct patterns |
| Multimodal Fusion | Late fusion with confidence weighting | Academic surveys |
| ROS 2 Integration | Actions + Topics + Services | ROS 2 docs |
| Safety | Five-layer safety architecture | ISO standards |

---

## References

1. Brohan, A., et al. "RT-1: Robotics Transformer for Real-World Control at Scale" (2022)
2. Driess, D., et al. "PaLM-E: An Embodied Multimodal Language Model" (2023)
3. Ahn, M., et al. "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances" (2022)
4. Liang, J., et al. "Code as Policies: Language Model Programs for Embodied Control" (2022)
5. Jiang, Y., et al. "VIMA: General Robot Manipulation with Multimodal Prompts" (2022)
6. Radford, A., et al. "Robust Speech Recognition via Large-Scale Weak Supervision" (2022) [Whisper]
