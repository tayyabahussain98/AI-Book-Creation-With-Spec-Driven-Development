<<<<<<< HEAD
---
sidebar_position: 4
title: Capstone Architecture Diagram
---

# Capstone Architecture Diagram

## Complete VLA System Architecture

```mermaid
flowchart TB
    subgraph UI["User Interface Layer"]
        Mic["Microphone"]
        Display["Display"]
        Buttons["Buttons"]
    end

    subgraph Perception["Perception Layer"]
        subgraph Voice["Voice Input Pipeline"]
            VAD["VAD Detector"]
            ASR["ASR Engine"]
            NLU["NLU Parser"]
            Intent["Intent Classifier"]
        end

        subgraph Vision["Visual Input Pipeline"]
            RGB["RGB Camera"]
            Depth["Depth Estimator"]
            Detector["Object Detector"]
            SceneGraph["Scene Graph"]
        end
    end

    subgraph Cognition["Cognition Layer"]
        subgraph VLA["VLA Model (Policy)"]
            VisionEnc["Vision Encoder"]
            LangEnc["Language Encoder"]
            CrossAttn["Cross-Attention"]
            ActionHead["Action Head"]
        end

        subgraph Planner["LLM Task Planner"]
            CmdParser["Command Parser"]
            PlanGen["Plan Generator"]
            Refine["Step Refiner"]
            Exec["Step Executor"]
        end
    end

    subgraph Action["Action Layer"]
        subgraph Motion["Motion Generation"]
            TrajPlan["Trajectory Planning"]
            IK["Inverse Kinematics"]
            TrajExec["Trajectory Exec"]
            Safety["Safety Check"]
        end

        subgraph SafetyMon["Safety Monitor"]
            ForceMon["Force Monitor"]
            PosMon["Position Monitor"]
            VelMon["Velocity Monitor"]
            EStop["E-Stop Trigger"]
        end
    end

    subgraph Hardware["Robot Hardware"]
        HeadCam["Head Cameras"]
        Arms["Arm Motors"]
        Hands["Hand Sensors"]
        Torso["Torso Motors"]
    end

    %% Voice path
    Mic --> VAD
    VAD --> ASR
    ASR --> NLU
    NLU --> Intent
    Intent --> CmdParser

    %% Vision path
    RGB --> Depth
    RGB --> Detector
    Depth --> Detector
    Detector --> SceneGraph
    SceneGraph --> VisionEnc

    %% VLA path
    VisionEnc --> CrossAttn
    LangEnc --> CrossAttn
    CrossAttn --> ActionHead

    %% Planner to VLA
    CmdParser --> PlanGen
    PlanGen --> Refine
    Refine --> Exec
    Exec --> TrajPlan

    %% Cross connections
    Intent --> LangEnc
    SceneGraph --> LangEnc
    TrajPlan --> IK
    IK --> TrajExec
    TrajExec --> Safety

    %% Safety path
    ForceMon --> EStop
    PosMon --> EStop
    VelMon --> EStop
    Safety --> EStop

    %% Motion to hardware
    TrajExec --> Arms
    TrajExec --> Hands
    Safety --> Arms
    Safety --> Hands
    VisionEnc --> HeadCam

    %% Display feedback
    ActionHead --> Display
    Exec --> Display
```

## Data Flow for Voice Command

```mermaid
sequenceDiagram
    participant User
    participant VLASystem
    participant Perception
    participant Cognition
    participant Action
    participant Hardware

    User->>VLASystem: "pick up the red cup and place it on the table"
    VLASystem->>Perception: Process audio
    Perception->>Perception: VAD → ASR
    Perception-->>VLASystem: "pick up the red cup..."

    VLASystem->>Cognition: Parse intent
    Cognition->>Cognition: Intent.PICK_PLACE
    Cognition->>Cognition: Generate plan
    Cognition->>Perception: Ground objects
    Perception-->>Cognition: cup_001 @ (0.5, 0.3, 0.1)

    Cognition-->>VLASystem: Grounded plan

    loop Execute each step
        VLASystem->>Action: Request trajectory
        Action->>Action: Safety check
        alt Unsafe
            Action-->>VLASystem: HALT
            VLASystem-->>User: "Cannot execute - safety concern"
        else Safe
            Action->>Hardware: Execute motion
            Hardware-->>Action: Completion
            Action-->>VLASystem: Step done
        end
    end

    VLASystem-->>User: "Done: placed cup on table"
```

## Safety Layer Hierarchy

```mermaid
flowchart TB
    subgraph L4["Level 4: Hardware E-Stop"]
        HW["Physical Kill Switch"]
        HW --> Power["Power Cut"]
    end

    subgraph L3["Level 3: Software E-Stop"]
        SW["Emergency Stop Topic"]
        SW --> Halt["Immediate Motion Halt"]
    end

    subgraph L2["Level 2: Boundary Enforcement"]
        WS["Workspace Limits"]
        OA["Obstacle Avoidance"]
        WS --> Reject["Reject Trajectory"]
        OA --> Modify["Modify Trajectory"]
    end

    subgraph L1["Level 1: Trajectory Verification"]
        Vel["Velocity Limits"]
        Acc["Acceleration Limits"]
        Force["Force Predictions"]
        Vel --> Check["Check Limits"]
        Acc --> Check
        Force --> Check
    end

    subgraph L0["Level 0: Nominal Operation"]
        Plan["Motion Planning"]
        Exec["Execution"]
    end

    L0 --> L1
    L1 --> L2
    L2 --> L3
    L3 --> L4

    Check -->|violation| L2
    Modify -->|modified| L1
    Reject -->|rejected| L0
```

## System State Machine

```mermaid
stateDiagram-v2
    [*] --> IDLE

    IDLE --> LISTENING : Voice detected
    LISTENING --> PROCESSING : Transcription complete
    LISTENING --> IDLE : Silence/Timeout

    PROCESSING --> EXECUTING : Plan ready
    PROCESSING --> IDLE : Parse failure
    PROCESSING --> ERROR : Exception

    EXECUTING --> IDLE : Task complete
    EXECUTING --> PAUSED : User interrupt
    EXECUTING --> ERROR : Execution failure
    EXECUTING --> EMERGENCY_STOP : Safety violation

    PAUSED --> EXECUTING : Resume
    PAUSED --> IDLE : Cancel

    ERROR --> IDLE : Reset

    EMERGENCY_STOP --> IDLE : Manual reset
    EMERGENCY_STOP --> ERROR : Persistent fault
```
=======
---
sidebar_position: 4
title: Capstone Architecture Diagram
---

# Capstone Architecture Diagram

## Complete VLA System Architecture

```mermaid
flowchart TB
    subgraph UI["User Interface Layer"]
        Mic["Microphone"]
        Display["Display"]
        Buttons["Buttons"]
    end

    subgraph Perception["Perception Layer"]
        subgraph Voice["Voice Input Pipeline"]
            VAD["VAD Detector"]
            ASR["ASR Engine"]
            NLU["NLU Parser"]
            Intent["Intent Classifier"]
        end

        subgraph Vision["Visual Input Pipeline"]
            RGB["RGB Camera"]
            Depth["Depth Estimator"]
            Detector["Object Detector"]
            SceneGraph["Scene Graph"]
        end
    end

    subgraph Cognition["Cognition Layer"]
        subgraph VLA["VLA Model (Policy)"]
            VisionEnc["Vision Encoder"]
            LangEnc["Language Encoder"]
            CrossAttn["Cross-Attention"]
            ActionHead["Action Head"]
        end

        subgraph Planner["LLM Task Planner"]
            CmdParser["Command Parser"]
            PlanGen["Plan Generator"]
            Refine["Step Refiner"]
            Exec["Step Executor"]
        end
    end

    subgraph Action["Action Layer"]
        subgraph Motion["Motion Generation"]
            TrajPlan["Trajectory Planning"]
            IK["Inverse Kinematics"]
            TrajExec["Trajectory Exec"]
            Safety["Safety Check"]
        end

        subgraph SafetyMon["Safety Monitor"]
            ForceMon["Force Monitor"]
            PosMon["Position Monitor"]
            VelMon["Velocity Monitor"]
            EStop["E-Stop Trigger"]
        end
    end

    subgraph Hardware["Robot Hardware"]
        HeadCam["Head Cameras"]
        Arms["Arm Motors"]
        Hands["Hand Sensors"]
        Torso["Torso Motors"]
    end

    %% Voice path
    Mic --> VAD
    VAD --> ASR
    ASR --> NLU
    NLU --> Intent
    Intent --> CmdParser

    %% Vision path
    RGB --> Depth
    RGB --> Detector
    Depth --> Detector
    Detector --> SceneGraph
    SceneGraph --> VisionEnc

    %% VLA path
    VisionEnc --> CrossAttn
    LangEnc --> CrossAttn
    CrossAttn --> ActionHead

    %% Planner to VLA
    CmdParser --> PlanGen
    PlanGen --> Refine
    Refine --> Exec
    Exec --> TrajPlan

    %% Cross connections
    Intent --> LangEnc
    SceneGraph --> LangEnc
    TrajPlan --> IK
    IK --> TrajExec
    TrajExec --> Safety

    %% Safety path
    ForceMon --> EStop
    PosMon --> EStop
    VelMon --> EStop
    Safety --> EStop

    %% Motion to hardware
    TrajExec --> Arms
    TrajExec --> Hands
    Safety --> Arms
    Safety --> Hands
    VisionEnc --> HeadCam

    %% Display feedback
    ActionHead --> Display
    Exec --> Display
```

## Data Flow for Voice Command

```mermaid
sequenceDiagram
    participant User
    participant VLASystem
    participant Perception
    participant Cognition
    participant Action
    participant Hardware

    User->>VLASystem: "pick up the red cup and place it on the table"
    VLASystem->>Perception: Process audio
    Perception->>Perception: VAD → ASR
    Perception-->>VLASystem: "pick up the red cup..."

    VLASystem->>Cognition: Parse intent
    Cognition->>Cognition: Intent.PICK_PLACE
    Cognition->>Cognition: Generate plan
    Cognition->>Perception: Ground objects
    Perception-->>Cognition: cup_001 @ (0.5, 0.3, 0.1)

    Cognition-->>VLASystem: Grounded plan

    loop Execute each step
        VLASystem->>Action: Request trajectory
        Action->>Action: Safety check
        alt Unsafe
            Action-->>VLASystem: HALT
            VLASystem-->>User: "Cannot execute - safety concern"
        else Safe
            Action->>Hardware: Execute motion
            Hardware-->>Action: Completion
            Action-->>VLASystem: Step done
        end
    end

    VLASystem-->>User: "Done: placed cup on table"
```

## Safety Layer Hierarchy

```mermaid
flowchart TB
    subgraph L4["Level 4: Hardware E-Stop"]
        HW["Physical Kill Switch"]
        HW --> Power["Power Cut"]
    end

    subgraph L3["Level 3: Software E-Stop"]
        SW["Emergency Stop Topic"]
        SW --> Halt["Immediate Motion Halt"]
    end

    subgraph L2["Level 2: Boundary Enforcement"]
        WS["Workspace Limits"]
        OA["Obstacle Avoidance"]
        WS --> Reject["Reject Trajectory"]
        OA --> Modify["Modify Trajectory"]
    end

    subgraph L1["Level 1: Trajectory Verification"]
        Vel["Velocity Limits"]
        Acc["Acceleration Limits"]
        Force["Force Predictions"]
        Vel --> Check["Check Limits"]
        Acc --> Check
        Force --> Check
    end

    subgraph L0["Level 0: Nominal Operation"]
        Plan["Motion Planning"]
        Exec["Execution"]
    end

    L0 --> L1
    L1 --> L2
    L2 --> L3
    L3 --> L4

    Check -->|violation| L2
    Modify -->|modified| L1
    Reject -->|rejected| L0
```

## System State Machine

```mermaid
stateDiagram-v2
    [*] --> IDLE

    IDLE --> LISTENING : Voice detected
    LISTENING --> PROCESSING : Transcription complete
    LISTENING --> IDLE : Silence/Timeout

    PROCESSING --> EXECUTING : Plan ready
    PROCESSING --> IDLE : Parse failure
    PROCESSING --> ERROR : Exception

    EXECUTING --> IDLE : Task complete
    EXECUTING --> PAUSED : User interrupt
    EXECUTING --> ERROR : Execution failure
    EXECUTING --> EMERGENCY_STOP : Safety violation

    PAUSED --> EXECUTING : Resume
    PAUSED --> IDLE : Cancel

    ERROR --> IDLE : Reset

    EMERGENCY_STOP --> IDLE : Manual reset
    EMERGENCY_STOP --> ERROR : Persistent fault
```
>>>>>>> 5cf5f6818272e652910633754d301908ff0bb236
