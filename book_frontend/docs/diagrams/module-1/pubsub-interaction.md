# Publisher-Subscriber Interaction Diagram Specification

**Purpose**: Illustrate the publish-subscribe communication pattern between a publisher node and subscriber node via a ROS 2 topic

**Diagram Type**: Sequence diagram / Message flow diagram

## Components

### 1. Publisher Node (`/simple_publisher`)
- **Type**: ROS 2 Node (Python rclpy)
- **Role**: Generates and publishes Float32 messages at 10 Hz
- **Topic Published**: `/sensor_data`
- **Message Type**: `std_msgs/Float32`
- **Frequency**: 10 Hz (100ms period)

### 2. ROS 2 Topic (`/sensor_data`)
- **Type**: Named message channel
- **Message Type**: `std_msgs/Float32` (single float value)
- **QoS**: Default (Reliable, Volatile, Keep Last 10)
- **Direction**: Publisher → Subscribers (one-to-many)

### 3. Subscriber Node (`/simple_subscriber`)
- **Type**: ROS 2 Node (Python rclpy)
- **Role**: Receives and logs Float32 messages
- **Topic Subscribed**: `/sensor_data`
- **Callback**: `listener_callback(msg)` invoked on message arrival
- **Action**: Logs received value to console

### 4. DDS Middleware (Implicit)
- **Type**: Underlying transport layer (Fast DDS, CycloneDDS, etc.)
- **Role**: Routes messages from publisher to all matching subscribers
- **Discovery**: Automatic (nodes find each other via DDS discovery protocol)

## Message Flow Sequence

### Step 1: Node Initialization
```
[Publisher Node]    [Subscriber Node]
      |                    |
      | rclpy.init()       | rclpy.init()
      | create_publisher() | create_subscription()
      |                    |
      └─────────────┬──────┘
            DDS Discovery
       (nodes announce themselves)
```

### Step 2: DDS Discovery and Connection
```
[Publisher] --announces--> "I publish to /sensor_data (Float32)"
[Subscriber] --announces--> "I subscribe to /sensor_data (Float32)"
[DDS] --matches--> Topic name and message type compatible
[DDS] --establishes connection--> Publisher ↔ Subscriber
```

### Step 3: Message Publishing (Timer Callback - Every 100ms)
```
Time: 0ms
[Publisher]
  timer_callback() invoked
  counter = 0.0
  msg = Float32(data=0.0)
  publisher_.publish(msg) ──────┐
                                 │
                                 ▼
                         [/sensor_data Topic]
                         Message buffered by DDS
                                 │
                                 ▼
                        [Subscriber]
                        listener_callback(msg) invoked
                        Log: "I heard: 0.00"
```

### Step 4: Continuous Publishing (10 Hz Loop)
```
Time: 100ms
[Publisher]
  timer_callback()
  counter = 1.0
  publish(Float32(1.0)) ─────────> [/sensor_data] ─────────> [Subscriber]
                                                               Log: "I heard: 1.00"

Time: 200ms
[Publisher]
  timer_callback()
  counter = 2.0
  publish(Float32(2.0)) ─────────> [/sensor_data] ─────────> [Subscriber]
                                                               Log: "I heard: 2.00"

... (continues every 100ms)
```

## Timing Details

| Time (ms) | Publisher Action | Topic State | Subscriber Action |
|-----------|------------------|-------------|-------------------|
| 0         | Publish Float32(0.0) | Message queued | Callback invoked: log 0.0 |
| 100       | Publish Float32(1.0) | Message queued | Callback invoked: log 1.0 |
| 200       | Publish Float32(2.0) | Message queued | Callback invoked: log 2.0 |
| 300       | Publish Float32(3.0) | Message queued | Callback invoked: log 3.0 |
| ...       | ...              | ...         | ...               |

**Note**: Actual message delivery timing depends on:
- DDS transport (shared memory ~100μs, Ethernet ~1-5ms)
- Subscriber callback execution time
- CPU scheduling and system load

## Decoupling Properties

### 1. Temporal Decoupling
- **Publisher can start first**: Publishes messages even if no subscribers exist yet
- **Subscriber can start first**: Waits for messages, automatically receives them when publisher starts
- **No synchronization required**: Nodes don't need to start at the same time

### 2. Spatial Decoupling
- Publisher doesn't know subscriber's location (same machine, different machine, etc.)
- Subscriber doesn't need publisher's IP address or port
- DDS handles routing transparently

### 3. Cardinality Decoupling
- **One publisher, zero subscribers**: Messages published but not delivered (no error)
- **One publisher, one subscriber**: Standard 1:1 communication
- **One publisher, N subscribers**: All subscribers receive the same messages (multicast)

## Failure Scenarios

### Scenario 1: Publisher Crashes
```
[Publisher] ──X (crashes)
[/sensor_data] --no new messages-->
[Subscriber] --continues running--> (no callback invocations, node still alive)
```
**Result**: Subscriber keeps running, callback stops being invoked. No cascading failure.

### Scenario 2: Subscriber Crashes
```
[Publisher] --continues publishing-->
[/sensor_data] --messages discarded--> (no active subscribers)
[Subscriber] ──X (crashed, no longer subscribed)
```
**Result**: Publisher continues publishing, unaware that subscriber is gone. No errors.

### Scenario 3: Network Partition
```
[Publisher] --publishes--> [Network] ──X (connection lost)
[Subscriber] --timeout--> No messages received
```
**Result**: Subscriber detects missing messages (if using RELIABLE QoS). Publisher may queue messages until reconnection.

## Visual Layout

```
┌─────────────────────┐
│  Publisher Node     │
│  /simple_publisher  │
│                     │
│  ┌──────────────┐   │
│  │Timer: 10 Hz  │   │
│  │(100ms period)│   │
│  └──────┬───────┘   │
│         │           │
│         ▼           │
│  ┌──────────────┐   │
│  │ Publish      │   │
│  │ Float32(n)   │   │
│  └──────┬───────┘   │
└─────────┼───────────┘
          │
          │ publish()
          ▼
    ┌──────────────────┐
    │  /sensor_data    │
    │  Topic           │
    │  (Float32)       │
    │  QoS: Reliable   │
    │  History: 10     │
    └──────┬───────────┘
          │
          │ message delivery
          ▼
┌─────────────────────┐
│  Subscriber Node    │
│  /simple_subscriber │
│                     │
│  ┌──────────────┐   │
│  │ Callback:    │   │
│  │ listener_    │   │
│  │ callback()   │   │
│  └──────┬───────┘   │
│         │           │
│         ▼           │
│  ┌──────────────┐   │
│  │ Log:         │   │
│  │ "I heard: n" │   │
│  └──────────────┘   │
└─────────────────────┘
```

## Key Insights

1. **Asynchronous Communication**: Publisher doesn't wait for subscriber acknowledgment. Fire-and-forget pattern enables high-frequency publishing without blocking.

2. **DDS Handles Routing**: Middleware automatically discovers nodes and routes messages. No manual connection management.

3. **Topic-Based Addressing**: Nodes communicate via named topics (`/sensor_data`), not direct node references. This decouples sender from receiver.

4. **QoS Policies Control Behavior**: Default QoS uses Reliable delivery (TCP-like), but can be changed to Best Effort (UDP-like) for high-frequency sensor data.

5. **One-to-Many Scalability**: Add more subscribers by simply creating nodes that subscribe to `/sensor_data`. No publisher code changes needed.

## Usage in Book

- **Referenced in**: Chapter 3 (Hands-On Examples 1 & 2: Publisher and Subscriber Patterns)
- **Purpose**: Help learners visualize the interaction between publisher and subscriber nodes, including timing, message flow, and decoupling properties
- **Learning Goal**: Learners understand that ROS 2 topics enable asynchronous, many-to-many communication without direct node dependencies
