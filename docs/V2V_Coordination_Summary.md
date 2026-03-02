# V2V Coordination System Summary

## 1. Core Components

### 1.1 V2V Message Protocol (`v2v_message.py`)

**Message Content (V2VMessage):**
- **Vehicle State**: Position (x, y, yaw), Velocity (speed, heading)
- **Perception Sharing**: List of detected obstacles (class, confidence, world coordinates, distance)
- **Intent Broadcasting**: Driving intent (cruising, turning_left, turning_right, yielding, stopped, emergency_brake)
- **Cooperation Request**: Active request for yield or priority passage
- **Timestamp & Vehicle ID**: Support message freshness validation

**Communication Modes:**
- In-process communication (CARLA multi-vehicle simulation)
- Socket UDP broadcast (Real-world PC ↔ Raspberry Pi deployment)
- ROS2 Topic (Reserved for future extension)

---

## 2. Cooperative Planner Features (`cooperative_planner.py`)

### 2.1 Time-to-Intersection Based Intersection Coordination
- Calculates Time-To-Intersection (TTI) for both vehicles approaching conflict point
- **First-Arrive-First-Go**: Vehicle arriving at conflict point first gets priority
- **When time difference < 2s**: Uses vehicle ID as deterministic tiebreaker to prevent deadlock
- Considers heading deviation penalty (vehicles needing turns get time penalty)

### 2.2 Distance-Aware Coordination (Key Innovation)
- **Hierarchical coordination strategy**:
  - `>50m`: Coordination inactive, only passively receive detection info
  - `30-50m`: Monitoring mode, aware but not actively coordinating
  - `15-30m`: Active coordination, handle intersection/following behaviors
  - `8-15m`: Warning mode, slow down and yield
  - `<8m`: Critical mode, emergency braking

### 2.3 Occlusion Compensation
- Fuses obstacle detections shared by other vehicles into local APF planner
- Solves "I can't see but other vehicles can" blind spot problem
- Applies confidence discount to shared information (confidence × 0.8)

### 2.4 Deadlock Detection & Resolution
- Detects when two vehicles are stationary and too close
- Triggers after 3-second timeout
- Vehicle with smaller ID actively backs up and yields

### 2.5 Platooning (Convoy Following)
- Detects vehicles traveling in same direction
- Automatically adjusts speed to maintain safe following distance
- Supports convoy cruising behavior

---

## 3. Innovation Contributions

| Innovation | Description |
|------------|-------------|
| **1. Lightweight Decentralized Coordination** | No central controller needed, pure distributed V2V negotiation, suitable for low-cost small robot platforms |
| **2. TTI-based Intersection Arbitration** | Traditional methods use distance; this project uses arrival time + heading penalty, more accurate |
| **3. Distance-Aware Hierarchical Coordination** | Dynamically adjusts coordination intensity based on actual distance, reduces unnecessary communication and computation |
| **4. Seamless Simulation-to-Reality Migration** | Same code supports CARLA simulation and real Raspberry Pi cars, dual-mode Socket/In-process |
| **5. Workshop Coordination Focus** | Different from highway autonomous driving, focuses on indoor/workshop multi-robot coordination scenarios (innovation requested in early feedback) |

---

## 4. Technical Highlights

1. **Fault-Tolerant Design**: Supports message dropout simulation, stale message filtering
2. **Modular Architecture**: V2V communication layer separated from planning layer, independently testable
3. **Real-Time Optimization**: UDP non-blocking communication, message processing latency <100ms
4. **Extensibility**: Reserved ROS2 interface for future integration with robotics middleware

---

## 5. Integration with Overall System

The V2V module integrates with other modules as follows:

```
Perception Module → SharedDetection → V2VMessage → Broadcast
                                          ↓
Planning Module ← Shared Obstacles ← CooperativePlanner ← Receive V2VMessage
       ↓
Control Module ← CooperativeDecision (action, speed_adjustment)
```

This enables true cooperative autonomous driving where multiple vehicles share perception and negotiate trajectories.
