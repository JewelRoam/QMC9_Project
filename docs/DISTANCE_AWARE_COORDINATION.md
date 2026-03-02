# Distance-Aware Vehicle Coordination

## Overview
This document describes the enhanced inter-vehicle coordination system that uses relative distance to intelligently activate cooperative behaviors.

## Key Innovation
The system now only activates full coordination when vehicles are close enough to benefit from it, reducing unnecessary computation and making the coordination more practical.

## Coordination Zones

```
Distance-based Coordination Zones:

    > 50m          : INACTIVE    (No coordination)
    30m - 50m      : MONITORING  (Passive tracking, platooning possible)
    15m - 30m      : ACTIVE      (Full coordination: intersection, deadlock detection)
    8m - 15m       : WARNING     (Close proximity alert)
    < 8m           : CRITICAL    (Emergency slow down)
```

## Implementation Details

### New Parameters in config.yaml
```yaml
cooperation:
  coordination_range: 50.0        # Max range for coordination (meters)
  active_coordination_range: 30.0 # Active coordination range (meters)
  warning_range: 15.0             # Warning/critical range (meters)
```

### Core Methods Added to CooperativePlanner

1. **`_update_nearby_vehicles()`**
   - Filters V2V messages by coordination range
   - Tracks vehicle distances and timestamps
   - Cleans up stale entries (>5 seconds)

2. **`_get_coordination_status()`**
   - Determines current coordination status based on nearest vehicle
   - Returns: `(status_str, num_nearby_vehicles)`
   - Status values: `inactive`, `monitoring`, `active`, `warning`, `critical`

3. **Enhanced `process()` method**
   - First filters vehicles by coordination range
   - Only applies complex coordination logic when beneficial
   - Maintains passive obstacle sharing at all ranges

## Visual Indicators

The multi-vehicle demo now displays coordination status with color coding:

| Status | Color | Meaning |
|--------|-------|---------|
| INACTIVE | Gray | No nearby vehicles requiring coordination |
| MONITORING | Cyan | Tracking vehicles, ready for platooning |
| ACTIVE | Green | Full coordination active |
| WARNING | Orange | Close proximity, caution needed |
| CRITICAL | Red | Very close, emergency braking |

## Benefits

1. **Computational Efficiency**: Complex coordination logic only runs when vehicles are close
2. **Practical Relevance**: Coordination only matters when vehicles can actually interact
3. **Clear Visual Feedback**: Users can see exactly when coordination is active
4. **Configurable Thresholds**: Easy to adjust ranges via config file

## Testing

Run the test script to verify functionality:
```bash
python test_distance_coordination.py
```

Expected output shows different behaviors at different distances:
- 100m: inactive (no action)
- 40m: monitoring (platooning possible)
- 25m: active (full coordination)
- 5m: critical (emergency slow down)

## Phase 2: Time-to-Intersection (TTI) Based Priority

The intersection coordination has been enhanced with intelligent priority calculation:

### How It Works

1. **Conflict Point Detection**: System calculates where two vehicle paths will intersect
2. **TTI Calculation**: Estimates time for each vehicle to reach the conflict point
   - Considers distance, speed, and heading alignment
   - Adds penalty for large heading deviations
3. **Priority Decision**:
   - Vehicle arriving **FIRST** has priority
   - If arrival times within 2 seconds → use ID tiebreaker for determinism
   - Clear time advantage (>2s) → definitive priority assignment

### Example Scenarios

```
Scenario 1: Ego arrives 3.4s earlier
  Result: Ego proceeds with clear priority

Scenario 2: Other arrives 1.0s earlier  
  Result: Ego yields (significant time disadvantage)

Scenario 3: Both arrive within 0.5s (close call)
  Result: Lower ID yields (deterministic tiebreaker)
```

### Benefits

- **Fair**: Faster/closer vehicle gets priority
- **Safe**: Conservative when timing is close
- **Deterministic**: ID tiebreaker prevents deadlock
- **Explainable**: Clear reasoning for every decision

## Future Enhancements (Phase 3)

1. **Intent prediction**: Predict other vehicle's future trajectories using ML
2. **Dynamic zone adjustment**: Adjust coordination ranges based on speed
3. **Multi-vehicle coordination**: Handle 3+ vehicles at complex intersections
4. **Learning-based priority**: Learn optimal yielding patterns from data
