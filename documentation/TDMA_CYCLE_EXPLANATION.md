# TDMA Cycle Explanation for Multi-AUV Missions

## Overview

This document explains how the TDMA (Time Division Multiple Access) cycle works in the multi-AUV acoustic communication system. The TDMA cycle is a time-based coordination mechanism that allows multiple AUVs to share the acoustic communication channel without collisions.

## Core Concept

The system divides time into **repeating cycles**, where each cycle contains multiple **time slots**. Each vehicle is assigned specific slots during which it can transmit acoustic messages and ranging pings.

```
Cycle Structure:
┌──────────┬──────────┬──────────┬──────────┐
│  Slot 0  │  Slot 1  │  Slot 2  │  Slot 3  │ ← One Complete Cycle
└──────────┴──────────┴──────────┴──────────┘
    ↑ Repeats continuously
```

## Key Configuration Parameters

The TDMA cycle is configured with the following parameters:

| Parameter | Description | Typical Values |
|-----------|-------------|----------------|
| `num_slots` | Total number of time slots in one cycle | 2, 4, 6, 8 |
| `slot_duration_seconds` | Length of each slot | 15-30 seconds |
| `active_slots` | Which slot(s) each vehicle can transmit in | "0", "1", "0,2", etc. |
| `cycle_start_time` | Reference epoch for synchronizing all vehicles | ISO 8601 timestamp |
| `guard_time_seconds` | Safety buffer between transmissions | 2-5 seconds |
| `packet_length_seconds` | Duration of acoustic packet transmission | 1-5 seconds |

### Configuration Example

From `payload_multi.launch`:
```xml
<node name="tdma_node" pkg="ros_acomms" type="tdma_node.py">
    <param name="num_slots" value="2" />
    <param name="active_slots" value="0"/>  <!-- Agent-specific -->
    <param name="slot_duration_seconds" value="25" />
    <param name="guard_time_seconds" value="2.0" />
    <param name="packet_length_seconds" value="1.0" />
</node>
```

## How the Cycle Works

### 1. Current Slot Calculation

The system continuously calculates which slot is currently active using modular arithmetic:

```python
# From tdma_node.py
now_secs = rospy.Time.now().to_sec()
offset_secs = now_secs - self.cycle_start_secs
in_cycle_secs = offset_secs % self.cycle_duration
current_slot = int(in_cycle_secs // self.slot_duration_seconds)
```

**Key calculations:**
- `cycle_duration = num_slots × slot_duration_seconds`
- `current_slot` = Current position in cycle (0 to num_slots-1)
- `remaining_slot_seconds` = Time left in current slot

### 2. Active Slot Determination

Each vehicle determines if it's currently active:

```python
we_are_active = self.slots[current_slot]['active'] and not software_mute
```

A vehicle is active when:
- Current slot matches one of its `active_slots`
- Software mute is not enabled
- The vehicle has initialized properly

### 3. Transmission Rules

A vehicle can transmit only when ALL conditions are met:

1. **Active Slot**: Currently in one of its assigned `active_slots`
2. **Sufficient Time**: `remaining_time > guard_time + packet_length`
3. **Guard Time Elapsed**: Enough time passed since last transmission
4. **No Collision**: Guard time prevents overlapping transmissions

```python
# From tdma_node.py
def can_transmit_now(self, msg, last_tx_time):
    if not msg.we_are_active:
        return False
    if msg.remaining_active_seconds < (self.guard_time_seconds + self.packet_length_seconds):
        return False
    if current_time <= (last_tx_time + self.guard_time_seconds + self.packet_length_seconds):
        return False
    return True
```

## Multi-Agent Example

### Two-Agent System

Configuration for a 2-agent system with 2 slots:

```
Agent 0: active_slots="0"  → Transmits only in slot 0
Agent 1: active_slots="1"  → Transmits only in slot 1

Timeline (50 seconds total cycle, 25 sec/slot):
┌─────────────────────┬─────────────────────┐
│      Slot 0         │      Slot 1         │ → Repeats
│   Agent 0 Active    │   Agent 1 Active    │
│     25 seconds      │     25 seconds      │
└─────────────────────┴─────────────────────┘
  0s                 25s                   50s
```

### Multi-Agent with Landmarks

For a 2-agent, 2-landmark system (common configuration):

```
Agent A: active_slots="0"
Agent B: active_slots="1"
Landmarks L0, L1: Always listening (no transmit slots)

Activities per slot (point-to-point transmissions on broadcast channel):
┌─────────────────────────┬─────────────────────────┐
│       Slot 0            │       Slot 1            │
│   Agent A transmits:    │   Agent B transmits:    │
│   - Data packet → dest  │   - Data packet → dest  │
│   - Ping → Agent B      │   - Ping → Agent A      │
│   - Ping → L0           │   - Ping → L0           │
│   - Ping → L1           │   - Ping → L1           │
│                         │                         │
│   Others listen:        │   Others listen:        │
│   - B receives pings    │   - A receives pings    │
│   - L0/L1 respond       │   - L0/L1 respond       │
│   - All can overhear    │   - All can overhear    │
└─────────────────────────┴─────────────────────────┘

Note: Each ping/packet has a specific destination address (point-to-point),
but all vehicles in acoustic range can overhear (broadcast medium).
```

## Communication Cycle Integration

The `comms_ltwt.py` manager coordinates activities within the TDMA cycle:

### Slot 0 - Cycle Reset

When the cycle returns to slot 0:
```python
# From comms_ltwt.py - on_tdma_status()
if current_slot == 0 and current_slot != self.tdma_status.current_slot:
    self.tdma_cycle_sequence += 1
    self.on_tdma_cycle_reset()
```

Actions:
- Increment cycle counter
- Reset ping attempt flags
- Build new partial graph message with:
  - Preintegrated IMU data
  - Range measurements from previous cycle
  - Relative pose estimates

### Slot 1 - Message Transmission

When entering slot 1:
```python
elif current_slot == 1 and current_slot != self.tdma_status.current_slot:
    self.partial_graph_pub.publish(self.staged_partial_graph)
```

Actions:
- Publish staged partial graph to message queue
- Queue compresses and encodes message for acoustic transmission

### During Active Slots

When in an active slot with sufficient time:
```python
elif we_are_active and remaining_sec_in_slot < 2.1*self.ping_timeout:
    if self.data_sent == False:
        self.data_sent = True
        self.poke_ping_cycle(10)
```

Actions:
1. **Send Data Packets**: TDMA node queries message queue for highest priority data
2. **Execute Ranging Cycle**: 
   - Ping other agents for cooperative localization (point-to-point)
   - Ping landmarks for absolute positioning (point-to-point)
   - Collect one-way travel time (OWTT) measurements
3. **Request IMU Preintegration**: Generate relative pose between pings

## Ranging: Point-to-Point vs Broadcast

### Addressing: Point-to-Point (Unicast)

Each ranging ping is **addressed to a specific destination**:

```python
# From comms_ltwt.py
ping_req = PingModemRequest()
ping_req.dest = target_addr  # Specific destination address
ping_req.rate = 1
ping_req.timeout_sec = self.ping_timeout
```

The `cycle_target_mapping` determines which specific targets each vehicle pings during its slot:
- Agent A might ping: Agent B, Landmark L0, Landmark L1
- Each ping is a separate point-to-point transmission

### Channel: Inherently Broadcast

However, the **acoustic channel is a shared broadcast medium** - all vehicles within acoustic range can physically receive any transmission. The code handles this:

```python
# From comms_ltwt.py - on_nmea_from_modem()
elif nmea_type == "$CACMA":  # Ping received acknowledgement
    src, dest, recieved_ping_time = parse_nmea_cacma(data)
    if dest == self.local_address:
        # This ping was addressed to us - process normally
        rospy.loginfo("Received Ping from %s" % chr(ord("A") + src))
        self.request_preintegration(rcvd_stamp, True)
    elif data[1] == "PNG":
        # We overheard a ping to someone else
        rospy.loginfo("Overheard Ping from %s to %s" % 
                     (chr(ord("A") + src), chr(ord("A") + dest)))
```

### Key Characteristics

| Aspect | Behavior |
|--------|----------|
| **Protocol** | Point-to-point (unicast) with specific destination addresses |
| **Physical Channel** | Broadcast (all vehicles in range receive the acoustic signal) |
| **Processing** | Only intended recipient processes for ranging/localization |
| **Overhearing** | Other vehicles can detect and log overheard transmissions |
| **Data Packets** | Also point-to-point with destination addresses |

This is analogous to WiFi or other wireless networks where:
- Transmissions have specific MAC addresses (point-to-point protocol)
- But the RF channel broadcasts to all nearby receivers (broadcast medium)
- Non-target nodes can overhear but typically ignore traffic not addressed to them

### Benefits of Point-to-Point Ranging

1. **Targeted Measurements**: Each vehicle collects specific range measurements it needs
2. **Cycle Planning**: Predetermined ping schedule via `cycle_target_mapping`
3. **Cooperative Localization**: Vehicles can request specific neighbors for ranging
4. **Landmark Positioning**: Each vehicle independently pings fixed landmarks
5. **Overhearing as Bonus**: Other vehicles may optionally use overheard pings for passive localization

## TDMA Status Publishing

The TDMA node publishes status at 5 Hz on the `tdma_status` topic:

```python
# TdmaStatus message fields
msg = TdmaStatus(
    we_are_active=we_are_active,           # Boolean: in active slot
    current_slot=current_slot,              # Int: current slot number
    remaining_slot_seconds=remaining_slot_seconds,  # Float: time left
    remaining_active_seconds=remaining_active_seconds,  # Float
    time_to_next_active=time_to_next_active,  # Float: seconds until active
    slot_duration_seconds=slot_duration_seconds,  # Float
    require_tags=list(...),                 # List: QoS requirements
    exclude_tags=list(...),                 # List: QoS exclusions
    minimum_priority=minimum_priority       # Int: priority threshold
)
```

This status is consumed by:
- **Comms Manager** (`comms_ltwt.py`): Coordinates pings and data transmission
- **Message Queue**: Determines when to queue packets
- **Other Nodes**: Monitor network timing and activity

## Cycle Synchronization

### Clock Synchronization

All vehicles must maintain synchronized clocks:
- **GPS Time Sync**: Set modem time from GPS on startup
- **Common Epoch**: All vehicles use same `cycle_start_time` parameter
- **Drift Tolerance**: Guard times accommodate small clock drift

### Configuration Consistency

All vehicles in the network must agree on:
- `num_slots`: Same number of slots
- `slot_duration_seconds`: Same slot duration
- `cycle_start_time`: Same reference epoch

Individual vehicles differ only in:
- `active_slots`: Which slots they transmit in
- `modem_address`: Unique address for each vehicle

### Example Network Configuration

```yaml
# Common parameters (all vehicles)
num_slots: 2
slot_duration_seconds: 25
cycle_start_time: "2024-01-01T00:00:00Z"

# Vehicle-specific parameters
vehicle_0:
  modem_address: 0
  active_slots: "0"

vehicle_1:
  modem_address: 1
  active_slots: "1"
```

## Advanced Features

### Multiple Active Slots

A vehicle can have multiple active slots:
```xml
<param name="active_slots" value="0,2,4"/>  <!-- Slots 0, 2, and 4 -->
```

This enables:
- Higher throughput for priority vehicles
- Flexible network topologies
- Load balancing across time

### Slot Range Notation

Supports range notation for slot configuration:
```python
"0:4:2"    # Slots 0, 2, 4 (start:stop:step)
"0,2,5:7"  # Slots 0, 2, 5, 6, 7 (mixed notation)
```

### Dynamic Reconfiguration

The `tdma_advanced_node` supports runtime reconfiguration:
- Adjust guard times
- Change packet lengths
- Enable/disable software mute
- Modify ping parameters

### Quality of Service (QoS) Tags

Slots can have QoS requirements:
- `minimum_priority`: Only high-priority messages
- `require_tags`: Message must have specific tags
- `exclude_tags`: Message must not have specific tags

## Timing Diagram Example

For a 2-agent system with 25-second slots:

```
Time (s):  0         5         10        15        20        25        30
           |         |         |         |         |         |         |
Agent 0:   [═Data═][Ping→B][Ping→L0][Ping→L1]│     Idle             │
Slot:      └────────────────  Slot 0  ─────────┴──────  Slot 1  ─────┘
Agent 1:   (overhear)                   Idle    │[═Data═][Ping→A]...   │
Agent 1:                                        receives pings from A

Legend:
═Data═      : Acoustic data packet transmission (point-to-point)
[Ping→X]    : Ranging ping TO specific target X (point-to-point)
(overhear)  : Can overhear transmissions (broadcast medium)
Idle        : Listening/receiving mode
│           : Slot boundary

Note: All transmissions use the broadcast acoustic channel but have 
specific destination addresses. Non-target vehicles can overhear.
```

## Implementation Files

Key implementation files in the workspace:

| File | Purpose |
|------|---------|
| `ros_acomms/src/tdma_node.py` | Base TDMA MAC implementation |
| `ros_acomms/src/tdma_advanced_node.py` | Advanced TDMA with pings and dynamic reconfig |
| `ros_acomms/src/tdma_scripted_node.py` | Scripted TDMA for testing |
| `spurdog_acomms/src/comms_ltwt.py` | Communication cycle manager |
| `spurdog_acomms/launch/payload_multi.launch` | Multi-agent launch configuration |

## Best Practices

1. **Slot Duration**: Choose slot duration based on:
   - Maximum acoustic packet length
   - Number of pings needed per slot
   - Acoustic propagation delays
   - Guard time requirements

2. **Guard Time**: Set guard time to account for:
   - Clock drift between vehicles
   - Acoustic multipath delays
   - Modem processing time

3. **Active Slot Assignment**: Design active slots to:
   - Prevent collisions (non-overlapping slots)
   - Minimize latency for critical vehicles
   - Balance network load

4. **Testing**: Always test TDMA configuration:
   - In simulation first
   - With hardware-in-the-loop
   - During pre-deployment tests

## Troubleshooting

Common issues and solutions:

| Issue | Cause | Solution |
|-------|-------|----------|
| Acoustic collisions | Overlapping active slots | Verify unique active slots per vehicle |
| Missed transmissions | Insufficient slot time | Increase `slot_duration_seconds` |
| Clock drift | Poor GPS sync | Check GPS fix quality and time sync |
| No activity | Wrong `cycle_start_time` | Verify all vehicles use same epoch |

## References

- ROS Acomms Documentation: `ros_acomms/docs/overview.rst`
- TDMA Test Files: `ros_acomms_tests/src/test_tdma_*.py`
- Launch Examples: `spurdog_acomms/launch/*.launch`
