# NetworkTables-Based AprilTag Steering Implementation

## Overview
This implementation adds autonomous AprilTag steering functionality that reads target data from NetworkTables instead of directly from PhotonVision. This allows for integration with a custom OrangePi vision application or other vision processing systems.

## Features Implemented

### 1. NetworkTables Vision Data Interface
**Location:** `Constants.h` (VisionConstants namespace)

The system reads vision target data from a NetworkTables table with the following keys:
- `hasTarget` (boolean) - Whether a valid target is detected
- `targetYaw` (double) - Horizontal angle to target in degrees
- `targetPitch` (double) - Vertical angle to target in degrees
- `targetArea` (double) - Target area percentage
- `targetID` (int) - AprilTag ID
- `timestamp` (double) - FPGA timestamp when data was captured

**NetworkTables Path:** `/VisionData/`

### 2. Data Freshness Validation
**Location:** `RobotContainer.cpp` (`getNetworkTablesVisionData()` method)

The system validates that vision data is recent and reliable:
- Maximum data age: **0.5 seconds** (configurable in `VisionConstants::kMaxVisionDataAge`)
- Checks timestamp against current FPGA time
- Only processes data from valid reef AprilTags (IDs: 6-11, 17-22)
- Displays status on SmartDashboard:
  - "Vision Status": Shows "FRESH", "STALE DATA", or no target
  - "Vision Data Age": Shows age of data in seconds

### 3. Button Binding
**Location:** `RobotContainer.cpp` (ConfigureButtonBindings method)

**Button:** Driver Controller **A Button**

**Behavior:**
- When pressed: Robot automatically rotates to face the AprilTag while driver controls translation (forward/backward/strafe)
- Driver maintains full control of robot movement direction
- Rotation is automated using PID control
- Falls back to normal driver control if no valid/fresh target is available

### 4. Safety Features
- **Stale Data Protection**: Refuses to act on vision data older than 0.5 seconds
- **Tag Validation**: Only responds to reef AprilTags (prevents steering toward wrong targets)
- **Graceful Fallback**: Returns to normal driver control when target is lost or data becomes stale
- **Visual Feedback**: SmartDashboard displays vision status and data age

## Integration with Custom Vision Application

Your OrangePi vision application should publish data to NetworkTables as follows:

```python
from networktables import NetworkTables

# Initialize NetworkTables
NetworkTables.initialize(server='10.TE.AM.2')  # Replace with your team number
vision_table = NetworkTables.getTable('VisionData')

# When a target is detected:
vision_table.putBoolean('hasTarget', True)
vision_table.putNumber('targetYaw', yaw_angle)      # degrees
vision_table.putNumber('targetPitch', pitch_angle)  # degrees
vision_table.putNumber('targetArea', area)          # percentage
vision_table.putNumber('targetID', apriltag_id)     # integer ID
vision_table.putNumber('timestamp', current_fpga_time)  # IMPORTANT: Use robot FPGA time!

# When no target is detected:
vision_table.putBoolean('hasTarget', False)
```

### Critical: Timestamp Synchronization
The timestamp MUST be the robot's FPGA timestamp, not the OrangePi's system time. Options:
1. Read `/SmartDashboard/FPGATime` from NetworkTables and use it
2. Have the robot periodically publish its time to a known location
3. Use the timestamp from when you read the target data (less accurate but simpler)

## Testing Procedure

### 1. Pre-Test Checklist
- [ ] Deploy code to robot
- [ ] Verify OrangePi vision application is running and publishing to NetworkTables
- [ ] Open SmartDashboard or Shuffleboard
- [ ] Add "Vision Status" and "Vision Data Age" indicators from SmartDashboard

### 2. Bench Test (Robot on Blocks)
1. Place an AprilTag (reef tag ID 6-11 or 17-22) in front of camera
2. Verify NetworkTables data appears in SmartDashboard
3. Press **A button** on driver controller
4. Verify robot attempts to rotate toward target
5. Move AprilTag left/right and verify robot follows

### 3. Field Test
1. Drive robot near a reef with AprilTags
2. Press **A button** while driving
3. Robot should:
   - Continue responding to driver translation inputs
   - Automatically rotate to face the nearest reef AprilTag
   - Smoothly track the target as you move
4. Verify fallback to normal control when:
   - Target is lost from view
   - Data becomes stale (> 0.5 seconds old)

### 4. Monitor for Issues
Watch SmartDashboard during testing:
- "Vision Status" should show "FRESH" when tracking
- "Vision Data Age" should be < 0.5 seconds
- If "STALE DATA" appears frequently, check:
  - Vision processing rate
  - Network latency
  - Timestamp synchronization

## Tuning Parameters

### In Constants.h (VisionConstants)
```cpp
kMaxVisionDataAge = 0.5;  // Maximum data age in seconds (default: 0.5s)
kNTSteeringP = 1.0;       // PID proportional gain (not currently used, reserved)
```

### In DriveSubsystem.cpp
The existing `m_alignPIDController` is used for rotation:
- Current values: P=1.0, I=0.0, D=0.0
- Modify in DriveSubsystem constructor if rotation is too aggressive/sluggish

### Adjust Rotation Speed
In `PhotonDrive()` method (line 226 in DriveSubsystem.cpp):
```cpp
units::radians_per_second_t rotationsPerSecond{rotation/75};
```
- Decrease the divisor (75) for faster rotation
- Increase for slower, smoother rotation

## Button Mapping Summary

| Button | Function |
|--------|----------|
| **A** | NetworkTables-based AprilTag steering (NEW) |
| **B** | PhotonVision-based AprilTag steering (existing) |
| **Left Bumper** | TractorBeam left (full autonomous drive to target) |
| **Right Bumper** | TractorBeam right (full autonomous drive to target) |
| **X** | Zero heading |
| **Back** | Toggle robot relative |
| **Start** | Toggle field relative |

## Troubleshooting

### Robot doesn't respond to A button
1. Check "Vision Status" on SmartDashboard - should show "FRESH"
2. Verify `hasTarget` is true in NetworkTables viewer
3. Check that target ID is a reef tag (6-11, 17-22)

### "STALE DATA" warning
1. Vision processing too slow - optimize OrangePi code
2. Network latency - check WiFi connection
3. Incorrect timestamp - verify FPGA time synchronization

### Robot rotates in wrong direction
1. Yaw angle sign may be inverted
2. Try negating the yaw value in vision application

### Rotation too fast/slow/jerky
1. Adjust PID gains in DriveSubsystem constructor
2. Adjust division factor in PhotonDrive method (currently /75)
3. Check vision update rate (should be > 20 Hz)

## Future Enhancements

Possible improvements:
1. Add distance-based steering (use pitch/area data)
2. Implement separate PID tuning for NT steering vs PhotonVision
3. Add SmartDashboard tuneable PID values
4. Implement predictive tracking for moving targets
5. Add multi-target selection logic

## Code Locations

- **Constants:** `src/main/include/Constants.h` (lines 208-225)
- **Header:** `src/main/include/RobotContainer.h` (lines 19-20, 65-77, 85)
- **Implementation:** `src/main/cpp/RobotContainer.cpp`
  - Constructor initialization: line 52
  - Vision data reader: lines 203-241
  - Button binding: lines 308-322
