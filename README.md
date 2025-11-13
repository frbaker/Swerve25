# Swerve26 - FRC 2026 Robot Code

Team 3267's robot code for the 2026 FRC season, featuring a REVrobotics MAXSwerve drivetrain.

## Overview

This repository contains a clean, production-ready swerve drive implementation ready for the 2026 FRC season. The codebase has been refreshed from the 2025 season with all game-specific code removed, leaving only the proven swerve drive system.

## Features

- **MAXSwerve Drive System**: Complete 4-module swerve drive with NEO motors
- **Modern C++20**: Clean, well-documented code following FRC best practices
- **Field-Relative & Robot-Relative Modes**: Flexible drive control options
- **Slew Rate Limiting**: Smooth acceleration for better driver control
- **Odometry & Pose Estimation**: Track robot position on the field
- **Command-Based Framework**: Modular, maintainable code structure
- **Simulation Support**: Test code without physical hardware

## Hardware Configuration

### Swerve Modules
- **Motors**: NEO brushless motors (drive & steering)
- **Controllers**: SPARK MAX motor controllers
- **Encoders**: Through-bore absolute encoders
- **Wheels**: 3" diameter wheels
- **Pinion**: 14T (configurable for 12T, 13T, or 14T)

### Electronics
- **RoboRIO 2**: Main robot controller
- **Pigeon2 IMU**: Gyroscope for heading (CAN ID 9)
- **8x SPARK MAX**: 4 drive + 4 steering controllers

### CAN IDs (Update in Constants.h)
```
Drive Motors:        Steering Motors:
- Front Left:  11    - Front Left:  10
- Rear Left:   13    - Rear Left:   12  
- Front Right: 15    - Front Right: 14
- Rear Right:  17    - Rear Right:  16

Pigeon2 IMU: 9
```

## Software Requirements

- **WPILib 2025.1.1** (ready for 2026 update when released)
- **Visual Studio Code** with WPILib extension
- **C++20 Compiler** (included with WPILib)

### Vendor Dependencies
The following vendor libraries are configured:
- REVLib (SPARK MAX motor controllers)
- Phoenix6 (CTRE Pigeon2 IMU)
- PathPlanner (Autonomous path following - optional)
- PhotonLib (Vision processing - optional)

## Getting Started

### 1. Clone the Repository
```bash
git clone <your-repo-url>
cd Swerve25
```

### 2. Configure Your Robot
Edit `src/main/include/Constants.h` and update:
- CAN IDs to match your robot's wiring
- Chassis dimensions (track width & wheel base)
- Module angular offsets (must be calibrated)
- PID gains (tune for your specific robot)

### 3. Build the Project
```bash
./gradlew build
```

### 4. Deploy to Robot
```bash
./gradlew deploy
```

## Calibration Guide

### Module Angular Offsets
1. Manually align all wheels to point forward (0 degrees)
2. Read the absolute encoder values for each module
3. Update `kFrontLeftChassisAngularOffset` etc. in Constants.h
4. The offsets compensate for module mounting positions

### PID Tuning
Located in `src/main/include/Constants.h`:
- **Drive PID**: Start with P=0.04, tune for smooth speed control
- **Turning PID**: Start with P=1.0, tune for accurate angle control

### Dimensions
Measure and update in Constants.h:
- **Track Width**: Distance between left/right wheel centers
- **Wheel Base**: Distance between front/back wheel centers

## Driver Controls

### Xbox Controller Layout
**Left Stick**: 
- Y-axis: Forward/Backward
- X-axis: Strafe Left/Right

**Right Stick**:
- X-axis: Rotate

**Buttons**:
- **X**: Zero gyro heading (reset field-relative)
- **Y**: Set X-formation (lock wheels)
- **B**: Reset drive encoders

*Field-relative driving is enabled by default*

## Code Structure

```
src/main/
├── cpp/
│   ├── Robot.cpp              # Main robot class
│   ├── RobotContainer.cpp     # Command bindings & subsystems
│   ├── Constants.cpp          # Constant implementations
│   └── subsystems/
│       ├── DriveSubsystem.cpp # Swerve drive control
│       └── MAXSwerveModule.cpp# Individual module control
├── include/
│   ├── Robot.h
│   ├── RobotContainer.h
│   ├── Constants.h            # ALL CONSTANTS - EDIT THIS
│   ├── Configs.h              # SPARK MAX configurations
│   └── subsystems/
│       ├── DriveSubsystem.h
│       └── MAXSwerveModule.h
└── deploy/                    # PathPlanner paths, config files
```

## For the 2026 Season

This codebase is a clean starting point. When the 2026 game is revealed:

1. **Add Game-Specific Subsystems**: Create new subsystem folders for manipulators, shooters, etc.
2. **Implement Autonomous**: Use PathPlanner to create autonomous routines
3. **Add Vision**: PhotonLib is already included for AprilTag tracking
4. **Tune Constants**: Update speeds, accelerations based on game requirements

## Simulation

To run robot simulation:
```bash
./gradlew simulateNative
```

Then connect to the simulator with:
- **FRC Driver Station**
- **Glass** (for visualizing robot state)

## Troubleshooting

### Build Errors
- Ensure WPILib 2025 is properly installed
- Check that vendor libraries are up to date: `./gradlew vendordep`
- Clean build: `./gradlew clean build`

### Module Not Turning Correctly
- Verify CAN IDs match physical wiring
- Check angular offsets are calibrated
- Confirm absolute encoders are properly connected

### Robot Drives Sideways
- Field-relative mode may need gyro zeroing (press X button)
- Check that gyro is mounted correctly and CAN ID is right
- Verify module order in DriveSubsystem constructor

## Resources

- [WPILib Documentation](https://docs.wpilib.org/)
- [REV Robotics MAXSwerve](https://docs.revrobotics.com/brushless/maxswerve)
- [CTRE Phoenix 6 Docs](https://v6.docs.ctr-electronics.com/)
- [PathPlanner](https://pathplanner.dev/)
- [Chief Delphi Forums](https://www.chiefdelphi.com/)

## Team Information

**Team**: 3267  
**Season**: 2026  
**Drivetrain**: REVrobotics MAXSwerve  
**Programming Language**: C++20  

## License

Copyright (c) FIRST and other WPILib contributors.
Open Source Software; you can modify and/or share it under the terms of
the WPILib BSD license file in the root directory of this project.

---

**Ready for the 2026 season!** When the game is revealed, this codebase provides a solid foundation to build upon.
