// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/SparkMax.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>
#include <numbers>

#pragma once
using namespace rev::spark;
/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    constexpr units::meters_per_second_t kMaxSpeed = 2_mps;
    //constexpr units::meters_per_second_t kMaxSpeed = 4.8_mps;
    constexpr units::radians_per_second_t kMaxAngularSpeed{2 * std::numbers::pi};

    constexpr double kDirectionSlewRate = 1.2;   // radians per second
    constexpr double kMagnitudeSlewRate = 1.8;   // percent per second (1 = 100%)
    constexpr double kRotationalSlewRate = 2.0;  // percent per second (1 = 100%)

    // Chassis configuration
    /*
    Todo: add TrackWidth and WheelBase measurements
    */
   // constexpr units::meter_t kTrackWidth = 0.6731_m;  //24 5/8" Distance between centers of right and left wheels on robot
    //constexpr units::meter_t kWheelBase = 0.6731_m;  // 28" Distance between centers of front and back wheels on robot


    constexpr units::meter_t kTrackWidth = 0.6515_m;  //24 5/8" Distance between centers of right and left wheels on robot
    constexpr units::meter_t kWheelBase = 0.7112_m;  // 28" Distance between centers of front and back wheels on robot

    // Angular offsets of the modules relative to the chassis in radians
    constexpr double kFrontLeftChassisAngularOffset = -std::numbers::pi / 2;
    constexpr double kFrontRightChassisAngularOffset = 0;
    constexpr double kRearLeftChassisAngularOffset = std::numbers::pi;
    constexpr double kRearRightChassisAngularOffset = std::numbers::pi / 2;


    /*
    Todo: Configure Can Ids and update accordingly
    */
    // Pigeon IMU
    constexpr int kPigeonIMU = 9;

    // SPARK MAX CAN IDs
    constexpr int kFrontLeftDrivingCanId = 16;
    constexpr int kRearLeftDrivingCanId = 17;
    constexpr int kFrontRightDrivingCanId = 13;
    constexpr int kRearRightDrivingCanId = 22;

    constexpr int kFrontLeftTurningCanId = 14;
    constexpr int kRearLeftTurningCanId = 20;
    constexpr int kFrontRightTurningCanId = 29;
    constexpr int kRearRightTurningCanId = 18;

    //needs to be adjusted per our robot
  const units::meter_t CAMERA_HEIGHT = 24_in;

// Angle between horizontal and the camera.
  const units::radian_t CAMERA_PITCH = 0_deg;

    //Each target may be at a different height
  const units::meter_t TARGET_HEIGHT = 17_cm;

}  // namespace DriveConstants


/*
Todo: Update the module data based on actual build
*/
namespace ModuleConstants {
    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of the steering motor in the MAXSwerve Module.
    constexpr bool kTurningEncoderInverted = true;

    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    constexpr int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    constexpr double kDrivingMotorFreeSpeedRps = 5676.0 / 60;  // NEO free speed is 5676 RPM
    constexpr units::meter_t kWheelDiameter = 0.0762_m; //3"wheels
    constexpr units::meter_t kWheelCircumference = kWheelDiameter * std::numbers::pi;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    constexpr double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    constexpr double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /kDrivingMotorReduction;

    constexpr double kDrivingEncoderPositionFactor = (kWheelDiameter.value() * std::numbers::pi) /kDrivingMotorReduction;  // meters
    constexpr double kDrivingEncoderVelocityFactor = ((kWheelDiameter.value() * std::numbers::pi) / kDrivingMotorReduction) /60.0;  // meters per second

    constexpr double kTurningEncoderPositionFactor = (2 * std::numbers::pi);  // radians
    constexpr double kTurningEncoderVelocityFactor = (2 * std::numbers::pi) / 60.0;  // radians per second

    constexpr units::radian_t kTurningEncoderPositionPIDMinInput = 0_rad;
    constexpr units::radian_t kTurningEncoderPositionPIDMaxInput = units::radian_t{kTurningEncoderPositionFactor};

    constexpr double kDrivingP = 0.04;
    constexpr double kDrivingI = 0;
    constexpr double kDrivingD = 0;
    constexpr double kDrivingFF = (1 / kDriveWheelFreeSpeedRps);
    constexpr double kDrivingMinOutput = -1;
    constexpr double kDrivingMaxOutput = 1;

    constexpr double kTurningP = 1;
    constexpr double kTurningI = 0;
    constexpr double kTurningD = 0;
    constexpr double kTurningFF = 0;
    constexpr double kTurningMinOutput = -1;
    constexpr double kTurningMaxOutput = 1;

    constexpr SparkMax::IdleMode kDrivingMotorIdleMode = SparkMax::IdleMode::kBrake;
    constexpr SparkMax::IdleMode kTurningMotorIdleMode = SparkMax::IdleMode::kBrake;

    constexpr units::ampere_t kDrivingMotorCurrentLimit = 50_A;
    constexpr units::ampere_t kTurningMotorCurrentLimit = 20_A;
}  // namespace ModuleConstants

/*
Todo: ensure these values makes sense for our auto
*/
namespace AutoConstants {
    constexpr auto kMaxSpeed = 1_mps;
    constexpr auto kMaxAcceleration = 3_mps_sq;
    constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
    constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

    constexpr double kPXController = 0.5;
    constexpr double kPYController = 0.5;
    constexpr double kPThetaController = 0.5;

    extern const frc::TrapezoidProfile<units::radians>::Constraints kThetaControllerConstraints;
}  // namespace AutoConstants

namespace OIConstants {
    constexpr int kDriverControllerPort = 0;
    constexpr int kCoDriverControllerPort = 1;
    constexpr double kDriveDeadband = 0.10;
}  // namespace OIConstants

namespace IntakeConstants {
    constexpr int kConveyorCanId = 62;
    constexpr double kIntakeSpeed = 0.3;
}

namespace ArmConstants {
 constexpr int kArmCanid = 61;
 constexpr int kArmCanid2 = 60; 
 constexpr double kArmSpeed = 0.1; 
 constexpr int kArmDio = 0;
}
namespace PivotConstants {
 constexpr int kPivotCanid = 59; 
 constexpr double kPivotSpeed = 0.3; 

}





