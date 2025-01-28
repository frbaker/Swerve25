// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/length.h>
#include <units/angle.h>

#include "Constants.h"
#include "utils/SwerveUtils.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>
#include <units/length.h>
#include <cmath>

using namespace pathplanner;
using namespace DriveConstants;


DriveSubsystem::DriveSubsystem()
    : m_frontLeft{kFrontLeftDrivingCanId, kFrontLeftTurningCanId,
                  kFrontLeftChassisAngularOffset},
      m_rearLeft{kRearLeftDrivingCanId, kRearLeftTurningCanId,
                 kRearLeftChassisAngularOffset},
      m_frontRight{kFrontRightDrivingCanId, kFrontRightTurningCanId,
                   kFrontRightChassisAngularOffset},
      m_rearRight{kRearRightDrivingCanId, kRearRightTurningCanId,
                  kRearRightChassisAngularOffset},
      m_odometry{kDriveKinematics,
                 frc::Rotation2d(units::radian_t{m_gyro.GetYaw().GetValue()}),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()}, 
                 frc::Pose2d{}},
                 m_alignPIDController(1.0, 0.0, 0.0), // Example values for Kp, Ki, Kd
                 m_distancePIDController(1.0, 0.0, 0.0) 
                 {

                  // Configure the AutoBuilder last
                 RobotConfig config = RobotConfig::fromGUISettings();

                  // Configure the AutoBuilder last
                  AutoBuilder::configure(
                      [this](){ return GetPose(); }, // Robot pose supplier
                      [this](frc::Pose2d pose){ resetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
                      [this](){ return getRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                      [this](auto speeds, auto feedforwards){ driveRobotRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                      std::make_shared<PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
                          PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                          PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                      ),
                      config, // The robot configuration
                      []() {
                          // Boolean supplier that controls when the path will be mirrored for the red alliance
                          // This will flip the path being followed to the red side of the field.
                          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                          auto alliance = frc::DriverStation::GetAlliance();
                          if (alliance) {
                              return alliance.value() == frc::DriverStation::Alliance::kRed;
                          }
                          return false;
                      },
                      this // Reference to this subsystem to set requirements
                  );
                 }
            

void DriveSubsystem::Periodic() {
  //frc::SmartDashboard::PutNumber("Pigeon", m_gyro.GetYaw().GetValue());
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(frc::Rotation2d(units::radian_t{m_gyro.GetYaw().GetValue()}),
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                     m_frontRight.GetPosition(), m_rearRight.GetPosition()});
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed, //A
                           units::radians_per_second_t rot,
                           bool fieldRelative,
                           bool rateLimit
                           ) {
  double xSpeedCommanded;
  double ySpeedCommanded;

  if (rateLimit) {
    // Convert XY to polar for rate limiting
    double inputTranslationDir = atan2(ySpeed.value(), xSpeed.value());
    double inputTranslationMag =
        sqrt(pow(xSpeed.value(), 2) + pow(ySpeed.value(), 2));

    // Calculate the direction slew rate based on an estimate of the lateral
    // acceleration
    double directionSlewRate;
    if (m_currentTranslationMag != 0.0) {
      directionSlewRate =
          abs(DriveConstants::kDirectionSlewRate / m_currentTranslationMag);
    } else {
      directionSlewRate = 500.0;  // some high number that means the slew rate
                                  // is effectively instantaneous
    }

    double currentTime = wpi::Now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    double angleDif = SwerveUtils::AngleDifference(inputTranslationDir,
                                                   m_currentTranslationDir);
    if (angleDif < 0.45 * std::numbers::pi) {
      m_currentTranslationDir = SwerveUtils::StepTowardsCircular(
          m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
    } else if (angleDif > 0.85 * std::numbers::pi) {
      if (m_currentTranslationMag >
          1e-4) {  // some small number to avoid floating-point errors with
                   // equality checking
        // keep currentTranslationDir unchanged
        m_currentTranslationMag = m_magLimiter.Calculate(0.0);
      } else {
        m_currentTranslationDir =
            SwerveUtils::WrapAngle(m_currentTranslationDir + std::numbers::pi);
        m_currentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
      }
    } else {
      m_currentTranslationDir = SwerveUtils::StepTowardsCircular(
          m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.Calculate(0.0);
    }
    m_prevTime = currentTime;

    xSpeedCommanded = m_currentTranslationMag * cos(m_currentTranslationDir);
    ySpeedCommanded = m_currentTranslationMag * sin(m_currentTranslationDir);
    m_currentRotation = m_rotLimiter.Calculate(rot.value());

  } else {
    xSpeedCommanded = xSpeed.value();
    ySpeedCommanded = ySpeed.value();
    m_currentRotation = rot.value();
  }

  // Convert the commanded speeds into the correct units for the drivetrain
  units::meters_per_second_t xSpeedDelivered =
      xSpeedCommanded * DriveConstants::kMaxSpeed;
  units::meters_per_second_t ySpeedDelivered =
      ySpeedCommanded * DriveConstants::kMaxSpeed;
  units::radians_per_second_t rotDelivered =
      m_currentRotation * DriveConstants::kMaxAngularSpeed;

  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                frc::Rotation2d(units::radian_t{m_gyro.GetYaw().GetValue()}))
          : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered});

  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetX() {
  m_frontLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
  m_frontRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
  frc::SmartDashboard::PutString("Running", "SetX");
}

void DriveSubsystem::TractorBeam(units::meter_t forward, bool left, units::degree_t yaw, double targetArea){
  //Calculate how far forward to drive
  double howFarDouble = forward.value();
  double forwardPid = m_distancePIDController.Calculate(howFarDouble, 0.0); //adjust the second param to stop sooner or later
  //units::meters_per_second_t howFar{forwardPid/60}; //adjust the division to speed up or slow down after seeing robot behavior
  units::meters_per_second_t howFar{0.3_mps}; //adjust the division to speed up or slow down after seeing robot behavior

  //Calculate strafe and the offset (6 inches either side of center of the april tag?)
  const units::meter_t DESIRED_OFFSET = 0.0_m; // 6 inches (0.1524 meters) to the left or right of the april tag (adjust after seeing robot behavior)
  units::radian_t desiredStrafeAngle = units::radian_t(atan2(DESIRED_OFFSET.value(), forward.value())); //right side of reef
  if (left) {
    units::radian_t desiredStrafeAngle = units::radian_t(atan2(-DESIRED_OFFSET.value(), forward.value())); //left side of reef
  }
  units::radian_t currentStrafeAngle = units::radian_t(atan2(0.0, forward.value())); // Assuming camera is centered; adjust if not
  units::radian_t strafeAngleError = desiredStrafeAngle - currentStrafeAngle;
  const double kp_strafe = 0.1; // Proportional gain for strafe, adjustments may be needed after seeing robot behavior
  double strafeAdjustment = strafeAngleError.value() * kp_strafe;
  units::meters_per_second_t strafeCommand = units::meters_per_second_t{strafeAdjustment};

  //units::meters_per_second_t strafeCommand = 0.0_m; //If we are having trouble after seeing robot behavior - try this instead to eliminate variables

  //Calculate rotation to be aligned to april tag
  double rotation = m_alignPIDController.Calculate(yaw.value(), 0.0);
  units::radians_per_second_t rotationsPerSecond{rotation/75};

  //Tractorbeam to the april tag (changed from 11.70)
  if (targetArea < 11.60){
    frc::SmartDashboard::PutNumber("targetAreaAfterPass", targetArea);
    Drive(howFar, strafeCommand, rotationsPerSecond, false, true); //field relative needs to be false
  }
  else{
    Drive(0_mps, 0_mps, 0_rad_per_s, FIELD_RELATIVE, true);
  }
}

void DriveSubsystem::PhotonDrive(units::meters_per_second_t forward, units::meters_per_second_t strafe, units::degree_t yaw){
  double rotation = m_alignPIDController.Calculate(yaw.value(), 0.0);
  units::radians_per_second_t rotationsPerSecond{rotation/75};
  Drive(
    forward,
    strafe,
    rotationsPerSecond,
    FIELD_RELATIVE,
    true
  );
  /*
    units::meters_per_second_t xSpeed,
    units::meters_per_second_t ySpeed, //A
    units::radians_per_second_t rot, 
    bool fieldRelative,
    bool rateLimit
  */
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         DriveConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

units::degree_t DriveSubsystem::GetHeading() /*const*/ {
  return frc::Rotation2d(units::radian_t{m_gyro.GetYaw().GetValue()}).Degrees();
}

void DriveSubsystem::ZeroHeading() { m_gyro.Reset(); }

frc::Pose2d DriveSubsystem::GetPose() { return m_odometry.GetPose(); }

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
      //A
}
void DriveSubsystem::resetPose(frc::Pose2d pose) {
    // Reset the odometry to the given pose
    m_odometry.ResetPosition(
        frc::Rotation2d(units::radian_t{m_gyro.GetYaw().GetValue()}),
        {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
        pose);
}

frc::ChassisSpeeds DriveSubsystem::getRobotRelativeSpeeds() {
    // Assuming you can get the speed of each module, calculate robot-relative speeds
    // This is an approximation; you'd need to know the current state of each module
    auto flState = m_frontLeft.GetState();
    auto frState = m_frontRight.GetState();
    auto blState = m_rearLeft.GetState();
    auto brState = m_rearRight.GetState();

    // Convert module states to chassis speeds using kinematics, assuming robot relative
    return kDriveKinematics.ToChassisSpeeds({flState, frState, blState, brState});
}

void DriveSubsystem::driveRobotRelative(frc::ChassisSpeeds speeds) {
    // Convert chassis speeds to module states and set them
    auto states = kDriveKinematics.ToSwerveModuleStates(speeds);
    kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

    // Set each module state
    m_frontLeft.SetDesiredState(states[0]);
    m_frontRight.SetDesiredState(states[1]);
    m_rearLeft.SetDesiredState(states[2]);
    m_rearRight.SetDesiredState(states[3]);
}