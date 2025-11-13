// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#include "MAXSwerveModule.h"

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed,
             units::radians_per_second_t rot,
             bool fieldRelative,
             bool rateLimit);

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  void SetX();

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  /**
   * Sets the drive MotorControllers to a power from -1 to 1.
   */
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  units::degree_t GetHeading() const;

  /**
   * Zeroes the heading of the robot.
   */
  void ZeroHeading();

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose() const;

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(const frc::Pose2d& pose);

  /**
   * Returns the kinematics for the drive base.
   */
  frc::SwerveDriveKinematics<4>& GetKinematics();

  /**
   * Returns the robot-relative chassis speeds.
   */
  frc::ChassisSpeeds GetRobotRelativeSpeeds() const;

  /**
   * Drives the robot using robot-relative chassis speeds.
   */
  void DriveRobotRelative(const frc::ChassisSpeeds& speeds);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // MAXSwerve modules
  MAXSwerveModule m_frontLeft;
  MAXSwerveModule m_frontRight;
  MAXSwerveModule m_rearLeft;
  MAXSwerveModule m_rearRight;

  // The Pigeon2 IMU sensor
  ctre::phoenix6::hardware::Pigeon2 m_gyro{DriveConstants::kPigeonCanId};

  // Slew rate filter variables for controlling lateral acceleration
  double m_currentRotation = 0.0;
  double m_currentTranslationDir = 0.0;
  double m_currentTranslationMag = 0.0;

  frc::SlewRateLimiter<units::scalar> m_magLimiter{
      DriveConstants::kMagnitudeSlewRate / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{
      DriveConstants::kRotationalSlewRate / 1_s};
  double m_prevTime = wpi::Now() * 1e-6;

  // Odometry class for tracking robot pose
  frc::SwerveDriveOdometry<4> m_odometry;

  // Kinematics for swerve drive
  frc::SwerveDriveKinematics<4> m_kinematics{
      frc::Translation2d{DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2},
      frc::Translation2d{DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2},
      frc::Translation2d{-DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2},
      frc::Translation2d{-DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2}};
};
