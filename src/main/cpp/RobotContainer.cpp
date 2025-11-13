// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/JoystickButton.h>

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Configure the button bindings
  ConfigureButtonBindings();
  ConfigureDefaultCommands();

  // Add autonomous options to the chooser
  m_chooser.SetDefaultOption("None", "None");
  frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);
}

void RobotContainer::ConfigureDefaultCommands() {
  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
            true,  // Field-relative driving
            true); // Rate limiting enabled
      },
      {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
  // Zero heading (X button)
  frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kX)
      .OnTrue(frc2::InstantCommand([this] { m_drive.ZeroHeading(); }, {&m_drive})
                  .ToPtr());

  // Set X formation (Y button) - locks wheels in X pattern to prevent movement
  frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kY)
      .WhileTrue(frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}).ToPtr());

  // Reset encoders (B button)
  frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kB)
      .OnTrue(frc2::InstantCommand([this] { m_drive.ResetEncoders(); }, {&m_drive})
                  .ToPtr());

  // TODO: Add more button bindings as needed for your 2026 robot
}

std::optional<frc2::CommandPtr> RobotContainer::GetAutonomousCommand() {
  // No autonomous command for now - ready for 2026 game
  // This will be configured once the 2026 game is revealed
  return std::nullopt;
}
