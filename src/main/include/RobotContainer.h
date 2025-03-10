// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/CommandPtr.h>
#include <rev/SparkMax.h>
#include <rev/SparkLowLevel.h>
#include "Constants.h"
#include <photon/PhotonCamera.h>
#include "subsystems/DriveSubsystem.h"
#include "subsystems/CoralCollector.h"
#include "subsystems/Elevator.h"
#include "subsystems/Pivot.h"
#include "subsystems/Climber.h"
#include "subsystems/LEDS.h"
/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
  frc::XboxController m_coDriverController{OIConstants::kCoDriverControllerPort};

  frc::DigitalInput m_ElevatorSwitch{0};
  frc::DigitalInput m_PivotSwitch{1};
  frc::DigitalInput m_PivotDownSwitch{2};
                      
  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;
  CoralCollector m_collector;
  Pivot m_pivot;
  Elevator m_elevator;
  Climber m_climber;
  LEDS m_leds;

  frc::SendableChooser<frc2::Command*> m_chooser;

  photon::PhotonCamera camera{"boom"};
  bool isValueInArray(int value, int array[], int size);
  void DriverControl();
  void ElevatorControl();
  void ClimberControl();
  void coDriverControl();
  photon::PhotonTrackedTarget hasValidAprilTagTarget();
  // The chooser for the autonomous routines
  

  void ConfigureButtonBindings();
  double elevatorOverrideHeight;
  bool fieldRelative;
};