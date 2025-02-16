// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <frc/controller/PIDController.h>
#include "Constants.h"
#include <rev/config/SparkMaxConfig.h>

using namespace rev::spark;
class Elevator : public frc2::SubsystemBase {
 public:
  Elevator();

  /**
   * Example command factory method.
   */
  void JoyControl(double goSpeed);
  frc2::CommandPtr GoToBottom();
  frc2::CommandPtr GoToTrough();
  frc2::CommandPtr GoToLevel2();
  frc2::CommandPtr GoToLevel3();
  frc2::CommandPtr GoToLevel4();
  void SetpointMovement();
  void SetPoint(double point);

  void Stop();
  double CurrentPosition();
  void ResetEncoder();
  
  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  SparkMax m_elevatorMotor{ElevatorConstants::kElevatorLeftCanId, SparkLowLevel::MotorType::kBrushless};
  SparkRelativeEncoder m_elevatorEncoder = m_elevatorMotor.GetEncoder();

  //SparkMax m_elevatorPivot{ElevatorConstants::kElevatorPivotCanId, SparkLowLevel::MotorType::kBrushless};
 
  frc::PIDController m_setPointPIDController;
  double sendElevatorTo = 0.0;
};
