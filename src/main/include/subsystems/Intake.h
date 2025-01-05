// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include "Constants.h"

using namespace rev::spark;
class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  /**
   * Example command factory method.
   */
  frc2::CommandPtr RunIntake();
  frc2::CommandPtr ReverseIntake();
  frc2::CommandPtr Stop();
  
  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  bool isLoaded();

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
  SparkMax m_conveyorMotor{IntakeConstants::kConveyorCanId, SparkLowLevel::MotorType::kBrushless};
  //rev::SparkMax m_conveyorMotor(int deviceID, rev::SparkLowLevel::MotorType type);
};
