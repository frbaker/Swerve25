// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include "Constants.h"
#include <rev/config/SparkMaxConfig.h>

using namespace rev::spark;
class CoralCollector : public frc2::SubsystemBase {
 public:
  CoralCollector();

  /**
   * Example command factory method.
   */
  void RunCoralCollector();
  frc2::CommandPtr RunCoralCollectorAuto();
  void ReverseCoralCollector();
  frc2::CommandPtr ReverseCoralCollectorAuto();
  void Stop();
  frc2::CommandPtr StopAuto();
  void RunCoralCollectorSlower();
  void ReverseCoralCollectorSlower();
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
  //SparkMax m_collector{CoralCollectorConstants::kCoralCollectorCanId, SparkLowLevel::MotorType::kBrushless};
  SparkMax m_coralCollectorLeft{CoralCollectorConstants::kCoralCollectorLeftCanId, SparkLowLevel::MotorType::kBrushless};
  SparkMax m_coralCollectorRight{CoralCollectorConstants::kCoralCollectorRightCanId, SparkLowLevel::MotorType::kBrushless};
  
  
};
