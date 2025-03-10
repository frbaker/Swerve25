// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include "Constants.h"
#include <frc/filter/SlewRateLimiter.h>
#include <frc/controller/PIDController.h>

using namespace rev::spark;
class Pivot : public frc2::SubsystemBase {
 public:
  Pivot();

  /**
   * Example command factory method.
   */
  void RunPivot();
  frc2::CommandPtr RunPivotAuto();
  void ReversePivot();
  frc2::CommandPtr ReversePivotAuto();
  void Stop();
  frc2::CommandPtr StopAuto();
  void ResetEncoder();
  void ResetEncoderDown();
  void SetPoint(double point);
  void SetpointMovement();
  
void RunReducedPivotSpeed();

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  bool is_arm_up();

double CurrentPosition();

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
  SparkMax m_Pivot{PivotConstants::kPivotCanid, SparkLowLevel::MotorType::kBrushless};
  SparkRelativeEncoder m_PivotEncoder = m_Pivot.GetEncoder();
  frc::SlewRateLimiter<units::scalar> m_pivotSlewLimiter{0.02 / 3_s};
  frc::PIDController m_setPointPIDController;
  double sendPivotTo;
};