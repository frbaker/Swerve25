// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/Pivot.h"
#include <frc/controller/PIDController.h>
#include "Constants.h"
#include <rev/config/SparkMaxConfig.h>

using namespace PivotConstants;

Pivot::Pivot():m_setPointPIDController(0.1, 0.0, 0.0) {
  SparkMaxConfig pivFollowerConfigObj;
  pivFollowerConfigObj.OpenLoopRampRate(1.75);
  m_Pivot.Configure(pivFollowerConfigObj, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kNoPersistParameters);
  m_PivotEncoder.SetPosition(0);
  sendPivotTo = 0;
}

void Pivot::RunPivot(){
  m_Pivot.Set(kPivotSpeed);
}
frc2::CommandPtr Pivot::RunPivotAuto(){
  return RunOnce([this]{
  m_Pivot.Set(kPivotSpeed);
  });
}
void Pivot::RunReducedPivotSpeed(){
  m_Pivot.Set(kPivotSpeedDown);
}
void Pivot::ReversePivot(){
  m_Pivot.Set(-kPivotSpeed);
}
frc2::CommandPtr Pivot::ReversePivotAuto(){
  return RunOnce([this] {
  m_Pivot.Set(-kPivotSpeed);
  });
}

void Pivot::Stop(){
  m_Pivot.Set(0.0);
}
frc2::CommandPtr Pivot::StopAuto(){
  return RunOnce([this] {
  m_Pivot.Set(0.0);
  });
}

double Pivot::CurrentPosition(){
  return m_PivotEncoder.GetPosition();
}

bool is_arm_up(){
  return false;
}

void Pivot::ResetEncoder(){
  m_PivotEncoder.SetPosition(0);
}

void Pivot::SetPoint(double point){
  if(point == 0){
    sendPivotTo = kTroughSetPoint;
  }
  if(point == 1){
    sendPivotTo = kLevelTwoThreeSetPoint;
  }
  if(point == 2){
    sendPivotTo = kLevelTwoThreeSetPoint;
  }
  if(point == 3){
    sendPivotTo = kLevelFourSetPoint;
  }
}

void Pivot::SetpointMovement(){
  double currentPosition = m_PivotEncoder.GetPosition();
  if (std::abs(currentPosition - sendPivotTo) <= kPivotTolerance){ //this works like a deadband so if we are close it doesn't keep running motors
    m_Pivot.Set(0.0);   
    }
  else{
    double setPointAdjustment = std::clamp(m_setPointPIDController.Calculate(currentPosition, sendPivotTo),-0.3, 0.3); // todo - may need to adjust the clamp values
    m_Pivot.Set(setPointAdjustment);
    frc::SmartDashboard::PutNumber("PivotSetPoint", setPointAdjustment);
    frc::SmartDashboard::PutNumber("SendPivotTo", sendPivotTo);
  }

}

/*bool Arm::RunArm() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
    if (is_arm_up()){
      m_arm.Set(kArmSpeed);
      m_arm2.Set(kArmSpeed);
      frc::SmartDashboard::PutString("Arm_Angle","Up"); 
      return true;
    }
    else {
      frc::SmartDashboard::PutString("Arm_Angle","Already Loaded");
      m_arm.Set(0);
      return false;
    }
    
  //RunOnce creates a command that calls a lambda once, and then finishes.
}

void Arm::ReverseArm() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  
    m_arm.Set(-kArmSpeed);
    m_arm2.Set(-kArmSpeed);
    frc::SmartDashboard::PutString("Arm_Angle","Down"); 
    
}

void Arm::Stop(){
  frc::SmartDashboard::PutString("Arm_Angle","notGoing"); 
  m_arm.Set(0);
  m_arm2.Set(0);
}

bool Arm::is_arm_up() {
  // Query some boolean state, such as a digital sensor.
  return m_clicker.Get();
}*/

void Pivot::Periodic() {
  // Implementation of subsystem periodic method goes here.
  frc::SmartDashboard::PutString("Arm_Subsystem","Periodic"); 
  frc::SmartDashboard::PutNumber("pivot angle", CurrentPosition());
}

void Pivot::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
