// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/Pivot.h"
#include "Constants.h"

using namespace PivotConstants;

Pivot::Pivot() {
  
}

void Pivot::RunPivot(){
  m_Pivot.Set(kPivotSpeed);
}
frc2::CommandPtr Pivot::RunPivotAuto(){
  m_Pivot.Set(kPivotSpeed);
}
void Pivot::RunReducedPivotSpeed(){
  m_Pivot.Set(kPivotSpeedDown);
}
void Pivot::ReversePivot(){
  m_Pivot.Set(-kPivotSpeed);
}
frc2::CommandPtr Pivot::ReversePivotAuto(){
  m_Pivot.Set(-kPivotSpeed);
}

void Pivot::Stop(){
  m_Pivot.Set(0.0);
}
frc2::CommandPtr Pivot::StopAuto(){
  m_Pivot.Set(0.0);
}

double Pivot::CurrentPosition(){
  return m_PivotEncoder.GetPosition();
}

bool is_arm_up(){
  return false;
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
}

void Pivot::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
