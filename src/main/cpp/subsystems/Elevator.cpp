// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/Elevator.h"
#include <frc/controller/PIDController.h>
#include "Constants.h"

using namespace ElevatorConstants;

/*
Todo: will need to update the type of motor used on the intake once it's known
Todo: Will need to make methods that make sense for what the intake will do
- the intake is planned to be a conveyor so will need to turn one direction at a specific speed
- I'm thinking it should be able to be reversed if we need to 
-it'd be nice to know if we have a note loaded 
*/
Elevator::Elevator():m_setPointPIDController(1.0, 0.0, 0.0) {
  // Implementation of subsystem constructor goes here.
  //frc::PWMSparkMax m_conveyorMotor(kconveyorMotorPort);
  SparkMaxConfig defaultConfig;
  SparkMaxConfig smtest2Config;
  smtest2Config.Apply(defaultConfig); //.Follow(m_SMTEST1);
  smtest2Config.Inverted(true);

  m_SMTEST1.Configure(defaultConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
  m_SMTEST2.Configure(smtest2Config, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);


}

void Elevator::JoyControl(double goSpeed) {
      m_elevatorMotor.Set(goSpeed);
      double currentPosition = m_elevatorEncoder.GetPosition();
      frc::SmartDashboard::PutNumber("Elevator Encoder Reading", currentPosition); 
}

frc2::CommandPtr Elevator::GoToBottom() {
  return RunOnce([ this ] {
    double currentPosition = m_elevatorEncoder.GetPosition();
    double setPointAdjustment = m_setPointPIDController.Calculate(currentPosition, 0.0);
    m_elevatorMotor.Set(setPointAdjustment);
    frc::SmartDashboard::PutString("Elevator","Going to bottom");
  });
}

frc2::CommandPtr Elevator::GoToTrough() {
  return RunOnce([ this ] { 
      double currentPosition = m_elevatorEncoder.GetPosition();
      double setPointAdjustment = m_setPointPIDController.Calculate(currentPosition, kTroughSetPoint);
      m_elevatorMotor.Set(setPointAdjustment);
      frc::SmartDashboard::PutString("Elevator","Going to trough");
    });
}
frc2::CommandPtr Elevator::GoToLevel2() {
  return RunOnce([ this ] { 
      double currentPosition = m_elevatorEncoder.GetPosition();
      double setPointAdjustment = m_setPointPIDController.Calculate(currentPosition, kLevelTwoSetPoint);
      m_elevatorMotor.Set(setPointAdjustment);
      frc::SmartDashboard::PutString("Elevator","Going to Level2");
    });
}
frc2::CommandPtr Elevator::GoToLevel3() {
  return RunOnce([ this ] { 
      double currentPosition = m_elevatorEncoder.GetPosition();
      double setPointAdjustment = m_setPointPIDController.Calculate(currentPosition, kLevelThreeSetPoint);
      m_elevatorMotor.Set(setPointAdjustment);
      frc::SmartDashboard::PutString("Elevator","Going to Level 3");
    });
}
frc2::CommandPtr Elevator::GoToLevel4() {
  return RunOnce([ this ] { 
      double currentPosition = m_elevatorEncoder.GetPosition();
      double setPointAdjustment = m_setPointPIDController.Calculate(currentPosition, kLevelFourSetPoint);
      m_elevatorMotor.Set(setPointAdjustment);
      frc::SmartDashboard::PutString("Elevator","Going to Level 4");
    });
}

void Elevator::SMaxTest(){
   m_SMTEST1.Set(0.1);
   m_SMTEST2.Set(0.1);
}

void Elevator::Stop(){
   m_elevatorMotor.Set(0);
  m_SMTEST1.Set(0.0);
  m_SMTEST2.Set(0.0);
  m_elevatorPivot.Set(0.0);
}

double Elevator::CurrentPosition(){
  return m_elevatorEncoder.GetPosition();
}

void Elevator::PivotCoralCollector(double power){
  m_elevatorPivot.Set(power);
}

void Elevator::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void Elevator::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
