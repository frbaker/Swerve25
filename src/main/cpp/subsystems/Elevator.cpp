// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/Elevator.h"
#include <frc/controller/PIDController.h>
#include "Constants.h"
using namespace ElevatorConstants;
/*
Tunning the PID loop
Gradually increase P value until speed is not slugish, but doesn't overshoot or oscillate 
After being happy with the P value, then adjust the I to increase precision if necessary (- if oscillations appear, decrease - do not be overly agressive with this. We may be fine leaving it at 0)
When happy with P and I, can adjust the D value - it will SMOOTH out the motion, reduce overshooting - but it can amplify any noise (especially if there is occasinal out of normal sensor readings) and make it slugish if not tunned well.
*/

Elevator::Elevator():m_setPointPIDController(0.1, 0.0, 0.0) { //TODO - tune the pid loop
  // Implementation of subsystem constructor goes here.
  /*SparkMaxConfig invertconf;
  invertconf.Inverted(true);

  m_elevatorMotor.Configure(invertconf, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);*/
  m_elevatorEncoder.SetPosition(0);
  sendElevatorTo = kTroughSetPoint;
  
}

void Elevator::JoyControl(double goSpeed) {
  double currentPosition = m_elevatorEncoder.GetPosition();
  frc::SmartDashboard::PutNumber("Elevator Encoder Reading", currentPosition); 
  
  m_elevatorMotor.Set(goSpeed); 
  
  if (goSpeed > 0) {
    //going up if we are not above maxHeight
    /*if (currentPosition < kElevatorMaxHeight) {
     m_elevatorMotor.Set(goSpeed);
     m_rightElevatorMotor.Set(-goSpeed);
    }
    */
    /*if (currentPosition >= kLevelThreeSetPoint){
      sendElevatorTo = kLevelFourSetPoint;
    }
    else if (currentPosition >= kLevelTwoSetPoint){
      sendElevatorTo = kLevelThreeSetPoint;
    }
    else if (currentPosition >= kTroughSetPoint){
      sendElevatorTo = kLevelTwoSetPoint;
    }
    else if (currentPosition >= 0.0) {
      sendElevatorTo = kTroughSetPoint;
    }*/
  }
  else{
    //doing down if we are not below MinHeight
   /* if (currentPosition > kElevatorMinHeight) {
     m_elevatorMotor.Set(goSpeed);
     m_rightElevatorMotor.Set(-goSpeed);
    }
    */
    /*
    if (currentPosition <= kTroughSetPoint){
      sendElevatorTo = 0.0;
    }
    else if (currentPosition <= kLevelTwoSetPoint){
      sendElevatorTo = kTroughSetPoint;
    }
    else if (currentPosition <= kLevelThreeSetPoint){
      sendElevatorTo = kLevelTwoSetPoint;
    }
    else if (currentPosition <= kLevelFourSetPoint) {
      sendElevatorTo = kLevelThreeSetPoint;
    }*/
  }
}

void Elevator::SetpointMovement(){
  /*frc::SmartDashboard::PutString("Running", "Up Another Levl");
  if (sendElevatorTo == 0.0) {
    sendElevatorTo = kTroughSetPoint;
  }
  else if (sendElevatorTo == kTroughSetPoint){
    sendElevatorTo = kLevelTwoSetPoint;
  }
  else if (sendElevatorTo == kLevelTwoSetPoint){
    sendElevatorTo = kLevelThreeSetPoint;
  }
  else if (sendElevatorTo == kLevelThreeSetPoint){
    sendElevatorTo = kLevelFourSetPoint;
  }
  frc::SmartDashboard::PutNumber("Current Elevator Pos", currentPosition);*/
  double currentPosition = m_elevatorEncoder.GetPosition();
  if (std::abs(currentPosition - sendElevatorTo) <= kElevatorTolerance){ //this works like a deadband so if we are close it doesn't keep running motors
    m_elevatorMotor.Set(0.0);   
    }
  else{
    double setPointAdjustment = std::clamp(m_setPointPIDController.Calculate(currentPosition, sendElevatorTo),-0.7, 0.7);
    m_elevatorMotor.Set(setPointAdjustment);
    frc::SmartDashboard::PutNumber("ElevatorSetPoint", setPointAdjustment);
    frc::SmartDashboard::PutNumber("SendElevatorTo", sendElevatorTo);
  }
}


void Elevator::Stop(){
   m_elevatorMotor.Set(0);
}

double Elevator::CurrentPosition(){
  return m_elevatorEncoder.GetPosition();
}

void Elevator::ResetEncoder(){
  m_elevatorEncoder.SetPosition(0);
}


void Elevator::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void Elevator::SetPoint(double point){
  if(point == 0){
    sendElevatorTo = kTroughSetPoint;
  }
    if(point == 1){
    sendElevatorTo = kLevelTwoSetPoint;
  }
  if(point == 2){
    sendElevatorTo = kLevelThreeSetPoint;
  }
  if(point == 3){
    sendElevatorTo = kLevelFourSetPoint;
  }
  frc::SmartDashboard::PutNumber("SendElevatorTo", sendElevatorTo);
}


void Elevator::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
