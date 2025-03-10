// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"


#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/time.h>


#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Elevator.h"
#include "subsystems/CoralCollector.h"
#include <photon/PhotonUtils.h>

#include <frc/DriverStation.h>

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>

using namespace pathplanner;
using namespace DriveConstants;

RobotContainer::RobotContainer() {
    NamedCommands::registerCommand("Setpoint3", m_elevator.SetPointCmd(2));
    NamedCommands::registerCommand("lowerPivot", m_pivot.ReversePivotAuto());
    NamedCommands::registerCommand("stopPivot", m_pivot.StopAuto());
    NamedCommands::registerCommand("reverseCoralCollector", m_collector.ReverseCoralCollectorAuto());
    /*NamedCommands::registerCommand("stopCollector", m_collector.StopAuto());
    NamedCommands::registerCommand("reverseCollector", m_collector.ReverseCoralCollectorAuto());
    NamedCommands::registerCommand("raisePivot", m_pivot.RunPivotAuto());
    NamedCommands::registerCommand("lowerPivot", m_pivot.ReversePivotAuto());
    NamedCommands::registerCommand("stopPivot", m_pivot.StopAuto());*/
    //NamedCommands::registerCommand("autoScore", std::move(m_drive.PhotonDrive2()));
    //NamedCommands::registerCommand("autoScore", std::move(PhotonDrive2Command(&m_drive)));

    //initial values set in constants.h 
    elevatorOverrideHeight = kElevatorForceDriveToCoDriverHeight;
    fieldRelative = FIELD_RELATIVE;
  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
    m_drive.SetDefaultCommand(frc2::RunCommand(
        [this] {
        DriverControl();
        },
        {&m_drive}));

    m_elevator.SetDefaultCommand(frc2::RunCommand(
        [this]{
            ElevatorControl();
        },
        {&m_elevator}));

    m_climber.SetDefaultCommand(frc2::RunCommand(
        [this]{
            ClimberControl();
        },
        {&m_climber}));
}

void RobotContainer::ElevatorControl() {
    /*units::unit_t<double> joyValue{m_coDriverController.GetRightY()};
    units::unit_t<double> ramping = elevatorRamp.Calculate(joyValue);
    m_elevator.JoyControl(frc::ApplyDeadband(ramping.value(), OIConstants::kDriveDeadband / 2));*/
    m_elevator.JoyControl(frc::ApplyDeadband(m_coDriverController.GetRightY(), OIConstants::kDriveDeadband));
    if(!m_ElevatorSwitch.Get()){ //False when pushed down 
        m_elevator.ResetEncoder();
    } //A
    if(!m_PivotSwitch.Get()){ //False when pushed down //A
        m_pivot.ResetEncoder(); //A
    } //A
    if(!m_PivotDownSwitch.Get()){
        m_pivot.ResetEncoderDown();
    }
    if(m_coDriverController.GetPOV() == 0){ //A
        m_elevator.SetPoint(3); //A
        frc::SmartDashboard::PutString("Level", "Level 4"); //A
    }
    if(m_coDriverController.GetPOV() == 90){ //A
        m_elevator.SetPoint(2); //A
        frc::SmartDashboard::PutString("Level", "Level 3"); //A
    }
    if(m_coDriverController.GetPOV() == 270){ //A
        m_elevator.SetPoint(1); //A 
        frc::SmartDashboard::PutString("Level", "Level 2"); //A
    } //A
    if(m_coDriverController.GetPOV() == 180){ //A
        m_elevator.SetPoint(0); //A
        frc::SmartDashboard::PutString("Level", "Trough"); //A
    } //A
}

void RobotContainer::ClimberControl(){ //A
    m_climber.RunClimber(frc::ApplyDeadband((-m_driverController.GetRightTriggerAxis()/3) * (m_climber.EncoderValue() > -50), OIConstants::kDriveDeadband)); //A
    if(m_driverController.GetAButtonPressed()){ //A
        fieldRelative = m_climber.MovePigeon(); //A
    } //A

   if(m_coDriverController.GetXButtonPressed()){ //A
        m_leds.TurnOnLED(m_coDriverController.GetXButtonPressed()); //A
    } //A

} //A



void RobotContainer::DriverControl() { 
        //when the elevator is up, the drive control is passed to the co-driver for small adjustments to line up to score
        if (m_elevator.CurrentPosition() > elevatorOverrideHeight) {
            //situations change rapidly - allow the driver to override and take back control if needed
            if (m_driverController.GetPOV()==0) {
                elevatorOverrideHeight = std::numeric_limits<double>::max(); //will give control back to driver
                 m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
                    fieldRelative, true
                );
            } 
            else if (m_driverController.GetPOV()==180) {
                elevatorOverrideHeight = kElevatorForceDriveToCoDriverHeight;
            } 
            else {
                coDriverControl();
             }
        }
        else {
        m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
            fieldRelative, true);
        
        }
}
void RobotContainer::coDriverControl() {
    m_drive.Drive(
            //TODO - this whole section needs to be coded
            //ideally the left and right triggers will tractor beam us into actual scoring position AFTER elevator up
            //after elevator is raised up -- do we need safety in place to avoid raising when too close?
            //if no april tag is seen, use joystick to move forward and triggers to strafe?
            -units::meters_per_second_t{(m_coDriverController.GetLeftY())/10},
            -units::meters_per_second_t{(m_coDriverController.GetLeftTriggerAxis())/10 + (m_coDriverController.GetRightTriggerAxis())/10},
            -units::radians_per_second_t{0.0}, //shouldn't be needed - but see if we can allow rotation control?
            fieldRelative, true);
}

photon::PhotonTrackedTarget RobotContainer::hasValidAprilTagTarget() {
    std::vector<photon::PhotonPipelineResult> results = camera.GetAllUnreadResults();
    if (!results.empty()) {
        photon::PhotonPipelineResult result = results.back(); //back gets only the most recent
        if (result.HasTargets()) {
            for (auto target : result.GetTargets()) {
                int targetID = target.GetFiducialId();
                bool found = std::any_of(std::begin(ElevatorConstants::reefTags), std::end(ElevatorConstants::reefTags), [targetID](int x) { return x == targetID; });
                if (found){
                    return target;
                }
            }
        }
    }
    // If no valid target found, return an empty PhotonTrackedTarget
    return photon::PhotonTrackedTarget();
}


bool RobotContainer::isValueInArray(int value, int array[], int size) {
    for (int i = 0; i < size; ++i) {
        if (array[i] == value) {
            return true;
        }
    }
    return false;
}

void RobotContainer::ConfigureButtonBindings() {
    //Zero Heading
    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX).WhileTrue(new frc2::RunCommand([this] {
        m_drive.ZeroHeading();
    }, {&m_drive}));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kBack).WhileTrue(new frc2::RunCommand([this] {
        fieldRelative = false;
    }, {&m_drive}));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kStart).WhileTrue(new frc2::RunCommand([this] {
        fieldRelative = true;
    }, {&m_drive}));

    //Tractor Beam - left - experimental - robot rotates and drives to target automagically - with an offset of 6 inches to the left
    //TODO - This needs to be updated, we need stop centered on the april tag with some distance, then control will be passed to the codriver 
    //raise elevator and move to final scoring position.
    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftBumper).WhileTrue(new frc2::RunCommand([this] { 
        elevatorOverrideHeight = kElevatorForceDriveToCoDriverHeight;
        photon::PhotonTrackedTarget target = hasValidAprilTagTarget();
        if (target.GetFiducialId() > 0) {
                        double targetArea = target.GetArea();
                        frc::SmartDashboard::PutNumber("tagetArea", targetArea);
                        units::meter_t distance = photon::PhotonUtils::CalculateDistanceToTarget(CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH, units::radian_t{target.GetPitch()});
                        m_drive.TractorBeam(distance, true, units::degree_t(target.GetYaw()), targetArea); //true goes to the left
                    }
                    else {
                       DriverControl();
                    }
    }, {&m_drive}));

    //Tractor Beam - right - experimental - robot rotates and drives to target automagically - with an offset of 6 inches to the right
    //TODO - This needs to be updated, we need stop centered on the april tag with some distance, then control will be passed to the codriver 
    //raise elevator and move to final scoring position.
    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightBumper).WhileTrue(new frc2::RunCommand([this] { 
        elevatorOverrideHeight = kElevatorForceDriveToCoDriverHeight;
        photon::PhotonTrackedTarget target = hasValidAprilTagTarget();
        if (target.GetFiducialId() > 0) {
                        double targetArea = target.GetArea();
                        //if the targeting region of target_height is set to top, this should be the height of the top of the target Thus, .17 meters plus 6.5 inches equals 0.3351 meters
                        //if this doesn't work, we can try the center of the target or the bottom of target
                        //Todo: adjust camera height based on actual measurement in constants.h, adjust target height to either top, center, or bottom?
                        units::meter_t distance = photon::PhotonUtils::CalculateDistanceToTarget(CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH, units::radian_t{target.GetPitch()});
                        m_drive.TractorBeam(distance, false, units::degree_t(target.GetYaw()), targetArea); //false goes to the right
                    }
                    else {
                       DriverControl();
                    }
    }, {&m_drive}));

    //Photon Drive - driver still controls driving - but robot rotates front of bot facing toward visible reef april tag
    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB).WhileTrue(new frc2::RunCommand([this] { 
        photon::PhotonTrackedTarget target = hasValidAprilTagTarget();
        if (target.GetFiducialId() > 0) {
                        m_drive.PhotonDrive(
                //driver controls direction of travel, but rotation faces reef april tag
                            -units::meters_per_second_t{frc::ApplyDeadband(m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
                            -units::meters_per_second_t{frc::ApplyDeadband(m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
                            units::degree_t(target.GetYaw()));
                    }
                    else {
                        DriverControl();
                    }
      }, {&m_drive}));


    /*frc2::JoystickButton(&m_coDriverController, frc::XboxController::Button::kX).WhileTrue(new frc2::RunCommand([this] {
        //it's possible that we are too close to the reef to safely raise the elevator
        //if an april tag is present - to avoid damaging the robot we run only if it is safe. 
        photon::PhotonTrackedTarget target = hasValidAprilTagTarget();
        if (target.GetFiducialId() > 0) {
            double targetArea = target.GetArea();
            if (targetArea < ElevatorConstants::kElevatorToCloseToReef){
                m_elevator.SetpointMovement();
            }
        }
        else{
            m_elevator.SetpointMovement();
           // m_pivot.SetpointMovement();
        }
    }, {&m_elevator}));*/

    /*frc2::JoystickButton(&m_coDriverController, frc::XboxController::Button::kA).WhileTrue(new frc2::RunCommand([this] {
         //it's possible that we are too close to the reef to safely lower the elevator
        //if an april tag is present - to avoid damaging the robot we run only if it is safe. 
        photon::PhotonTrackedTarget target = hasValidAprilTagTarget();
        if (target.GetFiducialId() > 0) {
            double targetArea = target.GetArea();
            if (targetArea < ElevatorConstants::kElevatorToCloseToReef){
                m_elevator.SetpointMovement();
            }
        }
        else{
            m_elevator.SetpointMovement();
        }
    }, {&m_elevator}));*/

    //TODO -- implement RunCoralCollector, and ReverseCoralCollector from the coralCollector class

    frc2::JoystickButton(&m_coDriverController, frc::XboxController::Button::kRightBumper).WhileTrue(new frc2::RunCommand([this] {
        if(m_pivot.CurrentPosition() > PivotConstants::kchangespeedpoint){
            m_collector.RunCoralCollectorSlower();
        }
        else {
            m_collector.RunCoralCollector();

        }
        
    }, {&m_collector})).OnFalse(new frc2::InstantCommand([this] {
        m_collector.Stop();
    }, {&m_collector})); //should turn it off when false

    frc2::JoystickButton(&m_coDriverController, frc::XboxController::Button::kLeftBumper).WhileTrue(new frc2::RunCommand([this] {
        if(m_pivot.CurrentPosition() > PivotConstants::kchangespeedpoint){
            m_collector.ReverseCoralCollectorSlower();
        }
        else{
            m_collector.ReverseCoralCollector();
        }
    }, {&m_collector})).OnFalse(new frc2::InstantCommand([this] {
        m_collector.Stop();
    }, {&m_collector})); //should turn it off when false

    frc2::JoystickButton(&m_coDriverController, frc::XboxController::Button::kY).WhileTrue(new frc2::RunCommand([this] {
        m_pivot.RunPivot();
    }, {&m_pivot})).OnFalse(new frc2::InstantCommand([this] {
        m_pivot.Stop();
    }, {&m_pivot}));;  // should turn it off when false

    frc2::JoystickButton(&m_coDriverController, frc::XboxController::Button::kB).WhileTrue(new frc2::RunCommand([this] {
        m_pivot.ReversePivot();
    }, {&m_pivot})).OnFalse(new frc2::InstantCommand([this] {
        m_pivot.Stop();
    }, {&m_pivot})); //should turn it off when false

    



}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    return PathPlannerAuto("From Right Wall").ToPtr();
  /*// Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
        // positive 2nd number = left, negative = right
        // Pass through these two interior waypoints, making an 's' curve path

      {
        frc::Translation2d{1_m, 0_m},
       frc::Translation2d{2_m, 0_m},},
       
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d{2_m, 0_m, 90_deg},
      // Pass the config
      config);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(swerveControllerCommand),
      frc2::InstantCommand([this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false); },{}),
      frc2::RunCommand([this] { 
        m_arm.RunArm(); 
    }, {&m_arm})/
        
  );
*/}
