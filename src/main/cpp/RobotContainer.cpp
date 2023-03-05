// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/button/JoystickButton.h>

#include "Balance.h"

#include "IntakeDeploy.h"
#include "IntakeIngest.h"
#include "IntakeRelease.h"
#include "IntakeStop.h"

#include "RotateTurntableCW.h"
#include "RotateTurntableCCW.h"

#include "RetrieveGamePiece.h"

#include "PlaceOnFloor.h"
#include "PlaceLow.h"
#include "PlaceHigh.h"
#include "PlaceHighCube.h"

#include "ReleaseCone.h"

#include "ExtendArm.h"
#include "RetractArm.h"

#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/PathPoint.h>
#include <pathplanner/lib/PathPlanner.h>

using namespace pathplanner;

RobotContainer::RobotContainer() 
  : m_drive()
{
  SetDefaultCommands();
  ConfigureBindings();

  SmartDashboard::PutNumber("MaxAutoBalanceSpeed", 0.9);
  frc::SmartDashboard::PutNumber("Balance Tolerance", 7.0);
  frc::SmartDashboard::PutNumber("BalanceEndTime", 1.0);
}

//#define USE_PATH_PLANNER_SWERVE_CMD
#ifdef USE_PATH_PLANNER_SWERVE_CMD
Command* RobotContainer::GetAutonomousCommand()
{
  auto pptraj = PathPlanner::loadPath("TestPath1", units::meters_per_second_t{1.0}, units::meters_per_second_squared_t{1.0});
  Trajectory trajectory = convertPathToTrajectory(pptraj);
  PrintTrajectory(trajectory);
  
  return GetSwerveCommandPath(trajectory);
  //return GetPathPlannerSwervePath(trajectory);
}
#else
CommandPtr RobotContainer::GetAutonomousCommand()
{
  // TODO Add dashboard selector for auto path
  std::vector<PathPlannerTrajectory> pathGroup = PathPlanner::loadPathGroup("PlaceAndBalance", {PathConstraints(2_mps, 2_mps_sq)});

  static std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;
  eventMap.emplace("Balance", std::make_shared<Balance>(m_drive, *this));
  eventMap.emplace("TravelPosition", std::make_shared<TravelPosition>(*this));
  eventMap.emplace("PlaceHigh", std::make_shared<PlaceHigh>(*this));
  eventMap.emplace("ClawOpen", std::make_shared<ClawOpen>(*this));
  eventMap.emplace("ClawClose", std::make_shared<ClawClose>(*this));
  eventMap.emplace("IntakeDeploy", std::make_shared<IntakeDeploy>(*this));
  eventMap.emplace("ReleaseCone", std::make_shared<ReleaseCone>(*this));
  eventMap.emplace("RetrievePosition", std::make_shared<RetrievePosition>(*this));
  eventMap.emplace("ExtendArm", std::make_shared<ExtendArm>(*this));
  eventMap.emplace("WaitLong", std::make_shared<WaitCommand>(1.2_s));
  eventMap.emplace("WaitShort", std::make_shared<WaitCommand>(0.5_s));
  eventMap.emplace("ClearancePosition", std::make_shared<ClearancePosition>(*this));

  // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this could be in RobotContainer along with your subsystems
  static SwerveAutoBuilder autoBuilder(
      [this]() { return GetDrive().GetPose(); }, // Function to supply current robot pose
      [this](auto initPose) { GetDrive().ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
      PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      [this](ChassisSpeeds speeds) { GetDrive().Drive(speeds.vx, speeds.vy, speeds.omega, true); }, // Output function that accepts field relative ChassisSpeeds
      eventMap, // Our event map
      { &m_drive }, // Drive requirements, usually just a single drive subsystem
      true // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  );

  return autoBuilder.fullAuto(pathGroup);
}
#endif

void RobotContainer::Periodic() {
  m_drive.Periodic();
  m_vision.Periodic();

  // m_pitchFactor = SmartDashboard::GetNumber("PitchFactor", m_pitchFactor);
  // m_maxAutoBalanceSpeed = SmartDashboard::GetNumber("MaxAutoBalanceSpeed", m_maxAutoBalanceSpeed);
}

void RobotContainer::SetDefaultCommands()
{
  m_drive.SetDefaultCommand(RunCommand
  (
    [this] 
    {
      // Don't send any input if autonomous is running
      if (m_isAutoRunning == false)
      {
        // const double kDeadband = 0.02;
        const double kDeadband = 0.1;
        const auto xInput = ApplyDeadband(m_primaryController.GetLeftY(), kDeadband);
        const auto yInput = ApplyDeadband(m_primaryController.GetLeftX(), kDeadband);
        const auto rotInput = ApplyDeadband(m_primaryController.GetRightX(), kDeadband);      
        const auto rotXInput = ApplyDeadband(m_primaryController.GetRightY(), kDeadband);
        const auto rotYInput = ApplyDeadband(m_primaryController.GetRightX(), kDeadband);

        const auto xSpeed = m_xspeedLimiter.Calculate(xInput) * kMaxSpeed;
        auto ySpeed = m_yspeedLimiter.Calculate(yInput) * kMaxSpeed;
        auto rot = m_rotLimiter.Calculate(rotInput) * kMaxAngularSpeed;      
        const double rotX = m_rotLimiter.Calculate(rotXInput);
        const double rotY = m_rotLimiter.Calculate(rotYInput);

        if (m_DriveStraightHook)
        {
          ySpeed = 0.0_mps;
          rot = 0.0_rad_per_s;
        }

        if (m_fieldRelative)
        {
          GetDrive().RotationDrive(xSpeed, ySpeed, rotX, rotY, m_fieldRelative);
        }
        else
        {
          GetDrive().Drive(xSpeed, ySpeed, rot, m_fieldRelative);
        }
      }
    },
    {&m_drive}
  ));
}

void RobotContainer::ConfigureBindings()
{
  ConfigPrimaryButtonBindings();
  ConfigSecondaryButtonBindings();
  // ConfigSecondaryButtonBindingsNewWay();
}

void RobotContainer::ConfigPrimaryButtonBindings()
{
  auto& primary = m_primaryController;

  // Primary
  // Keep the bindings in this order
  // A, B, X, Y, Left Bumper, Right Bumper, Back, Start
#ifdef USE_TEST_BUTTONS
  primary.A().WhileTrue(&m_wheelsBackward);
  primary.B().WhileTrue(&m_wheelsRight);
  primary.X().WhileTrue(&m_wheelsLeft);
  primary.Y().WhileTrue(&m_wheelsForward);
  primary.Y().OnTrue(&m_toggleDriveStraight);
  primary.Start().WhileTrue(&m_OverrideOn);
  primary.Back().WhileTrue(&m_OverrideOff);
#else
  primary.A().OnTrue(ClawOpen(*this).ToPtr());
  primary.B().OnTrue(ClawClose(*this).ToPtr());
  primary.X().OnTrue(RetrieveGamePiece(*this).ToPtr());
  primary.Y().OnTrue(ReleaseCone(*this).ToPtr());
#endif
  primary.LeftBumper().OnTrue(&m_toggleFieldRelative);
  primary.RightBumper().OnTrue(&m_toggleSlowSpeed);
}

void RobotContainer::ConfigSecondaryButtonBindings()
{
  auto& secondary = m_secondaryController;

  // Keep the bindings in this order
  // A, B, X, Y, Left Bumper, Right Bumper, Back, Start
  secondary.A().OnTrue(PlaceOnFloor(*this).ToPtr());                          
  secondary.B().OnTrue(PlaceLow(*this).ToPtr());                                 
  secondary.X().OnTrue(TravelPosition(*this).ToPtr());                                
  secondary.Y().OnTrue(PlaceHigh(*this).ToPtr());                                    

  secondary.LeftBumper().WhileTrue(IntakeIngest(*this).ToPtr());
  secondary.RightBumper().WhileTrue(IntakeRelease(*this).ToPtr());
  secondary.Start().WhileTrue(RotateTurntableCCW(*this).ToPtr());
  secondary.Back().WhileTrue(RotateTurntableCW(*this).ToPtr());
  
  // secondary.LeftStick().OnTrue();                            
  // secondary.RightStick().OnTrue();                           
  // secondary.LeftTrigger().WhileTrue();
  // secondary.RightTrigger().WhileTrue();

  auto loop = CommandScheduler::GetInstance().GetDefaultButtonLoop();
  secondary.POVUp(loop).Rising().IfHigh([this] { PlaceHighCube(*this).Schedule(); });
}

void RobotContainer::ConfigSecondaryButtonBindingsNewWay()
{
  auto& secondary = m_secondaryController;
  // Raspberry PI Pico with gp2040 firmware Button Box
  //
  // Row	Black			    Blue			    Green				      Yellow				      Red
  // 1	  Back			    Start			    Left Stick Button	Right Stick Button	Left Bumper
  // 2	  Right Trigger	Left Trigger	X					        Y					          Right Bumper
  // 3	  B				      A				      POV Left			    POV Right			      POV Up
  secondary.A().WhileTrue(IntakeIngest(*this).ToPtr());                          // Blue   row 3
  secondary.A().OnFalse(IntakeStop(*this).ToPtr());                              // Blue   row 3
  secondary.B().OnTrue(PlaceLow(*this).ToPtr());                                 // Black  row 3
  secondary.X().OnTrue(PlaceHigh(*this).ToPtr());                                // Green  row 2
  secondary.Y().OnTrue(RetrieveGamePiece(*this).ToPtr());                        // Yellow row 2

  secondary.LeftBumper().OnTrue(PlaceOnFloor(*this).ToPtr());                    // Red    row 1
  secondary.RightBumper().WhileTrue(IntakeRelease(*this).ToPtr());               // Red    row 2
  secondary.Start().WhileTrue(&m_rotateArm);                                     // Blue   row 1
  secondary.Back().WhileTrue(RotateTurntableCW(*this).ToPtr());                     // Black  row 1

  secondary.LeftStick().OnTrue(&m_extendArm);                            // Green  row 1
  secondary.RightStick().OnTrue(&m_retractArm);                           // Yellow row 1
  secondary.LeftTrigger().WhileTrue(TravelPosition(*this).ToPtr());       // Blue   row 2
  secondary.RightTrigger().WhileTrue(&m_toggleClaw);                      // Black  row 2

  auto loop = CommandScheduler::GetInstance().GetDefaultButtonLoop();
  secondary.POVLeft(loop).Rising().IfHigh([this] { m_deployment.ExtendBackPlate(); });  // Green  row 3
  secondary.POVRight(loop).Rising().IfHigh([this] { m_deployment.RetractBackPlate(); });// Yellow row 3
  //secondary.POVUp(loop).Rising().IfHigh([this] { PlaceHighCube(*this).Schedule(); });      // Red    row 3
  //secondary.POVUp(loop).Rising().IfHigh(RotateTurntableCW(*this).ToPtr()});      // Red    row 3
}

SequentialCommandGroup* RobotContainer::GetParkCommand()
{
  return new SequentialCommandGroup
  (
      ParallelDeadlineGroup
      (
            WaitUntilCommand([this]() { return GetDrive().GetPitch() < -7.0; })
          , RunCommand([this]() { GetDrive().Drive(-1.00_mps, 0.0_mps, 0.0_rad_per_s, false); }, {&m_drive})
      )
      , ParallelDeadlineGroup
      (
          WaitCommand(1.600_s)
        , RunCommand([this]() { GetDrive().Drive(-1.00_mps, 0.0_mps, 0.0_rad_per_s, false); }, {&m_drive})
      )
      , InstantCommand([this]() { GetDrive().Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, false); }, {&m_drive})
  );
}

ConditionalCommand* RobotContainer::GetParkAndBalanceCommand2()
{
  return new ConditionalCommand
  (
      RunCommand([this]() { GetDrive().Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, false); }, {&m_drive})    // Cmd if true
    , RunCommand([this]()                                                                            // Cmd if false
      { 
        //double driveSpeed = std::clamp(m_pitchFactor * GetDrive().GetPitch(), -m_maxAutoBalanceSpeed, m_maxAutoBalanceSpeed);
        double driveSpeed = 1.0;//std::clamp(0.1 * GetDrive().GetPitch(), -m_maxAutoBalanceSpeed, m_maxAutoBalanceSpeed);
        GetDrive().Drive(units::velocity::meters_per_second_t(driveSpeed), 0.0_mps, 0.0_rad_per_s, false); 
      }
      , {&m_drive})
    , [this]() { return GetDrive().GetPitch() > -1.0 && GetDrive().GetPitch() < 1.0; }    // Condition
  );
}

const TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints{kMaxAngularSpeed, kMaxAngularAcceleration};


SwerveControllerCommand<4>* RobotContainer::GetSwerveCommandPath(Trajectory trajectory)
{
  //PrintTrajectory(trajectory);

  ProfiledPIDController<units::radians> thetaController{0.01, 0.0, 0.0, kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t(-std::numbers::pi), units::radian_t(std::numbers::pi));

  SwerveControllerCommand<4>* swerveControllerCommand = new SwerveControllerCommand<4>(
      trajectory,                                                             // frc::Trajectory
      [this]() { return GetDrive().GetPose(); },                                 // std::function<frc::Pose2d()>
      m_drive.m_kinematics,                                               // frc::SwerveDriveKinematics<NumModules>
      PIDController(1.0, 0, 0.0),                // frc2::PIDController
      PIDController(1.0, 0, 0.0),                // frc2::PIDController
      thetaController,                                                        // frc::ProfiledPIDController<units::radians>
      [this](auto moduleStates) { GetDrive().SetModuleStates(moduleStates); },   // std::function< void(std::array<frc::SwerveModuleState, NumModules>)>
      {&m_drive}                                                              // std::initializer_list<Subsystem*> requirements
  );

  m_drive.SetHeading(trajectory.InitialPose().Rotation().Degrees());
  m_drive.ResetOdometry(trajectory.InitialPose());

  return swerveControllerCommand;
}

pathplanner::PPSwerveControllerCommand* RobotContainer::GetPathPlannerSwervePath(PathPlannerTrajectory trajectory)
{
  PPSwerveControllerCommand* ppSwerveControllerCommand = new PPSwerveControllerCommand(
      trajectory,                                                             // frc::Trajectory
      [this]() { return GetDrive().GetPose(); },                                 // std::function<frc::Pose2d()>
      m_drive.m_kinematics,                                                   // frc::SwerveDriveKinematics<NumModules>
      PIDController(1.0, 0.0, 0.0),                                       // frc2::PIDController
      PIDController(1.0, 0.0, 0.0),                                       // frc2::PIDController
      PIDController(1.0, 0.0, 0.0),                                       // frc2::PIDController
      [this](auto moduleStates) { GetDrive().SetModuleStates(moduleStates); },   // std::function< void(std::array<frc::SwerveModuleState, NumModules>)>
      {&m_drive}                                                              // std::initializer_list<Subsystem*> requirements
			//bool useAllianceColor = false);
  );

  m_drive.SetHeading(trajectory.getInitialHolonomicPose().Rotation().Degrees());
  m_drive.ResetOdometry(trajectory.getInitialHolonomicPose());

  return ppSwerveControllerCommand;
}



void RobotContainer::PrintTrajectory(Trajectory& trajectory)
{
  printf("Time,X,Y,HoloRot\n");
  for (auto &state:trajectory.States())
  {
      double time = state.t.to<double>();
      double x = state.pose.X().to<double>();
      double y = state.pose.Y().to<double>();
      double holoRot = state.pose.Rotation().Degrees().to<double>();
      printf("%.3f,%.3f,%.3f,%.3f\n", time, x, y, holoRot);
  }
}

Trajectory RobotContainer::convertPathToTrajectory(PathPlannerTrajectory ppTrajectory)
{
  std::vector<Trajectory::State> states;
  for (double time = 0.0; time < ppTrajectory.getTotalTime().to<double>(); time += 0.02)
  {
      PathPlannerTrajectory::PathPlannerState state = ppTrajectory.sample(time * 1_s);
      states.push_back({
          time * 1_s,
          state.velocity,
          state.acceleration, 
          Pose2d(
              state.pose.X(),
              state.pose.Y(),
              state.holonomicRotation
          ),
          units::curvature_t(0)
      });
  }
  return Trajectory(states);
}