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

#include "IntakeDeploy.h"
#include "IntakeIngest.h"
#include "IntakeRelease.h"
#include "IntakeStop.h"

#include "RotateTurntableCW.h"

#include "PlaceOnFloor.h"
#include "PlaceLow.h"
#include "PlaceHigh.h"

#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/PathPlanner.h>

using namespace pathplanner;


RobotContainer::RobotContainer() 
  : m_drive()
  , m_autoBuilder(
      [this]() { return GetDrive().GetPose(); }, // Function to supply current robot pose
      [this](auto initPose) { GetDrive().ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
      PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      [this](auto speeds) { GetDrive().Drive(speeds.vx, speeds.vy, speeds.omega, true); }, // Output function that accepts field relative ChassisSpeeds
      m_eventMap, // Our event map
      { &m_drive }, // Drive requirements, usually just a single drive subsystem
      true // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  )
{
  SetDefaultCommands();
  ConfigureBindings();

  // This is the pathplanner event map
  m_eventMap.emplace("marker1", std::make_shared<frc2::PrintCommand>("Passed Marker 1"));
  m_eventMap.emplace("intakeDown", std::make_shared<IntakeIngest>(*this));

  SmartDashboard::PutNumber("PitchFactor", m_pitchFactor);
  SmartDashboard::PutNumber("MaxAutoBalanceSpeed", m_maxAutoBalanceSpeed);
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
  std::vector<PathPlannerTrajectory> pathGroup = PathPlanner::loadPathGroup("TestPath1", {PathConstraints(1_mps, 1_mps_sq)});

  return m_autoBuilder.fullAuto(pathGroup);

  // std::vector<Pose2d> waypoints
  // {
  //     Pose2d(3.95_m, 4.17_m, -90_deg)
  //   , Pose2d(3.95_m, 2.17_m, -90_deg)
  //   // , Pose2d(2.95_m, 2.17_m, 180_deg)
  //   // , Pose2d(2.95_m, 4.17_m, 180_deg)
  //   // , Pose2d(3.95_m, 4.17_m, 180_deg)
  // };

  // auto config = TrajectoryConfig(units::velocity::meters_per_second_t{1.0}, units::meters_per_second_squared_t{1.0});
  // config.SetKinematics(m_drive.m_kinematics);
  // Trajectory trajectory = TrajectoryGenerator::GenerateTrajectory(waypoints[0], {}, waypoints[1], config);

  // return GetSwerveCommandPath(trajectory);
}
#endif

void RobotContainer::Periodic() {
  m_drive.Periodic();
  m_vision.Periodic();

  m_pitchFactor = SmartDashboard::GetNumber("PitchFactor", m_pitchFactor);
  m_maxAutoBalanceSpeed = SmartDashboard::GetNumber("MaxAutoBalanceSpeed", m_maxAutoBalanceSpeed);
}

void RobotContainer::SetDefaultCommands()
{
  m_drive.SetDefaultCommand(RunCommand(
    [this] 
    {
      //const double kDeadband = 0.02;
      const double kDeadband = 0.1;
      const auto xInput = ApplyDeadband(m_primaryController.GetLeftY(), kDeadband);
      const auto yInput = ApplyDeadband(m_primaryController.GetLeftX(), kDeadband);
      const auto rotInput = ApplyDeadband(m_primaryController.GetRightX(), kDeadband);      
      const auto rotXInput = ApplyDeadband(m_primaryController.GetRightY(), kDeadband);
      const auto rotYInput = ApplyDeadband(m_primaryController.GetRightX(), kDeadband);

      const auto xSpeed = m_xspeedLimiter.Calculate(xInput) * kMaxSpeed;
      const auto ySpeed = m_yspeedLimiter.Calculate(yInput) * kMaxSpeed;
      const auto rot = m_rotLimiter.Calculate(rotInput) * kMaxAngularSpeed;      
      const double rotX = m_rotLimiter.Calculate(rotXInput);
      const double rotY = m_rotLimiter.Calculate(rotYInput);
      
      if (m_fieldRelative)
      {
        GetDrive().RotationDrive(xSpeed, ySpeed, rotX, rotY, m_fieldRelative);
      }
      else
      {
        GetDrive().Drive(xSpeed, ySpeed, rot, m_fieldRelative);
      }
    },
    {&m_drive}
  ));
}

void RobotContainer::ConfigureBindings()
{
    ConfigPrimaryButtonBindings();
    ConfigSecondaryButtonBindings();
}

void RobotContainer::ConfigPrimaryButtonBindings()
{
    using xbox = XboxController::Button;

    auto& primary = m_primaryController;

    // Primary
    // Keep the bindings in this order
    // A, B, X, Y, Left Bumper, Right Bumper, Back, Start
#ifdef USE_TEST_BUTTONS
    JoystickButton(&primary, xbox::kA).WhileTrue(&m_wheelsBackward);
    JoystickButton(&primary, xbox::kB).WhileTrue(&m_wheelsRight);
    JoystickButton(&primary, xbox::kX).WhileTrue(&m_wheelsLeft);
    JoystickButton(&primary, xbox::kY).WhileTrue(&m_wheelsForward);
    JoystickButton(&primary, xbox::kStart).WhileTrue(&m_OverrideOn);
    JoystickButton(&primary, xbox::kBack).WhileTrue(&m_OverrideOff);
#else
    JoystickButton(&primary, xbox::kA).OnTrue(ClawOpen(*this).ToPtr());
    JoystickButton(&primary, xbox::kB).OnTrue(ClawClose(*this).ToPtr());
    
    JoystickButton(&primary, xbox::kX).OnTrue(&m_resetArmEncoder);
    // JoystickButton(&primary, xbox::kY).WhileTrue();
#endif
    // Triggers field relative driving
    // TODO If we set field relative as default, we also need to swap the 
    //      button bindings here (while button is true (pressed) it should clear field relative (be robo relative))
    JoystickButton(&primary, xbox::kLeftBumper).OnTrue(&m_toggleFieldRelative);
    // JoystickButton(&primary, xbox::kLeftBumper).WhileFalse(&m_clearFieldRelative);

    JoystickButton(&primary, xbox::kRightBumper).OnTrue(&m_toggleSlowSpeed);
}

void RobotContainer::ConfigSecondaryButtonBindings()
{
    using xbox = XboxController::Button;

    auto& secondary = m_secondaryController;

    // Keep the bindings in this order
    // A, B, X, Y, Left Bumper, Right Bumper, Back, Start
    JoystickButton(&secondary, xbox::kA).WhileTrue(IntakeIngest(*this).ToPtr());
    JoystickButton(&secondary, xbox::kB).OnTrue(PlaceLow(*this).ToPtr());
    JoystickButton(&secondary, xbox::kX).OnTrue(PlaceHigh(*this).ToPtr());
    JoystickButton(&secondary, xbox::kY).OnTrue(&m_retrieveGamePiece);

    JoystickButton(&secondary, xbox::kLeftBumper).OnTrue(PlaceOnFloor(*this).ToPtr());
    JoystickButton(&secondary, xbox::kRightBumper).WhileTrue(IntakeRelease(*this).ToPtr());
    
    JoystickButton(&secondary, xbox::kBack).OnTrue(RotateTurntableCW(*this).ToPtr());    
    // JoystickButton(&secondary, xbox::kStart).OnTrue();
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

ConditionalCommand* RobotContainer::GetParkAndBalanceCommand()
{
    return new ConditionalCommand
    (
        RunCommand([this]() { GetDrive().Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, false); }, {&m_drive})    // Cmd if true
      , RunCommand([this]()                                                                            // Cmd if false
        { 
          double driveSpeed = std::clamp(m_pitchFactor * GetDrive().GetPitch(), -m_maxAutoBalanceSpeed, m_maxAutoBalanceSpeed);
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

