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


RobotContainer::RobotContainer() : m_drive()
{
  SetDefaultCommands();
  ConfigureBindings();

  frc::SmartDashboard::PutNumber("PitchFactor", m_pitchFactor);
  frc::SmartDashboard::PutNumber("MaxAutoBalanceSpeed", m_maxAutoBalanceSpeed);
}

#define USE_PATH_PLANNER_SWERVE_CMD
#ifdef USE_PATH_PLANNER_SWERVE_CMD
frc2::Command* RobotContainer::GetAutonomousCommand()
{
  auto pptraj = PathPlanner::loadPath("TestPath1", units::meters_per_second_t{1.0}, units::meters_per_second_squared_t{1.0});
  frc::Trajectory trajectory = convertPathToTrajectory(pptraj);
  PrintTrajectory(trajectory);
  
  return GetSwerveCommandPath(trajectory);
  //return GetPathPlannerSwervePath(trajectory);
}
#else
//frc2::CommandPtr RobotContainer::GetAutonomousCommand()
frc2::Command* RobotContainer::GetAutonomousCommand()
{
  std::vector<frc::Pose2d> waypoints
  {
      frc::Pose2d(3.95_m, 4.17_m, -90_deg)
    , frc::Pose2d(3.95_m, 2.17_m, -90_deg)
    // , frc::Pose2d(2.95_m, 2.17_m, 180_deg)
    // , frc::Pose2d(2.95_m, 4.17_m, 180_deg)
    // , frc::Pose2d(3.95_m, 4.17_m, 180_deg)
  };

  auto config = frc::TrajectoryConfig(units::velocity::meters_per_second_t{1.0}, units::meters_per_second_squared_t{1.0});
  config.SetKinematics(m_drive.m_kinematics);
  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(waypoints[0], {}, waypoints[1], config);

  return GetSwerveCommandPath(trajectory);
}
#endif

void RobotContainer::Periodic() {
  m_drive.Periodic();
  m_vision.Periodic();

  m_pitchFactor = frc::SmartDashboard::GetNumber("PitchFactor", m_pitchFactor);
  m_maxAutoBalanceSpeed = frc::SmartDashboard::GetNumber("MaxAutoBalanceSpeed", m_maxAutoBalanceSpeed);
}

void RobotContainer::SetDefaultCommands()
{
  m_drive.SetDefaultCommand(frc2::RunCommand(
    [this] 
    {
      //const double kDeadband = 0.02;
      const double kDeadband = 0.1;
      const auto xInput = frc::ApplyDeadband(m_primaryController.GetLeftY(), kDeadband);
      const auto yInput = frc::ApplyDeadband(m_primaryController.GetLeftX(), kDeadband);
      const auto rotInput = frc::ApplyDeadband(m_primaryController.GetRightX(), kDeadband);

      const auto xSpeed = m_xspeedLimiter.Calculate(xInput) * kMaxSpeed;
      const auto ySpeed = m_yspeedLimiter.Calculate(yInput) * kMaxSpeed;
      const auto rot = m_rotLimiter.Calculate(rotInput) * kMaxAngularSpeed;
      
      GetDrive().Drive(xSpeed, ySpeed, rot, m_fieldRelative);
    },
    {&m_drive}
  ));
}

void RobotContainer::ConfigureBindings()
{
    using namespace frc;
    using namespace frc2;
    using xbox = frc::XboxController::Button;

    auto& primary = m_primaryController;

    // Primary
    // Keep the bindings in this order
    // A, B, X, Y, Left Bumper, Right Bumper, Back, Start
    JoystickButton(&primary, xbox::kA).WhileTrue(&m_wheelsBackward);
    // JoystickButton(&primary, xbox::kB).WhileTrue(&m_wheelsLeft);
    JoystickButton(&primary, xbox::kX).WhileTrue(&m_wheelsRight);
    JoystickButton(&primary, xbox::kY).WhileTrue(&m_wheelsForward);

    JoystickButton(&primary, xbox::kY).WhileTrue(GetParkAndBalanceCommand());
    JoystickButton(&primary, xbox::kB).OnTrue(GetParkCommand());

    JoystickButton(&primary, xbox::kStart).WhileTrue(&m_OverrideOn);
    JoystickButton(&primary, xbox::kBack).WhileTrue(&m_OverrideOff);

    //JoystickButton(&primary, xbox::kRightBumper).WhileTrue(&m_resyncAbsRelEnc);
    // Triggers field relative driving
    // TODO If we set field relative as default, we also need to swap the 
    //      button bindings here (while button is true (pressed) it should clear field relative (be robo relative))
    JoystickButton(&primary, xbox::kLeftBumper).WhileTrue(&m_setFieldRelative);
    JoystickButton(&primary, xbox::kLeftBumper).WhileFalse(&m_clearFieldRelative);

}

frc2::SequentialCommandGroup* RobotContainer::GetParkCommand()
{
    return new frc2::SequentialCommandGroup
    (
        frc2::ParallelDeadlineGroup
        (
              frc2::WaitUntilCommand([this]() { return GetDrive().GetPitch() < -7.0; })
            , frc2::RunCommand([this]() { GetDrive().Drive(-1.00_mps, 0.0_mps, 0.0_rad_per_s, false); }, {&m_drive})
        )
        , frc2::ParallelDeadlineGroup
        (
            frc2::WaitCommand(1.600_s)
          , frc2::RunCommand([this]() { GetDrive().Drive(-1.00_mps, 0.0_mps, 0.0_rad_per_s, false); }, {&m_drive})
        )
        , frc2::InstantCommand([this]() { GetDrive().Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, false); }, {&m_drive})
    );
}

frc2::ConditionalCommand* RobotContainer::GetParkAndBalanceCommand()
{
    return new frc2::ConditionalCommand
    (
        frc2::RunCommand([this]() { GetDrive().Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, false); }, {&m_drive})    // Cmd if true
      , frc2::RunCommand([this]()                                                                            // Cmd if false
        { 
          double driveSpeed = std::clamp(m_pitchFactor * GetDrive().GetPitch(), -m_maxAutoBalanceSpeed, m_maxAutoBalanceSpeed);
          GetDrive().Drive(units::velocity::meters_per_second_t(driveSpeed), 0.0_mps, 0.0_rad_per_s, false); 
        }
        , {&m_drive})
      , [this]() { return GetDrive().GetPitch() > -1.0 && GetDrive().GetPitch() < 1.0; }    // Condition
    );
}

const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints{kMaxAngularSpeed, kMaxAngularAcceleration};


frc2::SwerveControllerCommand<4>* RobotContainer::GetSwerveCommandPath(frc::Trajectory trajectory)
{
  //PrintTrajectory(trajectory);

  frc::ProfiledPIDController<units::radians> thetaController{0.01, 0.0, 0.0, kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t(-std::numbers::pi), units::radian_t(std::numbers::pi));

  frc2::SwerveControllerCommand<4>* swerveControllerCommand = new frc2::SwerveControllerCommand<4>(
      trajectory,                                                             // frc::Trajectory
      [this]() { return GetDrive().GetPose(); },                                 // std::function<frc::Pose2d()>
      m_drive.m_kinematics,                                               // frc::SwerveDriveKinematics<NumModules>
      frc2::PIDController(1.0, 0, 0.0),                // frc2::PIDController
      frc2::PIDController(1.0, 0, 0.0),                // frc2::PIDController
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
      frc2::PIDController(1.0, 0.0, 0.0),                                       // frc2::PIDController
      frc2::PIDController(1.0, 0.0, 0.0),                                       // frc2::PIDController
      frc2::PIDController(1.0, 0.0, 0.0),                                       // frc2::PIDController
      [this](auto moduleStates) { GetDrive().SetModuleStates(moduleStates); },   // std::function< void(std::array<frc::SwerveModuleState, NumModules>)>
      {&m_drive}                                                              // std::initializer_list<Subsystem*> requirements
			//bool useAllianceColor = false);
    );

    m_drive.SetHeading(trajectory.getInitialHolonomicPose().Rotation().Degrees());
    m_drive.ResetOdometry(trajectory.getInitialHolonomicPose());

    return ppSwerveControllerCommand;
}

void RobotContainer::PrintTrajectory(frc::Trajectory& trajectory)
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

frc::Trajectory RobotContainer::convertPathToTrajectory(PathPlannerTrajectory ppTrajectory)
{
    std::vector<frc::Trajectory::State> states;
    for (double time = 0.0; time < ppTrajectory.getTotalTime().to<double>(); time += 0.02)
    {
        PathPlannerTrajectory::PathPlannerState state = ppTrajectory.sample(time * 1_s);
        states.push_back({
            time * 1_s,
            state.velocity,
            state.acceleration, 
            frc::Pose2d(
                state.pose.X(),
                state.pose.Y(),
                state.holonomicRotation
            ),
            units::curvature_t(0)
        });
    }
    return frc::Trajectory(states);
}

