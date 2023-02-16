// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>

#include "ISubsystemAccess.h"

using namespace pathplanner;

class RobotContainer : public ISubsystemAccess
{
public:
  RobotContainer();
  
  //frc2::CommandPtr GetAutonomousCommand();
  frc2::Command* GetAutonomousCommand();
  void Periodic();

  // ISubsystemAcces Implementation
  ClawSubsystem&          GetClaw() override { return m_claw; }
  DeploymentSubsystem&    GetDeployment() override { return m_deployment; }
  DriveSubsystem&         GetDrive() override { return m_drive; }
  IntakeSubsystem&        GetIntake() override { return m_intake; }
  TurntableSubsystem&     GetTurntable() override { return m_turntable; }
  VisionSubsystem&        GetVision() override { return m_vision; }

private:
  void SetDefaultCommands();
  void ConfigureBindings();
  frc2::SequentialCommandGroup* GetParkCommand();
  frc2::ConditionalCommand* GetParkAndBalanceCommand();
  frc2::SwerveControllerCommand<4>* GetSwerveCommandPath(frc::Trajectory trajectory); 
  PPSwerveControllerCommand* GetPathPlannerSwervePath(PathPlannerTrajectory trajectory);
  void PrintTrajectory(frc::Trajectory& trajectory);
  frc::Trajectory convertPathToTrajectory(PathPlannerTrajectory ppTrajectory);

 private:
  // The robot's subsystems and commands are defined here...
  ClawSubsystem m_claw;
  DeploymentSubsystem m_deployment;
  DriveSubsystem m_drive;
  IntakeSubsystem m_intake;
  TurntableSubsystem m_turntable;
  VisionSubsystem m_vision;

  frc::XboxController m_primaryController{0};
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  // TODO If we set field relative as default, we also need to swap the 
  //      button bindings here (while button is true (pressed) it should clear field relative (be robo relative))
  //      in ConfigureBindings()
  bool m_fieldRelative = false; //true;
  
  frc2::InstantCommand m_setFieldRelative{[this] { m_fieldRelative = true; }, {}};
  frc2::InstantCommand m_clearFieldRelative{[this] { m_fieldRelative = false; }, {}};

  frc2::InstantCommand m_wheelsForward{[this] { m_drive.WheelsForward(); }, {&m_drive} };
  frc2::InstantCommand m_wheelsLeft{[this] { m_drive.WheelsLeft(); }, {&m_drive} };
  frc2::InstantCommand m_wheelsBackward{[this] { m_drive.WheelsBackward(); }, {&m_drive} };
  frc2::InstantCommand m_wheelsRight{[this] { m_drive.WheelsRight(); }, {&m_drive} };

  frc2::InstantCommand m_OverrideOn{[this] { m_drive.SetOverrideXboxInput(true); }, {&m_drive} };
  frc2::InstantCommand m_OverrideOff{[this] { m_drive.SetOverrideXboxInput(false); }, {&m_drive} };

  frc2::InstantCommand m_resyncAbsRelEnc{[this] { m_drive.ResyncAbsRelEnc(); }, {&m_drive} };

  double m_pitchFactor = 0.033;
  double m_maxAutoBalanceSpeed = 0.5;
};
