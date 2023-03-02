// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

//#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>
#include <unordered_map>

#include "ISubsystemAccess.h"
#include "DriveSubsystem.h"

#include "ClawOpen.h"
#include "ClawClose.h"
#include "IntakeDeploy.h"
#include "RetrievePosition.h"
#include "TravelPosition.h"

using namespace frc;
using namespace frc2;
using namespace pathplanner;

class RobotContainer : public ISubsystemAccess
{
public:
  RobotContainer();
  
  CommandPtr GetAutonomousCommand();
  void Periodic();
  void SetIsAutoRunning(bool isAutoRunning) { m_isAutoRunning = isAutoRunning; }

  // ISubsystemAcces Implementation
  ClawSubsystem&          GetClaw() override { return m_claw; }
  DeploymentSubsystem&    GetDeployment() override { return m_deployment; }
  IDriveSubsystem&        GetDrive() override { return m_drive; }
  IntakeSubsystem&        GetIntake() override { return m_intake; }
  TurntableSubsystem&     GetTurntable() override { return m_turntable; }
  VisionSubsystem&        GetVision() override { return m_vision; }

  wpi::log::DataLog&         GetLogger() override { return DataLogManager::GetLog(); }

private:
  void SetDefaultCommands();
  void ConfigureBindings();
  void ConfigPrimaryButtonBindings();
  void ConfigSecondaryButtonBindings();
  void ConfigSecondaryButtonBindingsNewWay();
  SequentialCommandGroup* GetParkCommand();
  std::shared_ptr<ConditionalCommand> GetParkAndBalanceCommand();
  ConditionalCommand* GetParkAndBalanceCommand2();
  SwerveControllerCommand<4>* GetSwerveCommandPath(Trajectory trajectory); 
  PPSwerveControllerCommand* GetPathPlannerSwervePath(PathPlannerTrajectory trajectory);
  void PrintTrajectory(Trajectory& trajectory);
  Trajectory convertPathToTrajectory(PathPlannerTrajectory ppTrajectory);

 private:
  // The robot's subsystems and commands are defined here...
  ClawSubsystem m_claw;
  DeploymentSubsystem m_deployment;
  DriveSubsystem m_drive;
  IntakeSubsystem m_intake;
  TurntableSubsystem m_turntable;
  VisionSubsystem m_vision;

  CommandXboxController m_primaryController{0};
  CommandXboxController m_secondaryController{1};
  SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  // TODO If we set field relative as default, we also need to swap the 
  //      button bindings here (while button is true (pressed) it should clear field relative (be robo relative))
  //      in ConfigureBindings()
  bool m_fieldRelative = false; //true;
  
  InstantCommand m_toggleFieldRelative{[this] { m_fieldRelative = !m_fieldRelative; }, {}};
  InstantCommand m_toggleSlowSpeed{[this] { GetDrive().ToggleSlowSpeed(); }, {&m_drive}};
  // frc2::InstantCommand m_runCompressor{[this] { m_compressor.EnableDigital(); m_bRunningCompressor = true;}, {} };
  SequentialCommandGroup m_retrieveGamePiece{ IntakeDeploy(*this), ClawOpen(*this), RetrievePosition(*this), ClawClose(*this), TravelPosition(*this) };

  InstantCommand m_extendArm{[this] { m_deployment.ExtendArm(); }, {&m_deployment} };
  InstantCommand m_retractArm{[this] { m_deployment.RetractArm(); }, {&m_deployment} };
  InstantCommand m_rotateArm{[this] { m_deployment.RotateArmToAngle(degree_t(SmartDashboard::GetNumber("GotoAngle", 0.0))); }, {&m_deployment} };

  InstantCommand m_wheelsForward{[this] { GetDrive().WheelsForward(); }, {&m_drive} };
  InstantCommand m_wheelsLeft{[this] { GetDrive().WheelsLeft(); }, {&m_drive} };
  InstantCommand m_wheelsBackward{[this] { GetDrive().WheelsBackward(); }, {&m_drive} };
  InstantCommand m_wheelsRight{[this] { GetDrive().WheelsRight(); }, {&m_drive} };

  InstantCommand m_OverrideOn{[this] { GetDrive().SetOverrideXboxInput(true); }, {&m_drive} };
  InstantCommand m_OverrideOff{[this] { GetDrive().SetOverrideXboxInput(false); }, {&m_drive} };
  InstantCommand m_resetArmEncoder{[this] { m_deployment.ResetEncoder(); }, {}};

  // std::unordered_map<std::string, std::shared_ptr<frc2::Command>> m_eventMap;
  // SwerveAutoBuilder m_autoBuilder;

  bool m_isAutoRunning = false;
};
