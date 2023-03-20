// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/WaitCommand.h>

#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>

#include <unordered_map>

#include "ISubsystemAccess.h"
#include "DriveSubsystem.h"

#include "DebugFlag.h"

using namespace frc;
using namespace frc2;
using namespace pathplanner;

class RobotContainer : public ISubsystemAccess
{
public:
  RobotContainer();
  
  CommandPtr GetAutonomousCommand();
  enum EAutoPath
  {
      kAutoPathPlaceAndBalance
    , kAutoPathPlaceAndExitTags1Or8
    , kAutoPathPlaceAndExitTags3Or6
    , kExitTags1Or8
    , kNone
    // Keep the emun in sync with the LUT
  };
  std::vector<std::string> m_pathPlannerLUT
  { 
      "PlaceAndBalance"       // These strings are the names of the PathPlanner .path files
    , "PlaceAndExitTags1Or8" 
    , "PlaceAndExitTags3Or6"
    , "ExitTags1Or8"
    , "None"
  };
  frc::SendableChooser<EAutoPath> m_chooser;
  void SetIsAutoRunning(bool isAutoRunning) { m_isAutoRunning = isAutoRunning; }

  void Periodic();

  // ISubsystemAcces Implementation
  ClawSubsystem&          GetClaw() override { return m_claw; }
  DeploymentSubsystem&    GetDeployment() override { return m_deployment; }
  IDriveSubsystem&        GetDrive() override { return m_drive; }
  IntakeSubsystem&        GetIntake() override { return m_intake; }
  VisionSubsystem&        GetVision() override { return m_vision; }

  wpi::log::DataLog&         GetLogger() override { return DataLogManager::GetLog(); }

private:
  void SetDefaultCommands();
  void ConfigureBindings();
  void ConfigPrimaryButtonBindings();
  void ConfigSecondaryButtonBindings();
// #define USE_PIT_BUTTON_BOX  
#ifdef USE_PIT_BUTTON_BOX  
  void ConfigPitButtonBoxBindings();
#endif

//#define BUTTON_BOX_DEVELOPMENT
#ifdef BUTTON_BOX_DEVELOPMENT
  void ConfigSecondaryButtonBindingsNewWay();
#endif

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
  VisionSubsystem m_vision;

  CommandXboxController m_primaryController{0};
  CommandXboxController m_secondaryController{1};
#ifdef USE_PIT_BUTTON_BOX  
  std::unique_ptr<CommandXboxController> m_pitButtonBox;
#endif
  SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s, -3 / 2_s};
  SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s, -3 / 3_s};
  SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};
  SlewRateLimiter<units::scalar> m_armRotLimiter{3 / 1_s};

  // TODO Make sure field relative starts how the drive team wants
  bool m_fieldRelative = false; //true;
  
  InstantCommand m_toggleFieldRelative{[this] { m_fieldRelative = !m_fieldRelative; }, {}};
  InstantCommand m_toggleSlowSpeed{[this] { GetDrive().ToggleSlowSpeed(); }, {&m_drive}};
  // frc2::InstantCommand m_runCompressor{[this] { m_compressor.EnableDigital(); m_bRunningCompressor = true;}, {} };
#ifdef USE_PIT_BUTTON_BOX  
  InstantCommand m_CfgPitButtonBoxCmd{[this] { ConfigPitButtonBoxBindings(); }, {}};
#endif

#ifdef USE_TEST_BUTTONS
  InstantCommand m_toggleDriveStraight{[this] 
  { 
    m_DriveStraightHook = !m_DriveStraightHook;
    printf("m_DriveStraightHook %s\n", m_DriveStraightHook ? "true" : "false");
  }, {} };
#endif

  InstantCommand m_extendArm{[this] { m_deployment.ExtendArm(); }, {&m_deployment} };
  InstantCommand m_retractArm{[this] { m_deployment.RetractArm(); }, {&m_deployment} };
  InstantCommand m_rotateArm{[this] { m_deployment.RotateArm(SmartDashboard::GetNumber("GotoTicks", 0.0)); }, {&m_deployment} };

  InstantCommand m_wheelsForward{[this] { GetDrive().WheelsForward(); }, {&m_drive} };
  InstantCommand m_wheelsLeft{[this] { GetDrive().WheelsLeft(); }, {&m_drive} };
  InstantCommand m_wheelsBackward{[this] { GetDrive().WheelsBackward(); }, {&m_drive} };
  InstantCommand m_wheelsRight{[this] { GetDrive().WheelsRight(); }, {&m_drive} };

  InstantCommand m_OverrideOn{[this] { GetDrive().SetOverrideXboxInput(true); }, {&m_drive} };
  InstantCommand m_OverrideOff{[this] { GetDrive().SetOverrideXboxInput(false); }, {&m_drive} };

  // std::unordered_map<std::string, std::shared_ptr<frc2::Command>> m_eventMap;
  // SwerveAutoBuilder m_autoBuilder;

  bool m_isAutoRunning = false;
  bool m_DriveStraightHook = false;

  DebugFlag m_dbgFlagDrvrCtrlrPitOverride{"DrvCtrlrPitOvrd", false};
};
