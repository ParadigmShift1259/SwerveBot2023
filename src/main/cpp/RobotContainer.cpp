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
#include "ClawRelease.h"
#include "ClawIngest.h"
#include "ClawIngestCube.h"
#include "ClawStop.h"
#include "IntakeDeploy.h"

#include "IntakeDeploy.h"
#include "IntakeRelease.h"
#include "IntakeStop.h"

#include "PlaceOnFloor.h"
#include "PlaceLow.h"
#include "PlaceHigh.h"
#include "PickUp.h"

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

  m_chooser.SetDefaultOption("Place and Balance Tag 1 or 8", EAutoPath::kAutoPathPlaceAndBalanceTags1Or8);
  m_chooser.AddOption("Place and Balance Tag 3 or 6", EAutoPath::kAutoPathPlaceAndBalanceTags3Or6);
  m_chooser.AddOption("Place and Balance", EAutoPath::kAutoPathPlaceAndBalance);
  m_chooser.AddOption("Place and Exit Tag 1 or 8", EAutoPath::kAutoPathPlaceAndExitTags1Or8);
  m_chooser.AddOption("Place and Exit Tag 3 or 6", EAutoPath::kAutoPathPlaceAndExitTags3Or6);
  m_chooser.AddOption("Exit Tag 1 or 8", EAutoPath::kAutoPathExitTags1Or8);
  m_chooser.AddOption("Exit Tag 3 or 6", EAutoPath::kAutoPathExitTags3Or6);
  m_chooser.AddOption("Balance Only Inside", EAutoPath::kAutoPathBalanceOnlyInside);
  m_chooser.AddOption("Balance Only Tag 1 or 8", EAutoPath::kAutoPathBalanceOnlyTags1Or8);
  m_chooser.AddOption("Balance Only Tag 3 or 6", EAutoPath::kAutoPathBalanceOnlyTags3Or6);
  m_chooser.AddOption("Place Only", EAutoPath::kAutoPathPlaceOnly);
  m_chooser.AddOption("None", EAutoPath::kNone);
  frc::SmartDashboard::PutData("Auto Path", &m_chooser);

  SmartDashboard::PutNumber("MaxAutoBalanceSpeed", 0.9);
  frc::SmartDashboard::PutNumber("Balance Tolerance", 7.0);
  frc::SmartDashboard::PutNumber("BalanceEndTime", 1.0);
  // frc::SmartDashboard::PutNumber("Retrieve Angle", 6.0);
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
  auto autoPath = m_chooser.GetSelected();
  auto pathFile = m_pathPlannerLUT[autoPath];
  std::vector<PathPlannerTrajectory> pathGroup = PathPlanner::loadPathGroup(pathFile, {PathConstraints(2_mps, 2_mps_sq)});
//  std::vector<PathPlannerTrajectory> pathGroup = PathPlanner::loadPathGroup("ExitTags1Or8", {PathConstraints(2_mps, 2_mps_sq)});

  static std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;
  eventMap.emplace("Balance", std::make_shared<Balance>(m_drive, *this));
  eventMap.emplace("TravelPosition", std::make_shared<TravelPosition>(*this));
//  eventMap.emplace("TravelWithIntakeIngest", m_travelWithIntakeIngest);
//  eventMap.emplace("TravelWithIntakeIngest", std::make_shared<ParallelCommandGroup>(TravelPosition(*this), IntakeIngest(*this)));
  eventMap.emplace("PlaceHigh", std::make_shared<PlaceHigh>(*this));
  eventMap.emplace("ClawRelease", std::make_shared<ClawRelease>(*this));
  eventMap.emplace("ClawIngest", std::make_shared<ClawIngest>(*this));
  eventMap.emplace("IntakeIngest", std::make_shared<IntakeIngest>(*this));
  eventMap.emplace("IntakeStop", std::make_shared<IntakeStop>(*this));
  eventMap.emplace("ReleaseCone", std::make_shared<ReleaseCone>(*this));
  eventMap.emplace("ExtendArm", std::make_shared<ExtendArm>(*this));
  eventMap.emplace("WaitLong", std::make_shared<WaitCommand>(1.2_s));
  eventMap.emplace("WaitShort", std::make_shared<WaitCommand>(0.5_s));

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

void RobotContainer::Periodic()
{
  m_drive.Periodic();
  m_vision.Periodic();

  // m_pitchFactor = SmartDashboard::GetNumber("PitchFactor", m_pitchFactor);
  // m_maxAutoBalanceSpeed = SmartDashboard::GetNumber("MaxAutoBalanceSpeed", m_maxAutoBalanceSpeed);
  SmartDashboard::PutBoolean("FieldRelative", m_fieldRelative);
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
        const double kDeadband = 0.1;
        const auto xInput = ApplyDeadband(m_primaryController.GetLeftY(), kDeadband);
        const auto yInput = ApplyDeadband(m_primaryController.GetLeftX(), kDeadband);
        const auto rotInput = ApplyDeadband(m_primaryController.GetRightX(), kDeadband);      
        const auto rotXInput = ApplyDeadband(m_primaryController.GetRightY(), kDeadband);
        const auto rotYInput = ApplyDeadband(m_primaryController.GetRightX(), kDeadband);

        const auto xSpeed = m_xspeedLimiter.Calculate(xInput) * m_drive.m_currentMaxSpeed; //kMaxSpeed;
        auto ySpeed = m_yspeedLimiter.Calculate(yInput) * m_drive.m_currentMaxSpeed; //kMaxSpeed;
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

  m_deployment.SetDefaultCommand(RunCommand
  (
    [this] 
    {
      // Don't send any input if autonomous is running
      if (m_isAutoRunning == false)
      {
        const double kDeadband = 0.25;
        const auto input = ApplyDeadband(m_secondaryController.GetLeftY(), kDeadband);
        if (fabs(input) > kDeadband)
        {
          m_deployment.RotateArmRelative(m_armRotLimiter.Calculate(input));
        }
      }
    },
    {&m_deployment}
  ));
}

void RobotContainer::ConfigureBindings()
{
  ConfigPrimaryButtonBindings();

#ifdef BUTTON_BOX_DEVELOPMENT
  ConfigSecondaryButtonBindingsNewWay();
#else
  ConfigSecondaryButtonBindings();
#endif
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
  auto loop = CommandScheduler::GetInstance().GetDefaultButtonLoop();
  primary.POVLeft(loop).Rising().IfHigh([this] { m_deployment.ExtendArm(); });  // Green  row 3
  primary.POVRight(loop).Rising().IfHigh([this] { m_deployment.RetractArm(); });// Yellow row 3
#else
  primary.A().OnTrue(ClawRelease(*this).ToPtr());
  primary.A().OnFalse(ClawStop(*this).ToPtr());
  // primary.B().OnTrue(ClawIngest(*this).ToPtr()); Doesn't work because photoeye doesn't work
  primary.B().WhileTrue(ClawIngest(*this).ToPtr());
  primary.X().OnTrue(ClawStop(*this).ToPtr());
  primary.Y().WhileTrue(IntakeRelease(*this).ToPtr());
#endif
  primary.LeftBumper().OnTrue(&m_toggleFieldRelative);
  primary.RightBumper().OnTrue(&m_toggleSlowSpeed);

  primary.LeftTrigger().OnTrue(&m_cancelAll);

//  if (m_dbgFlagDrvrCtrlrPitOverride)
  {
    //primary.Start().OnTrue(&m_rotateArm);
#ifdef USE_PIT_BUTTON_BOX  
    // Initialize button box bindingd
    primary.Back().OnTrue(&m_CfgPitButtonBoxCmd); // Calls ConfigPitButtonBoxBindings()
#endif
  }

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

  secondary.LeftBumper().WhileTrue(ClawIngestCube(*this).ToPtr());
  secondary.RightBumper().OnTrue(PickUp(*this).ToPtr());
  secondary.Start().OnTrue(IntakeIngest(*this).ToPtr());                                       
  secondary.Back().OnTrue(IntakeStop(*this).ToPtr()); 

  // secondary.LeftStick().OnTrue();                            
  // secondary.RightStick().OnTrue();                           
  // secondary.LeftTrigger().OnTrue(ClawIngest(*this).ToPtr()); Doesn't work because photoeye doesn't work
  secondary.LeftTrigger().WhileTrue(ClawIngest(*this).ToPtr());
  // secondary.RightTrigger().OnTrue();

  auto loop = CommandScheduler::GetInstance().GetDefaultButtonLoop();
  secondary.POVRight(loop).Rising().IfHigh([this] { m_deployment.ExtendArm(); });
  secondary.POVLeft(loop).Rising().IfHigh([this] { m_deployment.RetractArm(); });
}

#ifdef USE_PIT_BUTTON_BOX  
void RobotContainer::ConfigPitButtonBoxBindings()
{
  printf("ConfigPitButtonBoxBindings called\n");
  if (!m_pitButtonBox)
  {
    printf("Creating m_pitButtonBox\n");
    m_pitButtonBox = std::make_unique<CommandXboxController>(3);

    if (m_pitButtonBox)
    {
      printf("Configuring m_pitButtonBox bindings\n");
      // Raspberry PI Pico with gp2040 firmware Button Box
      //
      // Row	Black			    Blue			    Green				      Yellow				      Red
      // 1	  Back			    Start			    Left Stick Button	Right Stick Button	Left Bumper
      // 2	  Right Trigger	Left Trigger	X					        Y					          Right Bumper
      // 3	  B				      A				      POV Left			    POV Right			      POV Up
      m_pitButtonBox->A().WhileTrue(IntakeIngest(*this).ToPtr());                          // Blue   row 3
      printf("Configured m_pitButtonBox A button binding\n");
      m_pitButtonBox->A().OnFalse(IntakeStop(*this).ToPtr());                              // Blue   row 3
      m_pitButtonBox->B().OnTrue(PlaceLow(*this).ToPtr());                                 // Black  row 3
      m_pitButtonBox->X().OnTrue(PlaceHigh(*this).ToPtr());                                // Green  row 2
      //m_pitButtonBox->Y().OnTrue(RetrieveGamePiece(*this).ToPtr());                        // Yellow row 2

      m_pitButtonBox->LeftBumper().OnTrue(PlaceOnFloor(*this).ToPtr());                    // Red    row 1
      m_pitButtonBox->RightBumper().WhileTrue(IntakeRelease(*this).ToPtr());               // Red    row 2
      m_pitButtonBox->Start().WhileTrue(&m_rotateArm);                                     // Blue   row 1
      //m_pitButtonBox->Back().WhileTrue();                     // Black  row 1

      m_pitButtonBox->LeftStick().OnTrue(&m_extendArm);                            // Green  row 1
      m_pitButtonBox->RightStick().OnTrue(&m_retractArm);                           // Yellow row 1
      m_pitButtonBox->LeftTrigger().WhileTrue(TravelPosition(*this).ToPtr());       // Blue   row 2
      // m_pitButtonBox->RightTrigger().WhileTrue(&m_toggleClaw);                      // Black  row 2

      // auto loop = CommandScheduler::GetInstance().GetDefaultButtonLoop();
      // m_pitButtonBox->POVLeft(loop).Rising().IfHigh([this] {});  // Green  row 3
      // m_pitButtonBox->POVRight(loop).Rising().IfHigh([this] {});// Yellow row 3
      //m_pitButtonBox->POVUp(loop).Rising().IfHigh([this] {});      // Red    row 3
      //m_pitButtonBox->POVUp(loop).Rising().IfHigh({});      // Red    row 3
    }
  }
}
#endif

#ifdef BUTTON_BOX_DEVELOPMENT
void RobotContainer::ConfigSecondaryButtonBindingsNewWay()
{
  auto& secondary = m_secondaryController;
  // Raspberry PI Pico with gp2040 firmware Button Box
  //
  // Row	Black			    Blue			    Green				      Yellow				      Red
  // 1	  Back			    Start			    Left Stick Button	Right Stick Button	Left Bumper
  // 2	  Right Trigger	Left Trigger	X					        Y					          Right Bumper
  // 3	  B				      A				      POV Left			    POV Right			      POV Up
  // secondary.A().WhileTrue(IntakeIngest(*this).ToPtr());                          // Blue   row 3
  // secondary.A().OnFalse(IntakeStop(*this).ToPtr());                              // Blue   row 3
  secondary.A().OnTrue(TravelPosition(*this).ToPtr());                          // Blue   row 3
  secondary.B().OnTrue(PlaceLow(*this).ToPtr());                                 // Black  row 3
  secondary.X().OnTrue(PlaceHigh(*this).ToPtr());                                // Green  row 2
  secondary.Y().OnTrue(PickUp(*this).ToPtr());                        // Yellow row 2

  secondary.LeftBumper().OnTrue(PlaceOnFloor(*this).ToPtr());                    // Red    row 1
  // secondary.RightBumper().WhileTrue(IntakeRelease(*this).ToPtr());               // Red    row 2
  secondary.Start().WhileTrue(&m_rotateArm);                                     // Blue   row 1
  secondary.Back().WhileTrue(ClawIngest(*this).ToPtr());                     // Black  row 1

  secondary.LeftStick().OnTrue(&m_extendArm);                            // Green  row 1
  secondary.RightStick().OnTrue(&m_retractArm);                           // Yellow row 1
  secondary.LeftTrigger().OnTrue(TravelPosition(*this).ToPtr());       // Blue   row 2
  secondary.RightTrigger().WhileTrue(ClawRelease(*this).ToPtr());                      // Black  row 2

  // auto loop = CommandScheduler::GetInstance().GetDefaultButtonLoop();
  // secondary.POVLeft(loop).Rising().IfHigh([this] {});  // Green  row 3
  // secondary.POVRight(loop).Rising().IfHigh([this] {});// Yellow row 3
  //secondary.POVUp(loop).Rising().IfHigh([this] { PlaceHighCube(*this).Schedule(); });      // Red    row 3
  //secondary.POVUp(loop).Rising().IfHigh({});      // Red    row 3
}
#endif

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