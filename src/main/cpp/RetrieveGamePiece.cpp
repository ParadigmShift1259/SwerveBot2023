#include "RetrieveGamePiece.h"

#include "ConstantsDeploymentAngles.h"

#include "IntakeDeploy.h"

#include <frc/smartdashboard/SmartDashboard.h>

RetrieveGamePiece::RetrieveGamePiece(ISubsystemAccess& subsystemAccess) 
  : m_claw(subsystemAccess.GetClaw())
  , m_deployment(subsystemAccess.GetDeployment())
  , m_intake(subsystemAccess.GetIntake())
  , m_subsystemAccess(subsystemAccess)
{
  AddRequirements({&subsystemAccess.GetClaw(), &subsystemAccess.GetDeployment(), &subsystemAccess.GetIntake()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/RetrieveGamePiece/startCommand");
  m_logStartCommand.Append(true);
}

void RetrieveGamePiece::Execute()
{
  IntakeDeploy(m_subsystemAccess).Schedule();
  // RetrievePosition(*this)
  // ClawOpen(*this)
  // InstantCommand{[this] { m_deployment.ExtendArm(); }, {&m_deployment} }
  // WaitCommand{1.2_s}
  // ClawClose(*this)
  // WaitCommand{0.5_s}
  // ClearancePosition(*this)
  // TravelPosition(*this) // Retracts BackPlate and arm


  // m_intake.ExtendIntake();
  // m_deployment.RotateArmToAngle(kRetrieveAngle);
  // m_claw.Open();
  // m_deployment.ExtendArm();
  // // wait 1.2
  // m_claw.Close();
  // // wait 0.5
  // m_deployment.RotateArmToAngle(kClearanceAngle);
  // m_deployment.RetractBackPlate();
  // m_deployment.RetractArm();
  // m_deployment.RotateArmToAngle(kTravelAngle);
  

  // SequentialCommandGroup m_retrieveGamePiece
  // { 
  //       IntakeDeploy(*this)
  //     , RetrievePosition(*this)
  //     , ClawOpen(*this)
  //     , InstantCommand{[this] { m_deployment.ExtendArm(); }, {&m_deployment} }
  //     , WaitCommand{1.2_s}
  //     , ClawClose(*this)
  //     , WaitCommand{0.5_s}
  //     , ClearancePosition(*this)
  //     , TravelPosition(*this) // Retracts BackPlate and arm
  // };
}

bool RetrieveGamePiece::IsFinished()
{
  return m_deployment.IsAtDegreeSetpoint(kRetrieveAngle);
}

void RetrieveGamePiece::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}