#include "RetrieveGamePiece.h"

#include "ConstantsDeploymentAngles.h"

#include "ClawClose.h"
#include "ClawOpen.h"
#include "ClearancePosition.h"
#include "ExtendArm.h"
#include "IntakeDeploy.h"
#include "PlaceHigh.h"
#include "RetrievePosition.h"
#include "TravelPosition.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/InstantCommand.h>

RetrieveGamePiece::RetrieveGamePiece(ISubsystemAccess& subsystemAccess) 
  // : m_claw(subsystemAccess.GetClaw())
  // , m_deployment(subsystemAccess.GetDeployment())
  // , m_intake(subsystemAccess.GetIntake())
  //, m_subsystemAccess(subsystemAccess)
  : m_retrieveGamePiece(
        IntakeDeploy(subsystemAccess)
      , RetrievePosition(subsystemAccess)
      , ClawOpen(subsystemAccess)
      , ExtendArm(subsystemAccess)
      , WaitCommand{1.2_s}
      , ClawClose(subsystemAccess)
      , WaitCommand{0.5_s}
      , ClearancePosition(subsystemAccess)
      , TravelPosition(subsystemAccess) // Retracts BackPlate and arm
      , WaitCommand{2.0_s}
      , PlaceHigh(subsystemAccess)
      // , InstantCommand([this] { m_isFinished = true;}, {})
      )
{
  AddRequirements({&subsystemAccess.GetClaw(), &subsystemAccess.GetDeployment(), &subsystemAccess.GetIntake()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/RetrieveGamePiece/startCommand");
  m_logStartCommand.Append(true);
}

void RetrieveGamePiece::Initialize()
{
  m_retrieveGamePiece.Schedule();
}

void RetrieveGamePiece::Execute()
{
}

bool RetrieveGamePiece::IsFinished()
{
  //return m_isFinished;
  // return m_deployment.IsAtDegreeSetpoint(kPlaceHighAngle);
  return m_retrieveGamePiece.IsFinished();
}

void RetrieveGamePiece::End(bool interrupted)
{
  printf("RetrieveGamePiece Interrupted %s", interrupted ? "true" : "false");
  m_logStartCommand.Append(false);
}