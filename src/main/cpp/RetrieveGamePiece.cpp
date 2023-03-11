#include "RetrieveGamePiece.h"

#include "ConstantsDeploymentAngles.h"

#include "ClawClose.h"
#include "ClawOpen.h"
#include "ClearancePosition.h"
#include "ExtendArm.h"
#include "IntakeDeploy.h"
#include "PlaceHigh.h"
#include "RetractArm.h"
#include "RetrievePosition.h"
#include "TravelPosition.h"

//#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/InstantCommand.h>

RetrieveGamePiece::RetrieveGamePiece(ISubsystemAccess& subsystemAccess) 
  : m_retrieveGamePiece(
        TravelPosition(subsystemAccess)   // Retracts BackPlate and arm
      , RetrievePosition(subsystemAccess)
      , ClawOpen(subsystemAccess)
      , ExtendArm(subsystemAccess)
      , WaitCommand{0.5_s}                // Wait for arm to extend
      , ClawClose(subsystemAccess)
      , WaitCommand{0.5_s}                // Wait for claw to close
      , TravelPosition(subsystemAccess) // Retracts BackPlate and arm
      )
{
  AddRequirements({&subsystemAccess.GetClaw(), &subsystemAccess.GetDeployment(), &subsystemAccess.GetIntake()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/RetrieveGamePiece/startCommand");
}

void RetrieveGamePiece::Initialize()
{
  m_logStartCommand.Append(true);
  m_retrieveGamePiece.Schedule();
}

void RetrieveGamePiece::Execute()
{
}

bool RetrieveGamePiece::IsFinished()
{
  return m_retrieveGamePiece.IsFinished();
}

void RetrieveGamePiece::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}