#include "RetrievePosition.h"

#include "ConstantsDeploymentAngles.h"
#include <frc/smartdashboard/SmartDashboard.h>

//#include <frc/smartdashboard/SmartDashboard.h>

RetrievePosition::RetrievePosition(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/retrievePosition/startCommand");
}

void RetrievePosition::Initialize()
{
  m_logStartCommand.Append(true);
}

void RetrievePosition::Execute()
{
  m_deployment.RotateArmToAngle(kRetrieveAngle);
}

bool RetrievePosition::IsFinished()
{
  return m_deployment.IsAtDegreeSetpoint(kRetrieveAngle);
}

void RetrievePosition::End(bool interrupted)
{
  //m_deployment.ExtendArm();
  m_logStartCommand.Append(false);
}