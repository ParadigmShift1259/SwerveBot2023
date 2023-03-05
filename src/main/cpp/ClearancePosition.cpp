#include "ClearancePosition.h"

#include "ConstantsDeploymentAngles.h"

ClearancePosition::ClearancePosition(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/clearancePosition/startCommand");
  m_logStartCommand.Append(true);
}

void ClearancePosition::Execute()
{
  m_deployment.RetractBackPlate();
  m_deployment.RotateArmToAngle(kClearanceAngle);
}

bool ClearancePosition::IsFinished()
{
  return m_deployment.IsAtDegreeSetpoint(kClearanceAngle);
}

void ClearancePosition::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}