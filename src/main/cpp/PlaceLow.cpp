#include "PlaceLow.h"

#include "ConstantsDeploymentPositions.h"
#include "ConstantsDeploymentAbsolutes.h"

PlaceLow::PlaceLow(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/placeLow/startCommand");
}

void PlaceLow::Initialize()
{
  m_logStartCommand.Append(true);
  m_deployment.RetractArm();
  m_deployment.RotateArm(kPlaceLowPosition);
}

void PlaceLow::Execute()
{

}

bool PlaceLow::IsFinished()
{
  return m_deployment.IsAtAbsoluteSetpoint(kPlaceLowAbsolute);
}

void PlaceLow::End(bool interrupted)
{
  m_deployment.ResetEncoder(kPlaceLowPosition);
  m_logStartCommand.Append(false);
}