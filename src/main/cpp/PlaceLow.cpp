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
  m_deployment.RetractBackPlate(); 
  m_deployment.RetractArm();
  m_deployment.RotateArm(kPlaceLowPosition);
}

void PlaceLow::Execute()
{
  m_deployment.ResetEncoderWithAbsolute();
}

bool PlaceLow::IsFinished()
{
  return m_deployment.IsAtSetpoint(kPlaceLowPosition);
}

void PlaceLow::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}