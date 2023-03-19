
#include "PlaceHigh.h"

#include "ConstantsDeploymentPositions.h"
#include "ConstantsDeploymentAbsolutes.h"

PlaceHigh::PlaceHigh(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/placeHigh/startCommand");
}

void PlaceHigh::Initialize()
{
  m_logStartCommand.Append(true);
  m_deployment.RetractBackPlate();
  m_deployment.RetractArm();
  m_deployment.RotateArm(kPlaceHighPosition);
}

void PlaceHigh::Execute()
{
  m_deployment.ResetEncoderWithAbsolute();
}

bool PlaceHigh::IsFinished()
{
  return m_deployment.IsAtSetpoint(kPlaceHighPosition);
}

void PlaceHigh::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}