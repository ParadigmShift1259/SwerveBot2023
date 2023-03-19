#include "PlaceOnFloor.h"

#include "ConstantsDeploymentPositions.h"
#include "ConstantsDeploymentAbsolutes.h"

PlaceOnFloor::PlaceOnFloor(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/placeOnFloor/startCommand");
}

void PlaceOnFloor::Initialize()
{
  m_logStartCommand.Append(true);
  m_deployment.RetractBackPlate();
  m_deployment.RetractArm();
  m_deployment.RotateArm(kPlaceOnFloorPosition);
}

void PlaceOnFloor::Execute()
{
  m_deployment.ResetEncoderWithAbsolute();
}

bool PlaceOnFloor::IsFinished()
{
  return m_deployment.IsAtSetpoint(kPlaceOnFloorPosition);
}

void PlaceOnFloor::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}