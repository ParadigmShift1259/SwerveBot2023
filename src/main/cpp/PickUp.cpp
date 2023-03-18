#include "PickUp.h"

#include "ConstantsDeploymentPositions.h"
#include "ConstantsDeploymentAbsolutes.h"

PickUp::PickUp(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/PickUp/startCommand");
}

void PickUp::Initialize()
{
  m_logStartCommand.Append(true);
  m_deployment.RetractBackPlate(); 
  m_deployment.RetractArm();
  m_deployment.RotateArm(kShelfPosition);
}

void PickUp::Execute()
{
  m_deployment.ResetEncoderWithAbsolute();
}

bool PickUp::IsFinished()
{
  return m_deployment.IsAtSetpoint(kShelfAbsolute);
}

void PickUp::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}