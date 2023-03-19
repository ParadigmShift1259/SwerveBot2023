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
//  m_deployment.RotateArm(kShelfAbsolute);
  m_deployment.RotateArmSetAbsPos(kShelfAbsolute);
}

void PickUp::Execute()
{
  //m_deployment.ResetEncoderWithAbsolute();
  //m_deployment.RotateArm(kShelfAbsolute);
}

bool PickUp::IsFinished()
{
  //return m_deployment.IsAtSetpoint(kShelfAbsolute);
  return m_deployment.IsAtSetpoint(kShelfPosition);
}

void PickUp::End(bool interrupted)
{
//  m_deployment.RotateArmRelative(0.0);
  m_logStartCommand.Append(false);
}