#include "IntakeDeploy.h"

IntakeDeploy::IntakeDeploy(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
  , m_intake(subsystemAccess.GetIntake())
{
  AddRequirements({&subsystemAccess.GetDeployment(), &subsystemAccess.GetIntake()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/intakeDeploy/startCommand");
}

void IntakeDeploy::Initialize()
{
  m_logStartCommand.Append(true);
}

void IntakeDeploy::Execute() 
{
  if (m_deployment.IsInFrame())
  {
    m_intake.ExtendIntake();
  }
}

bool IntakeDeploy::IsFinished()
{
  return true;
}

void IntakeDeploy::End(bool interrupted) 
{
  m_logStartCommand.Append(false);
}