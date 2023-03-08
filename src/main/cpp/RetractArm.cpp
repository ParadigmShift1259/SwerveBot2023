#include "RetractArm.h"

RetractArm::RetractArm(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});
  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/RetractArm/startCommand");
}

void RetractArm::Initialize()
{
  m_logStartCommand.Append(true);
}

void RetractArm::Execute()
{
  m_deployment.RetractArm();
}

bool RetractArm::IsFinished()
{
  return true;
}

void RetractArm::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}