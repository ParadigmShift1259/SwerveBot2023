#include "ExtendArm.h"

ExtendArm::ExtendArm(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});
  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/ExtendArm/startCommand");
}

void ExtendArm::Initialize()
{
  m_logStartCommand.Append(true);
}

void ExtendArm::Execute()
{
  m_deployment.ExtendArm();
}

bool ExtendArm::IsFinished()
{
  return true;
}

void ExtendArm::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}