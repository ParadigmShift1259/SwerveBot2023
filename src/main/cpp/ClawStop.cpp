#include "ClawStop.h"

ClawStop::ClawStop(ISubsystemAccess& subsystemAccess) 
  : m_claw(subsystemAccess.GetClaw())
{
  AddRequirements({&subsystemAccess.GetClaw()});
  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/ClawStop/startCommand");
}

void ClawStop::Initialize()
{
  m_logStartCommand.Append(true);
}

void ClawStop::Execute()
{
  m_claw.Stop();
}

bool ClawStop::IsFinished()
{
  return true;
}

void ClawStop::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}