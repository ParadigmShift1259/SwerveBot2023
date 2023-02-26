#include "ClawOpen.h"

ClawOpen::ClawOpen(ISubsystemAccess& subsystemAccess) 
  : m_claw(subsystemAccess.GetClaw())
{
  AddRequirements({&subsystemAccess.GetClaw()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/placeHigh/startCommand");
}

void ClawOpen::Execute()
{
    m_claw.Open();
    m_logStartCommand.Append(false);
}

bool ClawOpen::IsFinished()
{
    return true;
}