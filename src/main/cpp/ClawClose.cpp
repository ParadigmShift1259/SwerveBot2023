#include "ClawClose.h"

ClawClose::ClawClose(ISubsystemAccess& subsystemAccess) 
  : m_claw(subsystemAccess.GetClaw())
{
  AddRequirements({&subsystemAccess.GetClaw()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/placeHigh/startCommand");
}

void ClawClose::Execute()
{
    m_claw.Close();
    m_logStartCommand.Append(false);
}

bool ClawClose::IsFinished()
{
    return true;
}
