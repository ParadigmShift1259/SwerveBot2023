#include "ClawClose.h"

ClawClose::ClawClose(ISubsystemAccess& subsystemAccess) 
  : m_claw(subsystemAccess.GetClaw())
{
  AddRequirements({&subsystemAccess.GetClaw()});
}

void ClawClose::Execute()
{
    m_claw.Close();
}

bool ClawClose::IsFinished()
{
    return true;
}
