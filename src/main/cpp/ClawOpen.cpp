#include "ClawOpen.h"

ClawOpen::ClawOpen(ISubsystemAccess& subsystemAccess) 
  : m_claw(subsystemAccess.GetClaw())
{
  AddRequirements({&subsystemAccess.GetClaw()});
}

void ClawOpen::Execute()
{
    m_claw.Open();
}

bool ClawOpen::IsFinished()
{
    return true;
}