
#include "IntakeRelease.h"

IntakeRelease::IntakeRelease(ISubsystemAccess& subsystemAccess)
 : m_intake(subsystemAccess.GetIntake())
{
  AddRequirements({&subsystemAccess.GetIntake()});
}

void IntakeRelease::Execute() {
  m_intake.Set(kReleaseSpeed);
}

bool IntakeRelease::IsFinished()
{
  return true;
}

void IntakeRelease::End(bool interrupted) {
  m_intake.Set(0);
  m_intake.IntakeOut(false);
}
