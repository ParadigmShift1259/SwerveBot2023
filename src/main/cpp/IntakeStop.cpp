
#include "IntakeStop.h"

IntakeStop::IntakeStop(ISubsystemAccess& subsystemAccess, bool retractIntake)
 : m_intake(subsystemAccess.GetIntake())
 , m_retractIntake(retractIntake)
{
  AddRequirements({&subsystemAccess.GetIntake()});
}

void IntakeStop::Execute() {
  m_intake.Set(0);
}

bool IntakeStop::IsFinished()
{
  return true;
}

void IntakeStop::End(bool interrupted) {
  m_intake.IntakeOut(m_retractIntake);
}
