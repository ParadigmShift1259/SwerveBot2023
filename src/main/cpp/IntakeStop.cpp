
#include "IntakeStop.h"

IntakeStop::IntakeStop(ISubsystemAccess& subsystemAccess, bool retractIntake)
 : m_intake(subsystemAccess.GetIntake())
 , m_retractIntake(retractIntake)
{
  AddRequirements({&subsystemAccess.GetIntake()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/placeHigh/startCommand");
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
  m_logStartCommand.Append(false);
}
