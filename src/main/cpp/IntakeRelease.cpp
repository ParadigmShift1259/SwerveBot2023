
#include "IntakeRelease.h"

IntakeRelease::IntakeRelease(ISubsystemAccess& subsystemAccess)
 : m_deployment(subsystemAccess.GetDeployment())
 , m_intake(subsystemAccess.GetIntake())
{
  AddRequirements({&subsystemAccess.GetDeployment(), &subsystemAccess.GetIntake()});
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
  // Only retract intake if deployment arm is out of the way
  m_intake.IntakeOut(!m_deployment.IsOkayToRetractIntake());
}
