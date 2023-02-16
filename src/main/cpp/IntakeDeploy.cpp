#include "IntakeDeploy.h"

IntakeDeploy::IntakeDeploy(ISubsystemAccess& subsystemAccess) 
  : m_intake(subsystemAccess.GetIntake())
{
  AddRequirements({&subsystemAccess.GetIntake()});
}

void IntakeDeploy::Execute() {
    m_intake.IntakeOut(true);
    m_bRunning = true;
}

bool IntakeDeploy::IsFinished()
{
  return m_bRunning;
}

void IntakeDeploy::End(bool interrupted) {
    m_bRunning = false;
}