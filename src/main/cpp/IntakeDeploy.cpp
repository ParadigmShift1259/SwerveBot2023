#include "IntakeDeploy.h"

IntakeDeploy::IntakeDeploy(ISubsystemAccess& subsystemAccess) 
  : m_intake(subsystemAccess.GetIntake())
{
  AddRequirements({&subsystemAccess.GetIntake()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/placeHigh/startCommand");
}

void IntakeDeploy::Execute() 
{
    m_intake.IntakeOut(true);
}

bool IntakeDeploy::IsFinished()
{
  return true;
}

void IntakeDeploy::End(bool interrupted) 
{
    m_logStartCommand.Append(false);
}