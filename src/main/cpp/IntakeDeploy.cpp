#include "IntakeDeploy.h"

IntakeDeploy::IntakeDeploy(ISubsystemAccess& subsystemAccess) 
  : m_intake(subsystemAccess.GetIntake())
{
  AddRequirements({&subsystemAccess.GetIntake()});
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

}