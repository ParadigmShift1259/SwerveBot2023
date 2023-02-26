#include "RetrievePosition.h"

#include "ConstantsDeploymentAngles.h"

RetrievePosition::RetrievePosition(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/placeHigh/startCommand");
}

void RetrievePosition::Execute()
{
    // m_deployment.ExtendArm();
    m_deployment.RotateArmToAngle(kRetrieveAngle);
}

bool RetrievePosition::IsFinished()
{
    return m_deployment.IsReverseLimitSwitchClosed();
}

void RetrievePosition::End(bool interrupted)
{
    m_logStartCommand.Append(false);
}