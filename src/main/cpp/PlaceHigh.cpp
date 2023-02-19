#include "PlaceHigh.h"

#include "ConstantsDeploymentAngles.h"
#include <frc/smartdashboard/SmartDashboard.h>

PlaceHigh::PlaceHigh(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});
  SmartDashboard::PutNumber("rot speed", kRotateSpeed);
}

void PlaceHigh::Execute()
{
    SmartDashboard::PutBoolean("fwd limit", IsFinished());
    m_deployment.ExtendArm();
    auto spd = SmartDashboard::GetNumber("rot speed", kRotateSpeed);
    m_deployment.RotateOutOfFrame(spd);
    //m_deployment.RotateOutOfFrame(kRotateSpeed);
}

bool PlaceHigh::IsFinished()
{
    return m_deployment.IsForwardLimitSwitchClosed();
}

void PlaceHigh::End(bool interrupted)
{
    //m_deployment.Stop();
}