#include "TravelPosition.h"

#include "ConstantsDeploymentAngles.h"

TravelPosition::TravelPosition(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
  , m_intake(subsystemAccess.GetIntake())
{
  AddRequirements({&subsystemAccess.GetDeployment(), &subsystemAccess.GetIntake()});
}

void TravelPosition::Execute()
{
    m_deployment.RetractArm();
    if (m_deployment.CurrentDegreePosition() < kTravelAngle)
    {
        m_deployment.RotateOutOfFrame(0.2);
    }
    else
    {
        m_deployment.RotateIntoFrame(0.2);
    }
}

bool TravelPosition::IsFinished()
{
    return m_deployment.IsAtDegreeSetpoint(kTravelAngle);
}

void TravelPosition::End(bool interrupted)
{
    m_deployment.Stop();
    m_intake.IntakeOut(false);
}