#include "TravelPosition.h"

#include "ConstantsDeploymentAngles.h"

TravelPosition::TravelPosition(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment(), &subsystemAccess.GetIntake()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/travelPosition/startCommand");
}

void TravelPosition::Execute()
{
  m_deployment.RetractBackPlate();
  m_deployment.RetractArm();
  m_deployment.RotateArmToAngle(kTravelAngle);
}

bool TravelPosition::IsFinished()
{
  return m_deployment.IsAtDegreeSetpoint(kTravelAngle);
}

void TravelPosition::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}