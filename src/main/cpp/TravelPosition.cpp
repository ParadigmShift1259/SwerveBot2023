#include "TravelPosition.h"

#include <frc2/command/WaitCommand.h>

#include "ConstantsDeploymentAngles.h"

TravelPosition::TravelPosition(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/travelPosition/startCommand");
}

void TravelPosition::Initialize()
{
  m_logStartCommand.Append(true);
  m_deployment.RetractBackPlate();
  m_deployment.RetractArm();
  frc2::WaitCommand(0.5_s);
  m_deployment.RotateArmToAngle(kTravelAngle);
}

void TravelPosition::Execute()
{

}

bool TravelPosition::IsFinished()
{
  return m_deployment.IsAtDegreeSetpoint(kTravelAngle);
}

void TravelPosition::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}