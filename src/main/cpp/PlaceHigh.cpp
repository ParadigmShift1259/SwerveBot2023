
#include "PlaceHigh.h"

#include "ConstantsDeploymentAngles.h"
//#include <frc/smartdashboard/SmartDashboard.h>

PlaceHigh::PlaceHigh(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/placeHigh/startCommand");
  
  m_logStartCommand.Append(true);
}

void PlaceHigh::Initialize()
{
  m_deployment.RetractBackPlate();
  m_deployment.RetractArm();
  m_deployment.RotateArmToAngle(kPlaceHighAngle);
}

void PlaceHigh::Execute()
{

}

bool PlaceHigh::IsFinished()
{
  return m_deployment.IsAtDegreeSetpoint(kPlaceHighAngle);
}

void PlaceHigh::End(bool interrupted)
{
  m_deployment.ExtendArm();
  m_logStartCommand.Append(false);
}