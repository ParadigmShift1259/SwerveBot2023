#include "DeploymentSubsystem.h"
using namespace std;
using namespace frc;

DeploymentSubsystem::DeploymentSubsystem()
    : m_motor(kDeploymentCANID)
    , m_solenoid(PneumaticsModuleType::REVPH, kDeploymentSolenoid)
{

}

void DeploymentSubsystem::Periodic() 
{

}

void DeploymentSubsystem::Set(double speed)
{

}