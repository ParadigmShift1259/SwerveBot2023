#include "DeploymentSubsystem.h"
using namespace std;
using namespace frc;

DeploymentSubsystem::DeploymentSubsystem()
    : m_motor(0)//kMotorPort)
    , m_solenoid(PneumaticsModuleType::REVPH, 0)//kSolenoidPort)
{

}

void DeploymentSubsystem::Periodic() 
{

}

void DeploymentSubsystem::Set(double speed)
{

}