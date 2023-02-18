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

void DeploymentSubsystem::RotateIntoFrame(double speed)
{
    m_motor.Set(ControlMode::PercentOutput, -speed);
}

void DeploymentSubsystem::RotateOutOfFrame(double speed)
{
    m_motor.Set(ControlMode::PercentOutput, speed);
}

void DeploymentSubsystem::Extend()
{
    m_solenoid.Set(true);
}

void DeploymentSubsystem::Retract()
{
    m_solenoid.Set(false);
}

bool DeploymentSubsystem::IsForwardLimitSwitchClosed()
{
    return m_motor.IsFwdLimitSwitchClosed() == 1;
}

bool DeploymentSubsystem::IsReverseLimitSwitchClosed()
{
    return m_motor.IsRevLimitSwitchClosed() == 1;
}

void DeploymentSubsystem::Stop()
{
    m_motor.Set(ControlMode::PercentOutput, 0.0);
}

bool DeploymentSubsystem::IsAtDegreeSetpoint(degree_t setpoint)
{
    double currentAngle = m_motor.GetSelectedSensorPosition() * kTicksPerDegree;
    return (fabs(setpoint.to<double>() - currentAngle) < 0.1);
}

degree_t DeploymentSubsystem::CurrentDegreePosition()
{
    return degree_t(m_motor.GetSelectedSensorPosition() * kTicksPerDegree);
}