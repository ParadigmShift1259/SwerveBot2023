#include "DeploymentSubsystem.h"

#include "ConstantsDeploymentAngles.h"

using namespace std;
using namespace frc;

DeploymentSubsystem::DeploymentSubsystem()
    : m_motor(kDeploymentCANID)
    , m_armSolenoid(PneumaticsModuleType::REVPH, kArmSolenoid)
    , m_backPlateSolenoid(PneumaticsModuleType::REVPH, kBackPlateSolenoid)
{

}

void DeploymentSubsystem::Periodic() 
{

}

void DeploymentSubsystem::RotateIntoFrame(double speed)
{
    m_backPlateSolenoid.Set(false);
    m_motor.Set(ControlMode::PercentOutput, -speed);
}

void DeploymentSubsystem::RotateOutOfFrame(double speed)
{
    m_backPlateSolenoid.Set(false);
    m_motor.Set(ControlMode::PercentOutput, speed);
}

void DeploymentSubsystem::ExtendArm()
{
    m_armSolenoid.Set(true);
}

void DeploymentSubsystem::RetractArm()
{
    m_armSolenoid.Set(false);
}

void DeploymentSubsystem::ExtendBackPlate()
{
    m_backPlateSolenoid.Set(true);
}

void DeploymentSubsystem::RetractBackPlate()
{
    m_backPlateSolenoid.Set(false);
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
    // TODO Figure out actual angle tolerance
    double currentAngle = m_motor.GetSelectedSensorPosition() * kTicksPerDegree;
    return (fabs(setpoint.to<double>() - currentAngle) < 0.1);
}

degree_t DeploymentSubsystem::CurrentDegreePosition()
{
    return degree_t(m_motor.GetSelectedSensorPosition() * kTicksPerDegree);
}

bool DeploymentSubsystem::IsOkayToRetractIntake()
{
    degree_t currentAngle(m_motor.GetSelectedSensorPosition() * kTicksPerDegree);
    return currentAngle > kTravelAngle;
}