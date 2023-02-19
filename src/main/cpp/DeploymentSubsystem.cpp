#include "DeploymentSubsystem.h"

#include "ConstantsDeploymentAngles.h"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace std;
using namespace frc;

DeploymentSubsystem::DeploymentSubsystem()
    : m_motor(kDeploymentCANID)
    , m_armSolenoid(PneumaticsModuleType::REVPH, kArmSolenoid)
    , m_backPlateSolenoid(PneumaticsModuleType::REVPH, kBackPlateSolenoid)
{
    m_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0);
    m_motor.SetNeutralMode(NeutralMode::Brake);
    // m_motor.SetInverted(true);
    m_motor.SetSelectedSensorPosition(1501.0);  // TODO setting enc count to hanging pos for testing

    // m_motor.Config_kP(0, 0.5);
    // m_motor.Config_kI(0, 0.0);
    // m_motor.Config_kD(0, 0.0);
    // m_motor.Config_kF(0, 0.0);
}

void DeploymentSubsystem::Periodic() 
{
    double pos = m_motor.GetSelectedSensorPosition();
    double currentAngle = pos * kDegreesPerTick;
    SmartDashboard::PutNumber("Arm enc", pos);
    SmartDashboard::PutNumber("Arm angle", currentAngle);
}

void DeploymentSubsystem::RotateIntoFrame(double speed)
{
    m_backPlateSolenoid.Set(false);
    m_motor.Set(ControlMode::PercentOutput, speed);
}

void DeploymentSubsystem::RotateOutOfFrame(double speed)
{
    m_backPlateSolenoid.Set(false);
    m_motor.Set(ControlMode::PercentOutput, -speed);
}

void DeploymentSubsystem::RotateArmToAngle(degree_t angle)
{
    m_backPlateSolenoid.Set(false);
    m_motor.Set(ControlMode::Position, angle.to<double>() * kTicksPerDegree);
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
    double currentAngle = m_motor.GetSelectedSensorPosition() * kDegreesPerTick;
    return (fabs(setpoint.to<double>() - currentAngle) < 0.1);
}

degree_t DeploymentSubsystem::CurrentDegreePosition()
{
    return degree_t(m_motor.GetSelectedSensorPosition() * kDegreesPerTick);
}

bool DeploymentSubsystem::IsOkayToRetractIntake()
{
    degree_t currentAngle(m_motor.GetSelectedSensorPosition() * kDegreesPerTick);
    return currentAngle > kTravelAngle;
}