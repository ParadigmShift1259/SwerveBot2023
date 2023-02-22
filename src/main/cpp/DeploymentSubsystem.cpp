#include "DeploymentSubsystem.h"

#include "ConstantsDeploymentAngles.h"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

DeploymentSubsystem::DeploymentSubsystem()
    : m_motor(kDeploymentCANID)
    , m_armSolenoid(PneumaticsModuleType::REVPH, kArmSolenoid)
    , m_backPlateSolenoid(PneumaticsModuleType::REVPH, kBackPlateSolenoid)
{
    m_motor.ConfigFactoryDefault();

    m_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0);
    m_motor.SetNeutralMode(NeutralMode::Brake);
    m_motor.SetInverted(true);
    m_motor.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10.0, 1.0);
    m_motor.EnableVoltageCompensation(true);
    m_motor.SetSensorPhase(true);

    // m_motor.ConfigForwardSoftLimitThreshold(180.0 * kDegreesPerTick);  // 70 degrees
    // m_motor.ConfigReverseSoftLimitThreshold(0.0 * kDegreesPerTick);  // 30 degrees
    // m_motor.ConfigForwardSoftLimitEnable(true);
    // m_motor.ConfigReverseSoftLimitEnable(true);

    m_motor.ConfigPeakOutputForward(0.4);
    m_motor.ConfigPeakOutputReverse(-0.4);
    m_motor.ConfigNominalOutputForward(0.1);
    m_motor.ConfigNominalOutputReverse(-0.1);

    m_motor.SetSelectedSensorPosition(-4000.0);  // TODO setting enc count to hanging pos for testing
    //m_motor.SetSelectedSensorPosition(1501.0);  // TODO setting enc count to hanging pos for testing

    m_motor.SelectProfileSlot(0, 0);
    m_motor.Config_kP(0, 0.5);
    m_motor.Config_kI(0, 0.01);
    m_motor.Config_kD(0, 0.1);
    m_motor.Config_kF(0, 0.0);
    m_motor.SetIntegralAccumulator(0.0);
}

void DeploymentSubsystem::Periodic() 
{
    double pos = m_motor.GetSelectedSensorPosition();
    double currentAngle = pos * kDegreesPerTick;
    double err = m_motor.GetClosedLoopError(0);
    double target = m_motor.GetClosedLoopTarget(0);
    double outputCurrent = m_motor.GetOutputCurrent();
    SmartDashboard::PutNumber("Arm enc", pos);
    SmartDashboard::PutNumber("Arm angle", currentAngle);
    SmartDashboard::PutNumber("Arm error", err);
    SmartDashboard::PutNumber("Arm target", target);
    SmartDashboard::PutBoolean("fwd limit", IsForwardLimitSwitchClosed());
    SmartDashboard::PutBoolean("rev limit", IsReverseLimitSwitchClosed());
    SmartDashboard::PutBoolean("output current", outputCurrent);
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
    //m_motor.NeutralOutput();
    m_backPlateSolenoid.Set(false);
    auto currentPos = m_motor.GetSelectedSensorPosition();
    degree_t currentAngle(currentPos * kDegreesPerTick);
    //auto newTickPos = currentPos + angle.to<double>() * kTicksPerDegree;  // Negate to indicate direction
    //auto newTickPos = copysign(1.0, (currentAngle - angle).to<double>()) * angle.to<double>() * kTicksPerDegree;  // Negate to indicate direction
    //auto newTickPos = copysign(1.0, (angle - currentAngle).to<double>()) * angle.to<double>() * kTicksPerDegree;  // Negate to indicate direction
    //auto newTickPos = -1.0 * angle.to<double>() * kTicksPerDegree;  // Negate to indicate direction
    auto newTickPos = angle.to<double>() * kTicksPerDegree;  // Negate to indicate direction
    printf("Rot angle %.3f currAngle %.3f delta %.3f ticks %.3f\n"
     , angle.to<double>()
     , currentAngle.to<double>()
     , (angle - currentAngle).to<double>()
     , newTickPos);
    m_motor.Set(ControlMode::Position, newTickPos);

    double err = m_motor.GetClosedLoopError(0);
    double target = m_motor.GetClosedLoopTarget(0);
    printf("Err %.3f target %.3f\n", err, target);
}

void DeploymentSubsystem::RotateArmToAngleRel(degree_t angle)
{
    m_backPlateSolenoid.Set(false);
     double pos = m_motor.GetSelectedSensorPosition();
     degree_t currentAngle(pos * kDegreesPerTick);
     printf("Rot angle %.3f currAngle %.3f delta %.3f ticks %.3f\n"
     , angle.to<double>()
     , currentAngle.to<double>()
     , (angle - currentAngle).to<double>()
     , (angle - currentAngle).to<double>() * kTicksPerDegree);
     m_motor.Set(ControlMode::Position, (angle - currentAngle).to<double>() * kTicksPerDegree);
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