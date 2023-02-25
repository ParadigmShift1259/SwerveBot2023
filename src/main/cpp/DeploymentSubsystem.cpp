#include "DeploymentSubsystem.h"

#include "ConstantsDeploymentAngles.h"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

constexpr double kDefaultP = 1.0;
constexpr double kDefaultI = 0.001;
constexpr double kDefaultD = 0.0;

DeploymentSubsystem::DeploymentSubsystem()
    : m_motor(kDeploymentCANID, CANSparkMaxLowLevel::MotorType::kBrushless)
    , m_armSolenoid(PneumaticsModuleType::REVPH, kArmSolenoid)
    , m_backPlateSolenoid(PneumaticsModuleType::REVPH, kBackPlateSolenoid)
{
    m_motor.RestoreFactoryDefaults();

    m_motor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_motor.SetInverted(true);
    m_motor.EnableVoltageCompensation(true);
    m_motor.SetSmartCurrentLimit(30);
 
    // not available in brushless m_enc.SetInverted(true);
    m_enc.SetPosition(0.0);  // TODO setting enc count to hanging pos for testing

    m_pid.SetOutputRange(-1.0, 1.0);
    m_pid.SetP(kDefaultP);
    m_pid.SetI(kDefaultI);
    m_pid.SetD(kDefaultD);
    m_pid.SetFF(0.0);
    m_pid.SetIAccum(0.0);
    m_pid.SetFeedbackDevice(m_enc);

    SmartDashboard::PutNumber("GotoAngle", 0.0);
    SmartDashboard::PutNumber("P", kDefaultP);
    SmartDashboard::PutNumber("I", kDefaultI);
    SmartDashboard::PutNumber("D", kDefaultD);
}

void DeploymentSubsystem::Periodic() 
{
    double pos = m_enc.GetPosition();
    double currentAngle = pos * kDegreesPerTick;
    double err = pos - m_setpointTicks;
    SmartDashboard::PutNumber("Arm enc", pos);
    SmartDashboard::PutNumber("Arm angle", currentAngle);
    SmartDashboard::PutNumber("Arm error", err);

    double outputCurrent = m_motor.GetOutputCurrent();
    double motorOutput = m_motor.GetAppliedOutput();
    double motorTemp = m_motor.GetMotorTemperature();
    SmartDashboard::PutNumber("output current", outputCurrent);
    SmartDashboard::PutNumber("motor output", motorOutput);
    SmartDashboard::PutNumber("motor temp", motorTemp);

    SmartDashboard::PutBoolean("fwd limit", IsForwardLimitSwitchClosed());
    SmartDashboard::PutBoolean("rev limit", IsReverseLimitSwitchClosed());
}

void DeploymentSubsystem::RotateIntoFrame(double speed)
{
    m_backPlateSolenoid.Set(false);
    //m_motor.Set(ControlMode::PercentOutput, speed);
}

void DeploymentSubsystem::RotateOutOfFrame(double speed)
{
    m_backPlateSolenoid.Set(false);
    //m_motor.Set(ControlMode::PercentOutput, -speed);
}

void DeploymentSubsystem::RotateArmToAngle(degree_t angle)
{
    m_pid.SetP(SmartDashboard::GetNumber("P", kDefaultP));
    m_pid.SetI(SmartDashboard::GetNumber("I", kDefaultI));
    m_pid.SetD(SmartDashboard::GetNumber("D", kDefaultD));

    //m_motor.NeutralOutput();
    m_backPlateSolenoid.Set(false);
    auto currentPos = m_enc.GetPosition();
    degree_t currentAngle(currentPos * kDegreesPerTick);
    //auto newTickPos = currentPos + angle.to<double>() * kTicksPerDegree;  // Negate to indicate direction
    //auto newTickPos = copysign(1.0, (currentAngle - angle).to<double>()) * angle.to<double>() * kTicksPerDegree;  // Negate to indicate direction
    //auto newTickPos = copysign(1.0, (angle - currentAngle).to<double>()) * angle.to<double>() * kTicksPerDegree;  // Negate to indicate direction
    //auto newTickPos = -1.0 * angle.to<double>() * kTicksPerDegree;  // Negate to indicate direction
    //auto newTickPos = angle.to<double>() * kTicksPerDegree;  // Negate to indicate direction
    angle = degree_t(SmartDashboard::GetNumber("GotoAngle", 0.0));
    m_setpointTicks = angle.to<double>() * kTicksPerDegree;
    printf("Rot angle %.3f currAngle %.3f delta %.3f ticks %.3f\n"
     , angle.to<double>()
     , currentAngle.to<double>()
     , (angle - currentAngle).to<double>()
     , m_setpointTicks);
    m_pid.SetReference(m_setpointTicks, CANSparkMaxLowLevel::ControlType::kPosition);

    //if (m_pid.GetControlMode() == ControlType::kPosition)
    // {
    //     double pos = m_enc.GetPosition();
    //     double currentAngle = pos * kDegreesPerTick;
    //     double err = pos - m_setpointTicks;
    //     printf("Err %.3f target %.3f angle %.3f\n", err, m_setpointTicks, currentAngle);
    // }
}

// void DeploymentSubsystem::RotateArmToAngleRel(degree_t angle)
// {
//     m_backPlateSolenoid.Set(false);
//      double pos = m_motor.GetSelectedSensorPosition();
//      degree_t currentAngle(pos * kDegreesPerTick);
//      printf("Rot angle %.3f currAngle %.3f delta %.3f ticks %.3f\n"
//      , angle.to<double>()
//      , currentAngle.to<double>()
//      , (angle - currentAngle).to<double>()
//      , (angle - currentAngle).to<double>() * kTicksPerDegree);
//      m_motor.Set(ControlMode::Position, (angle - currentAngle).to<double>() * kTicksPerDegree);
// }

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
    return false;//m_motor.GetForwardLimitSwitch(SparkMaxLimitSwitch::Type::kNormallyOpen).Get();
}

bool DeploymentSubsystem::IsReverseLimitSwitchClosed()
{
    return false; //m_motor.GetReverseLimitSwitch(SparkMaxLimitSwitch::Type::kNormallyOpen).Get();
}

void DeploymentSubsystem::Stop()
{
    m_motor.Set(0.0);
}

bool DeploymentSubsystem::IsAtDegreeSetpoint(degree_t setpoint)
{
    // TODO Figure out actual angle tolerance
    double currentAngle = m_enc.GetPosition() * kDegreesPerTick;
    return (fabs(setpoint.to<double>() - currentAngle) < 0.1);
}

degree_t DeploymentSubsystem::CurrentDegreePosition()
{
    return degree_t(m_enc.GetPosition() * kDegreesPerTick);
}

bool DeploymentSubsystem::IsOkayToRetractIntake()
{
    degree_t currentAngle(m_enc.GetPosition() * kDegreesPerTick);
    return currentAngle > kTravelAngle;
}
