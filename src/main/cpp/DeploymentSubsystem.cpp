#include "DeploymentSubsystem.h"

#include "ConstantsDeploymentAngles.h"

#include <units/math.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

constexpr double kDefaultP = 3.0;
constexpr double kDefaultI = 0.0;
constexpr double kDefaultD = 0.0;

DeploymentSubsystem::DeploymentSubsystem()
    : m_motor(kDeploymentCANID, CANSparkMaxLowLevel::MotorType::kBrushless)
    , m_armSolenoid(PneumaticsModuleType::CTREPCM, kArmSolenoid)
    , m_backPlateSolenoid(PneumaticsModuleType::CTREPCM, kBackPlateSolenoid)
{
    m_motor.RestoreFactoryDefaults();

    m_motor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_motor.SetInverted(true);
    m_motor.EnableVoltageCompensation(true);
    m_motor.SetSmartCurrentLimit(30);
 
    // not available in brushless m_enc.SetInverted(true);
    m_enc.SetPosition(kInitialPosition);

    m_pid.SetOutputRange(-8.0, 8.0);
    m_pid.SetP(kDefaultP);
    m_pid.SetI(kDefaultI);
    m_pid.SetD(kDefaultD);
    m_pid.SetFF(0.0);
    m_pid.SetIAccum(0.0);
    m_pid.SetFeedbackDevice(m_enc);

    SmartDashboard::PutNumber("GotoAngle", 0.0);
    SmartDashboard::PutNumber("MaxOutput", 2.0);
    SmartDashboard::PutNumber("P", kDefaultP);
    SmartDashboard::PutNumber("I", kDefaultI);
    SmartDashboard::PutNumber("D", kDefaultD);
}

void DeploymentSubsystem::Periodic() 
{
    double pos = m_enc.GetPosition();
    double currentAngle = TicksToDegreesDouble(pos);
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

    // double maxOutput = SmartDashboard::GetNumber("MaxOutput", 2.0);
    // m_pid.SetOutputRange(-maxOutput, maxOutput);
}

void DeploymentSubsystem::RotateArmToAngle(degree_t angle)
{
    m_pid.SetP(SmartDashboard::GetNumber("P", kDefaultP));
    m_pid.SetI(SmartDashboard::GetNumber("I", kDefaultI));
    m_pid.SetD(SmartDashboard::GetNumber("D", kDefaultD));

    //m_motor.NeutralOutput();
    m_backPlateSolenoid.Set(false);
    auto currentPos = m_enc.GetPosition();
    degree_t currentAngle = TicksToDegrees(currentPos);
    m_setpointTicks = DegreesToTicks(angle);
    printf("Rot angle %.3f currAngle %.3f delta %.3f ticks %.3f\n"
     , angle.to<double>()
     , currentAngle.to<double>()
     , (angle - currentAngle).to<double>()
     , m_setpointTicks);
    m_pid.SetReference(m_setpointTicks, CANSparkMaxLowLevel::ControlType::kPosition);
}

void DeploymentSubsystem::ExtendArm()
{
    m_armSolenoid.Set(kArmSolenoidExtend);
}

void DeploymentSubsystem::RetractArm()
{
    m_armSolenoid.Set(kArmSolenoidRetract);
}

void DeploymentSubsystem::ExtendBackPlate()
{
    m_backPlateSolenoid.Set(kBackPlateSolenoidExtend);
}

void DeploymentSubsystem::RetractBackPlate()
{
    m_backPlateSolenoid.Set(kBackPlateSolenoidRetract);
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
    degree_t currentAngle = TicksToDegrees(m_enc.GetPosition());
    return (units::math::fabs(setpoint - currentAngle) < 0.1_deg);
}

degree_t DeploymentSubsystem::CurrentDegreePosition()
{
    return TicksToDegrees(m_enc.GetPosition());
}

bool DeploymentSubsystem::IsOkayToRetractIntake()
{
    degree_t currentAngle = TicksToDegrees(m_enc.GetPosition());
    return currentAngle > kTravelAngle;
}
