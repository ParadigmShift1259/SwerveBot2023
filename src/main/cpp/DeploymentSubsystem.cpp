#include "DeploymentSubsystem.h"

#include "ConstantsDeploymentAngles.h"

#include <units/math.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

constexpr double kDefaultP = 1.0;
constexpr double kDefaultI = 0.0;
constexpr double kDefaultD = 0.0;

DeploymentSubsystem::DeploymentSubsystem()
    : m_motor(kDeploymentCANID, CANSparkMaxLowLevel::MotorType::kBrushless)
    , m_armSolenoid(PneumaticsModuleType::CTREPCM, kArmSolenoid)
    , m_backPlateSolenoid(PneumaticsModuleType::CTREPCM, kBackPlateSolenoid)
    , m_absEnc(4)
{
    wpi::log::DataLog& log = frc::DataLogManager::GetLog();
    
    m_logArmAngle = wpi::log::DoubleLogEntry(log, "/deployment/armAngle");
    m_logAbsEnc = wpi::log::DoubleLogEntry(log, "/deployment/absEnc");
    m_logOutputCurrent = wpi::log::DoubleLogEntry(log, "/deployment/outputCurrent");
    m_logMotorOutput = wpi::log::DoubleLogEntry(log, "/deployment/motorOutput");
    m_logMotorTemp = wpi::log::DoubleLogEntry(log, "/deployment/motorTemp");
    m_logFwdLimit = wpi::log::DoubleLogEntry(log, "/deployment/fwdLimit");
    m_logRevLimit = wpi::log::DoubleLogEntry(log, "/deployment/revLimit");

    m_absEnc.SetConnectedFrequencyThreshold(200);
    
    m_motor.RestoreFactoryDefaults();

    m_motor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_motor.SetInverted(false);
    m_motor.EnableVoltageCompensation(true);
    m_motor.SetSmartCurrentLimit(30);
 
    // not available in brushless m_enc.SetInverted(true);
    m_enc.SetPosition(0.0);
    m_enc.SetMeasurementPeriod(100);
    m_enc.SetInverted(true);
    printf("Alt enc CPR %u\n", m_enc.GetCountsPerRevolution());

    m_pid.SetOutputRange(-1.0, 1.0);
    m_pid.SetP(kDefaultP);
    m_pid.SetI(kDefaultI);
    m_pid.SetD(kDefaultD);
    m_pid.SetFF(0.0);
    m_pid.SetIAccum(0.0);
    m_pid.SetFeedbackDevice(m_enc);

    SmartDashboard::PutNumber("GotoAngle", 0.0);
    SmartDashboard::PutNumber("GotoTicks", 0.0);
    //SmartDashboard::PutNumber("MaxOutput", 2.0);
#define DEPLOY_PID_TUNING
#ifdef DEPLOY_PID_TUNING
    SmartDashboard::PutNumber("P", kDefaultP);
    SmartDashboard::PutNumber("I", kDefaultI);
    SmartDashboard::PutNumber("D", kDefaultD);
#endif
}

void DeploymentSubsystem::Periodic() 
{
    double pos = m_enc.GetPosition();
    double currentAngle = TicksToDegreesDouble(pos);
    double err = pos - m_setpointTicks;
    double absPos = m_absEnc.GetAbsolutePosition();
    SmartDashboard::PutNumber("Arm enc", pos);
    SmartDashboard::PutNumber("Arm angle", currentAngle);
    SmartDashboard::PutNumber("Arm error", err);
    SmartDashboard::PutNumber("Arm Abs Enc", absPos);

    m_logArmAngle.Append(currentAngle);
    m_logAbsEnc.Append(absPos);

    double outputCurrent = m_motor.GetOutputCurrent();
    double motorOutput = m_motor.GetAppliedOutput();
    double motorTemp = m_motor.GetMotorTemperature();
    m_logOutputCurrent.Append(outputCurrent);
    m_logMotorOutput.Append(motorOutput);
    m_logMotorTemp.Append(motorTemp);
    SmartDashboard::PutNumber("output current", outputCurrent);
    SmartDashboard::PutNumber("motor output", motorOutput);
    SmartDashboard::PutNumber("motor temp", motorTemp);

    // double maxOutput = SmartDashboard::GetNumber("MaxOutput", 2.0);
    // m_pid.SetOutputRange(-maxOutput, maxOutput);
}

void DeploymentSubsystem::RotateArmToAngle(degree_t angle)
{
#ifdef DEPLOY_PID_TUNING
    m_pid.SetP(SmartDashboard::GetNumber("P", kDefaultP));
    m_pid.SetI(SmartDashboard::GetNumber("I", kDefaultI));
    m_pid.SetD(SmartDashboard::GetNumber("D", kDefaultD));
#endif

    m_backPlateSolenoid.Set(false);
    m_setpointTicks = DegreesToTicks(angle);

    //auto currentPos = m_enc.GetPosition();
    //degree_t currentAngle = TicksToDegrees(currentPos);
    // printf("Rot angle %.3f currAngle %.3f delta %.3f ticks %.3f\n"
    //  , angle.to<double>()
    //  , currentAngle.to<double>()
    //  , (angle - currentAngle).to<double>()
    //  , m_setpointTicks);
    m_pid.SetReference(m_setpointTicks, CANSparkMaxLowLevel::ControlType::kPosition);
}

void DeploymentSubsystem::RotateArmToTicks(double ticks)
{
#ifdef DEPLOY_PID_TUNING
    m_pid.SetP(SmartDashboard::GetNumber("P", kDefaultP));
    m_pid.SetI(SmartDashboard::GetNumber("I", kDefaultI));
    m_pid.SetD(SmartDashboard::GetNumber("D", kDefaultD));
#endif

    m_pid.SetReference(ticks, CANSparkMax::ControlType::kPosition);
}

void DeploymentSubsystem::RotateArmRelative(double rotation)
{
    auto currentPos = m_enc.GetPosition();
    degree_t currentAngle = TicksToDegrees(currentPos);
    m_setpointTicks = DegreesToTicks(currentAngle + degree_t(rotation * kMaxOperatorDeg));
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

void DeploymentSubsystem::Stop()
{
    m_motor.Set(0.0);
}

bool DeploymentSubsystem::IsAtDegreeSetpoint(degree_t setpoint)
{
    // TODO Figure out actual angle tolerance
    degree_t currentAngle = TicksToDegrees(m_enc.GetPosition());
    //return (units::math::fabs(setpoint - currentAngle) < 0.1_deg);
    return (units::math::fabs(setpoint - currentAngle) < 0.5_deg);
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
