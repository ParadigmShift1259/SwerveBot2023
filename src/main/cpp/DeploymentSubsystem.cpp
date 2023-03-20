#include "DeploymentSubsystem.h"

#include "ConstantsDeploymentPositions.h"
#include "ConstantsDeploymentAbsolutes.h"

#include <units/math.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

constexpr double kMaxOutput = 6.0;
constexpr double kDefaultP = 4.0;
constexpr double kDefaultI = 0.0;
constexpr double kDefaultD = 0.0;

constexpr double kDefaultAbsP = 40.0;
constexpr double kDefaultAbsI = 0.0;
constexpr double kDefaultAbsD = 0.0;

DeploymentSubsystem::DeploymentSubsystem()
    : m_motor(kDeploymentCANID, CANSparkMaxLowLevel::MotorType::kBrushless)
    , m_armSolenoid(PneumaticsModuleType::CTREPCM, kArmSolenoid)
    , m_absEnc(4)
{
    wpi::log::DataLog& log = frc::DataLogManager::GetLog();
    
    // m_logArmEnc = wpi::log::DoubleLogEntry(log, "/deployment/armEnc");
    // m_logAbsEnc = wpi::log::DoubleLogEntry(log, "/deployment/absEnc");
    // m_logNeoEnc = wpi::log::DoubleLogEntry(log, "/deployment/neoEnc");
    m_logEncVal = wpi::log::DoubleArrayLogEntry(log, "/deployment/encVal");
    m_logOutputCurrent = wpi::log::DoubleLogEntry(log, "/deployment/outputCurrent");
    m_logMotorOutput = wpi::log::DoubleLogEntry(log, "/deployment/motorOutput");
    m_logMotorTemp = wpi::log::DoubleLogEntry(log, "/deployment/motorTemp");
    m_logFwdLimit = wpi::log::DoubleLogEntry(log, "/deployment/fwdLimit");
    m_logRevLimit = wpi::log::DoubleLogEntry(log, "/deployment/revLimit");

    m_absEnc.SetConnectedFrequencyThreshold(200);
    
    m_motor.RestoreFactoryDefaults();

    m_motor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_motor.SetInverted(true);
    m_motor.EnableVoltageCompensation(true);
    m_motor.SetSmartCurrentLimit(30);
 
    // not available in brushless m_enc.SetInverted(true);
    m_enc.SetMeasurementPeriod(100);
    m_enc.SetInverted(false);
    printf("Alt enc CPR %u\n", m_enc.GetCountsPerRevolution());

    m_pid.SetOutputRange(-kMaxOutput, kMaxOutput);
    m_pid.SetP(kDefaultP);
    m_pid.SetI(kDefaultI);
    m_pid.SetD(kDefaultD);
    m_pid.SetFF(0.0);
    m_pid.SetIAccum(0.0);
    m_pid.SetFeedbackDevice(m_enc);

    SmartDashboard::PutNumber("GotoAngle", 0.0);
    SmartDashboard::PutNumber("GotoTicks", 0.0);
    SmartDashboard::PutNumber("MaxOutput", 4.0);
#define DEPLOY_PID_TUNING
#ifdef DEPLOY_PID_TUNING
    SmartDashboard::PutNumber("P", kDefaultP);
    SmartDashboard::PutNumber("I", kDefaultI);
    SmartDashboard::PutNumber("D", kDefaultD);

    SmartDashboard::PutNumber("Pabs", kDefaultAbsP);
    SmartDashboard::PutNumber("Iabs", kDefaultAbsI);
    SmartDashboard::PutNumber("Dabs", kDefaultAbsD);

    SmartDashboard::PutNumber("tol", 0.005);
    SmartDashboard::PutNumber("mult", 1.0);
#endif

    m_timer.Reset();
    m_timer.Start();
}

void DeploymentSubsystem::Periodic() 
{
    auto time = m_timer.Get();
    if (time < 2.0_s)
    {
        ResetEncoderWithAbsolute();
    }
 
    double pos = m_enc.GetPosition();
    double err = pos - m_setpointTicks;
    double absPos = m_absEnc.GetAbsolutePosition();
    double neoPos = m_neoEnc.GetPosition();
    SmartDashboard::PutNumber("Arm enc", pos);
    SmartDashboard::PutNumber("Arm error", err);
    SmartDashboard::PutNumber("Arm Abs Enc", absPos);
    SmartDashboard::PutNumber("Arm Setpoint", m_setpointTicks);

    // m_logArmEnc.Append(pos);
    // m_logAbsEnc.Append(absPos);
    // m_logNeoEnc.Append(neoPos);
    m_logEncVal.Append({pos, absPos, neoPos});

    double currentPosition = m_absEnc.GetAbsolutePosition();

    double outputCurrent = m_motor.GetOutputCurrent();
    double motorOutput = m_motor.GetAppliedOutput();
    double motorTemp = m_motor.GetMotorTemperature();
    m_logOutputCurrent.Append(outputCurrent);
    m_logMotorOutput.Append(motorOutput);
    m_logMotorTemp.Append(motorTemp);
    SmartDashboard::PutNumber("output current", outputCurrent);
    SmartDashboard::PutNumber("motor output", motorOutput);
    SmartDashboard::PutNumber("motor temp", motorTemp);

    // double maxOutput = SmartDashboard::GetNumber("MaxOutput", 6.0);
    // m_pid.SetOutputRange(-maxOutput, maxOutput);
}

void DeploymentSubsystem::RotateArm(double rotations)
{
#ifdef DEPLOY_PID_TUNING
    m_pid.SetP(SmartDashboard::GetNumber("P", kDefaultP));
    m_pid.SetI(SmartDashboard::GetNumber("I", kDefaultI));
    m_pid.SetD(SmartDashboard::GetNumber("D", kDefaultD));
#endif

    static double lastRotation = -1.0;
    if (lastRotation == rotations)
    {
        return;
    }

    lastRotation = rotations;
    m_setpointTicks = rotations;
    m_pid.SetReference(m_setpointTicks, CANSparkMaxLowLevel::ControlType::kPosition);
}

void DeploymentSubsystem::RotateArmRelative(double rotation)
{
    auto currentPos = m_enc.GetPosition();
    // m_setpointTicks = std::clamp<double>(currentPos + rotation, kLowestPosition, kHighestPosition);
    m_setpointTicks = currentPos + rotation;
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

void DeploymentSubsystem::Stop()
{
    m_motor.Set(0.0);
}

bool DeploymentSubsystem::IsAtSetpoint(double setpoint)
{
    double currentPosition = m_enc.GetPosition();
    return fabs(currentPosition - setpoint) < 0.5;
}

bool DeploymentSubsystem::IsAtAbsoluteSetpoint(double setpoint)
{
    double currentAbsolutePosition = m_absEnc.GetAbsolutePosition();

    if (IsAtSetpoint(m_setpointTicks))
    {
        double extraRotations = SmartDashboard::GetNumber("mult", 1.0) * (currentAbsolutePosition - setpoint);
        m_setpointTicks = m_setpointTicks + extraRotations;
        m_pid.SetReference(m_setpointTicks, CANSparkMaxLowLevel::ControlType::kPosition);
    }

    return fabs(currentAbsolutePosition - setpoint) < 0.005;
}

void DeploymentSubsystem::ResetEncoderWithAbsolute()
{
    double absPos = m_absEnc.GetAbsolutePosition();
    
    if (absPos != 0.0)
    {
        double resetPosition = AbsoluteToRelative(absPos);
        m_enc.SetPosition(resetPosition);
    }
}

double DeploymentSubsystem::AbsoluteToRelative(double absPos)
{
    double relPos;

    relPos = 24.0 - 32.6 * absPos;

    // if (absPos < 0.6)
    // {
    //     relPos = kPlaceHighPosition / 0.48 * (0.85 - absPos);
    // }
    // else
    // {
    //     relPos = kPlaceHighPosition / 0.48 * (0.75 - absPos);
    // }
    
    return relPos;
}
