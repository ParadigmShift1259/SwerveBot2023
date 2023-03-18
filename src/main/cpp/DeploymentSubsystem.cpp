#include "DeploymentSubsystem.h"

#include "ConstantsDeploymentAngles.h"

#include <units/math.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

constexpr double kDefaultP = 1.0;
constexpr double kDefaultI = 0.0;
constexpr double kDefaultD = 0.0;
constexpr int kMotionSCurveStrength = 1;

constexpr double kMMCruiseVel = 30.0;
constexpr double kMMAccel = 30.0;
constexpr double kMinOutput = 0.0;
constexpr double kMaxOutput = 0.3;

DeploymentSubsystem::DeploymentSubsystem()
    : m_motor(kDeploymentCANID)
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
    
    m_motor.ConfigFactoryDefault();

    m_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0);
    m_motor.SetNeutralMode(NeutralMode::Brake);
    m_motor.SetSelectedSensorPosition(0.0);

    m_motor.SetInverted(false);
    m_motor.SetSensorPhase(true);
    m_motor.EnableVoltageCompensation(true);
    m_motor.ConfigContinuousCurrentLimit(30);
    m_motor.ConfigPeakCurrentLimit(0);
    m_motor.EnableCurrentLimit(true);
 
    m_motor.Config_kP(0, kDefaultP);
    m_motor.Config_kI(0, kDefaultI);
    m_motor.Config_kD(0, kDefaultD);
    m_motor.Config_kF(0, 0.0);
    m_motor.ConfigMotionSCurveStrength(kMotionSCurveStrength);

    m_motor.ConfigMotionCruiseVelocity(DegreesToTicks(degree_t(kMMCruiseVel / 10)));
    m_motor.ConfigMotionAcceleration(DegreesToTicks(degree_t(kMMAccel / 10)));

    // m_motor.ConfigForwardSoftLimitThreshold(DegreesToTicks(kHighestAngle));
    // m_motor.ConfigReverseSoftLimitThreshold(DegreesToTicks(kLowestAngle));
    // m_motor.ConfigForwardSoftLimitEnable(true);
    // m_motor.ConfigReverseSoftLimitEnable(true);

    // Minimum motor output
    m_motor.ConfigNominalOutputForward(kMinOutput);
    m_motor.ConfigNominalOutputReverse(-kMinOutput);
    
    // Maximum motor output
    m_motor.ConfigPeakOutputForward(kMaxOutput);
    m_motor.ConfigPeakOutputReverse(-kMaxOutput);

    m_motor.SetSelectedSensorPosition(kInitialPosition);

    SmartDashboard::PutNumber("GotoAngle", 0.0);
    SmartDashboard::PutNumber("GotoTicks", 0.0);
    //SmartDashboard::PutNumber("MaxOutput", 2.0);
//#define DEPLOY_PID_TUNING
#ifdef DEPLOY_PID_TUNING
    SmartDashboard::PutNumber("P", kDefaultP);
    SmartDashboard::PutNumber("I", kDefaultI);
    SmartDashboard::PutNumber("D", kDefaultD);
#endif
}

void DeploymentSubsystem::Periodic() 
{
    double pos = m_motor.GetSelectedSensorPosition();
    double currentAngle = TicksToDegreesDouble(pos);
    double err = pos - m_setpointTicks;
    double absPos = m_absEnc.GetAbsolutePosition();
    // double target = m_motor.GetClosedLoopTarget();
    SmartDashboard::PutNumber("Arm enc", pos);
    SmartDashboard::PutNumber("Arm angle", currentAngle);
    SmartDashboard::PutNumber("Arm error", err);
    SmartDashboard::PutNumber("Arm Abs Enc", absPos);
    // SmartDashboard::PutNumber("Arm Target", target);

    m_logArmAngle.Append(currentAngle);
    m_logAbsEnc.Append(absPos);

    double outputCurrent = m_motor.GetOutputCurrent();
    double motorOutput = m_motor.GetMotorOutputPercent();
    double motorTemp = m_motor.GetTemperature();
    m_logOutputCurrent.Append(outputCurrent);
    m_logMotorOutput.Append(motorOutput);
    m_logMotorTemp.Append(motorTemp);
    SmartDashboard::PutNumber("output current", outputCurrent);
    SmartDashboard::PutNumber("motor output", motorOutput);
    SmartDashboard::PutNumber("motor temp", motorTemp);

    bool fwdLimit = IsForwardLimitSwitchClosed();
    bool revLimit = IsReverseLimitSwitchClosed();
    m_logFwdLimit.Append(fwdLimit);
    m_logRevLimit.Append(revLimit);
    SmartDashboard::PutBoolean("fwd limit", fwdLimit);
    SmartDashboard::PutBoolean("rev limit", revLimit);

    // double maxOutput = SmartDashboard::GetNumber("MaxOutput", 2.0);
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
    m_motor.Set(ControlMode::MotionMagic, m_setpointTicks);
}

void DeploymentSubsystem::RotateArmToTicks(double ticks)
{
    m_motor.Set(ControlMode::Position, ticks);
}

void DeploymentSubsystem::RotateArmRelative(double rotation)
{
    auto currentPos = m_motor.GetSelectedSensorPosition();
    degree_t currentAngle = TicksToDegrees(currentPos);
    m_setpointTicks = DegreesToTicks(currentAngle + degree_t(rotation * kMaxOperatorDeg));
    m_motor.Set(ControlMode::MotionMagic, m_setpointTicks);
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
    return false;
}

bool DeploymentSubsystem::IsReverseLimitSwitchClosed()
{
    return false;
}

void DeploymentSubsystem::Stop()
{
    m_motor.Set(ControlMode::PercentOutput, 0.0);
}

bool DeploymentSubsystem::IsAtDegreeSetpoint(degree_t setpoint)
{
    // TODO Figure out actual angle tolerance
    degree_t currentAngle = TicksToDegrees(m_motor.GetSelectedSensorPosition());
    //return (units::math::fabs(setpoint - currentAngle) < 0.1_deg);
    return (units::math::fabs(setpoint - currentAngle) < 0.5_deg);
}

degree_t DeploymentSubsystem::CurrentDegreePosition()
{
    return TicksToDegrees(m_motor.GetSelectedSensorPosition());
}

bool DeploymentSubsystem::IsOkayToRetractIntake()
{
    degree_t currentAngle = TicksToDegrees(m_motor.GetSelectedSensorPosition());
    return currentAngle > kTravelAngle;
}
