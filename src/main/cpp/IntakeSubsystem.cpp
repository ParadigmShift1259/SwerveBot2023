
#include "IntakeSubsystem.h"

//#include <frc/SmartDashBoard/SmartDashboard.h>
//#include <frc/shuffleboard/Shuffleboard.h>

using namespace frc;

IntakeSubsystem::IntakeSubsystem() 
    : m_motor(kIntakeCANID)
    , m_solenoid(PneumaticsModuleType::CTREPCM, kIntakeSolenoid)
{
    m_motor.SetNeutralMode(NeutralMode::Coast);
    m_motor.SetInverted(true);
}

void IntakeSubsystem::Periodic()
{
    // SmartDashboard::PutNumber("D_I_Motor", m_motor.Get());
}

void IntakeSubsystem::Set(double speed)
{
    m_motor.Set(ControlMode::PercentOutput, speed);//kMotorReverseConstant);
}

void IntakeSubsystem::ExtendIntake()
{
    m_solenoid.Set(kIntakeExtend);
}

void IntakeSubsystem::RetractIntake()
{
    m_solenoid.Set(kIntakeRetract);
}
