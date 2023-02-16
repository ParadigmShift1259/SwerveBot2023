
#include "IntakeSubsystem.h"

#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

// using namespace IntakeConstants;
using namespace std;
using namespace frc;

IntakeSubsystem::IntakeSubsystem() 
    : m_motor(kIntakeCANID)
    , m_solenoid(PneumaticsModuleType::REVPH, kIntakeSolenoid)
{
    m_motor.SetNeutralMode(NeutralMode::Coast);
}

void IntakeSubsystem::Periodic()
{
    // SmartDashboard::PutNumber("D_I_Motor", m_motor.Get());
}

void IntakeSubsystem::Set(double speed)
{
    m_motor.Set(ControlMode::PercentOutput, speed * 0);//kMotorReverseConstant);
}

void IntakeSubsystem::IntakeOut(bool out)
{
    m_solenoid.Set(out);
}
