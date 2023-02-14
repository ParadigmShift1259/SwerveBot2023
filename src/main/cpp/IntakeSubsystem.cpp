
#include "IntakeSubsystem.h"

#include "Constants.h"
#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

// using namespace IntakeConstants;
using namespace std;
using namespace frc;

IntakeSubsystem::IntakeSubsystem() 
    : m_motor(0)//kMotorPort)
    , m_solenoid(PneumaticsModuleType::REVPH, 0)//kSolenoidPort)
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
