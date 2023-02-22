
#pragma once

#include <frc/Solenoid.h>
#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include "ConstantsDigitalOut.h"
#include "ConstantsCANIDs.h"

#include <units/angle.h>

using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
using namespace units;

class DeploymentSubsystem : public frc2::SubsystemBase 
{

    public:
        DeploymentSubsystem();

        /// Will be called periodically whenever the CommandScheduler runs.
        void Periodic() override;

        /// Drives the deployment mechanism at a given speed CCW into the robot frame
        /// \param speed         Desired motor speed to run, ranging from [0, 1]
        void RotateIntoFrame(double speed);

        /// Drives the deployment mechanism at a given speed CW out of the robot frame
        /// \param speed         Desired motor speed to run, ranging from [0, 1]
        void RotateOutOfFrame(double speed);


        /// Drives the deployment arm to a specific angle
        /// \param angle  Desired angle to rotate to [0, 140]
        void RotateArmToAngle(degree_t angle);
        void RotateArmToAngleRel(degree_t angle);

        /// Extends the deployment arm
        void ExtendArm();

        /// Retracts the deployment arm
        void RetractArm();

        /// Extends the back plate
        void ExtendBackPlate();

        /// Retracts the back plate
        void RetractBackPlate();

        /// Returns if the forward limit switch is currently closed
        /// \returns true if the limit switch is closed, false if the limit switch is open
        bool IsForwardLimitSwitchClosed();

        /// Returns if the reverse limit switch is currently closed
        /// \returns true if the limit switch is closed, false if the limit switch is open
        bool IsReverseLimitSwitchClosed();

        /// Stops running the deployment arm motor
        void Stop();

        /// Checks to see if the deployment arm is at a specified degree setpoint
        /// \param setpoint intended setpoint to check current position against
        /// \returns if current position is within an accepted margin error of the setpoint 
        bool IsAtDegreeSetpoint(degree_t setpoint);

        /// Retrieves current position of deployment arm  
        /// \returns position of deployment arm, in degrees 
        degree_t CurrentDegreePosition();

        /// Checks for interefence of the claw
        /// \returns true if ready to retracts
        bool IsOkayToRetractIntake();

        /// Zero out the arm encoder count
        void ResetEncoder() { m_motor.SetSelectedSensorPosition(0.0); }

    private:
        TalonSRX m_motor;
        frc::Solenoid m_armSolenoid;
        frc::Solenoid m_backPlateSolenoid;
        frc::Timer m_timer;

        // Empirically measured 4657 motor ticks for 140 degrees of arm rotation
        //static constexpr double kDegreesPerTick = 140.0 / 4657.0;
        // Empirically measured 5815 motor ticks for 180 degrees of arm rotation
        static constexpr double kDegreesPerTick = 180.0 / 5815.0;
        static constexpr double kTicksPerDegree = 1.0 / kDegreesPerTick;
};