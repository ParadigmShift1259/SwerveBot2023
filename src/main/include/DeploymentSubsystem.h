
#pragma once

#include <frc/Solenoid.h>
#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>

#include "ConstantsDigitalOut.h"
#include "ConstantsCANIDs.h"

#include <units/angle.h>

using namespace rev;
using namespace units;

class DeploymentSubsystem : public frc2::SubsystemBase 
{

    public:
        DeploymentSubsystem();

        /// Will be called periodically whenever the CommandScheduler runs.
        void Periodic() override;

        /// Drives the deployment arm to a specific angle
        /// \param angle  Desired angle to rotate to; see ConstantsDeploymentAngles.h
        void RotateArmToAngle(degree_t angle);

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
        void ResetEncoder() { m_enc.SetPosition(0.0); }

    private:
        double DegreesToTicks(degree_t degrees) { return degrees.to<double>() * kTicksPerDegree + kTickOffset; }
        degree_t TicksToDegrees(double ticks) { return degree_t{(ticks - kTickOffset) * kDegreesPerTick}; }
        double TicksToDegreesDouble(double ticks) { return (ticks - kTickOffset) * kDegreesPerTick; }

        CANSparkMax m_motor;
        SparkMaxLimitSwitch m_forwardLimitSwitch = m_motor.GetForwardLimitSwitch(SparkMaxLimitSwitch::Type::kNormallyOpen);
        SparkMaxLimitSwitch m_reverseLimitSwitch = m_motor.GetReverseLimitSwitch(SparkMaxLimitSwitch::Type::kNormallyOpen);
        SparkMaxRelativeEncoder m_enc{m_motor.GetEncoder()};
        SparkMaxPIDController m_pid{m_motor.GetPIDController()};
        double m_setpointTicks = 0.0;
        frc::Solenoid m_armSolenoid;
        frc::Solenoid m_backPlateSolenoid;
        frc::Timer m_timer;

        // Empirically measured 4657 motor ticks for 140 degrees of arm rotation
        //static constexpr double kDegreesPerTick = 140.0 / 4657.0;
        // Empirically measured 5815 motor ticks for 180 degrees of arm rotation
        // static constexpr double kDegreesPerTick = 180.0 / (17.85 + 33.81);
        // Empirically measured multiple data points and made linear regression
        // Linear Regression Slope
        static constexpr double kTicksPerDegree = 1.01;
        static constexpr double kTickOffset = 0.0;
        // Initial tick position when the arm is in travel position
        static constexpr double kInitialPosition = 4.8 * kTicksPerDegree;
        static constexpr double kDegreesPerTick = 1.0 / kTicksPerDegree;

        static constexpr bool kArmSolenoidExtend = true;
        static constexpr bool kArmSolenoidRetract = false;
        static constexpr bool kBackPlateSolenoidExtend = true;
        static constexpr bool kBackPlateSolenoidRetract = false;
};