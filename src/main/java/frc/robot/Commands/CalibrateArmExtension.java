// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmExtension;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CalibrateArmExtension extends Command {
    private double kStallVelocityThreshold = -0.5;
    private double kStallTimeThreshold = 0.25;
    private double kStallMotorPower = -0.1;
    private TalonFX m_motor = null;
    private Timer m_stallTimer = new Timer();
    private boolean m_isFinished = false;
    private TalonFXConfiguration m_originalMotorConfig = null;

    /** Creates a new CalibrateArmExtension. */
    public CalibrateArmExtension(TalonFX motor, ArmExtension armExtension) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_motor = motor;
        addRequirements(armExtension);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Retrieve the current motor configuration
        TalonFXConfiguration currentConfig = new TalonFXConfiguration();
        m_motor.getConfigurator().refresh(currentConfig);
        m_originalMotorConfig = currentConfig;

        // Disable the motor soft limit switch using Phoenix 6 API
        currentConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        m_motor.getConfigurator().apply(currentConfig);

        // Set the motor to slowly retract
        VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(-1.0)
                .withFeedForward(-kStallMotorPower);
        m_motor.setControl(request);
        m_stallTimer.reset();
        m_isFinished = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // rotor velocity ignore mechaniasm ratios
        double currentVelocity = m_motor.getRotorVelocity().getValueAsDouble();

        if (currentVelocity < kStallVelocityThreshold) {
            if (!m_stallTimer.isRunning()) {
                m_stallTimer.start();
            }
        } else {
            m_stallTimer.stop();
            m_stallTimer.reset();
        }

        if (m_stallTimer.hasElapsed(kStallTimeThreshold)) {
            // Motor has stalled for 0.5 seconds
            // Take appropriate action, e.g., stop the motor
            m_motor.set(0);
            m_isFinished = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Restore the original motor configuration
        m_motor.getConfigurator().apply(m_originalMotorConfig);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
