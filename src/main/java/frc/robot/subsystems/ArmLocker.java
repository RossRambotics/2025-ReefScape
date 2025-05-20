// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ArmLocker extends SubsystemBase {
    // private final Servo m_servo;
    TalonFX m_LeftMotor = new TalonFX(37, "rio");

    /** Creates a new ArmLocker. */
    public ArmLocker() {
        // Initialize the servo on PWM port 0
        // m_servo = new Servo(0);
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput = cfg.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        // Configure stator current limits
        CurrentLimitsConfigs statorCurrentLimit = new CurrentLimitsConfigs();
        statorCurrentLimit.StatorCurrentLimitEnable = true;
        statorCurrentLimit.StatorCurrentLimit = 15; // Current limit in amps
        cfg.CurrentLimits = statorCurrentLimit;

        /* Configure gear ratio */
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 1.0; // TODO: Calibrate motor rotations to sensor degrees

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
        slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                         // volts / rotation per second
        slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0; // No output for error derivative

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_LeftMotor.getConfigurator().apply(cfg);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
            // hi

        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Locks the hook by setting the servo to the lock position.
     */
    public void lock() {
        // m_servo.setAngle(23); // Adjust the angle as needed for the lock position
        // RobotContainer.m_armBase.climbMode();
        m_LeftMotor.set(0.1);
    }

    /**
     * Unlocks the hook by setting the servo to the unlock position.
     */
    public void unlock() {
        // m_servo.setAngle(80); // Adjust the angle as needed for the unlock position
        // RobotContainer.m_armBase.normalMode();
        m_LeftMotor.set(-0.1);
    }

    public void stop() {
        m_LeftMotor.stopMotor();
    }

    Command getArmLockCmd() {
        Command c = this.startEnd(
                () -> this.lock(),
                () -> this.stop())
                .withTimeout(5.0);
        return c;
    }

    Command getArmUnlockCmd() {
        Command c = this.startEnd(
                () -> this.unlock(),
                () -> this.stop())
                .withTimeout(1.0);
        return c;
    }
}
