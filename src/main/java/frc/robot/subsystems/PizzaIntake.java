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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PizzaIntake extends SubsystemBase {

    TalonFX m_LeftMotor = new TalonFX(35, "rio");

    /** Creates a new PizzaIntake. */
    public PizzaIntake() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput = cfg.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        // Configure stator current limits
        CurrentLimitsConfigs statorCurrentLimit = new CurrentLimitsConfigs();
        statorCurrentLimit.StatorCurrentLimitEnable = true;
        statorCurrentLimit.StatorCurrentLimit = 40; // Current limit in amps
        cfg.CurrentLimits = statorCurrentLimit;

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

    public void doStart() {
        m_LeftMotor.set(-0.65);
    }

    public void doBack() {
        m_LeftMotor.set(0.1);
    }

    public void doStop() {
        m_LeftMotor.set(0.0);
    }

    public Command getStopCommand() {
        return Commands.runOnce(() -> doStop()).withName("PizzaIntake.StopCommand");
    }

    public Command getStartCommand() {
        return Commands.runOnce(() -> doStart()).withName("PizzaIntake.StartCommand");
    }

    public Command getBackCommand() {
        return Commands.runOnce(() -> doBack()).withName("PizzaIntake.BackCommand");
    }
}
