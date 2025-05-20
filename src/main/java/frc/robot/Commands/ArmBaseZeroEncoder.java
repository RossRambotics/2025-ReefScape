// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmBaseZeroEncoder extends Command {
    private TalonFX m_motor = null;
    private Angle m_angle;

    /** Creates a new ArmBaseZeroEncoder. */
    public ArmBaseZeroEncoder(Subsystem armBase, TalonFX motor, Angle angle) {
        m_motor = motor;
        m_angle = angle;
        this.ignoringDisable(true);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(armBase);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_motor.setPosition(m_angle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
