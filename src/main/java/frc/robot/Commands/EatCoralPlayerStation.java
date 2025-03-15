// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EatCoralPlayerStation extends Command {
    Command m_startIntakeCmd = null;
    Command m_stopIntakeCmd = null;
    private Intake m_intake = null;

    /** Creates a new EatCoralPlayerStation. */
    public EatCoralPlayerStation(Intake intake) {
        m_intake = intake;
        m_stopIntakeCmd = m_intake.getStopCommand();
        m_startIntakeCmd = m_intake.getIntakeCommand();
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_startIntakeCmd.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_stopIntakeCmd.schedule();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (RobotContainer.m_intake.isCoralSensorDetected()) {
            return true;
        }
        return false;
    }
}
