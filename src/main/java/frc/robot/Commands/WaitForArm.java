// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmBase;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WaitForArm extends Command {
    private final ArmBase m_armBase = RobotContainer.m_armBase;
    private final Wrist m_wrist = RobotContainer.m_wrist;
    private final ArmExtension m_armExtension = RobotContainer.m_armExtension;
    private final Timer m_timer = new Timer();

    /** Creates a new WaitForArm. */
    public WaitForArm() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_armBase, m_wrist, m_armExtension);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_armBase.isStationary() && m_wrist.isStationary() && m_armExtension.isStationary()) {
            // If all subsystems are stationary, check the timer
            if (m_timer.hasElapsed(0.10)) {
                // If they have been stationary for X seconds, the command can finish
                return;
            }
        } else {
            // If any subsystem is not stationary, reset the timer
            m_timer.reset();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(0.25);
    }
}
