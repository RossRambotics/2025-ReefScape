// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunPathToTarget extends Command {
    private Command m_pathFindingCommand = null;
    private CommandSwerveDrivetrain m_drivetrain = null;
    private SwerveRequest.FieldCentricFacingAngle m_drive = null;

    /** Creates a new RunPathToTarget. */
    public RunPathToTarget(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentricFacingAngle drive) {
        m_drivetrain = drivetrain;
        m_drive = drive;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        Pose2d targetPose = RobotContainer.m_targeting.getTargetPose();
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                2.0, 2.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command c = AutoBuilder.pathfindToPose(
                targetPose,
                constraints // Goal end velocity in meters/sec
        );

        m_pathFindingCommand = c
                .andThen(new ReefLineUp3(m_drivetrain, m_drive, RobotContainer.m_targeting::getScoreTargetPose));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        m_pathFindingCommand.schedule();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_pathFindingCommand.cancel();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
