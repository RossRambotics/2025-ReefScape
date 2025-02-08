// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefLineUp extends Command {
    private CommandSwerveDrivetrain m_drivetrain;
    private SwerveRequest.FieldCentricFacingAngle m_drive;
    private Pose2d m_targetPose;

    /** Creates a new ReefLineUp. */
    public ReefLineUp(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentricFacingAngle drive) {
        m_drivetrain = drivetrain;
        m_drive = drive;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_targetPose = RobotContainer.m_tracking.getTargetPose();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double kP = 0.5;
        double kS = 0.15;

        // calculate error
        double distX = m_targetPose.getTranslation().getX() - m_drivetrain.getState().Pose.getX();
        double distY = m_targetPose.getTranslation().getY() - m_drivetrain.getState().Pose.getY();

        // apply deadband
        distX = MathUtil.applyDeadband(distX, 0.001, 5.0);
        distY = MathUtil.applyDeadband(distY, 0.001, 5.0);

        // apply static friction
        double sX = Math.copySign(kS, distX);
        double sY = Math.copySign(kS, distY);

        // apply proportional gain
        double velX = (distX * kP) + sX;
        double velY = (distY * kP) + sY;

        // clamp output
        velX = MathUtil.clamp(velX, -1.0, 1.0);

        // drive!
        m_drivetrain.setControl(m_drive
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                .withVelocityX(velX) // Drive forward with negative Y(forward)
                .withVelocityY(velY) // Drive left with negative X (left)
                .withTargetDirection(m_targetPose.getRotation()));

        DataLogManager.log("Vel X: " + velX + " Y: " + velY + "Error X: " + distX + " Y:" + distY);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(m_drive
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                .withVelocityX(0) // Drive forward with negative Y(forward)
                .withVelocityY(0) // Drive left with negative X (left)
                .withTargetDirection(m_targetPose.getRotation()));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
