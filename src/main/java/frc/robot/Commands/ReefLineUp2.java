// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefLineUp2 extends Command {
    private CommandSwerveDrivetrain m_drivetrain;
    private SwerveRequest.FieldCentricFacingAngle m_drive;
    private Pose2d m_targetPose;
    private final double kP = 1.0;
    private final double kI = 0.1;
    private final double kD = 0.0;
    private double kS = 0.15;
    private final double kTolerance = 0.02;
    private final PIDController m_xPID = new PIDController(kP, kI, kD);
    private final PIDController m_yPID = new PIDController(kP, kI, kD);
    private GenericEntry m_GE_PID_kS = null;
    private GenericEntry m_GE_PID_kP = null;
    private GenericEntry m_GE_PID_kI = null;
    private GenericEntry m_GE_PID_kD = null;
    private GenericEntry m_xError = null;
    private GenericEntry m_yError = null;
    private GenericEntry m_GE_bUpdatePID = null;

    /** Creates a new ReefLineUp. */
    public ReefLineUp2(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentricFacingAngle drive) {
        m_drivetrain = drivetrain;
        m_drive = drive;

        m_xPID.setTolerance(kTolerance);
        m_yPID.setTolerance(kTolerance);

        m_GE_PID_kS = Shuffleboard.getTab("ReefLineUp2").add("ReefLineUp_kS", kS).getEntry();
        m_GE_PID_kP = Shuffleboard.getTab("ReefLineUp2").add("ReefLineUp_kP", kP).getEntry();
        m_GE_PID_kI = Shuffleboard.getTab("ReefLineUp2").add("ReefLineUp_kI", kI).getEntry();
        m_GE_PID_kD = Shuffleboard.getTab("ReefLineUp2").add("ReefLineUp_kD", kD).getEntry();
        m_GE_bUpdatePID = Shuffleboard.getTab("ReefLineUp2").add("ReefLineUp_bUpdatePID", false).getEntry();
        m_xError = Shuffleboard.getTab("ReefLineUp2").add("ReefLineUp_xError", 0.0).getEntry();
        m_yError = Shuffleboard.getTab("ReefLineUp2").add("ReefLineUp_yError", 0.0).getEntry();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_targetPose = RobotContainer.m_tracking.getTargetPose();
        m_xPID.reset();
        m_yPID.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_GE_bUpdatePID.getBoolean(false)) {
            kS = m_GE_PID_kS.getDouble(0.15);
            m_xPID.setP(m_GE_PID_kP.getDouble(kP));
            m_xPID.setI(m_GE_PID_kI.getDouble(kI));
            m_xPID.setD(m_GE_PID_kD.getDouble(kD));
            m_yPID.setP(m_GE_PID_kP.getDouble(kP));
            m_yPID.setI(m_GE_PID_kI.getDouble(kI));
            m_yPID.setD(m_GE_PID_kD.getDouble(kD));
            m_GE_bUpdatePID.setBoolean(false);
        }

        // calculate error
        double distX = m_targetPose.getTranslation().getX() - m_drivetrain.getState().Pose.getX();
        double distY = m_targetPose.getTranslation().getY() - m_drivetrain.getState().Pose.getY();
        m_xError.setDouble(distX);
        m_yError.setDouble(distY);

        // prepare static friction
        double sX = Math.copySign(kS, distX);
        double sY = Math.copySign(kS, distY);

        // apply PID control
        double velX = m_xPID.calculate(distX) + sX;
        double velY = m_yPID.calculate(distY) + sY;

        // clamp output
        velX = MathUtil.clamp(velX, -1.0, 1.0);
        velY = MathUtil.clamp(velY, -1.0, 1.0);

        // drive!
        m_drivetrain.setControl(m_drive
                .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                .withVelocityX(-velX) // Drive forward with negative Y(forward)
                .withVelocityY(-velY) // Drive left with negative X (left)
                .withTargetDirection(m_targetPose.getRotation()));

        DataLogManager.log("Vel X: " + velX + " Y: " + velY + " Error X: " + distX + " Y:" + distY);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(m_drive
                .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
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
