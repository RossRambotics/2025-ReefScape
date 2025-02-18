// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.util.RandomExecutionLimiter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefLineUp4 extends Command {
    private CommandSwerveDrivetrain m_drivetrain;
    private SwerveRequest.FieldCentricFacingAngle m_drive;
    private Pose2d m_targetPose;
    private Supplier<Pose2d> mSupplierTargetPose;
    private final double kP = 6.5;
    private final double kI = 0.0;
    private final double kD = 0.1;
    private double kS = 0.15;
    private final double kTolerance = 0.02;
    private final PIDController m_xPID = new PIDController(kP, kI, kD);
    private final PIDController m_yPID = new PIDController(kP, kI, kD);
    private static GenericEntry m_GE_PID_kS = null;
    private static GenericEntry m_GE_PID_kP = null;
    private static GenericEntry m_GE_PID_kI = null;
    private static GenericEntry m_GE_PID_kD = null;
    private static GenericEntry m_xError = null;
    private static GenericEntry m_yError = null;
    private static GenericEntry m_GE_bUpdatePID = null;
    private static GenericEntry m_GE_isAtGoal = null;
    private RandomExecutionLimiter m_executionLimiter = new RandomExecutionLimiter();
    private double m_velX;
    private double m_velY;

    /** Creates a new ReefLineUp. */
    public ReefLineUp4(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentricFacingAngle drive,
            Supplier<Pose2d> targetPose) {
        m_drivetrain = drivetrain;
        m_drive = drive;
        mSupplierTargetPose = targetPose;

        m_xPID.setTolerance(kTolerance);
        m_yPID.setTolerance(kTolerance);
        m_xPID.reset();
        m_yPID.reset();

        if (m_GE_PID_kS == null) {
            m_GE_PID_kS = Shuffleboard.getTab("ReefLineUp3").add("ReefLineUp_kS", kS).getEntry();
            m_GE_PID_kP = Shuffleboard.getTab("ReefLineUp3").add("ReefLineUp_kP", kP).getEntry();
            m_GE_PID_kI = Shuffleboard.getTab("ReefLineUp3").add("ReefLineUp_kI", kI).getEntry();
            m_GE_PID_kD = Shuffleboard.getTab("ReefLineUp3").add("ReefLineUp_kD", kD).getEntry();
            m_GE_bUpdatePID = Shuffleboard.getTab("ReefLineUp3").add("ReefLineUp_bUpdatePID", false).getEntry();
            m_xError = Shuffleboard.getTab("ReefLineUp3").add("ReefLineUp_xError", 0.0).getEntry();
            m_yError = Shuffleboard.getTab("ReefLineUp3").add("ReefLineUp_yError", 0.0).getEntry();
            m_GE_isAtGoal = Shuffleboard.getTab("ReefLineUp3").add("ReefLineUp_isAtGoal", false).getEntry();
        }
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_targetPose = mSupplierTargetPose.get();
        m_xPID.reset();
        m_yPID.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // calculate error
        double distX = m_targetPose.getTranslation().getX() - m_drivetrain.getState().Pose.getX();
        double distY = m_targetPose.getTranslation().getY() - m_drivetrain.getState().Pose.getY();
        m_xError.setDouble(distX);
        m_yError.setDouble(distY);

        if (Math.abs(distX) < kTolerance && Math.abs(distY) < kTolerance) {
            m_GE_isAtGoal.setBoolean(true);
            return;
        } else {
            m_GE_isAtGoal.setBoolean(false);
        }

        // prepare static friction
        double sX = -Math.copySign(kS, distX);
        double sY = -Math.copySign(kS, distY);

        m_velX = m_xPID.calculate(distX) + sX;
        m_velY = m_yPID.calculate(distY) + sY;

        // clamp output
        m_velX = MathUtil.clamp(m_velX, -1.0, 1.0);
        m_velY = MathUtil.clamp(m_velY, -1.0, 1.0);

        // drive!
        m_drivetrain.applyRequest(() -> m_drive
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                .withVelocityX(-getVelX()) // Drive forward with negative Y(forward)
                .withVelocityY(-getVelY()) // Drive left with negative X (left)
                .withTargetDirection(RobotContainer.m_targeting.getTargetAngle()));

        // Check if we should execute this cycle
        if (!m_executionLimiter.shouldExecute()) {
            return;
        }
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

        // DataLogManager.log("Vel X: " + velX + " Y: " + velY + " Error X: " + distX +
        // " Y:" + distY);
    }

    public double getVelX() {
        return m_velX;
    }

    public double getVelY() {
        return m_velY;
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
