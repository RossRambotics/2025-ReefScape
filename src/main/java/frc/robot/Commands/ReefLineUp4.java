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
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
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
    private final double kStopTime = 0.2;
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
    private boolean m_isFinished = false;
    private Timer m_stopTimer = new Timer();

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
            m_GE_PID_kS = Shuffleboard.getTab("ReefLineUp4").add("ReefLineUp_kS", kS).getEntry();
            m_GE_PID_kP = Shuffleboard.getTab("ReefLineUp4").add("ReefLineUp_kP", kP).getEntry();
            m_GE_PID_kI = Shuffleboard.getTab("ReefLineUp4").add("ReefLineUp_kI", kI).getEntry();
            m_GE_PID_kD = Shuffleboard.getTab("ReefLineUp4").add("ReefLineUp_kD", kD).getEntry();
            m_GE_bUpdatePID = Shuffleboard.getTab("ReefLineUp4").add("ReefLineUp_bUpdatePID", false).getEntry();
            m_xError = Shuffleboard.getTab("ReefLineUp4").add("ReefLineUp_xError", 0.0).getEntry();
            m_yError = Shuffleboard.getTab("ReefLineUp4").add("ReefLineUp_yError", 0.0).getEntry();
            m_GE_isAtGoal = Shuffleboard.getTab("ReefLineUp4").add("ReefLineUp_isAtGoal", false).getEntry();
        }
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_targetPose = mSupplierTargetPose.get();
        m_drivetrain = RobotContainer.drivetrain;
        m_drive = RobotContainer.theTargetDrive;
        m_xPID.reset();
        m_yPID.reset();
        m_isFinished = false;
        m_stopTimer.stop();
        m_stopTimer.reset();
        m_GE_isAtGoal.setBoolean(false);

        if (!RobotContainer.isTuning) {
            return;
        }

        kS = m_GE_PID_kS.getDouble(kS);
        m_xPID.setP(m_GE_PID_kP.getDouble(kP));
        m_xPID.setI(m_GE_PID_kI.getDouble(kI));
        m_xPID.setD(m_GE_PID_kD.getDouble(kD));
        m_yPID.setP(m_GE_PID_kP.getDouble(kP));
        m_yPID.setI(m_GE_PID_kI.getDouble(kI));
        m_yPID.setD(m_GE_PID_kD.getDouble(kD));
        m_GE_bUpdatePID.setBoolean(false);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // assume the worst ... ;)
        RobotContainer.m_LEDs.setIsAligned(false);

        // calculate error
        double distX = m_targetPose.getTranslation().getX() - m_drivetrain.getState().Pose.getX();
        double distY = m_targetPose.getTranslation().getY() - m_drivetrain.getState().Pose.getY();
        m_xError.setDouble(distX);
        m_yError.setDouble(distY);

        if (Math.abs(distX) < kTolerance && Math.abs(distY) < kTolerance) {
            this.stop();
            RobotContainer.m_LEDs.setIsAligned(true);

            // If timer isn't running start it
            if (!m_stopTimer.isRunning()) {
                m_stopTimer.reset();
                m_stopTimer.start();
            }

            // If we have been stopped long enough so end command
            if (m_stopTimer.hasElapsed(kStopTime)) {
                m_GE_isAtGoal.setBoolean(true);
                m_isFinished = true;
            }

            // we are lined up, but waiting for the timer so skip the rest
            return;
        } else {
            // keep going and stop the timer
            m_GE_isAtGoal.setBoolean(false);
            m_isFinished = false;
            m_stopTimer.stop();
        }

        // apply PID control
        double velX = m_xPID.calculate(distX);
        double velY = m_yPID.calculate(distY);

        // prepare static friction
        double sX = Math.copySign(kS, velX);
        double sY = Math.copySign(kS, velY);

        // add static friction
        velX += sX;
        velY += sY;

        // clamp output
        velX = -MathUtil.clamp(velX, -1.0, 1.0);
        velY = -MathUtil.clamp(velY, -1.0, 1.0);

        // drive!
        m_drivetrain.setControl(m_drive
                .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                .withVelocityX(velX) // Drive forward with negative Y(forward)
                .withVelocityY(velY) // Drive left with negative X (left)
                .withTargetDirection(m_targetPose.getRotation()));

        if (!RobotContainer.isTuning) {
            return;
        }

        // Check if we should execute this cycle
        if (!m_executionLimiter.shouldExecute()) {
            return;
        }
        if (m_GE_bUpdatePID.getBoolean(false)) {
            kS = m_GE_PID_kS.getDouble(kS);
            m_xPID.setP(m_GE_PID_kP.getDouble(kP));
            m_xPID.setI(m_GE_PID_kI.getDouble(kI));
            m_xPID.setD(m_GE_PID_kD.getDouble(kD));
            m_yPID.setP(m_GE_PID_kP.getDouble(kP));
            m_yPID.setI(m_GE_PID_kI.getDouble(kI));
            m_yPID.setD(m_GE_PID_kD.getDouble(kD));
            m_GE_bUpdatePID.setBoolean(false);
        }

        DataLogManager.log("kS: " + kS + " sX: " + sX + " Vel X: " + velX + " Y: " + velY + " Error X: " + distX +
                " Y:" + distY);
    }

    private void stop() {
        m_drivetrain.setControl(m_drive
                .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                .withVelocityX(0) // Drive forward with negative Y(forward)
                .withVelocityY(0) // Drive left with negative X (left)
                .withTargetDirection(m_targetPose.getRotation()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.stop();
        RobotContainer.m_LEDs.setIsAligned(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
