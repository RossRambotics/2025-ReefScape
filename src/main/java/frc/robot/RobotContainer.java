// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Commands.ReefLineUp;
import frc.robot.Commands.ReefLineUp2;
import frc.robot.Commands.RunPathToTarget;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmBase;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ButtonBox;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RangeFinder;
import frc.robot.subsystems.SpeedNanny;
import frc.robot.subsystems.Targeting;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.ArmControl.ArmController;

public class RobotContainer {
    // Subsystems
    final static public ArmBase m_armBase = null;// new ArmBase();
    final static public ArmExtension m_armExtension = null;// new ArmExtension();
    final static public Wrist m_wrist = null;// new Wrist();
    final static public Mechanisms m_mechanisms = new Mechanisms();
    final static public Intake m_intake = null;// new Intake();
    // final static public RangeFinder m_rangeFinder = new RangeFinder();
    final static public ArmController m_armController = null;// new ArmController();
    final static public SpeedNanny m_speedNanny = new SpeedNanny();
    final static public Targeting m_tracking = new Targeting();
    final static public ButtonBox m_buttonBox = new ButtonBox();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle targetDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private static double modifyAxis(double value) {

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }

    private final SlewRateLimiter m_slewDriverX = new SlewRateLimiter(15);

    private double getDriverXVelocity() {
        double driverLeftX = modifyAxis(joystick.getLeftX());
        double slew = m_slewDriverX.calculate(driverLeftX * m_speedNanny.getSpeedLimit());

        return slew;
    }

    private final SlewRateLimiter m_slewDriverY = new SlewRateLimiter(15);
    private double m_kNudgeRate = 0.25;

    private double getDriverYVelocity() {
        double driverLeftY = modifyAxis(joystick.getLeftY());
        double slew = m_slewDriverY.calculate(driverLeftY * m_speedNanny.getSpeedLimit());

        return slew;
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                        .withVelocityX(-getDriverYVelocity()) // Drive forward with negative Y(forward)
                        .withVelocityY(-getDriverXVelocity()) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        // snaps the robot to target angle
        targetDrive.HeadingController = new PhoenixPIDController(5.0, 0.0, 0.20);
        joystick.leftBumper().whileTrue(
                drivetrain.applyRequest(() -> targetDrive
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                        .withVelocityX(-getDriverYVelocity()) // Drive forward with negative Y(forward)
                        .withVelocityY(-getDriverXVelocity()) // Drive left with negative X (left)
                        .withTargetDirection(m_tracking.getTargetAngle())));

        joystick.pov(0)
                .whileTrue(drivetrain.applyRequest(() -> drive
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate * m_kNudgeRate)
                        .withVelocityX(0.5)
                        .withVelocityY(0)));
        joystick.pov(180)
                .whileTrue(drivetrain.applyRequest(() -> drive
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate * m_kNudgeRate)
                        .withVelocityX(-0.5)
                        .withVelocityY(0)));
        joystick.pov(90)
                .whileTrue(drivetrain.applyRequest(() -> drive
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate * m_kNudgeRate)
                        .withVelocityX(0)
                        .withVelocityY(-0.5)));
        joystick.pov(270)
                .whileTrue(drivetrain.applyRequest(() -> drive
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate * m_kNudgeRate)
                        .withVelocityX(0)
                        .withVelocityY(0.5)));
        joystick.pov(45)
                .whileTrue(drivetrain.applyRequest(() -> drive
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate * m_kNudgeRate)
                        .withVelocityX(0.5)
                        .withVelocityY(-.5)));
        joystick.pov(135)
                .whileTrue(drivetrain.applyRequest(() -> drive
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate * m_kNudgeRate)
                        .withVelocityX(-0.5)
                        .withVelocityY(-.5)));
        joystick.pov(225)
                .whileTrue(drivetrain.applyRequest(() -> drive
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate * m_kNudgeRate)
                        .withVelocityX(-0.5)
                        .withVelocityY(.5)));
        joystick.pov(315)
                .whileTrue(drivetrain.applyRequest(() -> drive
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate * m_kNudgeRate)
                        .withVelocityX(0.5)
                        .withVelocityY(.5)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on back press
        joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        String pathName = "Tag.18.Left";
        Command c = null;
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            path.preventFlipping = true;
            c = AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("Can't Load Path: " + pathName, false);
        }

        if (c != null)
            joystick.a().whileTrue(c);

        joystick.b().whileTrue(new RunPathToTarget());
        joystick.x().whileTrue(new ReefLineUp2(drivetrain, targetDrive));

    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
