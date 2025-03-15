// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
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
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Commands.AutoCreepIntake;
import frc.robot.Commands.ReefLineUp;
import frc.robot.Commands.ReefLineUp2;
import frc.robot.Commands.ReefLineUp3;
import frc.robot.Commands.ReefLineUp4;
import frc.robot.Commands.RunPathToTarget;
import frc.robot.Commands.WaitForArm;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmBase;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmLocker;
import frc.robot.subsystems.ButtonBox;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.ManualArmControl;
import frc.robot.subsystems.RangeFinder;
import frc.robot.subsystems.SpeedNanny;
import frc.robot.subsystems.Targeting;
import frc.robot.subsystems.VisionForOdometry;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.ArmControl.ArmController;

import frc.util.SlewRateLimiterWithSupplier;

public class RobotContainer {
    // Subsystems
    final static public boolean isTuning = true;

    final static public ArmBase m_armBase = new ArmBase();
    final static public ArmExtension m_armExtension = new ArmExtension();
    final static public Wrist m_wrist = new Wrist();
    final static public Intake m_intake = new Intake();
    final static public ArmController m_armController = new ArmController();
    final static public VisionForOdometry m_visionForOdometry = new VisionForOdometry();
    final static public ManualArmControl m_manualArmControl = new ManualArmControl();
    final static public ArmLocker m_armLocker = new ArmLocker();

    // final static public RangeFinder m_rangeFinder = new RangeFinder();

    // final static public ArmBase m_armBase = null;// new ArmBase();
    // final static public ArmExtension m_armExtension = null;// new ArmExtension();
    // final static public Wrist m_wrist = null;// new Wrist();
    // final static public Intake m_intake = null;// new Intake();
    // final static public ArmController m_armController = null;// new
    // ArmController();
    // final static public RangeFinder m_rangeFinder = new RangeFinder();

    final static public SpeedNanny m_speedNanny = new SpeedNanny();
    final static public Targeting m_targeting = new Targeting();
    final static public ButtonBox m_buttonBox = new ButtonBox();
    final static public LEDs m_LEDs = new LEDs();
    final static public Mechanisms m_mechanisms = new Mechanisms();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    public static SwerveRequest.FieldCentricFacingAngle theTargetDrive = null;
    public final SwerveRequest.FieldCentricFacingAngle targetDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public static SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public static CommandSwerveDrivetrain drivetrain;// = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        drivetrain = TunerConstants.createDrivetrain();

        NamedCommands.registerCommand("Arm.Calibrate", m_armExtension.getCalibrateAndZero().withTimeout(0.5)
                .andThen(RobotContainer.m_armController.getTransition_Start()));
        NamedCommands.registerCommand("Arm.WaitForArm", new WaitForArm());
        NamedCommands.registerCommand("Arm.Back_L4", RobotContainer.m_armController.getAutoTransition_Back_L4());
        NamedCommands.registerCommand("Arm.HumanPlayer",
                RobotContainer.m_armController.getTransition_HumanPlayerCoral());
        NamedCommands.registerCommand("Arm.BackScore_L4", RobotContainer.m_armController.getTransition_BackScore_L4());
        NamedCommands.registerCommand("Reef.1", RobotContainer.m_buttonBox.getReef1Cmd());
        NamedCommands.registerCommand("Reef.2", RobotContainer.m_buttonBox.getReef2Cmd());
        NamedCommands.registerCommand("Reef.3", RobotContainer.m_buttonBox.getReef3Cmd());
        NamedCommands.registerCommand("Reef.4", RobotContainer.m_buttonBox.getReef4Cmd());
        NamedCommands.registerCommand("Reef.5", RobotContainer.m_buttonBox.getReef5Cmd());
        NamedCommands.registerCommand("Reef.6", RobotContainer.m_buttonBox.getReef6Cmd());
        NamedCommands.registerCommand("Reef.Left", RobotContainer.m_buttonBox.getLeftReefCmd());
        NamedCommands.registerCommand("Reef.Right", RobotContainer.m_buttonBox.getRightReefCmd());
        NamedCommands.registerCommand("Intake.OutTake", RobotContainer.m_intake.getOuttakeCommand().withTimeout(0.5));
        NamedCommands.registerCommand("Intake.InTake", RobotContainer.m_intake.getIntakeCommand().withTimeout(0.5));
        NamedCommands.registerCommand("Intake.Stop", RobotContainer.m_intake.getStopCommand());
        NamedCommands.registerCommand("Reef.LineUp", new ReefLineUp3(drivetrain,
                targetDrive, RobotContainer.m_targeting::getScoreTargetPose).withTimeout(1.0));
        NamedCommands.registerCommand("Arm.Carry", RobotContainer.m_armController.getTransition_Carry());
        NamedCommands.registerCommand("Reef.PathToTarget", new RunPathToTarget(drivetrain, targetDrive));

        NamedCommands.registerCommand("Score.L4",
                new WaitCommand(0.1)
                        // .andThen(RobotContainer.m_armController.getTransition_Back_L4())
                        .andThen(new ReefLineUp4(drivetrain,
                                targetDrive,
                                RobotContainer.m_targeting::getScoreTargetPose).withTimeout(5.01))
                        // .andThen(RobotContainer.m_armController.getTransition_Back_L4())
                        // .andThen(new WaitCommand(1.0))
                        .andThen(new PrintCommand("Before ScoreL4"))
                        .andThen(RobotContainer.m_armController.getTransition_BackScore_L4())
                        .andThen(new PrintCommand("After Score L4 beefore WaitForArm"))
                        .andThen(new WaitCommand(0.5))
                        .andThen(new WaitForArm())
                        .andThen(new PrintCommand("Before Outtake"))
                        .andThen(RobotContainer.m_intake.getOuttakeCommand())
                        .andThen(new PrintCommand("After Outtake"))
                        .andThen(RobotContainer.m_armController.getTransition_Carry())
                        .andThen(new WaitCommand(0.5))

        );

        new EventTrigger("Event.CoralStation").onTrue(
                RobotContainer.m_armController.getTransition_HumanPlayerCoral());

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        theTargetDrive = targetDrive;
        configureBindings();
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = MathUtil.applyDeadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }

    private final SlewRateLimiterWithSupplier m_slewDriverX = new SlewRateLimiterWithSupplier(
            m_speedNanny::getAccelerationLimit);

    private double getDriverXVelocity() {
        double driverLeftX = modifyAxis(joystick.getLeftX());
        double slew = m_slewDriverX.calculate(driverLeftX * m_speedNanny.getSpeedLimit());

        return slew;
    }

    private final SlewRateLimiterWithSupplier m_slewDriverY = new SlewRateLimiterWithSupplier(
            m_speedNanny::getAccelerationLimit);
    private double m_kNudgeRate = 0.25;

    private double getDriverYVelocity() {
        double driverLeftY = modifyAxis(joystick.getLeftY());
        double slew = m_slewDriverY.calculate(driverLeftY * m_speedNanny.getSpeedLimit());

        return slew;
    }

    private double getDriverRotationalRate() {
        double driverRightX = modifyAxis(joystick.getRightX());
        return driverRightX * m_speedNanny.getAngularRateLimit();
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
                        .withRotationalRate(-getDriverRotationalRate()) // Drive
                                                                        // counterclockwise
                                                                        // with
                // negative X (left)
                ));

        // snaps the robot to target angle
        targetDrive.HeadingController = new PhoenixPIDController(20.0, 0.0, 0.0);
        targetDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        targetDrive.HeadingController.setIZone(5.0);

        joystick.leftBumper().whileTrue(
                drivetrain.applyRequest(() -> targetDrive
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                        .withVelocityX(-getDriverYVelocity()) // Drive forward with negative Y(forward)
                        .withVelocityY(-getDriverXVelocity()) // Drive left with negative X (left)
                        .withTargetDirection(m_targeting.getTargetAngle())));

        double nudge = 0.5;
        joystick.pov(0)
                .whileTrue(drivetrain.applyRequest(() -> drive
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate * m_kNudgeRate)
                        .withVelocityX(
                                nudge)
                        .withVelocityY(0)));
        joystick.pov(180)
                .whileTrue(drivetrain.applyRequest(() -> drive
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate * m_kNudgeRate)
                        .withVelocityX(-nudge)
                        .withVelocityY(0)));
        joystick.pov(90)
                .whileTrue(drivetrain.applyRequest(() -> drive
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate * m_kNudgeRate)
                        .withVelocityX(0)
                        .withVelocityY(-nudge)));
        joystick.pov(270)
                .whileTrue(drivetrain.applyRequest(() -> drive
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate * m_kNudgeRate)
                        .withVelocityX(0)
                        .withVelocityY(nudge)));
        joystick.pov(45)
                .whileTrue(drivetrain.applyRequest(() -> drive
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate * m_kNudgeRate)
                        .withVelocityX(
                                nudge)
                        .withVelocityY(-nudge)));
        joystick.pov(135)
                .whileTrue(drivetrain.applyRequest(() -> drive
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate * m_kNudgeRate)
                        .withVelocityX(-nudge)
                        .withVelocityY(-nudge)));
        joystick.pov(225)
                .whileTrue(drivetrain.applyRequest(() -> drive
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate * m_kNudgeRate)
                        .withVelocityX(-nudge)
                        .withVelocityY(nudge)));
        joystick.pov(315)
                .whileTrue(drivetrain.applyRequest(() -> drive
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate * m_kNudgeRate)
                        .withVelocityX(
                                nudge)
                        .withVelocityY(nudge)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on back press
        joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.start().whileTrue(new AutoCreepIntake());

        drivetrain.registerTelemetry(logger::telemeterize);

        // String pathName = "Tag.18.Left";
        // Command c = null;
        // try {
        // PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        // path.preventFlipping = true;
        // c = AutoBuilder.followPath(path);
        // } catch (Exception e) {
        // DriverStation.reportError("Can't Load Path: " + pathName, false);
        // }

        // if (c != null)
        // joystick.a().whileTrue(c);

        // joystick.x().whileTrue(new ReefLineUp3(drivetrain, targetDrive,
        // m_targeting::getTargetPose));

        joystick.leftTrigger().whileTrue(new RunPathToTarget(drivetrain, targetDrive));

        joystick.leftBumper().whileTrue(
                drivetrain.applyRequest(() -> targetDrive
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                        .withVelocityX(-getDriverYVelocity()) // Drive forward with negative Y(forward)
                        .withVelocityY(-getDriverXVelocity()) // Drive left with negative X (left)
                        .withTargetDirection(m_targeting.getTargetAngle())));

        joystick.rightBumper().onTrue(m_armController.getTransition_Carry());
        joystick.rightTrigger().onTrue(m_armController.getNextNodeCmd());

        joystick.y().onTrue(m_armController.getTransition_HPCarry()
                .andThen(m_targeting.getTargetHumanPlayerStation()));

        // joystick.x().onTrue(m_armController.getTransition_GroundCoral());
        joystick.x().onTrue(m_targeting.getTargetLastReefIDCmd()
                .andThen(RobotContainer.m_armController.getTransition_Carry()));

        joystick.a().onTrue(m_intake.getIntakeCommand());
        joystick.a().onFalse(m_intake.getStopCommand());
        joystick.b().onTrue(m_intake.getOuttakeCommand());
        joystick.b().onFalse(m_intake.getStopCommand());

    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
