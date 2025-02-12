// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;

import frc.robot.RobotContainer;
import frc.robot.Commands.CalibrateArmExtension;
import frc.robot.sim.PhysicsSim;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;

public class ArmExtension extends SubsystemBase {
    final TalonFX m_LeftMotor = new TalonFX(32, "rio");
    // final TalonFX m_RightMotor = new TalonFX(33, "rio");

    private final double m_kRotationsToMeters = 0.038 * Math.PI * 1.2; // 2" diameter pulley (circumference = pi * d)
    private final double m_kGoalTolerance = 0.02; // 2 cm tolerance
    private final Timer m_timer = new Timer();

    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
    private GenericEntry m_GE_PID_kS = null;
    private GenericEntry m_GE_PID_kV = null;
    private GenericEntry m_GE_PID_kA = null;
    private GenericEntry m_GE_PID_kP = null;
    private GenericEntry m_GE_PID_kI = null;
    private GenericEntry m_GE_PID_kD = null;
    private GenericEntry m_GE_bUpdatePID = null;
    private GenericEntry m_GE_Position = null;
    private GenericEntry m_GE_Velocity = null;
    private GenericEntry m_GE_Goal = null;
    private Distance m_goal = Meters.of(0);
    private GenericEntry m_GE_Timer = null;

    /** Creates a new ArmPivot. */
    public ArmExtension() {

        // Turn on brake Mode
        m_LeftMotor.setNeutralMode(NeutralModeValue.Brake);

        // m_RightMotor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);

        // Configure the right motor to follow the left motor (but opposite direction)
        // m_RightMotor.setControl(new Follower(m_LeftMotor.getDeviceID(), false));

        /* Configure gear ratio */
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 5.0; // TODO: Calibrate motor rotations to sensor degrees

        /* Configure Motion Magic */
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(100))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(100))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(300));

        Slot0Configs slot0 = cfg.Slot0;
        slot0.GravityType = GravityTypeValue.Elevator_Static;
        slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 5.0; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0.0; // A velocity error of 1 rps results in 0.5 V output

        m_GE_PID_kS = Shuffleboard.getTab("ArmExt").add("ArmExt_kS", slot0.kS).getEntry();
        m_GE_PID_kV = Shuffleboard.getTab("ArmExt").add("ArmExt_kV", slot0.kV).getEntry();
        m_GE_PID_kA = Shuffleboard.getTab("ArmExt").add("ArmExt_kA", slot0.kA).getEntry();
        m_GE_PID_kP = Shuffleboard.getTab("ArmExt").add("ArmExt_kP", slot0.kP).getEntry();
        m_GE_PID_kI = Shuffleboard.getTab("ArmExt").add("ArmExt_kI", slot0.kI).getEntry();
        m_GE_PID_kD = Shuffleboard.getTab("ArmExt").add("ArmExt_kD", slot0.kD).getEntry();
        m_GE_bUpdatePID = Shuffleboard.getTab("ArmExt").add("ArmExt_UpdatePID", false).getEntry();
        m_GE_Position = Shuffleboard.getTab("ArmExt").add("ArmExt_Position", 0).getEntry();
        m_GE_Velocity = Shuffleboard.getTab("ArmExt").add("ArmExt_Velocity", 0).getEntry();
        m_GE_Goal = Shuffleboard.getTab("ArmExt").add("ArmExt_Goal", 0).getEntry();
        m_GE_Timer = Shuffleboard.getTab("ArmExt").add("ArmExt_Timer", 0).getEntry();

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_LeftMotor.getConfigurator().apply(cfg);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
            // hi

        }

        // this.initSysID(); // used for system identification
        Shuffleboard.getTab("ArmExt").add(this.getZeroArmExtCmd().withName("ArmExt.Zero"));
        Shuffleboard.getTab("ArmExt").add(this.GetCalibrateCmd().withName("ArmExt.Calibrate"));
        Shuffleboard.getTab("ArmExt").add(this.GetStopCmd().withName("ArmExt.Stop"));
    }

    private void setGoal(Distance distance) {
        if (distance.baseUnitMagnitude() != m_goal.baseUnitMagnitude()) {
            m_timer.reset();
            m_timer.start();
        }
        // TODO Convert meters to rotations
        double rotations = distance.in(Meters) / m_kRotationsToMeters;
        m_LeftMotor.setControl(m_mmReq.withPosition(rotations).withSlot(0));
        m_GE_Goal.setDouble(distance.in(Meters));
        m_goal = distance;
    }

    private Distance getPosition() {
        Angle current = m_LeftMotor.getPosition().getValue();
        Distance dist = Meters.of(current.in(Rotations) * m_kRotationsToMeters);
        return dist;
    }

    private Distance getError() {
        Distance dist = getPosition();
        Distance error = m_goal.minus(dist);
        error = Meters.of(error.abs(Meters));
        return error;
    }

    public Command getWaitUntilErrorLessThanCmd(Distance meters) {
        return new WaitUntilCommand(() -> {
            Distance error = getError();
            if (error.baseUnitMagnitude() <= meters.baseUnitMagnitude())
                return true;
            return false;
        });
    }

    public Command getSetGoalCommand(Distance distance) {
        return this.runOnce(() -> setGoal(distance));
    }

    public Command getExtendCommand() {
        return this.runOnce(() -> setGoal(Meters.of(2))).withName("ArmExtension.ExtendCommand");
    }

    public Command getDetractCommand() {
        return this.runOnce(() -> setGoal(Meters.of(0))).withName("ArmExtension.DetractCommand");
    }

    public Command getZeroArmExtCmd() {
        Command c = this.runOnce(() -> m_LeftMotor.setPosition(0));
        c.setName("ArmExt.Zero");
        // c.ignoringDisable(true);

        return c;
    }

    @Override
    public void periodic() {
        // Update PID?
        if (m_GE_bUpdatePID.getBoolean(false)) {
            Slot0Configs slot0 = new Slot0Configs();
            m_LeftMotor.getConfigurator().refresh(slot0);
            slot0.kS = m_GE_PID_kS.getDouble(slot0.kS);
            slot0.kV = m_GE_PID_kV.getDouble(slot0.kV);
            slot0.kA = m_GE_PID_kA.getDouble(slot0.kA);
            slot0.kP = m_GE_PID_kP.getDouble(slot0.kP);
            slot0.kI = m_GE_PID_kI.getDouble(slot0.kI);
            slot0.kD = m_GE_PID_kD.getDouble(slot0.kD);
            m_LeftMotor.getConfigurator().apply(slot0);
            m_GE_bUpdatePID.setBoolean(false);
            this.setGoal(Meters.of(m_GE_Goal.getDouble(0.0)));
        }
        // This method will be called once per scheduler run
        m_GE_Velocity.setDouble(m_LeftMotor.getVelocity().getValueAsDouble());
        m_GE_Position.setDouble(this.getPosition().in(Meter));
        m_GE_Timer.setDouble(m_timer.get());
        if (this.getError().in(Meters) <= m_kGoalTolerance) {
            m_timer.stop();
        }

    }

    public Command GetCalibrateCmd() {
        return new CalibrateArmExtension(m_LeftMotor, this);
    }

    public Command GetStopCmd() {
        return this.runOnce(() -> m_LeftMotor.set(0));
    }

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(m_LeftMotor, 0.00001);

        Shuffleboard.getTab("ArmExt").add(this.getExtendCommand());
        Shuffleboard.getTab("ArmExt").add(this.getDetractCommand());
    }

    @Override
    public void simulationPeriodic() {

        RobotContainer.m_mechanisms.updateExt(this.getPosition(), m_LeftMotor.getVelocity());
    }

    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                    null, // Use default timeout (10 s)
                          // Log state with Phoenix SignalLogger class
                    (state) -> SignalLogger.writeString("SysID-ArmExtension-state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> m_LeftMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                    null,
                    this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    // final private CommandXboxController m_joystick = new
    // CommandXboxController(4);

    private void initSysID() {
        // m_joystick.button(1).onTrue(Commands.runOnce(SignalLogger::start));
        // m_joystick.button(2).onTrue(Commands.runOnce(SignalLogger::stop));

        /*
         * Joystick Y = quasistatic forward
         * Joystick A = quasistatic reverse
         * Joystick B = dynamic forward
         * Joystick X = dyanmic reverse
         */
        // m_joystick.button(3).whileTrue(this.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // m_joystick.button(4).whileTrue(this.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // m_joystick.button(5).whileTrue(this.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // m_joystick.button(6).whileTrue(this.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
}
