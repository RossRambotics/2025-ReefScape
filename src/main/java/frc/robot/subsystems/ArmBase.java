// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.sim.PhysicsSim;
import frc.util.RandomExecutionLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static edu.wpi.first.units.Units.*;

public class ArmBase extends SubsystemBase {
    final TalonFX m_LeftMotor = new TalonFX(30, "rio");
    final TalonFX m_RightMotor = new TalonFX(31, "rio");
    final CANcoder m_armBaseCANcoder = new CANcoder(12, "rio");

    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
    private final double m_kGoalTolerance = 2.0; // 2 degree tolerance
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
    private Angle m_goal = Degrees.of(0);
    private Timer m_timer = new Timer();
    private GenericPublisher m_GE_Timer;
    private RandomExecutionLimiter m_executionLimiter = new RandomExecutionLimiter();

    final DynamicMotionMagicVoltage m_requestCoral = new DynamicMotionMagicVoltage(0, 3, 10, 20);
    final DynamicMotionMagicVoltage m_requestAlgae = new DynamicMotionMagicVoltage(0, 3, 2, 5);

    /** Creates a new ArmPivot. */
    public ArmBase() {
        // CAN Coder configuration
        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        cc_cfg.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Degrees.of(160));
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cc_cfg.MagnetSensor.withMagnetOffset(0.044);
        m_armBaseCANcoder.getConfigurator().apply(cc_cfg);

        TalonFXConfiguration fx_cfg = new TalonFXConfiguration();

        // Configure the right motor to follow the left motor (but opposite direction)
        m_RightMotor.setControl(new Follower(m_LeftMotor.getDeviceID(), true));

        /* Configure gear ratio */
        FeedbackConfigs fdb = fx_cfg.Feedback;
        double gearRatio = 112.0;

        if (Robot.isSimulation()) {
            // needed for internal sensor
            fdb.SensorToMechanismRatio = gearRatio;
        } else {
            // // use external encoder (CANCoder)
            fdb.SensorToMechanismRatio = 1.0; // 1:1 ratio
            fdb.FeedbackRemoteSensorID = m_armBaseCANcoder.getDeviceID();
            fdb.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            fdb.RotorToSensorRatio = gearRatio;
        }

        /* Configure Motion Magic */
        MotionMagicConfigs mm = fx_cfg.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(3.0))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10.0))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(20));

        // enable brake mode
        fx_cfg.MotorOutput = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake);

        Slot0Configs slot0 = fx_cfg.Slot0;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;
        slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 65.0; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0.0; // A velocity error of 1 rps results in 0.5 V output

        m_GE_PID_kS = Shuffleboard.getTab("ArmBase").add("ArmBase_kS", slot0.kS).getEntry();
        m_GE_PID_kV = Shuffleboard.getTab("ArmBase").add("ArmBase_kV", slot0.kV).getEntry();
        m_GE_PID_kA = Shuffleboard.getTab("ArmBase").add("ArmBase_kA", slot0.kA).getEntry();
        m_GE_PID_kP = Shuffleboard.getTab("ArmBase").add("ArmBase_kP", slot0.kP).getEntry();
        m_GE_PID_kI = Shuffleboard.getTab("ArmBase").add("ArmBase_kI", slot0.kI).getEntry();
        m_GE_PID_kD = Shuffleboard.getTab("ArmBase").add("ArmBase_kD", slot0.kD).getEntry();
        m_GE_bUpdatePID = Shuffleboard.getTab("ArmBase").add("ArmBase_UpdatePID", false).getEntry();
        m_GE_Position = Shuffleboard.getTab("ArmBase").add("ArmBase_Position", 0).getEntry();
        m_GE_Velocity = Shuffleboard.getTab("ArmBase").add("ArmBase_Velocity", 0).getEntry();
        m_GE_Goal = Shuffleboard.getTab("ArmBase").add("ArmBase_Goal", 0).getEntry();
        m_GE_Timer = Shuffleboard.getTab("ArmBase").add("ArmBase_Timer", 0).getEntry();

        // setup software limits
        SoftwareLimitSwitchConfigs swLimits = new SoftwareLimitSwitchConfigs();
        swLimits.ForwardSoftLimitEnable = true;
        swLimits.ForwardSoftLimitThreshold = Degrees.of(120).in(Rotations);
        swLimits.ReverseSoftLimitEnable = true;
        swLimits.ReverseSoftLimitThreshold = Degrees.of(-30).in(Rotations);
        // fx_cfg.SoftwareLimitSwitch = swLimits;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_LeftMotor.getConfigurator().apply(fx_cfg);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
            // hi
            // hey

        }

        Shuffleboard.getTab("ArmBase").add(this.getZeroArmAngleCmd());
        Shuffleboard.getTab("ArmBase").add(this.getTestArmDownCmd());
        Shuffleboard.getTab("ArmBase").add(this.getTestArmUpCmd());
        Shuffleboard.getTab("ArmBase").add(this.getStopArmCmd());
        Shuffleboard.getTab("ArmBase").add(this.getTestAngleCommand());
    }

    private void setGoal(Angle angle) {
        if (angle.baseUnitMagnitude() != m_goal.baseUnitMagnitude()) {
            m_timer.reset();
            m_timer.start();
        }
        if (RobotContainer.m_buttonBox.isCoralMode()) {
            m_LeftMotor.setControl(m_requestCoral.withPosition(angle.in(Rotations)).withSlot(0));
        } else {
            m_LeftMotor.setControl(m_requestAlgae.withPosition(angle.in(Rotations)).withSlot(0));
        }
        m_GE_Goal.setDouble(angle.in(Degrees));
        m_goal = angle;
    }

    public Angle getCurrentAngle() {
        return m_LeftMotor.getPosition().getValue();
    }

    private Angle getError() {
        Angle goal = m_goal;
        Angle current = m_LeftMotor.getPosition().getValue();
        Angle error = goal.minus(current);
        error = Degrees.of(error.abs(Degrees));
        return error;
    }

    public Command getSetGoalCommand(Angle angle) {
        return Commands.runOnce(() -> setGoal(angle));
    }

    public Command getZeroArmAngleCmd() {

        Command c = Commands.runOnce(() -> {
            m_LeftMotor.setPosition(Degrees.of(0));
            // this.setGoal(Degrees.of(0));
        })
                .ignoringDisable(true);

        // Command c = this.runOnce(() -> m_LeftMotor.setPosition(Degrees.of(0)));
        c.setName("ArmBase.Zero");
        // c.ignoringDisable(true);

        return c;
    }

    public Command getWaitUntilErrorLessThanCmd(Angle angle) {
        return new WaitUntilCommand(() -> {
            Angle error = getError();
            if (error.baseUnitMagnitude() <= angle.baseUnitMagnitude())
                return true;
            return false;
        });
    }

    // public Command getWaitUntilGreaterThanCommand(Angle start) {
    // return new WaitUntilCommand(() -> {
    // if (getError() >= 0)
    // return true;
    // return false;
    // });
    // }

    // public Command getWaitUntilLessThanCommand(Angle angle) {
    // return new WaitUntilCommand(() -> {
    // if (getError() <= 0)
    // return true;
    // return false;
    // });
    // }

    @Override
    public void periodic() {
        if (this.getError().in(Degree) <= m_kGoalTolerance) {
            m_timer.stop();
        }
        // Check if we should execute this cycle
        if (!m_executionLimiter.shouldExecute()) {
            return;
        }

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
            this.setGoal(Degrees.of(m_GE_Goal.getDouble(0.0)));
            m_GE_bUpdatePID.setBoolean(false);
        }
        // This method will be called once per scheduler run

        m_GE_Velocity.setDouble(m_LeftMotor.getVelocity().getValueAsDouble());
        m_GE_Position.setDouble(m_LeftMotor.getPosition().getValue().in(Degree));
        m_GE_Timer.setDouble(m_timer.get());

    }

    public Command getUpCommand() {
        return this.runOnce(() -> setGoal(Degrees.of(90))).withName("ArmBase.UpCommand");
    }

    public Command getDownCommand() {
        return this.runOnce(() -> setGoal(Degrees.of(0))).withName("ArmBase.DownCommand");
    }

    public Command getTestAngleCommand() {
        return this.runOnce(() -> setGoal(Degrees.of(25))).withName("ArmBase.TestAngle");
    }

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(m_LeftMotor, 0.0001);

        Shuffleboard.getTab("ArmBase").add(this.getUpCommand());

        Shuffleboard.getTab("ArmBase").add(this.getDownCommand());

    }

    private Command getTestArmUpCmd() {
        Command c = this.runOnce(() -> {
            m_LeftMotor.setVoltage(1);
            // m_RightMotor.setVoltage(-1);
        });
        c.setName("ArmBase.TestUp");
        return c;
    }

    private Command getTestArmDownCmd() {
        Command c = this.runOnce(() -> m_LeftMotor.setVoltage(-5));
        c.setName("ArmBase.TestDown");
        return c;
    }

    private Command getStopArmCmd() {
        Command c = this.runOnce(() -> {
            m_LeftMotor.stopMotor();
            m_RightMotor.stopMotor();
        });
        c.setName("ArmBase.Stop");
        return c;
    }

    @Override
    public void simulationPeriodic() {

        RobotContainer.m_mechanisms.update(m_LeftMotor.getPosition(), m_LeftMotor.getVelocity());
    }

    public void doManualMove(double theta) {
        double kManualMaxSpeed = 1.0;
        double change = theta * kManualMaxSpeed;
        Angle newGoal = m_goal.plus(Degrees.of(change));

        if (newGoal.in(Degrees) > 120) {
            newGoal = Degrees.of(120);
        } else if (newGoal.in(Degrees) < -30) {
            newGoal = Degrees.of(-30);
        }

        setGoal(newGoal);
    }

    public boolean isStationary() {
        return Math.abs(m_LeftMotor.getVelocity().getValue().in(RotationsPerSecond)) < 0.002;
    }
}
