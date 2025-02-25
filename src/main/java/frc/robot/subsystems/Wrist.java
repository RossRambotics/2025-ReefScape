// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.sim.PhysicsSim;
import frc.util.RandomExecutionLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static edu.wpi.first.units.Units.*;

public class Wrist extends SubsystemBase {
    final TalonFX m_LeftMotor = new TalonFX(34, "rio");
    final CANcoder m_wristCANcoder = new CANcoder(8, "rio");

    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
    private final double m_kGoalTolerance = 2.0; // 2 degree tolerance

    private GenericEntry m_GE_PID_kS = null;
    private GenericEntry m_GE_PID_kV = null;
    private GenericEntry m_GE_PID_kA = null;
    private GenericEntry m_GE_PID_kP = null;
    private GenericEntry m_GE_PID_kI = null;
    private GenericEntry m_GE_PID_kG = null;
    private GenericEntry m_GE_PID_kD = null;
    private GenericEntry m_GE_bUpdatePID = null;
    private GenericEntry m_GE_Position = null;
    private GenericEntry m_GE_Velocity = null;
    private GenericEntry m_GE_Goal = null;
    private GenericEntry m_GE_Timer = null;
    private Timer m_timer = new Timer();
    private RandomExecutionLimiter m_executionLimiter = new RandomExecutionLimiter();

    private Angle m_goal = Degrees.of(0);

    /** Creates a new Wrist. */
    public Wrist() {
        // CAN Coder configuration
        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        cc_cfg.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Degrees.of(160));
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cc_cfg.MagnetSensor.withMagnetOffset(Degrees.of(156 - 180 - 178));
        m_wristCANcoder.getConfigurator().apply(cc_cfg);

        // TalonFX configuration
        TalonFXConfiguration fx_cfg = new TalonFXConfiguration();

        /* Configure gear ratio */
        FeedbackConfigs fdb = fx_cfg.Feedback;
        double gearRatio = 25.0;

        // use internal encoder
        if (Robot.isSimulation()) {
            fdb.SensorToMechanismRatio = gearRatio;
        } else {
            // use external encoder (CANCoder)
            fdb.SensorToMechanismRatio = 1.0; // 1:1 ratio
            fdb.FeedbackRemoteSensorID = m_wristCANcoder.getDeviceID();
            fdb.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            fdb.RotorToSensorRatio = gearRatio;
        }

        fx_cfg.MotorOutput = fx_cfg.MotorOutput.withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        /* Configure Motion Magic */
        MotionMagicConfigs mm = fx_cfg.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(1))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(5))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(10));

        Slot0Configs slot0 = fx_cfg.Slot0;
        slot0.GravityType = GravityTypeValue.Elevator_Static;
        slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 50; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kG = 1.0;
        slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output

        m_GE_PID_kS = Shuffleboard.getTab("Wrist").add("Wrist_kS", slot0.kS).getEntry();
        m_GE_PID_kV = Shuffleboard.getTab("Wrist").add("Wrist_kV", slot0.kV).getEntry();
        m_GE_PID_kA = Shuffleboard.getTab("Wrist").add("Wrist_kA", slot0.kA).getEntry();
        m_GE_PID_kP = Shuffleboard.getTab("Wrist").add("Wrist_kP", slot0.kP).getEntry();
        m_GE_PID_kI = Shuffleboard.getTab("Wrist").add("Wrist_kI", slot0.kI).getEntry();
        m_GE_PID_kG = Shuffleboard.getTab("Wrist").add("Wrist_kG", slot0.kI).getEntry();
        m_GE_PID_kD = Shuffleboard.getTab("Wrist").add("Wrist_kD", slot0.kD).getEntry();
        m_GE_bUpdatePID = Shuffleboard.getTab("Wrist").add("Wrist_UpdatePID", false).getEntry();
        m_GE_Position = Shuffleboard.getTab("Wrist").add("Wrist_Position", 0).getEntry();
        m_GE_Velocity = Shuffleboard.getTab("Wrist").add("Wrist_Velocity", 0).getEntry();
        m_GE_Goal = Shuffleboard.getTab("Wrist").add("Wrist_Goal", 0).getEntry();
        m_GE_Timer = Shuffleboard.getTab("Wrist").add("Wrist_Timer", 0).getEntry();

        // setup software limits
        SoftwareLimitSwitchConfigs swLimits = new SoftwareLimitSwitchConfigs();
        swLimits.ForwardSoftLimitEnable = true;
        swLimits.ForwardSoftLimitThreshold = Degrees.of(125).in(Rotations);
        swLimits.ReverseSoftLimitEnable = true;
        swLimits.ReverseSoftLimitThreshold = Degrees.of(-175).in(Rotations);
        fx_cfg.SoftwareLimitSwitch = swLimits;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_LeftMotor.getConfigurator().apply(fx_cfg);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
            // hi

        }
        Shuffleboard.getTab("Wrist").add(this.getZeroWristAngleCmd());
    }

    private void setGoal(Angle angle) {
        if (angle.baseUnitMagnitude() != m_goal.baseUnitMagnitude()) {
            m_timer.reset();
            m_timer.start();
        }
        m_LeftMotor.setControl(m_mmReq.withPosition(angle.in(Rotations)).withSlot(0));
        m_GE_Goal.setDouble(angle.in(Degrees));
        m_goal = angle;

    }

    private Angle getError() {
        Angle goal = m_goal;
        Angle current = m_LeftMotor.getPosition().getValue();
        Angle error = goal.minus(current);
        error = Degrees.of(error.abs(Degrees));
        return error;
    }

    public Command getWaitUntilErrorLessThanCmd(Angle angle) {
        return new WaitUntilCommand(() -> {
            Angle error = getError();
            if (error.baseUnitMagnitude() <= angle.baseUnitMagnitude())
                return true;
            return false;
        });
    }

    public Command getSetGoalCommand(Angle angle) {
        return Commands.runOnce(() -> setGoal(angle));
    }

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
            slot0.kG = m_GE_PID_kG.getDouble(slot0.kG);
            slot0.kD = m_GE_PID_kD.getDouble(slot0.kD);
            m_LeftMotor.getConfigurator().apply(slot0);
            m_GE_bUpdatePID.setBoolean(false);
            this.setGoal(Degrees.of(m_GE_Goal.getDouble(0.0)));
        }
        // This method will be called once per scheduler run
        m_GE_Velocity.setDouble(m_LeftMotor.getVelocity().getValueAsDouble());
        m_GE_Position.setDouble(m_LeftMotor.getPosition().getValue().in(Degree));

        m_GE_Timer.setDouble(m_timer.get());
    }

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(m_LeftMotor, .000000001);

        Shuffleboard.getTab("Wrist").add(this.getOpenCommand());
        Shuffleboard.getTab("Wrist").add(this.getCloseCommand());

    }

    public Command getZeroWristAngleCmd() {
        Command c = Commands.runOnce(() -> m_LeftMotor.setPosition(Degrees.of(0)));
        c.setName("Wrist.ZeroAngle");
        // c.ignoringDisable(true);

        return c;
    }

    public Command getOpenCommand() {
        return Commands.runOnce(() -> setGoal(Degrees.of(0))).withName("Wrist.OpenCommand");
    }

    public Command getCloseCommand() {
        return Commands.runOnce(() -> setGoal(Degrees.of(40))).withName("Wrist.CloseCommand");
    }

    @Override
    public void simulationPeriodic() {

        RobotContainer.m_mechanisms.updateWrist(m_LeftMotor.getPosition(), m_LeftMotor.getVelocity());
    }

    public void doManualMove(double theta) {
        double kManualMaxSpeed = 1.0;
        double change = theta * kManualMaxSpeed;
        Angle newGoal = m_goal.plus(Degrees.of(change));

        if (newGoal.in(Degrees) > 60) {
            newGoal = Degrees.of(60);
        } else if (newGoal.in(Degrees) < -220) {
            newGoal = Degrees.of(-220);
        }

        setGoal(newGoal);
    }

    public boolean isStationary() {
        return Math.abs(m_LeftMotor.getVelocity().getValue().in(RotationsPerSecond)) < 0.2;
    }
}
