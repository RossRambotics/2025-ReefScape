// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

import frc.robot.RobotContainer;
import frc.robot.Commands.EatCoralPlayerStation;
import frc.robot.sim.PhysicsSim;
import frc.util.RandomExecutionLimiter;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.*;

public class Intake extends SubsystemBase {
    // Intake States
    // private CANrange m_CoralSensor = new CANrange(22);
    private int m_CurrentState = 0;
    final private int m_kIdle = 0;
    final private int m_kIntake = 1;
    final private int m_kOuttake = 2;
    final private int m_kUnclog = 3;
    final private int m_kFastshoot = 4;
    final private int m_kSlowshoot = 5;

    TalonFX m_LeftMotor = new TalonFX(35, "rio");
    // TalonFX m_RightMotor = new TalonFX(36, "rio");

    private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
    private DoubleEntry m_PID_kS = null;
    private DoubleEntry m_PID_kV = null;
    private DoubleEntry m_PID_kP = null;
    private DoubleEntry m_PID_kI = null;
    private DoubleEntry m_PID_kD = null;
    private BooleanEntry m_bUpdatePID = null;
    private BooleanEntry m_bUpdateGoals = null;
    private DoubleEntry m_Velocity_Left_RPS = null;
    private DoubleEntry m_Velocity_Right_RPS = null;
    private DoubleEntry m_Goal_Left_RPS = null;
    private DoubleEntry m_Goal_Right_RPS = null;
    private RandomExecutionLimiter m_executionLimiter = new RandomExecutionLimiter();

    /** Creates a new Intake. */
    public Intake() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput = cfg.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        // Configure stator current limits
        CurrentLimitsConfigs statorCurrentLimit = new CurrentLimitsConfigs();
        statorCurrentLimit.StatorCurrentLimitEnable = true;
        statorCurrentLimit.StatorCurrentLimit = 80; // Current limit in amps
        cfg.CurrentLimits = statorCurrentLimit;

        /* Configure gear ratio */
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 1.0; // TODO: Calibrate motor rotations to sensor degrees

        /* Configure Velocity Control */
        // Peak output of 8 volts
        cfg.Voltage.withPeakForwardVoltage(Volts.of(8))
                .withPeakReverseVoltage(Volts.of(-8));

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
        slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                         // volts / rotation per second
        slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0; // No output for error derivative

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("Intake");

        m_PID_kS = table.getDoubleTopic("Intake_kS").getEntry(slot0.kS);
        m_PID_kS.set(slot0.kS);
        m_PID_kV = table.getDoubleTopic("Intake_kV").getEntry(slot0.kV);
        m_PID_kV.set(slot0.kV);
        m_PID_kP = table.getDoubleTopic("Intake_kP").getEntry(slot0.kP);
        m_PID_kP.set(slot0.kP);
        m_PID_kI = table.getDoubleTopic("Intake_kI").getEntry(slot0.kI);
        m_PID_kI.set(slot0.kI);
        m_PID_kD = table.getDoubleTopic("Intake_kD").getEntry(slot0.kD);
        m_PID_kD.set(slot0.kD);
        m_bUpdatePID = table.getBooleanTopic("Intake_UpdatePID").getEntry(false);
        m_bUpdatePID.set(false);
        m_bUpdateGoals = table.getBooleanTopic("Intake_UpdateGoals").getEntry(false);
        m_bUpdateGoals.set(false);
        m_Goal_Left_RPS = table.getDoubleTopic("Intake_Goal_Left_RPS").getEntry(0);
        m_Goal_Left_RPS.set(0);
        m_Goal_Right_RPS = table.getDoubleTopic("Intake_Goal_Right_RPS").getEntry(0);
        m_Goal_Right_RPS.set(0);
        m_Velocity_Left_RPS = table.getDoubleTopic("Intake_Velocity_Left_RPS").getEntry(0);
        m_Velocity_Left_RPS.set(0);
        m_Velocity_Right_RPS = table.getDoubleTopic("Intake_Velocity_Right_RPS").getEntry(0);
        m_Velocity_Right_RPS.set(0);

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
        // status = StatusCode.StatusCodeNotInitialized;
        // for (int i = 0; i < 5; ++i) {
        // status = m_RightMotor.getConfigurator().apply(cfg);
        // if (status.isOK())
        // break;
        // }
        // if (!status.isOK()) {
        // System.out.println("Could not configure device. Error: " +
        // status.toString());
        // // hi
        // // hello

        // }

        Shuffleboard.getTab("Intake").add(this.getStopCommand());
        Shuffleboard.getTab("Intake").add(this.getIntakeCommand());
        Shuffleboard.getTab("Intake").add(this.getOuttakeCommand());
        Shuffleboard.getTab("Intake").add(this.getOuttakeAlgaeCommand());
        Shuffleboard.getTab("Intake").add(this.getIdleCommand());
        Shuffleboard.getTab("Intake").add(new EatCoralPlayerStation(this).withName("Eat Coral"));

        // var cfgcs = m_CoralSensor.getConfigurator();
        // var prox = new ProximityParamsConfigs()
        // .withProximityThreshold(0.02)
        // .withProximityHysteresis(0.01);
        // var fov = new FovParamsConfigs()
        // .withFOVRangeX(6.75) // 6.75 is the minimum FOV value
        // .withFOVRangeY(6.75); // 27.0 is the maximum FOV value
        // var tof = new ToFParamsConfigs()
        // .withUpdateMode(UpdateModeValue.ShortRangeUserFreq)
        // .withUpdateFrequency(50);

        // cfgcs.apply(prox);
        // cfgcs.apply(fov);
        // cfgcs.apply(tof);

        // Trigger coralSensorTrigger = new Trigger(this::isCoralSensorDetected);
        // coralSensorTrigger.onTrue(this.getStopCommand());
    }

    private Sendable getOuttakeAlgaeCommand() {
        return Commands.runOnce(() -> setGoal(-50, -50)).withName("Intake.OuttakeAlgaeCommand");
    }

    @Override
    public void periodic() {

        if (!RobotContainer.isTuning) {
            return;
        }

        // Check if we should execute this cycle
        if (!m_executionLimiter.shouldExecute()) {
            return;
        }
        // Update PID?
        if (m_bUpdatePID.get(false)) {
            Slot0Configs slot0 = new Slot0Configs();
            m_LeftMotor.getConfigurator().refresh(slot0);
            slot0.kS = m_PID_kS.get(slot0.kS);
            slot0.kV = m_PID_kV.get(slot0.kV);
            slot0.kP = m_PID_kP.get(slot0.kP);
            slot0.kI = m_PID_kI.get(slot0.kI);
            slot0.kD = m_PID_kD.get(slot0.kD);
            m_LeftMotor.getConfigurator().apply(slot0);
            m_bUpdatePID.set(false);

        }
        // update goals
        if (m_bUpdateGoals.get(false)) {
            setGoal(m_Goal_Left_RPS.get(0), m_Goal_Right_RPS.get(0));
            m_bUpdateGoals.set(false);
        }
        // This method will be called once per scheduler run

    }

    // Method to check if the m_CoralSensor is detected
    public boolean isCoralSensorDetected() {
        // return m_CoralSensor.getIsDetected().getValue(); // Replace with the actual
        // method to check sensor state
        return false;
    }

    private void setGoal(double leftVelocityRPS, double rightVelocityRPS) {
        // m_LeftMotor.setControl(m_velocityVoltage.withVelocity(leftVelocityRPS));
        // m_Goal_Left_RPS.set(leftVelocityRPS);

        // m_RightMotor.setControl(m_velocityVoltage.withVelocity(-rightVelocityRPS));
        // m_Goal_Right_RPS.set(rightVelocityRPS);
        m_LeftMotor.setVoltage(leftVelocityRPS / 8.3);
    }

    public Command getSetGoalCommand(double leftVelocityRPS, double rightVelocityRPS) {
        return Commands.runOnce(() -> setGoal(leftVelocityRPS, rightVelocityRPS));
    }

    public Command getStopCommand() {
        return Commands.runOnce(() -> doStop()).withName("Intake.StopCommand");
    }

    private void doStop() {
        m_LeftMotor.stopMotor();
        // if (RobotContainer.m_buttonBox.isCoralMode()) {
        // setGoal(0, 0);
        // } else {
        // setGoal(5, 5);
        // }
    }

    public Command getIdleCommand() {
        return Commands.runOnce(() -> setGoal(5, 5)).withName("Intake.IdleCommand");
    }

    public Command getIntakeCommand() {
        return Commands.runOnce(() -> doIntake()).withName("Intake.IntakeCommand");
    }

    private void doIntake() {
        if (RobotContainer.m_buttonBox.isCoralMode()) {
            setGoal(75, 40);
        } else {
            setGoal(10, 10);
        }
    }

    public Command getOuttakeCommand() {
        return Commands.runOnce(() -> doOuttake()).withName("Intake.OuttakeCommand");
    }

    private void doOuttake() {
        if (RobotContainer.m_buttonBox.isCoralMode()) {
            setGoal(-50, -10);
        } else {
            setGoal(-50, -50);
        }
    }

    public Command getUnclogCommand() {
        return Commands.runOnce(() -> setGoal(-8, -8))
                .andThen(new WaitCommand(3))
                .andThen(Commands.runOnce(() -> setGoal(1, 1))).withName("Intake.UnclogCommand");
    }

    public Command getFastshootCommand() {
        return Commands.runOnce(() -> setGoal(5, 5)).withName("Intake.FastshootCommand");
    }

    public Command getSlowshootCommand() {
        return Commands.runOnce(() -> setGoal(.8, .8)).withName("Intake.SlowshootCommand");
    }

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(m_LeftMotor, 0.1);
        // PhysicsSim.getInstance().addTalonFX(m_RightMotor, 0.1);

        // Shuffleboard.getTab("Intake").add(this.getIdleCommand());
        // Shuffleboard.getTab("Intake").add(this.getUnclogCommand());
        // Shuffleboard.getTab("Intake").add(this.getFastshootCommand());
        // Shuffleboard.getTab("Intake").add(this.getSlowshootCommand());

    }

    @Override
    public void simulationPeriodic() {

        m_Velocity_Left_RPS.set(m_LeftMotor.getVelocity().getValueAsDouble());
        // m_Velocity_Right_RPS.set(m_RightMotor.getVelocity().getValueAsDouble());

        RobotContainer.m_mechanisms.updateIntake(m_LeftMotor.getPosition(), m_LeftMotor.getPosition());
    }
}
