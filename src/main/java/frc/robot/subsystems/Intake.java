// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import frc.robot.RobotContainer;
import frc.robot.sim.PhysicsSim;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static edu.wpi.first.units.Units.*;

public class Intake extends SubsystemBase {
    // Intake States
    private int m_CurrentState = 0;
    final private int m_kIdle = 0;
    final private int m_kIntake = 1;
    final private int m_kOuttake = 2;
    final private int m_kUnclog = 3;
    final private int m_kFastshoot = 4;
    final private int m_kSlowshoot = 5;

    TalonFX m_LeftMotor = new TalonFX(35, "rio");
    TalonFX m_RightMotor = new TalonFX(36, "rio");

    private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
    private GenericEntry m_GE_PID_kS = null;
    private GenericEntry m_GE_PID_kV = null;
    private GenericEntry m_GE_PID_kP = null;
    private GenericEntry m_GE_PID_kI = null;
    private GenericEntry m_GE_PID_kD = null;
    private GenericEntry m_GE_bUpdatePID = null;
    private GenericEntry m_GE_bUpdateGoals = null;
    private GenericEntry m_GE_Position = null;
    private GenericEntry m_GE_Velocity_Left_RPS = null;
    private GenericEntry m_GE_Velocity_Right_RPS = null;
    private GenericEntry m_GE_Goal_Left_RPS = null;
    private GenericEntry m_GE_Goal_Right_RPS = null;

    /** Creates a new Intake. */
    public Intake() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

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

        m_GE_PID_kS = Shuffleboard.getTab("Intake").add("Intake_kS", slot0.kS).getEntry();
        m_GE_PID_kV = Shuffleboard.getTab("Intake").add("Intake_kV", slot0.kV).getEntry();
        m_GE_PID_kP = Shuffleboard.getTab("Intake").add("Intake_kP", slot0.kP).getEntry();
        m_GE_PID_kI = Shuffleboard.getTab("Intake").add("Intake_kI", slot0.kI).getEntry();
        m_GE_PID_kD = Shuffleboard.getTab("Intake").add("Intake_kD", slot0.kD).getEntry();
        m_GE_bUpdatePID = Shuffleboard.getTab("Intake").add("Intake_UpdatePID", false).getEntry();
        m_GE_bUpdateGoals = Shuffleboard.getTab("Intake").add("Intake_UpdateGoals", false).getEntry();
        m_GE_Position = Shuffleboard.getTab("Intake").add("Intake_Position", 0).getEntry();
        m_GE_Velocity_Left_RPS = Shuffleboard.getTab("Intake").add("Intake_Velocity_Left_RPS", 0).getEntry();
        m_GE_Velocity_Right_RPS = Shuffleboard.getTab("Intake").add("Intake_Velocity_Right_RPS", 0).getEntry();
        m_GE_Goal_Left_RPS = Shuffleboard.getTab("Intake").add("Intake_Goal_Left_RPS", 0).getEntry();
        m_GE_Goal_Right_RPS = Shuffleboard.getTab("Intake").add("Intake_Goal_Right_RPS", 0).getEntry();

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
        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_RightMotor.getConfigurator().apply(cfg);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
            // hi

        }

        Shuffleboard.getTab("Intake").add(this.getStopCommand());
        Shuffleboard.getTab("Intake").add(this.getIntakeCommand());
        Shuffleboard.getTab("Intake").add(this.getOuttakeCommand());
    }

    @Override
    public void periodic() {
        // Update PID?
        if (m_GE_bUpdatePID.getBoolean(false)) {
            Slot0Configs slot0 = new Slot0Configs();
            m_LeftMotor.getConfigurator().refresh(slot0);
            slot0.kS = m_GE_PID_kS.getDouble(slot0.kS);
            slot0.kV = m_GE_PID_kV.getDouble(slot0.kV);
            slot0.kP = m_GE_PID_kP.getDouble(slot0.kP);
            slot0.kI = m_GE_PID_kI.getDouble(slot0.kI);
            slot0.kD = m_GE_PID_kD.getDouble(slot0.kD);
            m_LeftMotor.getConfigurator().apply(slot0);
            m_GE_bUpdatePID.setBoolean(false);

        }
        // update goals
        if (m_GE_bUpdateGoals.getBoolean(false)) {
            setGoal(m_GE_Goal_Left_RPS.getDouble(0), m_GE_Goal_Right_RPS.getDouble(0));
            m_GE_bUpdateGoals.setBoolean(false);
        }
        // This method will be called once per scheduler run

    }

    private void setGoal(double leftVelocityRPS, double rightVelocityRPS) {
        m_LeftMotor.setControl(m_velocityVoltage.withVelocity(leftVelocityRPS));
        m_GE_Goal_Left_RPS.setDouble(leftVelocityRPS);

        m_RightMotor.setControl(m_velocityVoltage.withVelocity(-rightVelocityRPS));
        m_GE_Goal_Right_RPS.setDouble(rightVelocityRPS);
    }

    public Command getSetGoalCommand(double leftVelocityRPS, double rightVelocityRPS) {
        return this.runOnce(() -> setGoal(leftVelocityRPS, rightVelocityRPS));
    }

    public Command getStopCommand() {
        return this.runOnce(() -> setGoal(0, 0)).withName("Intake.StopCommand");
    }

    public Command getIdleCommand() {
        return this.runOnce(() -> setGoal(-0.003, -0.003)).withName("Intake.IdleCommand");
    }

    public Command getIntakeCommand() {
        return this.runOnce(() -> setGoal(20, 20)).withName("Intake.IntakeCommand");
    }

    public Command getOuttakeCommand() {
        return this.runOnce(() -> setGoal(-10, -10)).withName("Intake.OuttakeCommand");
    }

    public Command getUnclogCommand() {
        return this.runOnce(() -> setGoal(-8, -8))
                .andThen(new WaitCommand(3))
                .andThen(this.runOnce(() -> setGoal(1, 1))).withName("Intake.UnclogCommand");
    }

    public Command getFastshootCommand() {
        return this.runOnce(() -> setGoal(5, 5)).withName("Intake.FastshootCommand");
    }

    public Command getSlowshootCommand() {
        return this.runOnce(() -> setGoal(.8, .8)).withName("Intake.SlowshootCommand");
    }

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(m_LeftMotor, 0.1);
        PhysicsSim.getInstance().addTalonFX(m_RightMotor, 0.1);

        Shuffleboard.getTab("Intake").add(this.getIdleCommand());
        Shuffleboard.getTab("Intake").add(this.getUnclogCommand());
        Shuffleboard.getTab("Intake").add(this.getFastshootCommand());
        Shuffleboard.getTab("Intake").add(this.getSlowshootCommand());

    }

    @Override
    public void simulationPeriodic() {

        m_GE_Velocity_Left_RPS.setDouble(m_LeftMotor.getVelocity().getValueAsDouble());
        m_GE_Velocity_Right_RPS.setDouble(m_RightMotor.getVelocity().getValueAsDouble());

        RobotContainer.m_mechanisms.updateIntake(m_LeftMotor.getPosition(), m_RightMotor.getPosition());
    }
}
