// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
// Intake States
private int m_CurrentState=0;
final private int m_kIdle = 0;
final private int m_kIntake = 1;
final private int m_kOuttake = 2;
final private int m_kUnclog = 3;
final private int m_kFastshoot = 4;
final private int m_kSlowshoot = 5;

TalonFX m_LeftMotor = new TalonFX(1, "rio");
TalonFX m_RightMotor = new TalonFX(2, "rio");

  /** Creates a new Intake. */
  public Intake() {
    // configs.Slot0.kS = 0.1;
    // configs.Slot0.kV = 0.12;
    // configs.Slot0.kP = 0.11;
    // configs.Slot0.kI = 0.1;
    // configs.Slot0.kD = 0.1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void doIdle() {

  }
  private void doIntake() {

  }
  private void doOuttake() {

  }
  private void doUnclog() {

  }
  private void doFastshoot() {

  }
  private void doSlowshoot() {

  }

}
