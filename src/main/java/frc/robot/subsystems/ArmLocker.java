// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ArmLocker extends SubsystemBase {
    private final Servo m_servo;

    /** Creates a new ArmLocker. */
    public ArmLocker() {
        // Initialize the servo on PWM port 0
        m_servo = new Servo(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Locks the hook by setting the servo to the lock position.
     */
    public void lock() {
        m_servo.setAngle(23); // Adjust the angle as needed for the lock position
        RobotContainer.m_armBase.climbMode();
    }

    /**
     * Unlocks the hook by setting the servo to the unlock position.
     */
    public void unlock() {
        m_servo.setAngle(80); // Adjust the angle as needed for the unlock position
        RobotContainer.m_armBase.normalMode();
    }

    Command getArmLockCmd() {
        return Commands.runOnce(() -> this.lock());
    }

    Command getArmUnlockCmd() {
        return Commands.runOnce(() -> this.unlock());
    }
}
