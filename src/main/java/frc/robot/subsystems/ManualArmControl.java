// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ManualArmControl extends SubsystemBase {

    private final CommandXboxController m_joystick = new CommandXboxController(4);
    private boolean m_isManualControlEnabled = false;
    private boolean m_isFirstTimeEnabled = true;

    /** Creates a new ManualArmControl. */
    public ManualArmControl() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (!m_joystick.isConnected()) {
            return;
        }

        if (m_isFirstTimeEnabled) {
            m_isFirstTimeEnabled = false;

            // manual enable/disable
            m_joystick.start().onTrue(this.runOnce(() -> m_isManualControlEnabled = true));
            m_joystick.back().onTrue(this.runOnce(() -> m_isManualControlEnabled = false));

            // zero arm calibrations
            // a = armbase
            // m_joystick.a().onTrue(RobotContainer.m_armBase.getZeroArmAngleCmd());

            // b = arm extension
            // m_joystick.x().onTrue(this.runOnce(() ->
            // RobotContainer.m_armExtension.getZeroArmExtCmd()));

            // y = wrist
            // m_joystick.y().onTrue(this.runOnce(() ->
            // RobotContainer.m_wrist.getZeroWristAngleCmd()));

            return;
        }

        if (!m_isManualControlEnabled) {
            return;
        }

        RobotContainer.m_armBase.doManualMove(MathUtil.applyDeadband(-m_joystick.getLeftY(), 0.2));
        RobotContainer.m_wrist.doManualMove(MathUtil.applyDeadband(m_joystick.getRightY(), 0.2));
        RobotContainer.m_armExtension.doManualMove(MathUtil.applyDeadband(m_joystick.getLeftX(), 0.2));
    }
}
