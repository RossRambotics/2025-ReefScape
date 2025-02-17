// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.*;

public class SpeedNanny extends SubsystemBase {
    private final double m_kMaxAcceleration = 15.0;
    private final double m_kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                           // top speed
    private double m_kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
    // max angular velocity

    private double m_speedLimit = m_kMaxSpeed;
    private double m_accelerationLimit = 15.0;
    private double m_angularRateLimit = m_kMaxAngularRate;

    /** Creates a new SpeedNanny. */
    public SpeedNanny() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Calculate new speed limit based on arm angle & arm extension
        m_speedLimit = m_kMaxSpeed;
        m_angularRateLimit = m_kMaxAngularRate;
        m_accelerationLimit = m_kMaxAcceleration;

        double angle = RobotContainer.m_armBase.getCurrentAngle().in(Degrees);

        if (angle > 0.0 && angle <= 30.0) {
            m_speedLimit = m_kMaxSpeed * 0.5;
            m_angularRateLimit = m_kMaxAngularRate * 0.5;
        } else if (angle > 30.0) {
            m_speedLimit = m_kMaxSpeed * 0.25;
            m_angularRateLimit = m_kMaxAngularRate * 0.25;
        }

    }

    public double getSpeedLimit() {
        return m_speedLimit;
    }

    public double getAccelerationLimit() {
        return m_accelerationLimit;
    }

    public double getAngularRateLimit() {
        return m_angularRateLimit;
    }

}
