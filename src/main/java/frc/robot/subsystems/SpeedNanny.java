// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.*;

public class SpeedNanny extends SubsystemBase {
    private final double m_kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                           // top speed
    private double m_speedLimit = m_kMaxSpeed;

    /** Creates a new SpeedNanny. */
    public SpeedNanny() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Calculate new speed limit based on arm angle & arm extension
        m_speedLimit = m_kMaxSpeed;

    }

    public double getSpeedLimit() {
        return m_speedLimit;
    }

    public double getAccelerationLimit() {
        return 15.0;
    }

}
