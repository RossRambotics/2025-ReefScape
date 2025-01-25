// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.FloatSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

public class RangeFinder extends SubsystemBase {
    private CANrange m_range1 = new CANrange(23);
    private CANrange m_range2 = new CANrange(10);
    private CANrange m_range3 = new CANrange(12);
    private CANrange m_range4 = new CANrange(14);
    private CANrange m_range5 = new CANrange(22);
    private double m_velocity = 0.0;

    private boolean m_bRange1 = false;
    private boolean m_bRange2 = false;
    private boolean m_bRange3 = true;
    private boolean m_bRange4 = false;
    private boolean m_bRange5 = true;
    private GenericEntry m_GE_bRange1 = null;
    private GenericEntry m_GE_bRange2 = null;
    private GenericEntry m_GE_bRange3 = null;
    private GenericEntry m_GE_bRange4 = null;
    private GenericEntry m_GE_bRange5 = null;
    private GenericEntry m_GE_distRange1 = null;
    private GenericEntry m_GE_distRange2 = null;
    private GenericEntry m_GE_distRange3 = null;
    private GenericEntry m_GE_distRange4 = null;
    private GenericEntry m_GE_distRange5 = null;

    // private CANrange drive1 = new CANrange(1);
    // private CANrange drive2 = new CANrange(3);

    /** Creates a new RangeFinder. */
    public RangeFinder() {
        var cfg = m_range1.getConfigurator();
        var prox = new ProximityParamsConfigs()
                .withProximityThreshold(1.5)
                .withProximityHysteresis(0.05);
        var fov = new FovParamsConfigs()
                .withFOVRangeX(6.75) // 6.75 is the minimum FOV value
                .withFOVRangeY(6.75); // 27.0 is the maximum FOV value
        var tof = new ToFParamsConfigs()
                .withUpdateMode(UpdateModeValue.LongRangeUserFreq)
                .withUpdateFrequency(50);

        cfg.apply(prox);
        cfg.apply(fov);
        cfg.apply(tof);

        cfg = m_range2.getConfigurator();
        cfg.apply(prox);
        cfg.apply(fov);
        cfg.apply(tof);

        cfg = m_range3.getConfigurator();
        cfg.apply(prox);
        cfg.apply(fov);
        cfg.apply(tof);

        cfg = m_range4.getConfigurator();
        cfg.apply(prox);
        cfg.apply(fov);
        cfg.apply(tof);

        cfg = m_range5.getConfigurator();
        cfg.apply(prox);
        cfg.apply(fov);
        cfg.apply(tof);

        m_GE_bRange1 = Shuffleboard.getTab("RangeFinder").add("range_b1", m_bRange1)
                .withPosition(3, 1)
                .withWidget(BuiltInWidgets.kBooleanBox).getEntry();
        m_GE_bRange2 = Shuffleboard.getTab("RangeFinder").add("range_b2", m_bRange2)
                .withPosition(1, 1)
                .withWidget(BuiltInWidgets.kBooleanBox).getEntry();
        m_GE_bRange3 = Shuffleboard.getTab("RangeFinder").add("range_b3", m_bRange3)
                .withPosition(5, 1)
                .withWidget(BuiltInWidgets.kBooleanBox).getEntry();
        m_GE_bRange4 = Shuffleboard.getTab("RangeFinder").add("range_b4", m_bRange4)
                .withPosition(2, 1)
                .withWidget(BuiltInWidgets.kBooleanBox).getEntry();
        m_GE_bRange5 = Shuffleboard.getTab("RangeFinder").add("range_b5", m_bRange5)
                .withPosition(4, 1)
                .withWidget(BuiltInWidgets.kBooleanBox).getEntry();
        m_GE_distRange1 = Shuffleboard.getTab("RangeFinder").add("range_dist1", 0)
                .withPosition(3, 2)
                .getEntry();
        m_GE_distRange2 = Shuffleboard.getTab("RangeFinder").add("range_dist2", 0)
                .withPosition(1, 2)
                .getEntry();
        m_GE_distRange3 = Shuffleboard.getTab("RangeFinder").add("range_dist3", 0)
                .withPosition(5, 2)
                .getEntry();
        m_GE_distRange4 = Shuffleboard.getTab("RangeFinder").add("range_dist4", 0)
                .withPosition(2, 2)
                .getEntry();
        m_GE_distRange5 = Shuffleboard.getTab("RangeFinder").add("range_dist5", 0)
                .withPosition(4, 2)
                .getEntry();

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // System.out.println("range1... found target: " + m_range1.getIsDetected()
        // + " distance: " + m_range1.getDistance());
        // System.out.println("range2... found target: " + m_range2.getIsDetected()
        // + " distance: " + m_range2.getDistance());
        // System.out.println("range3... found target: " + m_range3.getIsDetected()
        // + " distance: " + m_range3.getDistance());
        // System.out.println("range4... found target: " + m_range4.getIsDetected()
        // + " distance: " + m_range4.getDistance());
        // System.out.println("range5... found target: " + m_range5.getIsDetected()
        // + " distance: " + m_range5.getDistance());

        m_bRange1 = m_range1.getIsDetected().getValue();
        m_GE_bRange1.setBoolean(m_bRange1);
        if (m_bRange1) {
            m_GE_distRange1.setDouble(m_range1.getDistance().getValueAsDouble());
        } else {
            m_GE_distRange1.setDouble(0.0);
        }

        m_bRange2 = m_range2.getIsDetected().getValue();
        m_GE_bRange2.setBoolean(m_bRange2);
        if (m_bRange2) {
            m_GE_distRange2.setDouble(m_range2.getDistance().getValueAsDouble());
        } else {
            m_GE_distRange2.setDouble(0.0);
        }

        m_bRange3 = m_range3.getIsDetected().getValue();
        m_GE_bRange3.setBoolean(m_bRange3);
        if (m_bRange3) {
            m_GE_distRange3.setDouble(m_range3.getDistance().getValueAsDouble());
        } else {
            m_GE_distRange3.setDouble(0.0);
        }

        m_bRange4 = m_range4.getIsDetected().getValue();
        m_GE_bRange4.setBoolean(m_bRange4);
        if (m_bRange4) {
            m_GE_distRange4.setDouble(m_range4.getDistance().getValueAsDouble());
        } else {
            m_GE_distRange4.setDouble(0.0);
        }

        m_bRange5 = m_range5.getIsDetected().getValue();
        m_GE_bRange5.setBoolean(m_bRange5);
        if (m_bRange5) {
            m_GE_distRange5.setDouble(m_range5.getDistance().getValueAsDouble());
        } else {
            m_GE_distRange5.setDouble(0.0);
        }

        double v = 0.0;
        double vFast = 0.25;
        double vSlow = 0.15;
        double vMedium = 0.2;

        if (m_bRange2 && m_bRange4 && !m_bRange3 && !m_bRange5) {
            // medium right
            v = -vMedium;
        } else if (!m_bRange2 && m_bRange4 && !m_bRange3 && !m_bRange5) {
            // slow right
            v = -vSlow;
        } else if (m_bRange2 && !m_bRange4 && !m_bRange3 && !m_bRange5) {
            // fast right
            v = -vFast;
        } else if (m_bRange3 && m_bRange5 && !m_bRange2 && !m_bRange4) {
            // medium left
            v = vMedium;
        } else if (!m_bRange3 && m_bRange5 && !m_bRange2 && !m_bRange4) {
            // slow left
            v = vSlow;
        } else if (m_bRange3 && !m_bRange5 && !m_bRange2 && !m_bRange4) {
            // fast left
            v = vFast;
        } else if (m_bRange1 && !m_bRange2 && !m_bRange4 && !m_bRange3 && !m_bRange5) {
            // stay
            v = 0.0;
        } else {
            // lost
            v = 0.0;
        }
        // System.out.println("v: " + Double.toString(v));
        m_velocity = v;
    }

    public double getVelocity() {
        return m_velocity;
    }
}
