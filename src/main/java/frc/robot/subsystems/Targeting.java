// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.objdetect.CascadeClassifier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Targeting extends SubsystemBase {
    private GenericEntry m_TargetID = null;
    private GenericEntry m_TargetAngle = null;
    private GenericEntry m_TargetIDFound = null;

    /** Creates a new Targeting. */
    public Targeting() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setTargetAngle() {

        int targetID = (int) m_TargetID.getDouble(-1);
        double goal = 0.0;
        switch (targetID) {
            case 1:
                m_TargetAngle.setDouble(0.0);
                break;
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
            case 10:
            case 11:
            case 12:
                m_TargetAngle.setDouble(54.0);
                break;
            case 13:
                m_TargetAngle.setDouble(-54.5);
                break;
            case 14:
                m_TargetAngle.setDouble(180.0);
                break;
            case 15:
                m_TargetAngle.setDouble(180.0);
                break;
            case 16:
                m_TargetAngle.setDouble(270.0);
                break;
            case 17:
                m_TargetAngle.setDouble(55.8);
                break;
            case 18:
                m_TargetAngle.setDouble(0.0);
                break;
            case 19:
                m_TargetAngle.setDouble(-55.8);
                break;
            case 20:
                m_TargetAngle.setDouble(-119.5);
                break;
            case 21:
                m_TargetAngle.setDouble(180.0);
                break;
            case 22:
                m_TargetAngle.setDouble(118.5);
                break;

        }

    }

}
