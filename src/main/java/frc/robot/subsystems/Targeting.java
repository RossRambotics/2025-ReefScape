// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.objdetect.CascadeClassifier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;

public class Targeting extends SubsystemBase {

    // Define the enumerated type
    public enum ScoreTarget {
        kLeftCoral,
        kCenterAlgae,
        kRightCoral
    }

    private ScoreTarget m_ScoreTarget = ScoreTarget.kLeftCoral;
    private GenericEntry m_TargetID = null;
    private GenericEntry m_TargetAngle = null;
    private GenericEntry m_TargetIDFound = null;
    private GenericEntry m_GE_bUpdateTarget = null;
    private boolean m_isFirstTime = true;
    private Alliance m_alliance = Alliance.Red;

    private Pose2d m_TargetPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

    /** Creates a new Targeting. */
    public Targeting() {
        m_TargetID = Shuffleboard.getTab("Targeting").add("TargetID", 0).getEntry();
        m_TargetAngle = Shuffleboard.getTab("Targeting").add("TargetAngle", 0.0).getEntry();
        m_TargetIDFound = Shuffleboard.getTab("Targeting").add("TargetIDFound", false).getEntry();
        m_GE_bUpdateTarget = Shuffleboard.getTab("Targeting").add("UpdateTarget", false).getEntry();

    }

    @Override
    public void periodic() {
        if (m_isFirstTime && DriverStation.isEnabled()) {
            m_isFirstTime = false;
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                this.setTargetID(9);
            } else {
                this.setTargetID(18);
            }
        }

        // This method will be called once per scheduler run
        if (m_GE_bUpdateTarget.getBoolean(false)) {
            setTargetAngle();
            m_GE_bUpdateTarget.setBoolean(false);
        }
    }

    public void setScoreTarget(ScoreTarget target) {
        m_ScoreTarget = target;
    }

    private final double kCoralYoffset = 0.5;
    private final double kCoralXoffset = 0.25;
    private final double kAlgaeXoffset = 0.75;

    public Pose2d getScoreTargetPose() {
        // start with the target pose
        Pose2d pose = new Pose2d(m_TargetPose.getTranslation().getX(), m_TargetPose.getTranslation().getY(),
                m_TargetPose.getRotation());
        Translation2d offset;

        // apply the offset based on the target (coral or algae)
        switch (m_ScoreTarget) {
            case kLeftCoral:
                offset = new Translation2d(kCoralXoffset, kCoralYoffset);
                break;

            case kCenterAlgae:
                offset = new Translation2d(kAlgaeXoffset, 0);
                break;

            case kRightCoral:
                offset = new Translation2d(kCoralXoffset, -kCoralYoffset);
                break;
            default:
                return m_TargetPose;
        }

        // rotate the offset by the target pose angle and add it to the target pose to
        // calculate the score target pose
        offset = offset.rotateBy(pose.getRotation().plus(Rotation2d.fromDegrees(180)));
        return new Pose2d(pose.getTranslation().plus(offset), pose.getRotation());
    }

    public Rotation2d getTargetAngle() {
        return Rotation2d.fromDegrees(m_TargetAngle.getDouble(0.0));
    }

    public Pose2d getTargetPose() {
        return m_TargetPose;
    }

    /**
     * Set the target ID for the red and blue side. Defaults to the red side if the
     * alliance is not set.
     * 
     * @param redID
     * @param blueID
     */
    public void setTargetIDRedBlue(int redID, int blueID) {
        if (m_alliance == Alliance.Red) {
            m_TargetID.setDouble(redID);
        } else {
            m_TargetID.setDouble(blueID);
        }
        this.setTargetAngle();
    }

    public void setTargetID(int targetID) {
        m_TargetID.setDouble(targetID);
        this.setTargetAngle();
    }

    private void setTargetAngle() {

        int targetID = (int) m_TargetID.getDouble(-1);
        double goal = 0.0;
        switch (targetID) {
            // red side (i think ;) )
            case 1:
                m_TargetAngle.setDouble(-125.5);
                break;
            case 2:
                m_TargetAngle.setDouble(125.5);
                break;
            case 3:
                m_TargetAngle.setDouble(90);
                break;
            case 4:
                m_TargetAngle.setDouble(90);
                break;
            case 5:
                m_TargetAngle.setDouble(-90);
                break;
            case 6:
                m_TargetAngle.setDouble(55.8);
                break;
            case 7:
                m_TargetAngle.setDouble(180.0);
                break;
            case 8:
                m_TargetAngle.setDouble(-119.5);
                break;
            case 9:
                m_TargetAngle.setDouble(0);
                m_TargetPose = new Pose2d(14.5, 4, Rotation2d.fromDegrees(0.0));
                break;
            case 10:
                m_TargetAngle.setDouble(0.0);
                break;
            case 11:
                m_TargetAngle.setDouble(55.8);
                break;

            // blue side
            case 12:
                m_TargetAngle.setDouble(54.0);
                m_TargetPose = new Pose2d(1.300, 1.225, Rotation2d.fromDegrees(54.0));
                break;
            case 13:
                m_TargetAngle.setDouble(-54.5);
                m_TargetPose = new Pose2d(1.300, 6.916, Rotation2d.fromDegrees(54.0));
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
                m_TargetAngle.setDouble(60.0);
                m_TargetPose = new Pose2d(3.733, 2.707, Rotation2d.fromDegrees(60.0));
                break;
            case 18:
                m_TargetAngle.setDouble(180);
                m_TargetPose = new Pose2d(2.960, 4.033, Rotation2d.fromDegrees(180));
                break;
            case 19:
                m_TargetAngle.setDouble(-60);
                m_TargetPose = new Pose2d(3.733, 5.357, Rotation2d.fromDegrees(-60.0));
                break;
            case 20:
                m_TargetAngle.setDouble(-120);
                m_TargetPose = new Pose2d(5.385, 5.357, Rotation2d.fromDegrees(-120.0));
                break;
            case 21:
                m_TargetAngle.setDouble(180.0);
                m_TargetPose = new Pose2d(6.050, 4.033, Rotation2d.fromDegrees(180));
                break;
            case 22:
                m_TargetAngle.setDouble(120);
                m_TargetPose = new Pose2d(5.213, 2.707, Rotation2d.fromDegrees(120.0));
                break;

        }

    }

    public void setAlliance(Alliance allianceColor) {
        m_alliance = allianceColor;
    }

}
