// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.objdetect.CascadeClassifier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.RandomExecutionLimiter;
import edu.wpi.first.units.measure.Angle;

public class Targeting extends SubsystemBase {

    // Define the enumerated type
    public enum ScoreTarget {
        kLeftCoral,
        kCenterAlgae,
        kRightCoral,
        kPlayerStation
    }

    // Define the enumerated type
    public enum HumanPlayerStation {
        kLeftStation,
        kRightStation
    }

    // Define the enumerated type
    public enum LineUpOrientation {
        kForward,
        kBackward
    }

    private LineUpOrientation m_lineUpOrientation = LineUpOrientation.kBackward;
    private HumanPlayerStation m_HumanPlayerStation = HumanPlayerStation.kLeftStation;
    private ScoreTarget m_ScoreTarget = ScoreTarget.kLeftCoral;
    private GenericEntry m_TargetID = null;
    private GenericEntry m_TargetAngle = null;
    private GenericEntry m_TargetIDFound = null;
    private GenericEntry m_GE_bUpdateTarget = null;
    private boolean m_isFirstTime = true;
    private Alliance m_alliance = Alliance.Red;
    private AprilTagFieldLayout m_aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private double kAprilTagWidth = 0.17 / 2.0;
    private RandomExecutionLimiter m_executionLimiter = new RandomExecutionLimiter();

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

        // Check if we should execute this cycle
        if (!m_executionLimiter.shouldExecute()) {
            return;
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

    private final double kCoralYoffset = 0.175; // left / right
    private final double kCoralXoffset = 0.03; // front / back
    private final double kAlgaeXoffset = 0.75;

    public Pose2d getScoreTargetPose() {
        // start with the target pose
        Pose2d pose = new Pose2d(m_TargetPose.getTranslation().getX(), m_TargetPose.getTranslation().getY(),
                m_TargetPose.getRotation());
        Translation2d offset;

        // apply the offset based on the target (coral or algae)
        ScoreTarget target = m_ScoreTarget;
        int targetID = (int) m_TargetID.getDouble(-1);

        switch (targetID) {
            case 1, 2, 12, 13:
                target = ScoreTarget.kPlayerStation;
                break;
            default:
                break;
        }
        switch (target) {
            case kLeftCoral:
                offset = new Translation2d(kCoralXoffset, kCoralYoffset);
                break;

            case kCenterAlgae:
                offset = new Translation2d(kAlgaeXoffset, 0);
                break;

            case kRightCoral:
                offset = new Translation2d(kCoralXoffset, -kCoralYoffset);
                break;
            case kPlayerStation:
                offset = new Translation2d(0, 0);
                break;
            default:
                return m_TargetPose;
        }

        // rotate the offset by the target pose angle and add it to the target pose to
        // calculate the score target pose
        if (m_lineUpOrientation == LineUpOrientation.kBackward) {
            offset = offset.rotateBy(pose.getRotation().plus(Rotation2d.fromDegrees(180)));
        } else {
            offset = offset.rotateBy(pose.getRotation().plus(Rotation2d.fromDegrees(0)));
        }

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

        Pose3d tagPose = m_aprilTagFieldLayout.getTagPose(targetID).isPresent()
                ? m_aprilTagFieldLayout.getTagPose(targetID).get()
                : new Pose3d(new Pose2d(0, 0, Rotation2d.fromDegrees(180)));
        Pose2d pose = new Pose2d(tagPose.getX(), tagPose.getY(), tagPose.getRotation().toRotation2d());

        // robot is 32 in long and we are trying to put it 14 inches from the reef, so
        // we need to be
        // 16 (half of length) + 14 inches from the reef or 30 inches from the reef. 30
        // inches in meters is 0.762.
        double distance = -0.762; // distance in meters
        Rotation2d rotation = pose.getRotation(); // get the rotation

        // Calculate the new x and y coordinates using the rotation
        double newX = pose.getX() - distance * rotation.getCos();
        double newY = pose.getY() - distance * rotation.getSin();

        // Create the new Pose2d
        Pose2d robotPose;

        // adjust robotPose based on player station
        switch (targetID) {
            case 1, 2, 12, 13:
                robotPose = new Pose2d(newX, newY, rotation.plus(Rotation2d.fromDegrees(180)));
                break;
            default:
                // Create the new Pose2d
                if (m_lineUpOrientation == LineUpOrientation.kForward) {
                    rotation = rotation.plus(Rotation2d.fromDegrees(180));
                }
                robotPose = new Pose2d(newX, newY, rotation);
                break;
        }

        m_TargetPose = robotPose;
        m_TargetAngle.setDouble(robotPose.getRotation().getDegrees());
    }

    public void setAlliance(Alliance allianceColor) {
        m_alliance = allianceColor;
    }

    public void setHumanPlayerStation(HumanPlayerStation station) {
        m_HumanPlayerStation = station;
    }

    public void setLineUpOrientation(LineUpOrientation orientation) {
        m_lineUpOrientation = orientation;
        this.setTargetAngle();
    }

    public Command getTargetHumanPlayerStation() {
        return this.runOnce(() -> setHumanPlayerStationAprilID(m_HumanPlayerStation))
                .withName("Targeting.HumanPlayerStation");

    }

    private void setHumanPlayerStationAprilID(HumanPlayerStation station) {
        switch (station) {
            case kLeftStation:
                if (m_alliance == Alliance.Red) {
                    setTargetID(1);
                } else {
                    setTargetID(13);
                }
                return;
            case kRightStation:
                if (m_alliance == Alliance.Red) {
                    setTargetID(2);
                } else {
                    setTargetID(12);
                }
                return;
            default:
                return;
        }
    }

}
