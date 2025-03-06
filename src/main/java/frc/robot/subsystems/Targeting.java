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
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.RandomExecutionLimiter;
import edu.wpi.first.units.measure.Angle;

public class Targeting extends SubsystemBase {

    // Define the enumerated type
    public enum ScoreTarget {
        kCoralLeft,
        kAlgaeCenter,
        kCoralRight,
        kPlayerStation,
        kProcessor,
        kNetLeft,
        kNetCenter,
        kNetRight,
        kCage
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
    private ScoreTarget m_ScoreTarget = ScoreTarget.kCoralLeft;
    private IntegerEntry m_TargetID = null;
    private DoubleEntry m_TargetAngle = null;
    private BooleanEntry m_TargetIDFound = null;
    private BooleanEntry m_GE_bUpdateTarget = null;
    private StringEntry m_GE_Selected_PlayerStation = null;
    private StringEntry m_GE_Selected_Reef = null;
    private BooleanEntry m_alignCenter = null;
    private BooleanEntry m_alignLeft = null;
    private BooleanEntry m_alignRight = null;
    private BooleanEntry m_isReef_1 = null;
    private BooleanEntry m_isReef_2 = null;
    private BooleanEntry m_isReef_3 = null;
    private BooleanEntry m_isReef_4 = null;
    private BooleanEntry m_isReef_5 = null;
    private BooleanEntry m_isReef_6 = null;

    private boolean m_isFirstTime = true;
    private Alliance m_alliance = Alliance.Red;
    private AprilTagFieldLayout m_aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private double kAprilTagWidth = 0.17 / 2.0;
    private RandomExecutionLimiter m_executionLimiter = new RandomExecutionLimiter();

    private Pose2d m_TargetPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

    /** Creates a new Targeting. */
    public Targeting() {

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("Targeting");

        m_TargetID = table.getIntegerTopic("TargetID").getEntry(0);
        m_TargetID.set(0);
        m_TargetAngle = table.getDoubleTopic("TargetAngle").getEntry(0);
        m_TargetAngle.set(0);
        m_TargetIDFound = table.getBooleanTopic("TargetIDFound").getEntry(false);
        m_TargetIDFound.set(false);
        m_GE_bUpdateTarget = table.getBooleanTopic("UpdateTarget").getEntry(false);
        m_GE_bUpdateTarget.set(false);
        m_GE_Selected_PlayerStation = table.getStringTopic("Selected_PlayerStation").getEntry("Left");
        m_GE_Selected_PlayerStation.set("Left");
        m_GE_Selected_Reef = table.getStringTopic("Selected_Reef").getEntry("none");
        m_GE_Selected_Reef.set("none");

        m_alignCenter = table.getBooleanTopic("AlignCenter").getEntry(false);
        m_alignCenter.set(false);
        m_alignRight = table.getBooleanTopic("AlignRight").getEntry(false);
        m_alignRight.set(false);
        m_alignLeft = table.getBooleanTopic("AlignLeft").getEntry(false);
        m_alignLeft.set(false);

        m_isReef_1 = table.getBooleanTopic("isReef1Target").getEntry(false);
        m_isReef_1.set(false);
        m_isReef_2 = table.getBooleanTopic("isReef2Target").getEntry(false);
        m_isReef_2.set(false);
        m_isReef_3 = table.getBooleanTopic("isReef3Target").getEntry(false);
        m_isReef_3.set(false);
        m_isReef_4 = table.getBooleanTopic("isReef4Target").getEntry(false);
        m_isReef_4.set(false);
        m_isReef_5 = table.getBooleanTopic("isReef5Target").getEntry(false);
        m_isReef_5.set(false);
        m_isReef_6 = table.getBooleanTopic("isReef6Target").getEntry(false);
        m_isReef_6.set(false);

        // Shuffleboard.getTab("Targeting").add(this.getTargetLastReefIDCmd());
    }

    @Override
    public void periodic() {
        if (m_isFirstTime && DriverStation.isEnabled()) {
            m_isFirstTime = false;
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                this.setTargetID(9);
                m_ReefTargetID = 9;
            } else {
                this.setTargetID(18);
                m_ReefTargetID = 18;
            }
        }

        // Check if we should execute this cycle
        if (!m_executionLimiter.shouldExecute()) {
            return;
        }
        // This method will be called once per scheduler run
        if (m_GE_bUpdateTarget.get(false)) {
            setTargetAngle();
            m_GE_bUpdateTarget.set(false);
        }
    }

    public void setScoreTarget(ScoreTarget target) {
        m_ScoreTarget = target;

        switch (target) {
            case kCoralLeft:
            case kNetLeft:
                m_alignCenter.set(false);
                m_alignLeft.set(true);
                m_alignRight.set(false);
                break;

            case kAlgaeCenter:
            case kNetCenter:
                m_alignCenter.set(true);
                m_alignLeft.set(false);
                m_alignRight.set(false);
                break;

            case kCoralRight:
            case kNetRight:
                m_alignCenter.set(false);
                m_alignLeft.set(false);
                m_alignRight.set(true);
                break;

            default:
                return;
        }
    }

    private final double kCoralYoffset = 0.175; // left / right
    private final double kCcoralXoffset = 0.03 - 0.08; // front / back
    private final double kAlgaeXoffset = 0.03; // front / back
    private final double kNetYoffset = 1.0; // left / right
    private final double kNetXoffset = 0.00; // front / back
    private int m_ReefTargetID;

    public Pose2d getScoreTargetPose() {
        double coralXoffset = kCcoralXoffset;
        // if (m_lineUpOrientation == LineUpOrientation.kForward) {
        // coralXoffset = kCcoralXoffset * 2;
        // }

        // start with the target pose
        Pose2d pose = new Pose2d(m_TargetPose.getTranslation().getX(), m_TargetPose.getTranslation().getY(),
                m_TargetPose.getRotation());
        Translation2d offset;

        // apply the offset based on the target (coral or algae)
        ScoreTarget target = m_ScoreTarget;
        int targetID = (int) m_TargetID.get(-1);

        switch (targetID) {
            case 1, 2, 12, 13:
                target = ScoreTarget.kPlayerStation;
                break;
            default:
                break;
        }
        switch (target) {
            case kCoralLeft:
                offset = new Translation2d(coralXoffset, kCoralYoffset);
                m_alignCenter.set(false);
                m_alignLeft.set(true);
                m_alignRight.set(false);
                break;

            case kAlgaeCenter:
                offset = new Translation2d(kAlgaeXoffset, 0);
                m_alignCenter.set(true);
                m_alignLeft.set(false);
                m_alignRight.set(false);
                break;

            case kCoralRight:
                offset = new Translation2d(coralXoffset, -kCoralYoffset);
                m_alignCenter.set(false);
                m_alignLeft.set(false);
                m_alignRight.set(true);
                break;
            case kPlayerStation:
                offset = new Translation2d(0, 0);
                break;
            case kProcessor:
                offset = new Translation2d(0, 0);
                break;
            case kNetLeft:
                offset = new Translation2d(kNetXoffset, kNetYoffset);
                m_alignCenter.set(false);
                m_alignLeft.set(true);
                m_alignRight.set(false);
                break;
            case kNetCenter:
                offset = new Translation2d(kNetXoffset, 0);
                m_alignCenter.set(true);
                m_alignLeft.set(false);
                m_alignRight.set(false);
                break;
            case kNetRight:
                offset = new Translation2d(kNetXoffset, -kNetYoffset);
                m_alignCenter.set(false);
                m_alignLeft.set(false);
                m_alignRight.set(true);
            case kCage:
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
        return Rotation2d.fromDegrees(m_TargetAngle.get(0.0));
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
            m_ReefTargetID = redID;
            m_TargetID.set(redID);
        } else {
            m_ReefTargetID = blueID;
            m_TargetID.set(blueID);
        }
        this.setTargetAngle();
        this.updateSelectedReef();
    }

    public void updateSelectedReef() {
        m_isReef_1.set(false);
        m_isReef_2.set(false);
        m_isReef_3.set(false);
        m_isReef_4.set(false);
        m_isReef_5.set(false);
        m_isReef_6.set(false);

        int targetID = (int) m_TargetID.get(-1);
        switch (targetID) {
            case 21:
            case 10:
                m_GE_Selected_Reef.set("Reef 1");
                m_isReef_1.set(true);
                break;
            case 22:
            case 9:
                m_GE_Selected_Reef.set("Reef 2");
                m_isReef_2.set(true);
                break;
            case 17:
            case 8:
                m_GE_Selected_Reef.set("Reef 3");
                m_isReef_3.set(true);
                break;
            case 18:
            case 7:
                m_GE_Selected_Reef.set("Reef 4");
                m_isReef_4.set(true);
                break;
            case 19:
            case 6:
                m_GE_Selected_Reef.set("Reef 5");
                m_isReef_5.set(true);
                break;
            case 20:
            case 11:
                m_GE_Selected_Reef.set("Reef 6");
                m_isReef_6.set(true);
                break;
            default:
                m_GE_Selected_Reef.set("Unknown");
                break;
        }
    }

    public void targetLastReefID() {
        m_TargetID.set(m_ReefTargetID);
        this.setTargetAngle();
    }

    public void setTargetID(int targetID) {
        m_TargetID.set(targetID);
        this.setTargetAngle();
    }

    private void setTargetAngle() {

        int targetID = (int) m_TargetID.get(-1);

        Pose3d tagPose = m_aprilTagFieldLayout.getTagPose(targetID).isPresent()
                ? m_aprilTagFieldLayout.getTagPose(targetID).get()
                : new Pose3d(new Pose2d(0, 0, Rotation2d.fromDegrees(180)));
        Pose2d pose = new Pose2d(tagPose.getX(), tagPose.getY(), tagPose.getRotation().toRotation2d());

        // robot is 32 in long and we are trying to put it 14 inches from the reef, so
        // we need to be
        // 16 (half of length) + 14 inches from the reef or 30 inches from the reef. 30
        // inches in meters is 0.762.
        double distance = -0.762 + 0.127; // distance in meters
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
        m_TargetAngle.set(robotPose.getRotation().getDegrees());
    }

    public void setAlliance(Alliance allianceColor) {
        m_alliance = allianceColor;
    }

    public void setHumanPlayerStation(HumanPlayerStation station) {
        m_HumanPlayerStation = station;
        if (station == HumanPlayerStation.kLeftStation) {
            m_GE_Selected_PlayerStation.set("Left");
        } else {
            m_GE_Selected_PlayerStation.set("Right");
        }
    }

    public void setLineUpOrientation(LineUpOrientation orientation) {
        m_lineUpOrientation = orientation;
        this.setTargetAngle();
    }

    public Command getTargetHumanPlayerStation() {
        return Commands.runOnce(() -> setHumanPlayerStationAprilID(m_HumanPlayerStation))
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

    public Command getTargetLastReefIDCmd() {
        return Commands.runOnce(() -> targetLastReefID())
                .withName("Targeting.TargetLastReefID");
    }

}
