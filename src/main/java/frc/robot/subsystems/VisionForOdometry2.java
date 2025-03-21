// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class VisionForOdometry2 extends SubsystemBase {

    private GenericEntry m_GE_Back_TagCount = null;
    private GenericEntry m_GE_Back_IgnoredCount = null;
    private Pose2d m_LastBackPose = new Pose2d();
    private NetworkTable m_LL_Back = null;
    private NetworkTableEntry m_throttle = null;
    private boolean m_isTagFound = false;

    /** Creates a new Vision. */
    public VisionForOdometry2() {
        m_GE_Back_TagCount = Shuffleboard.getTab("Vision").add("Vision_Back_TagCount", 0).getEntry();
        m_GE_Back_IgnoredCount = Shuffleboard.getTab("Vision").add("Vision_Back_IgnoredCount", 0).getEntry();
        m_GE_Back_IgnoredCount.setDouble(0);
        m_GE_Back_TagCount.setDouble(0);

        m_LL_Back = NetworkTableInstance.getDefault().getTable("limelight-back");
        m_throttle = m_LL_Back.getEntry("throttle_set");

    }

    public void fullSpeed() {
        m_throttle.setDouble(0);
    }

    public void idle() {
        m_throttle.setDouble(50);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        var driveState = RobotContainer.drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

        LimelightHelpers.SetRobotOrientation("limelight-back", headingDeg, 0, 0, 0, 0, 0);
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");

        // make sure we have a valid measurement and we are not moving too fast
        if (llMeasurement != null && llMeasurement.tagCount > 0 && omegaRps < 2.0) {
            m_isTagFound = true;
            m_GE_Back_TagCount.setDouble(llMeasurement.tagCount);

            // Check if pose is with in 1 meter of last pose
            double lastPoseDistance = m_LastBackPose.getTranslation().getDistance(llMeasurement.pose.getTranslation());
            if (lastPoseDistance > 1.5) {
                m_GE_Back_IgnoredCount.setDouble(m_GE_Back_IgnoredCount.getDouble(0) + 1);

                // pose changed too rapidly, ignore it but average it out in case it is valid
                m_LastBackPose = new Pose2d(m_LastBackPose.getX() + llMeasurement.pose.getX() / 2,
                        m_LastBackPose.getY() + llMeasurement.pose.getY() / 2,
                        m_LastBackPose.getRotation().plus(llMeasurement.pose.getRotation()).div(2));
            } else {
                // use the distance from the robot to the tag to adjust std devs
                double tagDistance = llMeasurement.pose.getTranslation().getDistance(driveState.Pose.getTranslation());

                // adjust std devs based on distance
                if (tagDistance < 0.1) {
                    // very close small error has big impact so increase std dev
                    RobotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.9, .9, 9999999));
                } else if (tagDistance < 0.75) {
                    // optimal reading distance trust vision more
                    RobotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                } else {
                    // further away, trust vision less
                    RobotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                }

                RobotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose,
                        Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));

                // update last pose since deemed as valid
                m_LastBackPose = llMeasurement.pose;
            }
        } else {
            // no valid measurement
            m_isTagFound = false;
            m_GE_Back_TagCount.setDouble(0);
        }
    }

    public boolean isTagFound() {
        return m_isTagFound;
    }
}
