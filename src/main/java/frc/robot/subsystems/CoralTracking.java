// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.util.DistanceCalculator.DistanceCalculator;
import frc.util.DistanceCalculator.VisionMeasurement;

public class CoralTracking extends SubsystemBase {
    private GenericEntry m_isGamePieceFound = null; // does the game piece tracking camera see a gamepiece
    private GenericEntry m_GamePieceDistance = null; // distance to nearest gamepiece
    private GenericEntry m_GamePieceOffset = null; // robot-centric x distance to the center of the game piece

    private DistanceCalculator m_CoralDistanceCalculator = new DistanceCalculator();

    private NetworkTable m_LL_GamePiece = null;
    private double m_CurrentHeading = 0.0;
    private boolean m_isCoralTracking = false;

    /** Creates a new Tracking. */
    public CoralTracking() {

        // Establish Game Piece tracking LL
        m_LL_GamePiece = NetworkTableInstance.getDefault().getTable("limelight-front");

        // Establish shuffleboard variables
        m_isGamePieceFound = Shuffleboard.getTab("Tracking")
                .add("isGamePieceFound", false).getEntry();
        m_GamePieceDistance = Shuffleboard.getTab("Tracking")
                .add("GamePieceDistance", -1.0).getEntry();
        m_GamePieceOffset = Shuffleboard.getTab("Tracking")
                .add("GamePieceOffset", 0.0).getEntry();

        // Coral bounds setting (make anything closer 0.10m and anything further 4.0m)
        m_CoralDistanceCalculator.addSolution(new VisionMeasurement(-100, 0.10));
        m_CoralDistanceCalculator.addSolution(new VisionMeasurement(-24, 0.10));
        m_CoralDistanceCalculator.addSolution(new VisionMeasurement(20.0, 4.0));
        m_CoralDistanceCalculator.addSolution(new VisionMeasurement(100.0, 4.0));

        // Coral LL3 measurements
        // TODO calibrate measurements
        m_CoralDistanceCalculator.addSolution(new VisionMeasurement(-20.04, 0.10));
        m_CoralDistanceCalculator.addSolution(new VisionMeasurement(-18.24, 0.20));
        m_CoralDistanceCalculator.addSolution(new VisionMeasurement(-16.27, 0.30));
        m_CoralDistanceCalculator.addSolution(new VisionMeasurement(-12.72, 0.40));
        m_CoralDistanceCalculator.addSolution(new VisionMeasurement(-9.15, 0.50));
        m_CoralDistanceCalculator.addSolution(new VisionMeasurement(2.98, 1.0));
        m_CoralDistanceCalculator.addSolution(new VisionMeasurement(8.51, 1.5));
        m_CoralDistanceCalculator.addSolution(new VisionMeasurement(12.17, 2.0));
        m_CoralDistanceCalculator.addSolution(new VisionMeasurement(14.47, 2.5));
        m_CoralDistanceCalculator.addSolution(new VisionMeasurement(16.22, 3.0));
        m_CoralDistanceCalculator.addSolution(new VisionMeasurement(17.26, 3.5));
        m_CoralDistanceCalculator.addSolution(new VisionMeasurement(18.59, 4.0));

        DataLogManager.log("0.00: " +
                m_CoralDistanceCalculator.compute(0.00).m_distance);
    }

    public void setCurrentHeading(double degrees) {
        m_CurrentHeading = degrees;
    }

    @Override
    public void periodic() {
        double dv = 0.0;

        // This method will be called once per scheduler run

        // do we see a game piece
        dv = m_LL_GamePiece.getEntry("tv").getDouble(0);
        if (dv > 0) {
            m_isGamePieceFound.setBoolean(true);
        } else {
            m_isGamePieceFound.setBoolean(false);
        }

        // if we are tracking a game piece calculate distance and offset
        if (this.isGamePieceFound()) {

            this.calcGamePieceDistance();
        }
    }

    private void calcGamePieceDistance() {
        // tx Range is -/+29.8 degrees
        // ty Range is -/+24.85 degrees
        double tx = m_LL_GamePiece.getEntry("tx").getDouble(0);
        double ty = m_LL_GamePiece.getEntry("ty").getDouble(0);

        double distance = m_CoralDistanceCalculator.compute(ty).m_distance;
        double offset = distance * Math.tan(Math.toRadians(tx));

        // double distance = (400.546 * Math.tan(Math.toRadians(0.114604 * (ty +
        // 8.1186)))) + 938.939;
        // distance = distance / 1000;
        // double offset = distance * Math.tan(Math.toRadians(tx));

        // Since targeting camera is pointing forward these are NOT inverted
        m_GamePieceDistance.setDouble(distance);
        m_GamePieceOffset.setDouble(offset);
    }

    public boolean isCoralTracking() {
        return m_isCoralTracking;
    }

    public boolean isGamePieceFound() {
        return m_isGamePieceFound.getBoolean(false);
    }

    public double getGamePiece_VelocityY() {
        double answer = 0;
        double offset = -m_GamePieceOffset.getDouble(0.0);

        double deadzone = 0.05; // TODO tune this
        double kP = 2.0; // TODO tune this
        double kS = 0.1; // TODO tune this

        if (offset < 0.0) {
            kS = kS * -1;
        }

        answer = (offset * kP) + kS;

        // if (answer <= 3) {
        // answer = 3;
        // }

        if (Math.abs(offset) < deadzone) {
            answer = 0;
        }

        DataLogManager.log("Game Piece Left/Right: " + answer);

        return answer;
    }

    public double getGamePiece_VelocityX() {
        double answer = 0;
        double offset = m_GamePieceDistance.getDouble(0.0);

        double deadzone = 0.05; // TODO tune this
        double kP = 1.0; // TODO tune this
        double kS = 0.1; // TODO tune this

        if (offset < 0.0) {
            kS = kS * -1;
        }

        answer = (offset * kP) + kS;

        if (answer <= 3) {
            answer = 3;
        }

        if (Math.abs(offset) < deadzone) {
            answer = 0;
        }

        DataLogManager.log("Game Piece Front/Back: " + answer);

        return answer;
    }

    public double getGamePiece_RotationalRate() {
        // if we don't see a game piece don't turn
        if (!this.isGamePieceFound()) {
            return 0;
        }

        double answer = 0;
        double offset = -Math.toRadians(m_LL_GamePiece.getEntry("tx").getDouble(0));

        double deadzone = 0.05; // TODO tune this
        double kP = 5; // TODO tune this
        double kS = 0.1; // TODO tune this

        if (offset < 0.0) {
            kS = kS * -1;
        }

        answer = (offset * kP) + kS;

        if (Math.abs(offset) < deadzone) {
            answer = 0;
        }

        DataLogManager.log("Game Piece Rotation: " + answer);

        return answer;
    }

    public Command NoteTrackingMode() {
        Command c;

        c = new FunctionalCommand(
                () -> m_isCoralTracking = true,
                () -> RobotContainer.m_LEDs.coralTrackingMode(),
                interrupted -> m_isCoralTracking = false,
                () -> {
                    return false;
                })
                .withName("NoteTrackingMode");

        return c;
    }

    public Command NoTrackingMode() {
        Command c;

        c = new FunctionalCommand(
                () -> {
                    m_isCoralTracking = false;
                },
                () -> RobotContainer.m_LEDs.noTrackingMode(),
                interrupted -> m_isCoralTracking = false,
                () -> {
                    return false;
                })
                .withName("NoTrackingMode");

        return c;
    }

}