// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Targeting.HumanPlayerStation;
import frc.robot.subsystems.Targeting.ScoreTarget;

public class ButtonBox extends SubsystemBase {

    private final CommandXboxController joystick1 = new CommandXboxController(1);
    private final CommandXboxController joystick2 = new CommandXboxController(2);

    /** Creates a new ButtonBox. */
    public ButtonBox() {

        Shuffleboard.getTab("ButtonBox").add(this.getResetArmCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getCalibrateArmCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getIntakeInCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getIntakeOutCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getShootProcessorCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getTravelToReefCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getTravelToProcessorCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getBackL4Cmd());
        Shuffleboard.getTab("ButtonBox").add(this.getBackL3Cmd());
        Shuffleboard.getTab("ButtonBox").add(this.getIntakeAlgaeCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getRemoveAlgaeCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getReleaseClimbCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getClimbCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getClimbArmCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getHighArmCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getPlayerStationLeftCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getPlayerStationRightCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getReef1Cmd());
        Shuffleboard.getTab("ButtonBox").add(this.getReef2Cmd());
        Shuffleboard.getTab("ButtonBox").add(this.getReef3Cmd());
        Shuffleboard.getTab("ButtonBox").add(this.getReef4Cmd());
        Shuffleboard.getTab("ButtonBox").add(this.getReef5Cmd());
        Shuffleboard.getTab("ButtonBox").add(this.getReef6Cmd());
        Shuffleboard.getTab("ButtonBox").add(this.getNetCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getProcessorCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getLeftReefCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getRightReefCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getAlgaeReefCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getIntakeStopCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getBackWardsOrientationCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getForwardOrientationCmd());

        joystick2.button(1).onTrue(RobotContainer.m_armController.getTransition_ClimbReady());
        joystick2.button(2).onTrue(RobotContainer.m_armController.getTransition_ClimbLockOn());
        joystick2.button(3).onTrue(RobotContainer.m_armController.getTransition_Climb());
        joystick1.button(9).onTrue(RobotContainer.m_armController.getTransition_FrontScore_L1());
        joystick1.button(10).onTrue(RobotContainer.m_armController.getTransition_FrontScore_L2());
        joystick1.button(11).onTrue(RobotContainer.m_armController.getTransition_BackScore_L3());
        joystick1.button(12).onTrue(RobotContainer.m_armController.getTransition_BackScore_L4());
        joystick1.button(7).onTrue(RobotContainer.m_armController.getTransition_NetAlgae());
        joystick1.button(8).onTrue(RobotContainer.m_armController.getTransition_GroundAlgae());
        joystick1.button(6).onTrue(RobotContainer.m_intake.getIntakeCommand());
        joystick1.button(6).onFalse(RobotContainer.m_intake.getStopCommand());
        joystick1.button(5).onTrue(RobotContainer.m_intake.getOuttakeCommand());
        joystick1.button(5).onFalse(RobotContainer.m_intake.getStopCommand());
        joystick1.button(2).onTrue(RobotContainer.m_intake.getStopCommand());
        joystick1.button(4).onTrue(RobotContainer.m_armExtension.getCalibrateAndZero());
        joystick1.button(4).onFalse(RobotContainer.m_armExtension.GetStopCmd());
        joystick1.button(3).onTrue(RobotContainer.m_armController.getTransition_BackAligment());
        joystick2.button(7).onTrue(RobotContainer.m_buttonBox.getPlayerStationLeftCmd());
        joystick2.button(6).onTrue(RobotContainer.m_buttonBox.getPlayerStationRightCmd());
        joystick2.button(9).onTrue(RobotContainer.m_buttonBox.getReef1Cmd());
        joystick2.button(10).onTrue(RobotContainer.m_buttonBox.getReef2Cmd());
        joystick2.button(11).onTrue(RobotContainer.m_buttonBox.getReef3Cmd());
        joystick2.button(12).onTrue(RobotContainer.m_buttonBox.getReef4Cmd());
        joystick1.button(1).onTrue(RobotContainer.m_buttonBox.getReef5Cmd());
        joystick2.button(8).onTrue(RobotContainer.m_buttonBox.getReef6Cmd());

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public Command getBackWardsOrientationCmd() {
        Command c = new PrintCommand("Backwards")
                .andThen(runOnce(() -> RobotContainer.m_targeting.setLineUpOrientation(
                        Targeting.LineUpOrientation.kBackward)));
        c.setName("Backwards Orientation");
        return c;
    }

    public Command getForwardOrientationCmd() {
        Command c = new PrintCommand("Forward")
                .andThen(runOnce(() -> RobotContainer.m_targeting.setLineUpOrientation(
                        Targeting.LineUpOrientation.kForward)));
        c.setName("Forward Orientation");
        return c;
    }

    // working on...
    public Command getResetArmCmd() {
        Command c = new PrintCommand("Reset Arm")
                .andThen(RobotContainer.m_armController.getTransition_BackAligment());
        c.setName("Reset Arm");
        return c;
    }

    // origional
    /*
     * public Command getResetArmCmd() {
     * Command c = new PrintCommand("Reset Arm");
     * c.setName("Reset Arm");
     * return c;
     */
    public Command getCalibrateArmCmd() {
        Command c = new PrintCommand("Calibrate Arm")
                .andThen(RobotContainer.m_armExtension.getCalibrateAndZero());
        c.setName("Calibrate Arm");
        return c;
    }

    public Command getIntakeInCmd() {
        Command c = new PrintCommand("Intake In")
                .andThen(RobotContainer.m_intake.getIntakeCommand());
        c.setName("Intake In");
        return c;
    }

    public Command getIntakeOutCmd() {
        Command c = new PrintCommand("Intake Out")
                .andThen(RobotContainer.m_intake.getOuttakeCommand());
        c.setName("Intake Out");
        return c;
    }

    public Command getShootProcessorCmd() {
        Command c = new PrintCommand("Shoot Processor");
        c.setName("Shoot Processor");
        return c;
    }

    public Command getTravelToReefCmd() {
        Command c = new PrintCommand("Travel To Reef");
        c.setName("Travel To Reef");
        return c;
    }

    public Command getTravelToProcessorCmd() {
        Command c = new PrintCommand("Travel To Processor");
        c.setName("Travel To Processor");
        return c;
    }

    public Command getBackL4Cmd() {
        Command c = new PrintCommand("Back L4")
                .andThen(RobotContainer.m_armController.getTransition_Back_L4());
        c.setName("Back L4");
        return c;
    }

    public Command getBackL3Cmd() {
        Command c = new PrintCommand("Back L3")
                .andThen(RobotContainer.m_armController.getTransition_Back_L3());
        c.setName("Back L3");
        return c;
    }

    public Command getIntakeAlgaeCmd() {
        Command c = new PrintCommand("Intake Algae")
                .andThen(RobotContainer.m_intake.getIntakeCommand());
        c.setName("Intake Algae");
        return c;
    }

    public Command getRemoveAlgaeCmd() {
        Command c = new PrintCommand("Remove Algae");
        c.setName("Remove Algae");
        return c;
    }

    public Command getReleaseClimbCmd() {
        Command c = new PrintCommand("Release Climb");
        c.setName("Release Climb");
        return c;
    }

    public Command getClimbCmd() {
        Command c = new PrintCommand("Climb");
        c.setName("Climb");
        return c;
    }

    public Command getClimbArmCmd() {
        Command c = new PrintCommand("Climb Arm");
        c.setName("Climb Arm");
        return c;
    }

    public Command getHighArmCmd() {
        Command c = new PrintCommand("High Arm");
        c.setName("High Arm");
        return c;
    }

    public Command getPlayerStationLeftCmd() {
        Command c = new PrintCommand("Player Station Left");
        c = c.andThen(
                Commands.runOnce(() -> RobotContainer.m_targeting
                        .setHumanPlayerStation(HumanPlayerStation.kLeftStation)));

        c.setName("Player Station Left");
        return c;
    }

    public Command getPlayerStationRightCmd() {
        Command c = new PrintCommand("Player Station Right");
        c = c.andThen(Commands.runOnce(() -> RobotContainer.m_targeting
                .setHumanPlayerStation(HumanPlayerStation.kRightStation)));

        c.setName("Player Station Right");
        return c;
    }

    public Command getReef1Cmd() {
        Command c = new PrintCommand("Reef 1");
        c = c.andThen(Commands.runOnce(() -> RobotContainer.m_targeting.setTargetIDRedBlue(10, 21)));

        c.setName("Reef 1");
        return c;
    }

    public Command getReef2Cmd() {
        Command c = new PrintCommand("Reef 2");
        c = c.andThen(Commands.runOnce(() -> RobotContainer.m_targeting.setTargetIDRedBlue(9, 22)));

        c.setName("Reef 2");
        return c;
    }

    public Command getReef3Cmd() {
        Command c = new PrintCommand("Reef 3");
        c = c.andThen(Commands.runOnce(() -> RobotContainer.m_targeting.setTargetIDRedBlue(8, 17)));

        c.setName("Reef 3");
        return c;
    }

    public Command getReef4Cmd() {
        Command c = new PrintCommand("Reef 4");
        c = c.andThen(Commands.runOnce(() -> RobotContainer.m_targeting.setTargetIDRedBlue(7, 18)));

        c.setName("Reef 4");
        return c;
    }

    public Command getReef5Cmd() {
        Command c = new PrintCommand("Reef 5");
        c = c.andThen(Commands.runOnce(() -> RobotContainer.m_targeting.setTargetIDRedBlue(6, 19)));

        c.setName("Reef 5");
        return c;
    }

    public Command getReef6Cmd() {
        Command c = new PrintCommand("Reef 6");
        c = c.andThen(Commands.runOnce(() -> RobotContainer.m_targeting.setTargetIDRedBlue(11, 20)));

        c.setName("Reef 6");
        return c;
    }

    public Command getNetCmd() {
        Command c = new PrintCommand("Net");
        c.setName("Net");
        return c;
    }

    public Command getLeftReefCmd() {
        Command c = new PrintCommand("Left Reef")
                .andThen(Commands.runOnce(() -> RobotContainer.m_targeting.setScoreTarget(ScoreTarget.kLeftCoral)));

        c.setName("Left Reef");
        return c;
    }

    public Command getRightReefCmd() {
        Command c = new PrintCommand("Right Reef")
                .andThen(Commands.runOnce(() -> RobotContainer.m_targeting.setScoreTarget(ScoreTarget.kRightCoral)));
        c.setName("Right Reef");
        return c;
    }

    public Command getAlgaeReefCmd() {
        Command c = new PrintCommand("Algae Reef")
                .andThen(Commands.runOnce(() -> RobotContainer.m_targeting.setScoreTarget(ScoreTarget.kCenterAlgae)));
        c.setName("Algae Reef");
        return c;
    }

    public Command getProcessorCmd() {
        Command c = new PrintCommand("Processor");
        c.setName("Processor");
        return c;
    }

    public Command getIntakeStopCmd() {
        Command c = new PrintCommand("Intake Stop")
                .andThen(RobotContainer.m_intake.getStopCommand());
        c.setName("Intake Stop");
        return c;
    }
}
