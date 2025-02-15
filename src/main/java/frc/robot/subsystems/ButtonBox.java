// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ButtonBox extends SubsystemBase {
    /** Creates a new ButtonBox. */
    public ButtonBox() {

        Shuffleboard.getTab("ButtonBox").add(this.getResetArmCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getCalibrateArmCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getIntakeInCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getIntakeOutCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getShootProcessorCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getTravelToReefCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getTravelToProcessorCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getHighCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getMiddleCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getLowCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getTroughCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getIntakeAlgaeCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getRemoveAlgaeCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getReleaseClimbCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getClimbCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getClimbArmCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getHighArmCmd());
        Shuffleboard.getTab("ButtonBox").add(this.getPlayerStation1Cmd());
        Shuffleboard.getTab("ButtonBox").add(this.getPlayerStation2Cmd());
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
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public Command getResetArmCmd() {
        Command c = new PrintCommand("Reset Arm");
        c.setName("Reset Arm");
        return c;
    }

    public Command getCalibrateArmCmd() {
        Command c = new PrintCommand("Calibrate Arm");
        c.setName("Calibrate Arm");
        return c;
    }

    public Command getIntakeInCmd() {
        Command c = new PrintCommand("Intake In");
        c.setName("Intake In");
        return c;
    }

    public Command getIntakeOutCmd() {
        Command c = new PrintCommand("Intake Out");
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

    public Command getHighCmd() {
        Command c = new PrintCommand("High");
        c.setName("High");
        return c;
    }

    public Command getMiddleCmd() {
        Command c = new PrintCommand("Middle");
        c.setName("Middle");
        return c;
    }

    public Command getLowCmd() {
        Command c = new PrintCommand("Low");
        c.setName("Low");
        return c;
    }

    public Command getTroughCmd() {
        Command c = new PrintCommand("Trough");
        c.setName("Trough");
        return c;
    }

    public Command getIntakeAlgaeCmd() {
        Command c = new PrintCommand("Intake Algae");
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

    public Command getPlayerStation1Cmd() {
        Command c = new PrintCommand("Player Station 1");
        c.setName("Player Station 1");
        return c;
    }

    public Command getPlayerStation2Cmd() {
        Command c = new PrintCommand("Player Station 2");
        c.setName("Player Station 2");
        return c;
    }

    public Command getReef1Cmd() {
        Command c = new PrintCommand("Reef 1");
        c.setName("Reef 1");
        return c;
    }

    public Command getReef2Cmd() {
        Command c = new PrintCommand("Reef 2");
        c.setName("Reef 2");
        return c;
    }

    public Command getReef3Cmd() {
        Command c = new PrintCommand("Reef 3");
        c.setName("Reef 3");
        return c;
    }

    public Command getReef4Cmd() {
        Command c = new PrintCommand("Reef 4");
        c.setName("Reef 4");
        return c;
    }

    public Command getReef5Cmd() {
        Command c = new PrintCommand("Reef 5");
        c.setName("Reef 5");
        return c;
    }

    public Command getReef6Cmd() {
        Command c = new PrintCommand("Reef 6");
        c.setName("Reef 6");
        return c;
    }

    public Command getNetCmd() {
        Command c = new PrintCommand("Net");
        c.setName("Net");
        return c;
    }

    public Command getLeftReefCmd() {
        Command c = new PrintCommand("Left Reef");
        c.setName("Left Reef");
        return c;
    }

    public Command getRightReefCmd() {
        Command c = new PrintCommand("Right Reef");
        c.setName("Right Reef");
        return c;
    }

    public Command getAlgaeReefCmd() {
        Command c = new PrintCommand("Algae Reef");
        c.setName("Algae Reef");
        return c;
    }

    public Command getProcessorCmd() {
        Command c = new PrintCommand("Processor");
        c.setName("Processor");
        return c;
    }
}
