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

}
