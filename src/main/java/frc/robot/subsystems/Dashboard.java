// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dashboard extends SubsystemBase {
    /** Creates a new Dashboard. */
    public Dashboard() {
        Shuffleboard.getTab("Dashboard").add(this.getAngle0Cmd());
        Shuffleboard.getTab("Dashboard").add(this.getAngle60Cmd());
        Shuffleboard.getTab("Dashboard").add(this.getAngle120Cmd());
        Shuffleboard.getTab("Dashboard").add(this.getAngle180Cmd());
        Shuffleboard.getTab("Dashboard").add(this.getAngle240Cmd());
        Shuffleboard.getTab("Dashboard").add(this.getAngle300Cmd());

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public Command getAngle0Cmd() {
        Command c = new PrintCommand("setting angle to 0")
                .andThen(Commands.runOnce(() -> setAlignAngle(Degrees.of(0))));
        c.setName("setAngle0");

        return c;
    }

    public Command getAngle60Cmd() {
        Command c = new PrintCommand("setting angle to 60")
                .andThen(Commands.runOnce(() -> setAlignAngle(Degrees.of(60))));
        c.setName("setAngle060");

        return c;
    }

    public Command getAngle120Cmd() {
        Command c = new PrintCommand("setting angle to 120")
                .andThen(Commands.runOnce(() -> setAlignAngle(Degrees.of(120))));
        c.setName("setAngle120");

        return c;
    }

    public Command getAngle180Cmd() {
        Command c = new PrintCommand("setting angle to 180")
                .andThen(Commands.runOnce(() -> setAlignAngle(Degrees.of(180))));
        c.setName("setAngle180");

        return c;
    }

    public Command getAngle240Cmd() {
        Command c = new PrintCommand("setting angle to 240")
                .andThen(Commands.runOnce(() -> setAlignAngle(Degrees.of(240))));
        c.setName("setAngle240");

        return c;
    }

    public Command getAngle300Cmd() {
        Command c = new PrintCommand("setting angle to 300")
                .andThen(Commands.runOnce(() -> setAlignAngle(Degrees.of(300))));
        c.setName("setAngle300");

        return c;
    }

    private Angle m_alignAngle;

    private Angle getAlignAngle() {
        return Degrees.of(90);
    }

    public Rotation2d getAlignAngleRot() {
        return new Rotation2d(m_alignAngle);
    }

    private void setAlignAngle(Angle a) {
        m_alignAngle = a;
    }
}
