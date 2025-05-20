package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Class to keep all the mechanism-specific objects together and out of the main
 * example
 */
public class Mechanisms {
    double HEIGHT = 2; // Controls the height of the mech2d SmartDashboard
    double WIDTH = 2; // Controls the height of the mech2d SmartDashboard

    Mechanism2d mech = new Mechanism2d(WIDTH, HEIGHT);

    // Position

    MechanismLigament2d arm = mech.getRoot("pivotPoint", 0.7, 0.3)
            .append(new MechanismLigament2d("arm", .1, 0, 20, new Color8Bit(Color.kAliceBlue)));

    // MechanismLigament2d side1 = arm
    // .append(new MechanismLigament2d("side1", 0.15307, 112.5, 6, new
    // Color8Bit(Color.kAliceBlue)));
    // MechanismLigament2d side2 = side1
    // .append(new MechanismLigament2d("side2", 0.15307, 45, 6, new
    // Color8Bit(Color.kAliceBlue)));
    // MechanismLigament2d side3 = side2
    // .append(new MechanismLigament2d("side3", 0.15307, 45, 6, new
    // Color8Bit(Color.kAliceBlue)));
    // MechanismLigament2d side4 = side3
    // .append(new MechanismLigament2d("side4", 0.15307, 45, 6, new
    // Color8Bit(Color.kAliceBlue)));
    // MechanismLigament2d side5 = side4
    // .append(new MechanismLigament2d("side5", 0.15307, 45, 6, new
    // Color8Bit(Color.kAliceBlue)));
    // MechanismLigament2d side6 = side5
    // .append(new MechanismLigament2d("side6", 0.15307, 45, 6, new
    // Color8Bit(Color.kAliceBlue)));
    // MechanismLigament2d side7 = side6
    // .append(new MechanismLigament2d("side7", 0.15307, 45, 6, new
    // Color8Bit(Color.kAliceBlue)));
    // MechanismLigament2d side8 = side7
    // .append(new MechanismLigament2d("side8", 0.15307, 45, 6, new
    // Color8Bit(Color.kAliceBlue)));

    MechanismLigament2d armBase = arm
            .append(new MechanismLigament2d("armbase", 0.45, 0, 6,
                    new Color8Bit(Color.kDarkMagenta)));

    MechanismLigament2d armExt = armBase
            .append(new MechanismLigament2d("extension", 1.0, 0, 4,
                    new Color8Bit(Color.kMaroon)));

    MechanismLigament2d wrist = armExt
            .append(new MechanismLigament2d("wrist", 0.15, 90, 10,
                    new Color8Bit(Color.kHotPink)));

    MechanismLigament2d intakeL = wrist
            .append(new MechanismLigament2d("rollerL", .05, 45, 10, new Color8Bit(Color.kGreenYellow)));

    /**
     * Runs the mech2d widget in GUI.
     *
     * This utilizes GUI to simulate and display a TalonFX and exists to allow users
     * to test and understand
     * features of our products in simulation using our examples out of the box.
     * Users may modify to have a
     * display interface that they find more intuitive or visually appealing.
     */
    public void update(StatusSignal<Angle> position, StatusSignal<AngularVelocity> velocity) {

        arm.setAngle(position.getValue().in(Degrees));
        SmartDashboard.putData("mech2d", mech); // Creates mech2d in SmartDashboard
    }

    public void updateExt(Distance position, StatusSignal<AngularVelocity> velocity) {
        armExt.setLength(0.1 + 1 + (position.magnitude() / 80));
    }

    public void updateWrist(StatusSignal<Angle> position, StatusSignal<AngularVelocity> velocity) {
        wrist.setAngle(-position.getValue().in(Degrees));
    }

    public void updateIntake(StatusSignal<Angle> positionL, StatusSignal<Angle> positionR) {
        intakeL.setAngle(45 + positionL.getValue().in(Degrees));
    }
}
