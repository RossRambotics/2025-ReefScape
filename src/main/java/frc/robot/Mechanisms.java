package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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
    // Velocity
    MechanismLigament2d VelocityMech = mech.getRoot("velocityLineReferencePosition", 0.75, 0.5)
            .append(new MechanismLigament2d("velocityLine", 1, 90, 6, new Color8Bit(Color.kAliceBlue)));

    MechanismLigament2d midline = mech.getRoot("midline", 0.7, 0.5)
            .append(new MechanismLigament2d("midline", 0.1, 0, 3, new Color8Bit(Color.kCyan)));

    // Position

    MechanismLigament2d arm = mech.getRoot("pivotPoint", 0.25, 0.5)
            .append(new MechanismLigament2d("arm", .2, 0, 0, new Color8Bit(Color.kAliceBlue)));

    MechanismLigament2d side1 = arm
            .append(new MechanismLigament2d("side1", 0.15307, 112.5, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d side2 = side1
            .append(new MechanismLigament2d("side2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d side3 = side2
            .append(new MechanismLigament2d("side3", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d side4 = side3
            .append(new MechanismLigament2d("side4", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d side5 = side4
            .append(new MechanismLigament2d("side5", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d side6 = side5
            .append(new MechanismLigament2d("side6", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d side7 = side6
            .append(new MechanismLigament2d("side7", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d side8 = side7
            .append(new MechanismLigament2d("side8", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));

    MechanismLigament2d armExt = side8
            .append(new MechanismLigament2d("extension", .0, -45 - 45 / 2, 4,
                    new Color8Bit(Color.kGreen)));

    MechanismLigament2d wrist = armExt
            .append(new MechanismLigament2d("wrist", 0.15, 90, 10,
                    new Color8Bit(Color.kHotPink)));

    MechanismLigament2d intakeL = mech.getRoot("intakeLpivot", 1.0, 1.25)
            .append(new MechanismLigament2d("rollerL", .2, 0, 0, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d intakeLside1 = intakeL
            .append(new MechanismLigament2d("intakeLside1", 0.15307, 112.5, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d intakeLside2 = intakeLside1
            .append(new MechanismLigament2d("intakeLside2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d intakeLside3 = intakeLside2
            .append(new MechanismLigament2d("intakeLside3", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d intakeLside4 = intakeLside3
            .append(new MechanismLigament2d("intakeLside4", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d intakeLside5 = intakeLside4
            .append(new MechanismLigament2d("intakeLide5", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d intakeLside6 = intakeLside5
            .append(new MechanismLigament2d("intakeLside6", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d intakeLside7 = intakeLside6
            .append(new MechanismLigament2d("intakeLside7", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d intakeLside8 = intakeLside7
            .append(new MechanismLigament2d("intakeLside8", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));

    MechanismLigament2d intakeR = mech.getRoot("intakeRpivot", 1.5, 1.25)
            .append(new MechanismLigament2d("rollerL", .2, 0, 0, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d intakeRside1 = intakeR
            .append(new MechanismLigament2d("intakeRside1", 0.15307, 112.5, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d intakeRside2 = intakeRside1
            .append(new MechanismLigament2d("intakeRside2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d intakeRside3 = intakeRside2
            .append(new MechanismLigament2d("intakeRside3", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d intakeRside4 = intakeRside3
            .append(new MechanismLigament2d("intakeRside4", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d intakeRside5 = intakeRside4
            .append(new MechanismLigament2d("intakeRide5", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d intakeRside6 = intakeRside5
            .append(new MechanismLigament2d("intakeRside6", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d intakeRside7 = intakeRside6
            .append(new MechanismLigament2d("intakeRside7", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    MechanismLigament2d intakeRside8 = intakeRside7
            .append(new MechanismLigament2d("intakeRside8", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));

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
        VelocityMech.setLength(velocity.getValue().in(RotationsPerSecond)); // Divide by 120 to scale motion to
                                                                            // fit in the window
        arm.setAngle(position.getValue().in(Degrees));
        SmartDashboard.putData("mech2d", mech); // Creates mech2d in SmartDashboard
    }

    public void updateExt(StatusSignal<Angle> position, StatusSignal<AngularVelocity> velocity) {
        armExt.setLength(position.getValue().magnitude() / 3);
    }

    public void updateWrist(StatusSignal<Angle> position, StatusSignal<AngularVelocity> velocity) {
        wrist.setAngle(position.getValue().in(Degrees) + 90);
    }

    public void updateIntake(StatusSignal<Angle> positionL, StatusSignal<Angle> positionR) {
        intakeL.setAngle(positionL.getValue().in(Degrees));
        intakeR.setAngle(positionR.getValue().in(Degrees));
    }
}
