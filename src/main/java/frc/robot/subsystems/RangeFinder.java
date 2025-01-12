// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.FloatSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.CANrange;

public class RangeFinder extends SubsystemBase {
  private CANrange range1 = new CANrange(22);
  private CANrange range2 = new CANrange(23);
  private CANrange range3 = new CANrange(24);
  private CANrange range4 = new CANrange(25);
  private CANrange range5 = new CANrange(26);
  private double m_velocity = 0.0;
  // private CANrange drive1 = new CANrange(1);
  // private CANrange drive2 = new CANrange(3);

  /** Creates a new RangeFinder. */
  public RangeFinder() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println("range1... found target: " + range1.getIsDetected()
        + " distance: " + range1.getDistance());
    System.out.println("range2... found target: " + range2.getIsDetected()
        + " distance: " + range2.getDistance());
    System.out.println("range3... found target: " + range3.getIsDetected()
        + " distance: " + range3.getDistance());
    System.out.println("range4... found target: " + range4.getIsDetected()
        + " distance: " + range4.getDistance());
    System.out.println("range5... found target: " + range5.getIsDetected()
        + " distance: " + range5.getDistance());

    // boolean b1 = range1.getIsDetected().getValue();
    // boolean b2 = range2.getIsDetected().getValue();
    // boolean b3 = range3.getIsDetected().getValue();
    // boolean b4 = range4.getIsDetected().getValue();
    // boolean b5 = range5.getIsDetected().getValue();

    boolean b1 = false;
    boolean b2 = false;
    boolean b3 = true;
    boolean b4 = false;
    boolean b5 = true;

    double v = 0.0;
    double vFast = 0.25;
    double vSlow = 0.15;
    double vMedium = 0.2;

    if (b2 && b4 && !b3 && !b5) {
      // medium right
      v = -vMedium;
      System.out.println("medium right");
    } else if (!b2 && b4 && !b3 && !b5) {
      // slow right
      v = -vSlow;
      System.out.println("slow right");
    } else if (b2 && !b4 && !b3 && !b5) {
      // fast right
      v = -vFast;
      System.out.println("fast right");
    } else if (b3 && b5 && !b2 && !b4) {
      // medium left
      v = vMedium;
      System.out.println("medium left");
    } else if (!b3 && b5 && !b2 && !b4) {
      // slow left
      v = vSlow;
      System.out.println("slow left");
    } else if (b3 && !b5 && !b2 && !b4) {
      // fast left
      v = vFast;
      System.out.println("fast left");
    } else if (b1 && !b2 && !b4 && !b3 && !b5) {
      // stay
      v = 0.0;
      System.out.println("stay");
    } else {
      // lost
      System.out.println("lost");
      v = 0.0;
    }
    System.out.println("v: " + Double.toString(v));
    m_velocity = v;
  }

  public double getVelocity() {
    return m_velocity;
  }
}
