// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class LEDs extends SubsystemBase {

    private final CANdle m_candle = new CANdle(30);

    private final static int kLED_COLUMNS = 33;
    private final static int kLED_ROWS = 8;
    private final static int kSTRIP_START = 0;
    private final static int kSTRIP_LENGTH = 68;
    private final static int kPANEL_START = 16;

    private final static int kLED_TOTAL = kLED_COLUMNS * kLED_ROWS + kSTRIP_LENGTH;

    private static final int LedCount = 0;

    private static final boolean LimelightTarget_Detector = false;

    private boolean m_isPanelDisabled = false;
    private boolean m_isAligned = false;
    private GenericEntry m_pixel = null;
    boolean m_isCoralTrackingMode = false;

    Animation m_police = null;

    /** Creates a new LedPannel. */
    public LEDs() {
        CANdleConfiguration configALL = new CANdleConfiguration();
        configALL.disableWhenLOS = false;
        configALL.stripType = LEDStripType.GRB;
        configALL.brightnessScalar = 0.5; // dim the LEDs to 10% brightness
        // configALL.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configALL, 100);
        m_Timer.start();
        m_candle.setLEDs(0, 0, 0);
        m_pixel = Shuffleboard.getTab("LEDs")
                .add("Pixel", 0).getEntry();
        Shuffleboard.getTab("LEDs").add(this.PixelOn());
        Shuffleboard.getTab("LEDs").add(this.PixelOff());

        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 8, 32 * 8);

    }

    RainbowAnimation animation = new RainbowAnimation(.5, .5, kSTRIP_LENGTH);

    private static Timer m_Timer = new Timer();
    private static boolean m_even = true;

    private static int m_counter = 0;
    private boolean m_once = true;

    private int m_notePosLast = 0;

    public void setIsAligned(boolean b) {
        m_isAligned = b;
    }

    @Override
    public void periodic() {
        // limit to 10x a second
        if (m_Timer.advanceIfElapsed(0.1)) {
            // Only run if not disabled
            m_even = !m_even;
            m_once = true;

            if (false) {
                // Turn panel to black
                m_candle.animate(animation, 0);
            } else {
                if (m_once) {
                    m_once = false;
                    m_candle.clearAnimation(0);
                }
                // if coral tracking mode enabled skip reef lights
                if (m_isCoralTrackingMode) {
                    return;
                }

                // do reef alignment lights
                if (m_isAligned) {
                    this.showGreen();
                } else if (RobotContainer.m_visionForOdometry.isTagFound()) {
                    this.showBlue();
                } else {
                    this.showRed();
                }
            }

        }

    }

    public Command PixelOn() {
        int pixel = (int) m_pixel.getInteger(0);
        return runOnce(() -> m_candle.setLEDs(200, 70, 127, 0, 12, 1)).withName("PixelOn");
    }

    public Command PixelOff() {
        int pixel = (int) m_pixel.getInteger(0);
        return runOnce(() -> m_candle.setLEDs(0, 0, 0, 0, 12, 1)).withName("PixelOff");
    }

    public void showBlue() {
        m_candle.setLEDs(0, 0, 225, 0, kSTRIP_START + 0, kSTRIP_LENGTH);
    }

    public void showRed() {
        m_candle.setLEDs(225, 0, 0, 0, kSTRIP_START + 0, kSTRIP_LENGTH);
    }

    public void showBlack() {
        m_candle.setLEDs(0, 0, 0, 0, kSTRIP_START + 0, kSTRIP_LENGTH);
    }

    public void showGreen() {
        m_candle.setLEDs(0, 225, 0, 0, kSTRIP_START + 0, kSTRIP_LENGTH);
    }

    public void showWhite() {
        m_candle.setLEDs(0, 0, 0, 80, kSTRIP_START + 0, kSTRIP_LENGTH);
    }

    public Void showPoliceLights() {
        m_Timer.hasElapsed(0.2);
        if (m_even) {
            // System.out.println("Red");
            m_candle.setLEDs(225, 0, 0, 0, kPANEL_START + 0, 24);
        } else {
            // System.out.println("Blue");
            m_candle.setLEDs(0, 0, 225, 0, kPANEL_START + 0, 24);
        }
        return null;
    }

    public Void showPoliceLights2() {
        m_Timer.hasElapsed(0.2);

        if (m_even) {

            m_candle.setLEDs(0, 0, 225, 0, kSTRIP_START + 8 + 0, 6);
            m_candle.setLEDs(225, 0, 0, 0, kSTRIP_START + 8 + 6, 7);
            m_candle.setLEDs(0, 0, 225, 0, kSTRIP_START + 8 + 13, 1);
        } else {
            m_candle.setLEDs(225, 0, 0, 0, kSTRIP_START + 8 + 0, 6);
            m_candle.setLEDs(0, 0, 225, 0, kSTRIP_START + 8 + 6, 7);
            m_candle.setLEDs(225, 0, 0, 0, kSTRIP_START + 8 + 13, 1);
        }
        return null;
    }

    public Void showTrackingLights() {
        m_Timer.hasElapsed(0.2);
        if (m_even) {
            // System.out.println("green");
            m_candle.setLEDs(0, 255, 0, 0, kSTRIP_START + 0, kSTRIP_LENGTH);
        } else {
            // System.out.println("yellow");
            m_candle.setLEDs(225, 225, 0, 0, kSTRIP_START + 0, kSTRIP_LENGTH);
        }
        return null;
    }

    public void showNoNote() {
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 72, 32 * 8);
    }

    public void showNoteDown() {
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 72, 32 * 10);

        // this is half of the note going down

        // This is the orange blob

        m_candle.setLEDs(225, 40, 0, 40, kPANEL_START + 78, 4);
        // m_candle.setLEDs(225, 40, 0, 40, kPANEL_START + 190, 4);
        m_candle.setLEDs(225, 40, 0, 40, kPANEL_START + 93, 2);
        m_candle.setLEDs(225, 40, 0, 40, kPANEL_START + 98, 2);
        m_candle.setLEDs(225, 40, 0, 40, kPANEL_START + 124, 2);
        m_candle.setLEDs(225, 40, 0, 40, kPANEL_START + 114, 2);
        m_candle.setLEDs(225, 40, 0, 40, kPANEL_START + 108, 2);
        m_candle.setLEDs(225, 40, 0, 40, kPANEL_START + 130, 2);
        m_candle.setLEDs(225, 40, 0, 40, kPANEL_START + 140, 2);
        m_candle.setLEDs(225, 40, 0, 40, kPANEL_START + 146, 2);
        m_candle.setLEDs(225, 40, 0, 40, kPANEL_START + 156, 2);
        m_candle.setLEDs(225, 40, 0, 40, kPANEL_START + 162, 2);
        m_candle.setLEDs(225, 40, 0, 40, kPANEL_START + 176, 2);
        m_candle.setLEDs(225, 40, 0, 40, kPANEL_START + 190, 2);
        m_candle.setLEDs(225, 40, 0, 40, kPANEL_START + 173, 2);
    }

    public void showNoteUp() {

        // Turn panel to black
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 72, 32 * 10);

        // this is half of the note going up

        // This is the orange blob

        m_candle.setLEDs(225, 40, 0, 40, kPANEL_START + 72, 128);

        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 74, 11);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 92, 4);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 96, 4);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 108, 8);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 124, 8);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 140, 8);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 156, 8);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 172, 8);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 187, 11);

        // this is the middle section of the orange bulb
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 104, 2);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 102, 2);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 88, 1);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 118, 4);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 136, 2);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 126, 6);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 104, 2);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 128, 4);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 144, 1);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 150, 4);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 166, 4);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 182, 1);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 183, 1);
        m_candle.setLEDs(225, 40, 0, 40, kPANEL_START + 182, 1);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 134, 2);

    }

    public void showYIKE() {
        // this is the letter Y
        m_counter = m_counter + 1;
        int c = m_counter % 16;
        c = c * 16;
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 8 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 32 + c, 4);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 21 + c, 2);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 26 + c, 2);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 42 + c, 2);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 53 + c, 2);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 56 + c, 1);

        // this is the letter I
        int color_rI = 50;
        m_candle.setLEDs(color_rI, 5, 6, 0, kPANEL_START + 103 + c, 10);
        m_candle.setLEDs(color_rI, 0, 0, 0, kPANEL_START + 87 + c, 2);
        m_candle.setLEDs(color_rI, 0, 0, 0, kPANEL_START + 127 + c, 2);
        m_candle.setLEDs(color_rI, 0, 0, 0, kPANEL_START + 119 + c, 2);
        m_candle.setLEDs(color_rI, 0, 0, 0, kPANEL_START + 135 + c, 1);
        m_candle.setLEDs(color_rI, 0, 0, 0, kPANEL_START + 95 + c, 2);
        m_candle.setLEDs(color_rI, 0, 0, 0, kPANEL_START + 80 + c, 1);
        // this is the letter k
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 144 + c, 8);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 156 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 173 + c, 3);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 163 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 165 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 169 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 155 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 169 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 183 + c, 1);
        // this is the letter e
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 200 + c, 9);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 215 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 170 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 212 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 228 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 219 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 216 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 215 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 207 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 232 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 231 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 340 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 324 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 325 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 239 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 223 + c, 1);
        m_candle.setLEDs(200, 70, 0, 0, kPANEL_START + 224 + c, 1);
    }

    public void showYahh() {
        // this is the letter Y
        m_candle.setLEDs(200, 70, 0, 0, 8, 1);
        m_candle.setLEDs(200, 70, 0, 0, 32, 4);
        m_candle.setLEDs(200, 70, 0, 0, 21, 2);
        m_candle.setLEDs(200, 70, 0, 0, 26, 2);
        m_candle.setLEDs(200, 70, 0, 0, 42, 2);
        m_candle.setLEDs(200, 70, 0, 0, 53, 2);
        m_candle.setLEDs(200, 70, 0, 0, 56, 1);

        // this letter is A

        m_candle.setLEDs(200, 70, 0, 0, 103, 1);
        m_candle.setLEDs(200, 70, 0, 0, 80, 1);

        // the rest of the lines are the middle and filling in the letter
        m_candle.setLEDs(200, 70, 0, 0, 108, 1);
        m_candle.setLEDs(200, 70, 0, 0, 92, 1);
        m_candle.setLEDs(200, 70, 0, 0, 99, 1);
        m_candle.setLEDs(200, 70, 0, 0, 105, 2);
        m_candle.setLEDs(200, 70, 0, 0, 112, 5);
        m_candle.setLEDs(200, 70, 0, 0, 81, 4);
        m_candle.setLEDs(200, 70, 0, 0, 89, 2);

        // this is the letter Y2
        m_candle.setLEDs(200, 70, 0, 0, 184, 1);
        m_candle.setLEDs(200, 70, 0, 0, 136, 1);
        m_candle.setLEDs(200, 70, 0, 0, 149, 2);
        m_candle.setLEDs(200, 70, 0, 0, 115, 2);
        m_candle.setLEDs(200, 70, 0, 0, 154, 2);
        m_candle.setLEDs(200, 70, 0, 0, 160, 4);
        m_candle.setLEDs(200, 70, 0, 0, 181, 2);
        m_candle.setLEDs(200, 70, 0, 0, 170, 2);

        // the next few lines are for the !x3 point
        m_candle.setLEDs(200, 70, 0, 0, 200, 5);
        m_candle.setLEDs(200, 70, 0, 0, 207, 1);
    }

    public void showNote() {
        // this is the shape of the note
        m_candle.setLEDs(255, 40, 0, 40, kPANEL_START + 72, 128);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 72, 3);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 77, 3);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 192, 3);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 197, 3);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 80, 1);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 87, 1);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 184, 1);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 191, 1);

        // this the the middle being black
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 91, 2);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 98, 4);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 106, 4);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 114, 4);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 122, 4);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 130, 4);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 138, 4);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 146, 4);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 154, 4);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 162, 4);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 170, 4);
        m_candle.setLEDs(0, 0, 0, 0, kPANEL_START + 179, 2);

    }

    public void coralTrackingMode() {
        m_isCoralTrackingMode = true;

        // if we have a note go orange
        if (RobotContainer.m_intake.isCoralSensorDetected()) {
            this.showWhite();
            return;
        }
        // if (RobotContainer.m_coralTracking.isGamePieceFound()) {
        // this.showTrackingLights();
        // } else {
        // this.showRed();
        // }
    }

    public void noTrackingMode() {
        m_isCoralTrackingMode = false;

        if (RobotContainer.m_intake.isCoralSensorDetected()) {
            this.showWhite();
        } else {
            this.showBlack();
        }
    }
}