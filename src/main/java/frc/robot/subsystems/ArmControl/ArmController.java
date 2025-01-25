// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ArmControl;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.util.GraphCommand.GraphCommand;
import frc.util.GraphCommand.GraphCommand.GraphCommandNode;

public class ArmController extends SubsystemBase {
    final private GraphCommand m_armGraph = new GraphCommand();
    private boolean m_isFirstTime = true;
    private GenericEntry m_GE_nodeName;
    private GenericEntry m_GE_nextNodeName;

    private GraphCommandNode A, B, C;

    /** Creates a new ArmController. */
    public ArmController() {
        m_GE_nodeName = Shuffleboard.getTab("ArmController").add("ArmController.CurrentNode", "<none>").getEntry();
        m_GE_nextNodeName = Shuffleboard.getTab("ArmController").add("ArmController.NextNode", "<none>").getEntry();

    }

    private void initialize() {
        A = m_armGraph.new GraphCommandNode("A",
                ArmController.getArmCommand(Degrees.of(0),
                        Meters.of(0),
                        Degrees.of(-90)),
                null,
                null);
        B = m_armGraph.new GraphCommandNode("B",
                ArmController.getArmCommand(Degrees.of(90),
                        Meters.of(0),
                        Degrees.of(-90))
                        .andThen(RobotContainer.m_armBase.getWaitUntilGreaterThanCommand(Degrees.of(45.0))),
                null,
                new PrintCommand("B is done"));

        m_armGraph.setGraphRootNode(A);
        m_armGraph.setCurrentNode(A);
        A.AddNode(B, 1);

        m_armGraph.initialize();
        m_armGraph.addRequirements(this);
        this.setDefaultCommand(m_armGraph);
        Shuffleboard.getTab("ArmController").add(this.getTransition_B());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (m_isFirstTime) {
            m_isFirstTime = false;
            this.initialize();
        }

        // send data to the SmartDashboard
        if (DriverStation.isDisabled()) {
            return;
        }

        if (m_armGraph.getCurrentNode() == null) {
            m_GE_nodeName.setString("<none>");
        } else {
            m_GE_nodeName.setString(m_armGraph.getCurrentNode().getNodeName());
        }

        if (m_armGraph.getCurrentNode().getNextNode() == null) {
            m_GE_nextNodeName.setString("<none>");
        } else {
            m_GE_nextNodeName.setString(m_armGraph.getCurrentNode().getNextNode().getNodeName());
        }
    }

    public Command getTransition_B() {
        return this.runOnce(() -> m_armGraph.setTargetNode(B));
    }

    final static public Command getArmCommand(Angle armBaseAngle, Distance armLength, Angle wristAngle) {
        return new ParallelCommandGroup(RobotContainer.m_armBase.getSetGoalCommand(armBaseAngle.in(Degree)),
                RobotContainer.m_armExtension.getSetGoalCommand(armLength.in(Meter)),
                RobotContainer.m_wrist.getSetGoalCommand(wristAngle.in(Degree)));
    }

}
