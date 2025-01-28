// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ArmControl;

import edu.wpi.first.networktables.GenericEntry;
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

    private GraphCommandNode Start_1, Start, Node_Alignment, Back_L4, Back_L3, Back_L2, Back_L1, Back_R4, Back_R3,
            Back_R2,
            Back_R1, Front_TL, Front_TR, FrontHP_PickUP_Coral, Front_PickUP_Coral;

    /** Creates a new ArmController. */
    public ArmController() {
        m_GE_nodeName = Shuffleboard.getTab("ArmController").add("ArmController.CurrentNode", "<none>").getEntry();
        m_GE_nextNodeName = Shuffleboard.getTab("ArmController").add("ArmController.NextNode", "<none>").getEntry();

    }

    private void initialize() {
        Start_1 = m_armGraph.new GraphCommandNode("Start_1",
                ArmController.getArmCommand(Degrees.of(0),
                        Meters.of(0),
                        Degrees.of(0)),
                null,
                null);
        Start = m_armGraph.new GraphCommandNode("Start",
                ArmController.getArmCommand(Degrees.of(0),
                        Meters.of(0),
                        Degrees.of(0)),
                null,
                null);
        Node_Alignment = m_armGraph.new GraphCommandNode("Node_Alignment",
                ArmController.getArmCommand(Degrees.of(90),
                        Meters.of(0),
                        Degrees.of(-90))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0)))
                        .andThen(RobotContainer.m_armExtension.getWaitUntilErrorLessThanCmd(Meters.of(0.5))),
                null,
                new PrintCommand("Node_Alignment is done"));
        Back_L4 = m_armGraph.new GraphCommandNode("Back_L4",
                ArmController.getArmCommand(Degrees.of(90),
                        Meters.of(3),
                        Degrees.of(0))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(45.0))),

                null,
                new PrintCommand("Back_L4 is done"));
        Back_L3 = m_armGraph.new GraphCommandNode("Back_L3",
                ArmController.getArmCommand(Degrees.of(90),
                        Meters.of(2.25),
                        Degrees.of(0))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(45.0))),
                null,
                new PrintCommand("Back_L3 is done"));
        Back_L2 = m_armGraph.new GraphCommandNode("Back_L2",
                ArmController.getArmCommand(Degrees.of(90),
                        Meters.of(1.5),
                        Degrees.of(0))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(45.0))),
                null,
                new PrintCommand("Back_L2 is done"));
        Back_L1 = m_armGraph.new GraphCommandNode("Back_L1",
                ArmController.getArmCommand(Degrees.of(90),
                        Meters.of(.75),
                        Degrees.of(0))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(45.0))),
                null,
                new PrintCommand("Back_L1 is done"));

        Back_R4 = m_armGraph.new GraphCommandNode("Back_R4",
                ArmController.getArmCommand(Degrees.of(90),
                        Meters.of(3),
                        Degrees.of(0))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(45.0))),
                null,
                new PrintCommand("Back_R4 is done"));
        Back_R3 = m_armGraph.new GraphCommandNode("Back_R3",
                ArmController.getArmCommand(Degrees.of(90),
                        Meters.of(2.25),
                        Degrees.of(0))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(45.0))),
                null,
                new PrintCommand("Back_R3 is done"));
        Back_R2 = m_armGraph.new GraphCommandNode("Back_R2",
                ArmController.getArmCommand(Degrees.of(90),
                        Meters.of(1.5),
                        Degrees.of(0))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(45.0))),
                null,
                new PrintCommand("Back_R2 is done"));
        Back_R1 = m_armGraph.new GraphCommandNode("Back_R1",
                ArmController.getArmCommand(Degrees.of(90),
                        Meters.of(.75),
                        Degrees.of(0))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(45.0))),
                null,
                new PrintCommand("Back_R1 is done"));

        Front_TL = m_armGraph.new GraphCommandNode("Front_TL",
                ArmController.getArmCommand(Degrees.of(25),
                        Meters.of(1.5),
                        Degrees.of(-90))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("Front_TL is done"));

        Front_TR = m_armGraph.new GraphCommandNode("Front_TR",
                ArmController.getArmCommand(Degrees.of(25),
                        Meters.of(1.5),
                        Degrees.of(-90))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("Front_TR is done"));

        FrontHP_PickUP_Coral = m_armGraph.new GraphCommandNode("FrontHP_PickUP_Coral",
                ArmController.getArmCommand(Degrees.of(-25),
                        Meters.of(2),
                        Degrees.of(0))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("FrontHP_PickUP_Coral"));

        Front_PickUP_Coral = m_armGraph.new GraphCommandNode("Front_PickUP_Coral",
                ArmController.getArmCommand(Degrees.of(-25),
                        Meters.of(2),
                        Degrees.of(0))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("Front_PickUP_Coral"));

        Start.AddNode(Node_Alignment, 1);
        Node_Alignment.AddNode(Back_L4, 1);
        Node_Alignment.AddNode(Back_L3, 1);
        Node_Alignment.AddNode(Back_L2, 1);
        Node_Alignment.AddNode(Back_L1, 1);
        Node_Alignment.AddNode(Back_R4, 1);
        Node_Alignment.AddNode(Back_R3, 1);
        Node_Alignment.AddNode(Back_R2, 1);
        Node_Alignment.AddNode(Back_R1, 1);
        Node_Alignment.AddNode(Front_TL, 1);
        Node_Alignment.AddNode(Front_TR, 1);

        m_armGraph.setGraphRootNode(Start);
        m_armGraph.setCurrentNode(Start);
        m_armGraph.initialize();
        m_armGraph.addRequirements(this);
        m_armGraph.setTargetNode(Start);
        this.setDefaultCommand(m_armGraph);

        Shuffleboard.getTab("ArmController").add(this.getTransition_Start());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Node_Alignment());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Back_L4());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Back_L3());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Back_L2());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Back_L1());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Back_R4());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Back_R3());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Back_R2());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Back_R1());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Front_TL());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Front_TR());
        Shuffleboard.getTab("ArmController").add(this.getTransition_FrontHP_PickUP_Coral());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Front_PickUP_Coral());

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

    public Command getTransition_Start() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(Start));
        c.setName("Start");
        return c;
    }

    public Command getTransition_Node_Alignment() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(Node_Alignment));
        c.setName("Note_Alignment");
        return c;
    }

    public Command getTransition_Back_L4() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(Back_L4));
        c.setName("Back_L4");
        return c;
    }

    public Command getTransition_Back_L3() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(Back_L3));
        c.setName("Back_L3");
        return c;
    }

    public Command getTransition_Back_L2() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(Back_L2));
        c.setName("Back_L2");
        return c;
    }

    public Command getTransition_Back_L1() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(Back_L1));
        c.setName("Back_L1");
        return c;
    }

    public Command getTransition_Back_R4() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(Back_R4));
        c.setName("Back_R4");
        return c;
    }

    public Command getTransition_Back_R3() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(Back_R3));
        c.setName("Back_R3");
        return c;
    }

    public Command getTransition_Back_R2() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(Back_R2));
        c.setName("Back_R2");
        return c;
    }

    public Command getTransition_Back_R1() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(Back_R1));
        c.setName("Back_R1");
        return c;
    }

    public Command getTransition_Front_TL() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(Front_TL));
        c.setName("Front_TL");
        return c;
    }

    public Command getTransition_Front_TR() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(Front_TR));
        c.setName("Front_TR");
        return c;
    }

    public Command getTransition_FrontHP_PickUP_Coral() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(FrontHP_PickUP_Coral));
        c.setName("FrontHP_PickUP_Coral");
        return c;
    }

    public Command getTransition_Front_PickUP_Coral() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(Front_PickUP_Coral));
        c.setName("Front_PickUP_Coral");
        return c;
    }

    final static public Command getArmCommand(Angle armBaseAngle, Distance armLength, Angle wristAngle) {
        return new ParallelCommandGroup(RobotContainer.m_armBase.getSetGoalCommand(armBaseAngle),
                RobotContainer.m_armExtension.getSetGoalCommand(armLength),
                RobotContainer.m_wrist.getSetGoalCommand(wristAngle));
    }

}
