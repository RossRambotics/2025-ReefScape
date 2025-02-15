// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ArmControl;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
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

    private GraphCommandNode BackScore_L4, BackScore_L3, BackScore_L2, BackScore_L1, Back_L4, Back_L3, Back_L2, Back_L1,
            BackAligment, Carry, FrontAligment, FrontScore_L3, Front_L3, S1, Start, FrontScore_L2, FrontScore_L1,
            Front_L2, Front_L1, HumanPlayerCoral, GroundCoral, GroundAlgae, ProcessorAlgae, NetAlgae, RemoveAlgaeHigh,
            RemoveAlgaeLow, Climb;

    /** Creates a new ArmController. */
    public ArmController() {
        m_GE_nodeName = Shuffleboard.getTab("ArmController").add("ArmController.CurrentNode", "<none>").getEntry();
        m_GE_nextNodeName = Shuffleboard.getTab("ArmController").add("ArmController.NextNode", "<none>").getEntry();

    }

    private void initialize() {

        Start = m_armGraph.new GraphCommandNode("Start",
                ArmController.getArmCommand(Degrees.of(0),
                        Meters.of(0),
                        Degrees.of(0)),
                null,
                null);
        BackAligment = m_armGraph.new GraphCommandNode("BackAlignment",
                ArmController.getArmCommand(Degrees.of(90),
                        Meters.of(-77.3),
                        Degrees.of(0))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0)))
                        .andThen(RobotContainer.m_armExtension.getWaitUntilErrorLessThanCmd(Meters.of(0.5))),
                null,
                new PrintCommand("BackAlignment is done"));
        Back_L4 = m_armGraph.new GraphCommandNode("Back_L4",
                ArmController.getArmCommand(Degrees.of(90),
                        Meters.of(0),
                        Degrees.of(0))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(45.0))),
                null,
                new PrintCommand("Back_L4 is done"));
        Back_L3 = m_armGraph.new GraphCommandNode("Back_L3",
                ArmController.getArmCommand(Degrees.of(85),
                        Meters.of(-60),
                        Degrees.of(0))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(45.0))),
                null,
                new PrintCommand("Back_L3 is done"));
        Back_L2 = m_armGraph.new GraphCommandNode("Back_L2",
                ArmController.getArmCommand(Degrees.of(85),
                        Meters.of(-77.3),
                        Degrees.of(0))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(45.0))),
                null,
                new PrintCommand("Back_L2 is done"));
        Back_L1 = m_armGraph.new GraphCommandNode("Back_L1",
                ArmController.getArmCommand(Degrees.of(85),
                        Meters.of(-77.3),
                        Degrees.of(0))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(45.0))),
                null,
                new PrintCommand("Back_L1 is done"));

        Front_L2 = m_armGraph.new GraphCommandNode("Front_L2",
                ArmController.getArmCommand(Degrees.of(35),
                        Meters.of(-66),
                        Degrees.of(60))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("Front_L2 is done"));

        Front_L1 = m_armGraph.new GraphCommandNode("Front_L1",
                ArmController.getArmCommand(Degrees.of(10),
                        Meters.of(-77.3),
                        Degrees.of(-20))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("Front_L1 is done"));

        HumanPlayerCoral = m_armGraph.new GraphCommandNode("HumanPlayerCoral",
                ArmController.getArmCommand(Degrees.of(18),
                        Meters.of(-77),
                        Degrees.of(-23))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("HumanPlayerCoral"));

        GroundCoral = m_armGraph.new GraphCommandNode("GroundCoral",
                ArmController.getArmCommand(Degrees.of(-20),
                        Meters.of(-65),
                        Degrees.of(-20))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("GroundCoral"));

        BackScore_L4 = m_armGraph.new GraphCommandNode("BackScore_L4",
                ArmController.getArmCommand(Degrees.of(90),
                        Meters.of(0),
                        Degrees.of(-115))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("BackScore_L4"));

        BackScore_L3 = m_armGraph.new GraphCommandNode("BackScore_L3",
                ArmController.getArmCommand(Degrees.of(90),
                        Meters.of(-60),
                        Degrees.of(-115))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("BackScore_L3"));

        BackScore_L2 = m_armGraph.new GraphCommandNode("BackScore_L2",
                ArmController.getArmCommand(Degrees.of(90),
                        Meters.of(-77.3),
                        Degrees.of(-136))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("BackScore_L2"));

        BackScore_L1 = m_armGraph.new GraphCommandNode("BackScore_L1",
                ArmController.getArmCommand(Degrees.of(90),
                        Meters.of(-77.3),
                        Degrees.of(-140))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("BackScore_L1"));

        Climb = m_armGraph.new GraphCommandNode("Climb",
                ArmController.getArmCommand(Degrees.of(90),
                        Meters.of(-77.3),
                        Degrees.of(-140))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("Climb"));

        Carry = m_armGraph.new GraphCommandNode("Carry",
                ArmController.getArmCommand(Degrees.of(20),
                        Meters.of(-77.3),
                        Degrees.of(-130))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("Carry"));
        FrontAligment = m_armGraph.new GraphCommandNode("FrontAligment",
                ArmController.getArmCommand(Degrees.of(0),
                        Meters.of(-77.3),
                        Degrees.of(-90))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("FrontAligment"));

        S1 = m_armGraph.new GraphCommandNode("S1",
                ArmController.getArmCommand(Degrees.of(0),
                        Meters.of(0),
                        Degrees.of(0)),
                null,
                null);
        new PrintCommand("S1");
        FrontScore_L2 = m_armGraph.new GraphCommandNode("FrontScore_L2",
                ArmController.getArmCommand(Degrees.of(35),
                        Meters.of(-66),
                        Degrees.of(60))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("FrontScore_L2"));
        FrontScore_L1 = m_armGraph.new GraphCommandNode("FrontScore_L1",
                ArmController.getArmCommand(Degrees.of(10),
                        Meters.of(-77.3),
                        Degrees.of(20))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("FrontScore_L1"));
        Front_L3 = m_armGraph.new GraphCommandNode("Front_L3",
                ArmController.getArmCommand(Degrees.of(35),
                        Meters.of(-66),
                        Degrees.of(60))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("Front_L3"));
        FrontScore_L3 = m_armGraph.new GraphCommandNode("FrontScore_L3",
                ArmController.getArmCommand(Degrees.of(10),
                        Meters.of(-77.3),
                        Degrees.of(20))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("FrontScore_L3"));
        GroundAlgae = m_armGraph.new GraphCommandNode("GroundAlgae",
                ArmController.getArmCommand(Degrees.of(-11),
                        Meters.of(0.8),
                        Degrees.of(-85))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("GroundAlgae"));
        ProcessorAlgae = m_armGraph.new GraphCommandNode("ProcessorAlgae",
                ArmController.getArmCommand(Degrees.of(0),
                        Meters.of(0.5),
                        Degrees.of(-90))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("ProcessorAlgae"));
        NetAlgae = m_armGraph.new GraphCommandNode("NetAlgae",
                ArmController.getArmCommand(Degrees.of(95),
                        Meters.of(3),
                        Degrees.of(-95))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("NetAlgae"));
        RemoveAlgaeHigh = m_armGraph.new GraphCommandNode("RemoveAlgaeHigh",
                ArmController.getArmCommand(Degrees.of(95),
                        Meters.of(3),
                        Degrees.of(-95))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("RemoveAlgaeHigh"));
        RemoveAlgaeLow = m_armGraph.new GraphCommandNode("RemoveAlgaeLow",
                ArmController.getArmCommand(Degrees.of(95),
                        Meters.of(3),
                        Degrees.of(-95))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("RemoveAlgaeLow"));

        S1.AddNode(Start, 1, true);
        Start.AddNode(BackAligment, 1);
        BackAligment.AddNode(Back_L4, 1);
        BackAligment.AddNode(Back_L3, 1);
        BackAligment.AddNode(Back_L2, 1);
        BackAligment.AddNode(Back_L1, 1);
        BackAligment.AddNode(Front_L2, 1);
        BackAligment.AddNode(Front_L1, 1);
        BackAligment.AddNode(Climb, 1);
        Back_L4.AddNode(BackScore_L4, 1);
        Back_L3.AddNode(BackScore_L3, 1);
        Back_L2.AddNode(BackScore_L2, 1);
        Back_L1.AddNode(BackScore_L1, 1);
        Front_L3.AddNode(FrontScore_L3, 1);
        Front_L2.AddNode(FrontScore_L2, 1);
        Front_L1.AddNode(FrontScore_L1, 1);
        BackAligment.AddNode(Carry, 1);
        BackAligment.AddNode(FrontAligment, 1);
        Carry.AddNode(FrontAligment, 1);
        Start.AddNode(FrontAligment, 1);
        FrontAligment.AddNode(Front_L3, 1);
        FrontAligment.AddNode(Front_L2, 1);
        FrontAligment.AddNode(Front_L1, 1);
        FrontAligment.AddNode(HumanPlayerCoral, 1);
        FrontAligment.AddNode(GroundAlgae, 1);
        FrontAligment.AddNode(GroundCoral, 1);
        FrontAligment.AddNode(ProcessorAlgae, 1);
        FrontAligment.AddNode(NetAlgae, 1);
        FrontAligment.AddNode(RemoveAlgaeHigh, 1);
        FrontAligment.AddNode(RemoveAlgaeLow, 1);

        m_armGraph.setGraphRootNode(Start);
        m_armGraph.setCurrentNode(Start);
        m_armGraph.initialize();
        m_armGraph.addRequirements(this);
        m_armGraph.setTargetNode(Start);
        this.setDefaultCommand(m_armGraph);

        Shuffleboard.getTab("ArmController").add(this.getTransition_Start());
        Shuffleboard.getTab("ArmController").add(this.getTransition_BackAligment());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Back_L4());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Back_L3());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Back_L2());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Back_L1());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Front_L2());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Front_L3());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Front_L1());
        Shuffleboard.getTab("ArmController").add(this.getTransition_HumanPlayerCoral());
        Shuffleboard.getTab("ArmController").add(this.getTransition_GroundCoral());
        Shuffleboard.getTab("ArmController").add(this.getTransition_BackScore_L4());
        Shuffleboard.getTab("ArmController").add(this.getTransition_BackScore_L3());
        Shuffleboard.getTab("ArmController").add(this.getTransition_BackScore_L2());
        Shuffleboard.getTab("ArmController").add(this.getTransition_BackScore_L1());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Carry());
        Shuffleboard.getTab("ArmController").add(this.getTransition_FrontAligment());
        Shuffleboard.getTab("ArmController").add(this.getTransition_S1());
        Shuffleboard.getTab("ArmController").add(this.getTransition_FrontScore_L3());
        Shuffleboard.getTab("ArmController").add(this.getTransition_FrontScore_L2());
        Shuffleboard.getTab("ArmController").add(this.getTransition_FrontScore_L1());
        Shuffleboard.getTab("ArmController").add(this.getTransition_GroundAlgae());
        Shuffleboard.getTab("ArmController").add(this.getTransition_ProcessorAlgae());
        Shuffleboard.getTab("ArmController").add(this.getTransition_NetAlgae());
        Shuffleboard.getTab("ArmController").add(this.getTransition_RemoveAlgaeHigh());
        Shuffleboard.getTab("ArmController").add(this.getTransition_RemoveAlgaeLow());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Climb());
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

    public Command getTransition_BackAligment() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(BackAligment));
        c.setName("BackAligment");
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

    public Command getTransition_Front_L2() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(Front_L2));
        c.setName("Front_L2");
        return c;
    }

    public Command getTransition_Front_L3() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(Front_L3));
        c.setName("Front_L3");
        return c;
    }

    public Command getTransition_Front_L1() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(Front_L1));
        c.setName("Front_L1");
        return c;
    }

    public Command getTransition_HumanPlayerCoral() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(HumanPlayerCoral));
        c.setName("HumanPlayerCoral");
        return c;
    }

    public Command getTransition_GroundCoral() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(GroundCoral));
        c.setName("GroundCoral");
        return c;
    }

    public Command getTransition_BackScore_L4() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(BackScore_L4));
        c.setName("BackScore_L4");
        return c;
    }

    public Command getTransition_BackScore_L3() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(BackScore_L3));
        c.setName("BackScore_L3");
        return c;
    }

    public Command getTransition_BackScore_L2() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(BackScore_L2));
        c.setName("BackScore_L2");
        return c;
    }

    public Command getTransition_BackScore_L1() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(BackScore_L1));
        c.setName("BackScore_L1");
        return c;
    }

    public Command getTransition_Climb() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(Climb));
        c.setName("Climb");
        return c;
    }

    public Command getTransition_Carry() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(Carry));
        c.setName("Carry");
        return c;
    }

    public Command getTransition_FrontAligment() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(FrontAligment));
        c.setName("FrontAligment");
        return c;
    }

    public Command getTransition_S1() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(S1));
        c.setName("S1");
        return c;
    }

    public Command getTransition_FrontScore_L2() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(FrontScore_L2));
        c.setName("FrontScore_L2");
        return c;
    }

    public Command getTransition_FrontScore_L3() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(FrontScore_L3));
        c.setName("FrontScore_L3");
        return c;
    }

    public Command getTransition_FrontScore_L1() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(FrontScore_L1));
        c.setName("FrontScore_L1");
        return c;
    }

    public Command getTransition_GroundAlgae() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(GroundAlgae));
        c.setName("GroundAlgae");
        return c;
    }

    public Command getTransition_ProcessorAlgae() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(ProcessorAlgae));
        c.setName("ProcessorAlgae");
        return c;
    }

    public Command getTransition_NetAlgae() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(NetAlgae));
        c.setName("NetAlgae");
        return c;
    }

    public Command getTransition_RemoveAlgaeHigh() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(RemoveAlgaeHigh));
        c.setName("RemoveAlgaeHigh");
        return c;
    }

    public Command getTransition_RemoveAlgaeLow() {
        Command c = this.runOnce(() -> m_armGraph.setTargetNode(RemoveAlgaeLow));
        c.setName("RemoveAlgaeLow");
        return c;
    }

    final static public Command getArmCommand(Angle armBaseAngle, Distance armLength, Angle wristAngle) {
        return new ParallelCommandGroup(RobotContainer.m_armBase.getSetGoalCommand(armBaseAngle),
                RobotContainer.m_armExtension.getSetGoalCommand(armLength),
                RobotContainer.m_wrist.getSetGoalCommand(wristAngle));
    }

}
