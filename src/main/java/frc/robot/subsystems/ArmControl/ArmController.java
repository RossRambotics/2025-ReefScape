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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Targeting.LineUpOrientation;
import frc.util.RandomExecutionLimiter;
import frc.util.GraphCommand.GraphCommand;
import frc.util.GraphCommand.GraphCommand.GraphCommandNode;

public class ArmController extends SubsystemBase {
    final private GraphCommand m_armGraph = new GraphCommand();
    private boolean m_isFirstTime = true;
    private GenericEntry m_GE_nodeName;
    private GenericEntry m_GE_nextNodeName;
    private RandomExecutionLimiter m_executionLimiter = new RandomExecutionLimiter();

    private GraphCommandNode BackScore_L4, BackScore_L3, Back_L4_Up, Back_L4_Middle, Back_L3,
            BackAligment, Carry, FrontAligment, HPCarry, S1, Start, FrontScore_L2, FrontScore_L1,
            Front_L2, Front_L1, HumanPlayerCoral, GroundCoral, GroundAlgae, ProcessorAlgae, NetAlgae, RemoveAlgaeHigh,
            RemoveAlgaeLow, Climb, ClimbReady, ClimbLockOn, NetAlgaeAlignment1, NetAlgaeAlignment2, AlgaeCarry, L4_Down;

    /** Creates a new ArmController. */
    public ArmController() {
        m_GE_nodeName = Shuffleboard.getTab("ArmController").add("ArmController.CurrentNode", "<none>").getEntry();
        m_GE_nextNodeName = Shuffleboard.getTab("ArmController").add("ArmController.NextNode", "<none>").getEntry();

    }

    private void initialize() {

        Start = m_armGraph.new GraphCommandNode("Start",
                ArmController.getArmCommand(Degrees.of(0),
                        Meters.of(-78),
                        Degrees.of(-130)),
                null,
                null);
        BackAligment = m_armGraph.new GraphCommandNode("BackAlignment",
                ArmController.getArmCommand(Degrees.of(85),
                        Meters.of(-70),
                        Degrees.of(-115))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0)))
                        .andThen(RobotContainer.m_armExtension.getWaitUntilErrorLessThanCmd(Meters.of(25.0))),
                null,
                new PrintCommand("BackAlignment is done"));
        Back_L4_Up = m_armGraph.new GraphCommandNode("Back_L4_Up",
                ArmController.getArmCommand(Degrees.of(82),
                        Meters.of(5),
                        Degrees.of(-116))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(25.0)))
                        .andThen(RobotContainer.m_wrist.getWaitUntilErrorLessThanCmd(Degrees.of(5.0))),
                null,
                new PrintCommand("Back_L4_Up is done"));
        Back_L4_Middle = m_armGraph.new GraphCommandNode("Back_L4_Middle",
                ArmController.getArmCommand(Degrees.of(82),
                        Meters.of(5),
                        Degrees.of(-116)),
                // .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(25.0)))
                // .andThen(RobotContainer.m_wrist.getWaitUntilErrorLessThanCmd(Degrees.of(5.0))),
                null,
                new PrintCommand("Back_L4_Middle is done"));
        Back_L3 = m_armGraph.new GraphCommandNode("Back_L3",
                ArmController.getArmCommand(Degrees.of(85),
                        Meters.of(-74.49),
                        Degrees.of(-115))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(45.0))),
                null,
                new PrintCommand("Back_L3 is done"));

        Front_L2 = m_armGraph.new GraphCommandNode("Front_L2",
                ArmController.getArmCommand(Degrees.of(33.76),
                        Meters.of(-59.2),
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
                ArmController.getArmCommand(Degrees.of(30.08),
                        Meters.of(-77),
                        Degrees.of(-23))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("HumanPlayerCoral"));

        GroundCoral = m_armGraph.new GraphCommandNode("GroundCoral",
                ArmController.getArmCommand(Degrees.of(-23.89),
                        Meters.of(-67.95),
                        Degrees.of(-20.73))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("GroundCoral"));

        BackScore_L4 = m_armGraph.new GraphCommandNode("BackScore_L4",
                ArmController.getArmCommand(Degrees.of(91.42),
                        Meters.of(5),
                        Degrees.of(-149.62))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(25.0)))
                        .andThen(RobotContainer.m_wrist.getWaitUntilErrorLessThanCmd(Degrees.of(5.0))),
                null,
                new PrintCommand("BackScore_L4"));

        BackScore_L3 = m_armGraph.new GraphCommandNode("BackScore_L3",
                ArmController.getArmCommand(Degrees.of(89.26),
                        Meters.of(-77.3),
                        Degrees.of(-122.07))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("BackScore_L3"));

        Climb = m_armGraph.new GraphCommandNode("Climb",
                ArmController.getArmCommand(Degrees.of(-35),
                        Meters.of(-77.3),
                        Degrees.of(-66))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("Climb"));

        ClimbLockOn = m_armGraph.new GraphCommandNode("ClimbLockOn",
                ArmController.getArmCommand(Degrees.of(65),
                        Meters.of(-77.3),
                        Degrees.of(10))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("ClimbLockOn"));

        ClimbReady = m_armGraph.new GraphCommandNode("ClimbReady",
                ArmController.getArmCommand(Degrees.of(45),
                        Meters.of(-77.3),
                        Degrees.of(10))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("ClimbReady"));

        Carry = m_armGraph.new GraphCommandNode("Carry",
                ArmController.getArmCommand(Degrees.of(-15),
                        Meters.of(-77.3),
                        Degrees.of(-140))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("Carry"));

        L4_Down = m_armGraph.new GraphCommandNode("L4_Down",
                ArmController.getArmCommand(Degrees.of(50),
                        Meters.of(-50),
                        Degrees.of(-115))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0)))
                        .andThen(RobotContainer.m_armExtension.getWaitUntilErrorLessThanCmd(Meters.of(10.0))),
                null,
                new PrintCommand("L4_Down"));
        HPCarry = m_armGraph.new GraphCommandNode("HPCarry",
                ArmController.getArmCommand(Degrees.of(-20),
                        Meters.of(-77.3),
                        Degrees.of(-130))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("HPCarry"));
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
                ArmController.getArmCommand(Degrees.of(33.76),
                        Meters.of(-59.2),
                        Degrees.of(60.0))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("FrontScore_L2"));
        FrontScore_L1 = m_armGraph.new GraphCommandNode("FrontScore_L1",
                ArmController.getArmCommand(Degrees.of(10),
                        Meters.of(-77.3),
                        Degrees.of(-20))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("FrontScore_L1"));

        GroundAlgae = m_armGraph.new GraphCommandNode("GroundAlgae",
                ArmController.getArmCommand(Degrees.of(-24.5),
                        Meters.of(-68.5),
                        Degrees.of(-5.25))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("GroundAlgae"));
        ProcessorAlgae = m_armGraph.new GraphCommandNode("ProcessorAlgae",
                ArmController.getArmCommand(Degrees.of(5.02),
                        Meters.of(-63.87),
                        Degrees.of(58.12))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("ProcessorAlgae"));
        NetAlgae = m_armGraph.new GraphCommandNode("NetAlgae",
                ArmController.getArmCommand(Degrees.of(85),
                        Meters.of(6),
                        Degrees.of(45))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("NetAlgae"));
        NetAlgaeAlignment1 = m_armGraph.new GraphCommandNode("NetAlgaeAlignment1",
                ArmController.getArmCommand(Degrees.of(45),
                        Meters.of(-30),
                        Degrees.of(-45))
                        .andThen(new WaitCommand(1.0)),
                null,
                new PrintCommand("NetAlgaeAlignment1"));
        NetAlgaeAlignment2 = m_armGraph.new GraphCommandNode("NetAlgaeAlignment2",
                ArmController.getArmCommand(Degrees.of(85),
                        Meters.of(-60),
                        Degrees.of(-15))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(
                                20.0))),
                null,
                new PrintCommand("NetAlgaeAlignment2"));
        RemoveAlgaeHigh = m_armGraph.new GraphCommandNode("RemoveAlgaeHigh",
                ArmController.getArmCommand(Degrees.of(42.96),
                        Meters.of(-55.77),
                        Degrees.of(9.03))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("RemoveAlgaeHigh"));
        RemoveAlgaeLow = m_armGraph.new GraphCommandNode("RemoveAlgaeLow",
                ArmController.getArmCommand(Degrees.of(17.54),
                        Meters.of(-77.3),
                        Degrees.of(-17.47))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("RemoveAlgaeLow"));
        AlgaeCarry = m_armGraph.new GraphCommandNode("AlgaeCarry",
                ArmController.getArmCommand(Degrees.of(-12.4),
                        Meters.of(-77.3),
                        Degrees.of(-84.62))
                        .andThen(RobotContainer.m_armBase.getWaitUntilErrorLessThanCmd(Degrees.of(70.0))),
                null,
                new PrintCommand("AlgaeCarry"));

        S1.AddNode(Start, 1, true);
        Start.AddNode(BackAligment, 1);
        // Carry.AddNode(Back_L4, 1);
        BackAligment.AddNode(Back_L3, 1);
        BackAligment.AddNode(Front_L2, 1);
        BackAligment.AddNode(Front_L1, 1);
        Carry.AddNode(ClimbReady, 1);
        Carry.AddNode(Back_L4_Up, 1.1, true);
        ClimbReady.AddNode(ClimbLockOn, 1);
        ClimbReady.setNextNode(ClimbLockOn);
        ClimbLockOn.AddNode(Climb, 1);
        ClimbLockOn.setNextNode(Climb);
        Back_L4_Up.AddNode(BackScore_L4, 1);
        Back_L4_Up.AddNode(Back_L3, 1);
        Back_L4_Up.AddNode(L4_Down, 1);
        BackScore_L4.AddNode(Back_L4_Middle, 0.9, true);
        Back_L4_Middle.AddNode(BackAligment, 0.9, true);
        Back_L4_Middle.AddNode(L4_Down, 0.9, true);
        L4_Down.AddNode(GroundCoral, 1);
        BackAligment.AddNode(Carry, 1);
        Back_L4_Up.setNextNode(BackScore_L4);
        BackScore_L4.setNextNode(Back_L4_Up);
        Back_L3.AddNode(BackScore_L3, 1);
        Back_L3.setNextNode(BackScore_L3);
        Back_L3.AddNode(Back_L4_Up, 1.1);
        BackScore_L3.setNextNode(Back_L3);
        Front_L2.AddNode(FrontScore_L2, 1);
        Front_L2.setNextNode(FrontScore_L2);
        FrontScore_L2.setNextNode(Front_L2);
        Front_L1.AddNode(FrontScore_L1, 1);
        Front_L1.setNextNode(FrontScore_L1);
        FrontScore_L1.setNextNode(Front_L1);
        BackAligment.AddNode(Carry, 1);
        BackAligment.AddNode(FrontAligment, 1);
        Carry.AddNode(FrontAligment, 1);
        Carry.AddNode(ProcessorAlgae, 1);
        Start.AddNode(FrontAligment, 1);
        FrontAligment.AddNode(Front_L2, 1);
        FrontAligment.AddNode(Front_L1, 1);
        FrontAligment.AddNode(HPCarry, 1);
        HPCarry.AddNode(HumanPlayerCoral, 1);
        HPCarry.setNextNode(HumanPlayerCoral);
        FrontAligment.AddNode(GroundAlgae, 1);
        FrontAligment.AddNode(GroundCoral, 1);
        RemoveAlgaeHigh.AddNode(ProcessorAlgae, 1);
        RemoveAlgaeHigh.setNextNode(ProcessorAlgae);
        RemoveAlgaeLow.AddNode(ProcessorAlgae, 1);
        RemoveAlgaeLow.setNextNode(ProcessorAlgae);
        ProcessorAlgae.AddNode(NetAlgaeAlignment2, 1);
        ProcessorAlgae.AddNode(ProcessorAlgae, 1);
        NetAlgae.AddNode(BackAligment, 1);
        FrontAligment.AddNode(RemoveAlgaeHigh, 1);
        FrontAligment.AddNode(RemoveAlgaeLow, 1);
        NetAlgaeAlignment2.AddNode(NetAlgae, 1);
        GroundAlgae.AddNode(ProcessorAlgae, 1);
        GroundAlgae.setNextNode(ProcessorAlgae);
        // BackAligment.AddNode(BackAlignmentBack, 0);

        m_armGraph.setGraphRootNode(S1);
        m_armGraph.setCurrentNode(S1);
        m_armGraph.initialize();
        m_armGraph.addRequirements(this);
        m_armGraph.setTargetNode(S1);
        this.setDefaultCommand(m_armGraph);

        Shuffleboard.getTab("ArmController").add(this.getTransition_Start());
        Shuffleboard.getTab("ArmController").add(this.getTransition_BackAligment());
        // Shuffleboard.getTab("ArmController").add(this.getTransition_BackAligmentBack());
        Shuffleboard.getTab("ArmController").add(this.getTransition_L4_Down());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Back_L4());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Back_L3());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Front_L2());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Front_L1());
        Shuffleboard.getTab("ArmController").add(this.getTransition_HumanPlayerCoral());
        Shuffleboard.getTab("ArmController").add(this.getTransition_GroundCoral());
        Shuffleboard.getTab("ArmController").add(this.getTransition_BackScore_L4());
        Shuffleboard.getTab("ArmController").add(this.getTransition_BackScore_L3());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Carry());
        Shuffleboard.getTab("ArmController").add(this.getTransition_FrontAligment());
        Shuffleboard.getTab("ArmController").add(this.getTransition_S1());
        Shuffleboard.getTab("ArmController").add(this.getTransition_FrontScore_L2());
        Shuffleboard.getTab("ArmController").add(this.getTransition_FrontScore_L1());
        Shuffleboard.getTab("ArmController").add(this.getTransition_GroundAlgae());
        Shuffleboard.getTab("ArmController").add(this.getTransition_ProcessorAlgae());
        Shuffleboard.getTab("ArmController").add(this.getTransition_NetAlgae());
        Shuffleboard.getTab("ArmController").add(this.getTransition_RemoveAlgaeHigh());
        Shuffleboard.getTab("ArmController").add(this.getTransition_RemoveAlgaeLow());
        Shuffleboard.getTab("ArmController").add(this.getTransition_Climb());
        Shuffleboard.getTab("ArmController").add(this.getTransition_ClimbReady());
        Shuffleboard.getTab("ArmController").add(this.getTransition_ClimbLockOn());
        Shuffleboard.getTab("ArmController").add(this.getNextNodeCmd());
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

        // Check if we should execute this cycle
        if (!m_executionLimiter.shouldExecute()) {
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

    public boolean isTransitioning() {
        return m_armGraph.isTransitioning();
    }

    public Command getNextNodeCmd() {
        Command c = Commands.runOnce(() -> advanceGraph());
        c.setName("NextNode");
        return c;
    }

    // private boolean m_isWaitingForDriver = false;

    // private boolean isWaitingForDriver() {
    // return m_isWaitingForDriver;
    // }

    // private Command getWaitForDriverCmd() {
    // Command c = this.runOnce(() -> m_isWaitingForDriver = false)
    // .andThen(new WaitUntilCommand(this::isWaitingForDriver));
    // c.setName("WaitForDriver");
    // return c;
    // }

    private void advanceGraph() {
        m_armGraph.setTargetNode(m_armGraph.getCurrentNode().getNextNode());
    }

    public Command getTransition_Start() {
        Command c = Commands.runOnce(() -> m_armGraph.setTargetNode(Start));
        c.setName("Start");
        return c;
    }

    public Command getTransition_BackAligment() {
        Command c = Commands.runOnce(() -> m_armGraph.setTargetNode(BackAligment));
        c.setName("BackAligment");
        return c;
    }

    public Command getAutoTransition_Back_L4() {
        Command c = Commands.runOnce(() -> m_armGraph.setTargetNode(Back_L4_Up));
        c.setName("Back_L4");
        return c;
    }

    public Command getTransition_Back_L4() {
        Command c = Commands.runOnce(() -> doTransition_Back_L4());
        c.setName("Back_L4");
        return c;
    }

    public Command getTransition_L4_Down() {
        Command c = Commands.runOnce(() -> m_armGraph.setTargetNode(L4_Down));
        c.setName("L4_Down");
        return c;
    }

    public void doTransition_Back_L4() {
        Carry.setNextNode(BackScore_L4);
        Back_L3.setNextNode(Back_L4_Up);
        BackScore_L4.setNextNode(Back_L4_Up);
        Back_L4_Up.setNextNode(BackScore_L4);
        RobotContainer.m_targeting.setLineUpOrientation(LineUpOrientation.kBackward);
    }

    public Command getTransition_Back_L3() {
        Command c = Commands.runOnce(() -> doTransition_Back_L3());
        c.setName("Back_L3");
        return c;
    }

    public void doTransition_Back_L3() {
        Carry.setNextNode(BackScore_L3);
        Back_L4_Up.setNextNode(Back_L3);
        BackScore_L3.setNextNode(Back_L3);
        Back_L3.setNextNode(BackScore_L3);
        RobotContainer.m_targeting.setLineUpOrientation(LineUpOrientation.kBackward);
    }

    public Command getTransition_Front_L2() {
        Command c = Commands.runOnce(() -> doTransition_Front_L2());
        c.setName("Front_L2");
        return c;
    }

    public void doTransition_Front_L2() {
        Carry.setNextNode(Front_L2);
        Front_L1.setNextNode(Front_L2);
        Front_L2.setNextNode(FrontScore_L2);
        FrontScore_L2.setNextNode(Front_L2);
        RobotContainer.m_targeting.setLineUpOrientation(LineUpOrientation.kForward);
    }

    public Command getTransition_Front_L1() {
        Command c = Commands.runOnce(() -> doTransition_Front_L1());
        c.setName("Front_L1");
        return c;
    }

    public void doTransition_Front_L1() {
        Carry.setNextNode(Front_L1);
        Front_L2.setNextNode(Front_L1);
        Front_L1.setNextNode(FrontScore_L1);
        FrontScore_L1.setNextNode(Front_L1);
        RobotContainer.m_targeting.setLineUpOrientation(LineUpOrientation.kForward);
    }

    public Command getTransition_HumanPlayerCoral() {
        Command c = Commands.runOnce(() -> m_armGraph.setTargetNode(HumanPlayerCoral));
        // .andThen(RobotContainer.m_buttonBox.getBackWardsOrientationCmd());
        c.setName("HumanPlayerCoral");
        return c;
    }

    public Command getTransition_GroundCoral() {
        Command c = Commands.runOnce(() -> m_armGraph.setTargetNode(GroundCoral));
        c.setName("GroundCoral");
        return c;
    }

    public Command getTransition_BackScore_L4() {
        Command c = Commands.runOnce(() -> m_armGraph.setTargetNode(BackScore_L4));
        c.setName("BackScore_L4");
        return c;
    }

    public Command getTransition_BackScore_L3() {
        Command c = Commands.runOnce(() -> m_armGraph.setTargetNode(BackScore_L3));
        c.setName("BackScore_L3");
        return c;
    }

    public Command getTransition_Climb() {
        Command c = Commands.runOnce(() -> m_armGraph.setTargetNode(Climb));
        c.setName("Climb");
        return c;
    }

    public Command getTransition_ClimbLockOn() {
        Command c = Commands.runOnce(() -> m_armGraph.setTargetNode(ClimbLockOn));
        c.setName("ClimbLockOn");
        return c;
    }

    public Command getTransition_ClimbReady() {
        Command c = Commands.runOnce(() -> m_armGraph.setTargetNode(ClimbReady));
        c.setName("ClimbReady");
        return c;
    }

    public Command getTransition_Carry() {
        Command c = Commands.runOnce(() -> m_armGraph.setTargetNode(Carry));
        c.setName("Carry");
        return c;
    }

    public Command getTransition_HPCarry() {
        Command c = Commands.runOnce(() -> m_armGraph.setTargetNode(HPCarry));
        c.setName("HPCarry");
        return c;
    }

    public Command getTransition_FrontAligment() {
        Command c = Commands.runOnce(() -> m_armGraph.setTargetNode(FrontAligment));
        c.setName("FrontAligment");
        return c;
    }

    public Command getTransition_S1() {
        Command c = Commands.runOnce(() -> m_armGraph.setTargetNode(S1));
        c.setName("S1");
        return c;
    }

    public Command getTransition_FrontScore_L2() {
        Command c = Commands.runOnce(() -> m_armGraph.setTargetNode(FrontScore_L2));
        c.setName("FrontScore_L2");
        return c;
    }

    public Command getTransition_FrontScore_L1() {
        Command c = Commands.runOnce(() -> m_armGraph.setTargetNode(FrontScore_L1));
        c.setName("FrontScore_L1");
        return c;
    }

    public Command getTransition_GroundAlgae() {
        Command c = Commands.runOnce(() -> m_armGraph.setTargetNode(GroundAlgae));
        c.setName("GroundAlgae");
        return c;
    }

    public Command getTransition_ProcessorAlgae() {
        Command c = Commands.runOnce(() -> doProcessorAlgae());
        c.setName("ProcessorAlgae");
        return c;
    }

    public void doProcessorAlgae() {
        Carry.setNextNode(ProcessorAlgae);
        RobotContainer.m_targeting.setLineUpOrientation(LineUpOrientation.kForward);
        RobotContainer.m_targeting.setTargetIDRedBlue(3, 16);
    }

    public Command getTransition_NetAlgae() {
        Command c = Commands.runOnce(() -> doNetAlgae());
        c.setName("NetAlgae");
        return c;
    }

    public void doNetAlgae() {
        ProcessorAlgae.setNextNode(NetAlgae);
        RobotContainer.m_targeting.setTargetIDRedBlue(5, 14);
    }

    public Command getTransition_RemoveAlgaeHigh() {
        Command c = Commands.runOnce(() -> doRemoveAlgaeHigh());
        c.setName("RemoveAlgaeHigh");
        return c;
    }

    public void doRemoveAlgaeHigh() {
        Carry.setNextNode(RemoveAlgaeHigh);
        RemoveAlgaeLow.setNextNode(RemoveAlgaeHigh);
        RemoveAlgaeHigh.setNextNode(ProcessorAlgae);
        RobotContainer.m_targeting.setLineUpOrientation(LineUpOrientation.kForward);
    }

    public Command getTransition_RemoveAlgaeLow() {
        Command c = Commands.runOnce(() -> doRemoveAlgaeLow());
        c.setName("RemoveAlgaeLow");
        return c;
    }

    public void doRemoveAlgaeLow() {
        Carry.setNextNode(RemoveAlgaeLow);
        RemoveAlgaeHigh.setNextNode(RemoveAlgaeLow);
        RemoveAlgaeLow.setNextNode(ProcessorAlgae);
        RobotContainer.m_targeting.setLineUpOrientation(LineUpOrientation.kForward);
    }

    final static public Command getArmCommand(Angle armBaseAngle, Distance armLength, Angle wristAngle) {
        return new ParallelCommandGroup(RobotContainer.m_armBase.getSetGoalCommand(armBaseAngle),
                RobotContainer.m_armExtension.getSetGoalCommand(armLength),
                RobotContainer.m_wrist.getSetGoalCommand(wristAngle));
    }

}
