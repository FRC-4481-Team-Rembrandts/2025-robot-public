/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.System;
import frc.robot.subsystems.algaegripper.AlgaeGripper;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.feedthroughgripper.FeedThroughGripper;
import frc.robot.subsystems.pivot.Pivot;
import java.util.Map;

public class ComplexCommands {
    // Complete system
    private static final frc.robot.System SYSTEM = System.getInstance();

    // Subsystems
    private static final Elevator ELEVATOR = SYSTEM.getElevator();
    private static final Pivot PIVOT = SYSTEM.getPivot();
    private static final FeedThroughGripper FEED_THROUGH_GRIPPER = SYSTEM.getFeedThroughGripper();
    private static final AlgaeGripper ALGAE_GRIPPER = SYSTEM.getAlgaeGripper();

    public static Command requestL1Command() {
        return Commands.runOnce(() -> SYSTEM.setRequestedArmState(System.ArmState.L1))
                .andThen(() -> SYSTEM.setCoralArmState(System.ArmState.L1));
    }

    public static Command requestL2Command() {
        return Commands.runOnce(() -> SYSTEM.setRequestedArmState(System.ArmState.L2))
                .andThen(() -> SYSTEM.setCoralArmState(System.ArmState.L2));
    }

    public static Command requestL3Command() {
        return Commands.runOnce(() -> SYSTEM.setRequestedArmState(System.ArmState.L3))
                .andThen(() -> SYSTEM.setCoralArmState(System.ArmState.L3));
    }

    public static Command requestL4Command() {
        return Commands.runOnce(() -> SYSTEM.setRequestedArmState(System.ArmState.L4))
                .andThen(() -> SYSTEM.setCoralArmState(System.ArmState.L4));
    }

    public static Command requestAlgae2IntakeCommand() {
        return Commands.runOnce(() -> SYSTEM.setRequestedArmState(System.ArmState.ALGAE2));
    }

    public static Command requestAlgae3IntakeCommand() {
        return Commands.runOnce(() -> SYSTEM.setRequestedArmState(System.ArmState.ALGAE3));
    }

    public static Command requestBargeCommand() {
        return Commands.runOnce(() -> SYSTEM.setRequestedArmState(System.ArmState.BARGE));
    }

    public static Command goToAlgaeL2CommandAuto() {
        return Commands.parallel(
                        PIVOT.goToAlgaeL2Command().until(PIVOT.onPositionTrigger()),
                        ELEVATOR.goToAlgaeL2Command().until(ELEVATOR.onPositionTrigger()))
                .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.ALGAE2))
                .withName("Go to algae L2");
    }

    public static Command goToAlgaeL3CommandAuto() {
        return Commands.parallel(
                        PIVOT.goToAlgaeL3Command().until(PIVOT.onPositionTrigger()),
                        ELEVATOR.goToAlgaeL3Command().until(ELEVATOR.onPositionTrigger()))
                .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.ALGAE3))
                .withName("Go to algae L3");
    }

    public static Command goToProcessorCommand() {
        return Commands.sequence(
                        PIVOT.goToProcessorCommand().until(PIVOT.onPositionTrigger()),
                        ELEVATOR.goToProcessorCommand().until(ELEVATOR.onPositionTrigger()))
                .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.PROCESSOR))
                .withName("Go to processor");
    }

    public static Command intakingCommand() {
        return Commands.sequence(
                        PIVOT.goToHorizontalCommand()
                                .until(PIVOT.onPositionTrigger())
                                .onlyIf(PIVOT.isPivotDown()
                                        .negate()
                                        .and(() -> SYSTEM.getCurrentArmState() != System.ArmState.BARGE)),
                        ELEVATOR.goToCoralStationCommand().until(ELEVATOR.onPositionTrigger()),
                        PIVOT.goToCoralStationCommand().until(PIVOT.onPositionTrigger()))
                .andThen(() -> SYSTEM.setCurrentArmState(System.ArmState.INTAKE))
                .withName("intaking");
    }

    public static Command intakingCommandAutonomous() {
        return PIVOT.goToCoralStationAutonomousCommand()
                .until(PIVOT.onPositionTrigger())
                .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.INTAKE))
                .withName("intakingAutonomous");
    }

    public static Command prepareGamepieceForScoring() {
        return new SelectCommand<>(
                        Map.ofEntries(
                                Map.entry(
                                        System.ArmState.L1,
                                        Commands.sequence(PIVOT.goToL1Command().until(PIVOT.onPositionTrigger()))
                                                .withName("prepare arm L1")),
                                Map.entry(
                                        System.ArmState.L2,
                                        Commands.sequence(PIVOT.goToL2Command().until(PIVOT.onPositionTrigger()))
                                                .withName("prepare arm L2")),
                                Map.entry(
                                        System.ArmState.L3,
                                        Commands.sequence(PIVOT.goToL3Command().until(PIVOT.onPositionTrigger()))
                                                .withName("prepare arm L3")),
                                Map.entry(
                                        System.ArmState.L4,
                                        Commands.sequence(PIVOT.goToL4Command().until(PIVOT.onPositionTrigger()))
                                                .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.L4_PREP))
                                                .withName("prepare arm L4")),
                                Map.entry(
                                        System.ArmState.BARGE,
                                        Commands.sequence(PIVOT.goToBargePrepCommand()
                                                        .until(PIVOT.onPositionTrigger()))
                                                .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.BARGE_PREP))
                                                .withName("prepare arm Barge"))),
                        SYSTEM::getRequestedArmState)
                .withName("prepare arm");
    }

    public static Command safeSetArmForReefActionWhenArmUp() {
        return Commands.sequence(
                        Commands.runOnce(() -> SYSTEM.isSafeToFinishArmMovementForScoring(false)),
                        new SelectCommand<>(
                                Map.ofEntries(
                                        Map.entry(
                                                System.ArmState.L1,
                                                Commands.sequence(
                                                        PIVOT.goToHorizontalCommand()
                                                                .until(PIVOT.onPositionTrigger())
                                                                .onlyIf(ELEVATOR.isElevatorUpTrigger()),
                                                        ELEVATOR.goToL1Command().until(ELEVATOR.onPositionTrigger()),
                                                        Commands.runOnce(
                                                                () -> SYSTEM.isSafeToFinishArmMovementForScoring(
                                                                        true)))),
                                        Map.entry(
                                                System.ArmState.L2,
                                                Commands.sequence(
                                                        PIVOT.goToHorizontalCommand()
                                                                .until(PIVOT.onPositionTrigger())
                                                                .onlyIf(ELEVATOR.isElevatorUpTrigger()),
                                                        ELEVATOR.goToL2Command().until(ELEVATOR.onPositionTrigger()),
                                                        Commands.runOnce(
                                                                () -> SYSTEM.isSafeToFinishArmMovementForScoring(
                                                                        true)))),
                                        Map.entry(
                                                System.ArmState.L3,
                                                Commands.sequence(
                                                        PIVOT.goToHorizontalCommand()
                                                                .until(PIVOT.onPositionTrigger())
                                                                .onlyIf(ELEVATOR.isElevatorUpTrigger()),
                                                        ELEVATOR.goToL3Command().until(ELEVATOR.onPositionTrigger()),
                                                        Commands.runOnce(
                                                                () -> SYSTEM.isSafeToFinishArmMovementForScoring(
                                                                        true)))),
                                        Map.entry(
                                                System.ArmState.L4,
                                                Commands.sequence(
                                                                PIVOT.goToL4Command()
                                                                        .until(PIVOT.onPositionTrigger()),
                                                                ELEVATOR.goToL4Command()
                                                                        .until(ELEVATOR.onPositionTrigger()))
                                                        .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.L4))
                                                        .withName("coral to scoring L4")),
                                        Map.entry(
                                                System.ArmState.ALGAE2,
                                                Commands.sequence(
                                                                PIVOT.goToHorizontalCommand()
                                                                        .until(PIVOT.onPositionTrigger())
                                                                        .onlyIf(ELEVATOR.isElevatorUpTrigger()),
                                                                ELEVATOR.goToAlgaeL2Command()
                                                                        .until(ELEVATOR.onPositionTrigger()),
                                                                Commands.runOnce(
                                                                        () ->
                                                                                SYSTEM
                                                                                        .isSafeToFinishArmMovementForScoring(
                                                                                                true)))
                                                        .withName("Go to algae L2")),
                                        Map.entry(
                                                System.ArmState.ALGAE3,
                                                Commands.sequence(
                                                                PIVOT.goToHorizontalCommand()
                                                                        .until(PIVOT.onPositionTrigger())
                                                                        .onlyIf(ELEVATOR.isElevatorUpTrigger()),
                                                                ELEVATOR.goToAlgaeL3Command()
                                                                        .until(ELEVATOR.onPositionTrigger()),
                                                                Commands.runOnce(
                                                                        () ->
                                                                                SYSTEM
                                                                                        .isSafeToFinishArmMovementForScoring(
                                                                                                true)))
                                                        .withName("Go to algae L3"))),
                                SYSTEM::getRequestedArmState))
                .withName("Safe set up");
    }

    public static Command safeSetArmForReefActionWhenArmDown() {
        return Commands.sequence(
                        Commands.runOnce(() -> SYSTEM.isSafeToFinishArmMovementForScoring(false)),
                        new SelectCommand<>(
                                Map.ofEntries(
                                        Map.entry(
                                                System.ArmState.L1,
                                                Commands.sequence(
                                                                PIVOT.goToStowedCommand()
                                                                        .until(PIVOT.onPositionTrigger())
                                                                        .onlyIf(
                                                                                ELEVATOR.isElevatorUpTrigger()
                                                                                        .or(
                                                                                                () ->
                                                                                                        SYSTEM
                                                                                                                        .getCurrentArmState()
                                                                                                                == System
                                                                                                                        .ArmState
                                                                                                                        .INTAKE)),
                                                                ELEVATOR.goToL1Command()
                                                                        .until(ELEVATOR.onPositionTrigger()),
                                                                PIVOT.goToL1Command()
                                                                        .until(PIVOT.onPositionTrigger()))
                                                        .andThen(() -> SYSTEM.setCoralArmState(System.ArmState.L1))
                                                        .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.L1))
                                                        .withName("coral to scoring L1")),
                                        Map.entry(
                                                System.ArmState.L2,
                                                Commands.sequence(
                                                                PIVOT.goToStowedCommand()
                                                                        .until(PIVOT.onPositionTrigger())
                                                                        .onlyIf(
                                                                                ELEVATOR.isElevatorUpTrigger()
                                                                                        .or(
                                                                                                () ->
                                                                                                        SYSTEM
                                                                                                                        .getCurrentArmState()
                                                                                                                == System
                                                                                                                        .ArmState
                                                                                                                        .INTAKE)),
                                                                ELEVATOR.goToL2Command()
                                                                        .until(ELEVATOR.onPositionTrigger()),
                                                                PIVOT.goToL2Command()
                                                                        .until(PIVOT.onPositionTrigger()))
                                                        .andThen(() -> SYSTEM.setCoralArmState(System.ArmState.L2))
                                                        .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.L2))
                                                        .withName("coral to scoring L2")),
                                        Map.entry(
                                                System.ArmState.L3,
                                                Commands.sequence(
                                                                PIVOT.goToStowedCommand()
                                                                        .until(PIVOT.onPositionTrigger())
                                                                        .onlyIf(
                                                                                ELEVATOR.isElevatorUpTrigger()
                                                                                        .or(
                                                                                                () ->
                                                                                                        SYSTEM
                                                                                                                        .getCurrentArmState()
                                                                                                                == System
                                                                                                                        .ArmState
                                                                                                                        .INTAKE)),
                                                                PIVOT.goToL3Command()
                                                                        .until(PIVOT.onPositionTrigger()),
                                                                ELEVATOR.goToL3Command()
                                                                        .until(ELEVATOR.onPositionTrigger()))
                                                        .andThen(() -> SYSTEM.setCoralArmState(System.ArmState.L3))
                                                        .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.L3))
                                                        .withName("coral to scoring L3")),
                                        Map.entry(
                                                System.ArmState.L4,
                                                Commands.sequence(
                                                        PIVOT.goToStowedCommand()
                                                                .until(PIVOT.onPositionTrigger())
                                                                .onlyIf(
                                                                        () -> SYSTEM.getCurrentArmState()
                                                                                == System.ArmState.INTAKE),
                                                        ELEVATOR.goToL4Command().until(ELEVATOR.onPositionTrigger()),
                                                        Commands.runOnce(
                                                                () -> SYSTEM.isSafeToFinishArmMovementForScoring(
                                                                        true)))),
                                        Map.entry(
                                                System.ArmState.ALGAE2,
                                                Commands.sequence(
                                                                PIVOT.goToAlgaeL2Command()
                                                                        .until(PIVOT.onPositionTrigger()),
                                                                ELEVATOR.goToAlgaeL2Command()
                                                                        .until(ELEVATOR.onPositionTrigger()))
                                                        .finallyDo(
                                                                () -> SYSTEM.setCurrentArmState(System.ArmState.ALGAE2))
                                                        .withName("Go to algae L2")),
                                        Map.entry(
                                                System.ArmState.ALGAE3,
                                                Commands.sequence(
                                                                PIVOT.goToAlgaeL3Command()
                                                                        .until(PIVOT.onPositionTrigger()),
                                                                ELEVATOR.goToAlgaeL3Command()
                                                                        .until(ELEVATOR.onPositionTrigger()))
                                                        .finallyDo(
                                                                () -> SYSTEM.setCurrentArmState(System.ArmState.ALGAE3))
                                                        .withName("Go to algae L3"))),
                                SYSTEM::getRequestedArmState))
                .withName("Safe set down");
    }

    public static Command finishSetArmForReefActionWhenSafe() {
        return new SelectCommand<>(
                        Map.ofEntries(
                                Map.entry(
                                        System.ArmState.L1,
                                        Commands.sequence(PIVOT.goToL1Command().until(PIVOT.onPositionTrigger()))
                                                .andThen(() -> SYSTEM.setCoralArmState(System.ArmState.L1))
                                                .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.L1))
                                                .withName("coral to scoring L1")),
                                Map.entry(
                                        System.ArmState.L2,
                                        Commands.sequence(PIVOT.goToL2Command().until(PIVOT.onPositionTrigger()))
                                                .andThen(() -> SYSTEM.setCoralArmState(System.ArmState.L2))
                                                .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.L2))
                                                .withName("coral to scoring L2")),
                                Map.entry(
                                        System.ArmState.L3,
                                        Commands.sequence(PIVOT.goToL3Command().until(PIVOT.onPositionTrigger()))
                                                .andThen(() -> SYSTEM.setCoralArmState(System.ArmState.L3))
                                                .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.L3))
                                                .withName("coral to scoring L3")),
                                Map.entry(
                                        System.ArmState.L4,
                                        Commands.sequence(
                                                        PIVOT.goToL4Command().until(PIVOT.onPositionTrigger()),
                                                        ELEVATOR.goToL4Command().until(ELEVATOR.onPositionTrigger()))
                                                .andThen(() -> SYSTEM.setCoralArmState(System.ArmState.L4))
                                                .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.L4))
                                                .withName("coral to scoring L4")),
                                Map.entry(
                                        System.ArmState.ALGAE2,
                                        Commands.sequence(
                                                        PIVOT.goToAlgaeL2Command()
                                                                .until(PIVOT.onPositionTrigger()),
                                                        ELEVATOR.goToAlgaeL2Command()
                                                                .until(ELEVATOR.onPositionTrigger()))
                                                .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.ALGAE2))
                                                .withName("Go to algae L2")),
                                Map.entry(
                                        System.ArmState.ALGAE3,
                                        Commands.sequence(
                                                        PIVOT.goToAlgaeL3Command()
                                                                .until(PIVOT.onPositionTrigger()),
                                                        ELEVATOR.goToAlgaeL3Command()
                                                                .until(ELEVATOR.onPositionTrigger()))
                                                .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.ALGAE3))
                                                .withName("Go to algae L3"))),
                        SYSTEM::getRequestedArmState)
                .andThen(Commands.runOnce(() -> SYSTEM.isSafeToFinishArmMovementForScoring(false)))
                .withName("Safe finish");
    }

    public static Command prepPivotBargeCommand() {
        return Commands.sequence(PIVOT.goToBargePrepCommand().until(PIVOT.onPositionTrigger()))
                .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.BARGE_PREP))
                .withName("prepare pivot Barge");
    }

    public static Command prepElevatorBargeCommand() {
        return Commands.sequence(ELEVATOR.goToBargeCommand().until(ELEVATOR.onPositionTrigger()))
                .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.BARGE_PREP))
                .withName("prepare elevator Barge");
    }

    public static Command scoreAlgaeInBargeCommand() {
        return Commands.parallel(
                        PIVOT.goToBargeCommand().until(PIVOT.onPositionTrigger()),
                        Commands.sequence(
                                ALGAE_GRIPPER.inCommand().withTimeout(0.001),
                                Commands.waitUntil(PIVOT.isPivotReadyToShootBarge()),
                                ALGAE_GRIPPER.outCommand().withTimeout(0.001)))
                .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.BARGE))
                .andThen(() -> SYSTEM.setRequestedArmState(SYSTEM.getCoralArmState()))
                .withName("score Barge");
    }

    public static Command setCoralForL4AutonomousCommand() {
        return Commands.parallel(
                        PIVOT.goToL4Command().until(PIVOT.onPositionTrigger()),
                        ELEVATOR.goToL4Command().until(ELEVATOR.onPositionTrigger()))
                .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.L4));
    }

    public static Command goToStowed() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(System.ArmState.L1, intakingCommand().withName("Go to intake from L1")),
                        Map.entry(System.ArmState.L2, intakingCommand().withName("Go to intake from L2")),
                        Map.entry(System.ArmState.L3, intakingCommand().withName("Go to intake from L3")),
                        Map.entry(
                                System.ArmState.L4,
                                Commands.sequence(
                                                PIVOT.goToHorizontalCommand().until(PIVOT.onPositionTrigger()),
                                                ELEVATOR.goToCoralStationCommand()
                                                        .until(ELEVATOR.onPositionTrigger()))
                                        .withName("Go to stowed from L4")),
                        Map.entry(System.ArmState.BARGE, intakingCommand().withName("Go to intake from barge")),
                        Map.entry(System.ArmState.BARGE_PREP, intakingCommand().withName("Go to stowed from barge")),
                        Map.entry(System.ArmState.STOWED, Commands.none()),
                        Map.entry(System.ArmState.INTAKE, Commands.none()),
                        Map.entry(System.ArmState.ALGAE2, intakingCommand().withName("Go to intake from Algae Low")),
                        Map.entry(System.ArmState.ALGAE3, intakingCommand().withName("Go to intake from Algae High")),
                        Map.entry(System.ArmState.L4_PREP, Commands.none()),
                        Map.entry(System.ArmState.PROCESSOR, Commands.none()),
                        Map.entry(System.ArmState.CLIMBING, Commands.none())),
                SYSTEM::getCurrentArmState);
    }

    public static Command goToClimbCommand() {
        return Commands.sequence(
                PIVOT.goToHorizontalCommand().until(PIVOT.onPositionTrigger()),
                ELEVATOR.goToClimbCommand().until(ELEVATOR.onPositionTrigger()),
                PIVOT.goToClimbCommand()
                        .until(PIVOT.onPositionTrigger())
                        .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.CLIMBING)));
    }

    public static Command stayAtClimbCommand() {
        return PIVOT.disableCommand();
    }

    public static Command outtakeCoralCommand() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(System.ArmState.L1, FEED_THROUGH_GRIPPER.outL1Command()),
                        Map.entry(System.ArmState.L2, FEED_THROUGH_GRIPPER.outL2Command()),
                        Map.entry(System.ArmState.L3, FEED_THROUGH_GRIPPER.outL3Command()),
                        Map.entry(System.ArmState.L4, FEED_THROUGH_GRIPPER.outL4Command())),
                SYSTEM::getRequestedArmState);
    }

    /**
     * Command that releases the coral
     *
     * @return command that releases the coral
     */
    private static Command scoreGamepieceCommand() {
        return new ConditionalCommand(
                        scoreAlgaeInBargeCommand().andThen(goToStowed()),
                        new ConditionalCommand(
                                scoreCoralL4Command(),
                                scoreCoralOtherCommand(),
                                () -> SYSTEM.getRequestedArmState() == System.ArmState.L4),
                        () -> SYSTEM.getCurrentArmState() == System.ArmState.BARGE
                                || SYSTEM.getCurrentArmState() == System.ArmState.BARGE_PREP)
                .withName("Release gamepiece");
    }

    public static Command scoreGamePieceOrSuperCycleCommand() {
        return new ConditionalCommand(
                superCycleL3Command()
                        .until(SYSTEM.robotWithinReefContact().negate())
                        .andThen(goToStowed()),
                scoreGamepieceCommand(),
                () -> (SYSTEM.getCurrentArmState() == System.ArmState.L3));
    }

    private static Command superCycleL3Command() {
        return outtakeCoralCommand()
                .until(FEED_THROUGH_GRIPPER.coralPresent().negate().debounce(0.05))
                .andThen(FEED_THROUGH_GRIPPER.disableCommand());
    }

    /**
     * Command that releases the coral for L4 Does a sort of 'bottle flip' for better accuracy
     *
     * @return Command that releases the coral
     */
    private static Command scoreCoralL4Command() {
        return ComplexCommands.outtakeCoralCommand()
                .withTimeout(0.05)
                .andThen(ComplexCommands.goToStowed()
                        .alongWith(Commands.sequence(new WaitCommand(0.2), FEED_THROUGH_GRIPPER.disableCommand())));
    }

    /**
     * Command to score coral on a branch other than L4.
     *
     * @return Command to score a coral on another branch than L4
     */
    private static Command scoreCoralOtherCommand() {
        return ComplexCommands.outtakeCoralCommand()
                .until(FEED_THROUGH_GRIPPER.coralPresent().negate().debounce(0.15))
                .andThen(ComplexCommands.goToStowed().alongWith(FEED_THROUGH_GRIPPER.disableCommand()));
    }

    private ComplexCommands() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
