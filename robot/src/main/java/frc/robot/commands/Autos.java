/*
 * Copyright (c) 2024-2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.commands;

import static frc.robot.Constants.*;

import com.teamrembrandts.auto.AutoRoutine;
import com.teamrembrandts.auto.Disabled;
import com.teamrembrandts.math.trajectory.ChoreoTrajectoryAdapter;
import com.teamrembrandts.math.trajectory.TrajectoryFollower;
import com.teamrembrandts.subsystems.drive.Drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.System;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.algaegripper.AlgaeGripper;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.feedthroughgripper.FeedThroughGripper;
import frc.robot.subsystems.pivot.Pivot;

public final class Autos {

    // Complete system
    private static final frc.robot.System SYSTEM = System.getInstance();

    // Subsystems
    private static final Drive DRIVETRAIN = SYSTEM.getDrivetrain();
    private static final Elevator ELEVATOR = SYSTEM.getElevator();
    private static final Pivot PIVOT = SYSTEM.getPivot();
    private static final FeedThroughGripper FEED_THROUGH_GRIPPER = SYSTEM.getFeedThroughGripper();
    private static final AlgaeGripper ALGAE_GRIPPER = SYSTEM.getAlgaeGripper();

    @AutoRoutine(name = "Red Left 4xL4 ")
    public static Command quattroPounderRedLeft() {
        return quattroPounder("Q_pounder_red_left_4");
    }

    @AutoRoutine(name = "Red Right 4xL4")
    public static Command quattroPounderRedRight() {
        return quattroPounder("Q_pounder_red_right_4");
    }

    @AutoRoutine(name = "Blue Left 4xL4")
    public static Command quattroPounderBlueLeft() {
        return quattroPounder("Q_pounder_blue_left_4");
    }

    @AutoRoutine(name = "Blue Right 4xL4")
    public static Command quattroPounderBlueRight() {
        return quattroPounder("Q_pounder_blue_right_4");
    }

    @AutoRoutine(name = "Red Mid Algae Auto")
    public static Command mcKroket() {
        return Commands.sequence(
                        scoreOnReef("Mac_kroket_Red", 0, true, 2.1),
                        ComplexCommands.goToStowed(),
                        pickAlgaeFromReef("Mac_kroket_Red", 1, false, 0.05, false),
                        scoreAlgaeInBarge("Mac_kroket_Red", 2),
                        pickAlgaeFromReef("Mac_kroket_Red", 3, false, 0.05, true),
                        scoreAlgaeInBarge("Mac_kroket_Red", 4),
                        ComplexCommands.goToStowed(),
                        DRIVETRAIN.followTrajectory(
                                new ChoreoTrajectoryAdapter("Mac_kroket_Red", 5),
                                TRAJECTORY_TRANSLATION_STATIC_KP_AUTON,
                                TRAJECTORY_TRANSLATION_DYNAMIC_KP_AUTON,
                                TRAJECTORY_ROTATION_KP_AUTON,
                                AUTON_DISTANCE_TOLERANCE,
                                AUTON_ANGULAR_TOLERANCE,
                                1.0))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    @AutoRoutine(name = "Blue Mid Algae Auto")
    public static Command mcKroketBlue() {
        return Commands.sequence(
                        scoreOnReef("Mac_kroket_Blue", 0, true, 2.1),
                        ComplexCommands.goToStowed(),
                        pickAlgaeFromReef("Mac_kroket_Blue", 1, false, 0.05, false),
                        scoreAlgaeInBarge("Mac_kroket_Blue", 2),
                        pickAlgaeFromReef("Mac_kroket_Blue", 3, false, 0.05, true),
                        scoreAlgaeInBarge("Mac_kroket_Blue", 4),
                        ComplexCommands.goToStowed(),
                        DRIVETRAIN.followTrajectory(
                                new ChoreoTrajectoryAdapter("Mac_kroket_Blue", 5),
                                TRAJECTORY_TRANSLATION_STATIC_KP_AUTON,
                                TRAJECTORY_TRANSLATION_DYNAMIC_KP_AUTON,
                                TRAJECTORY_ROTATION_KP_AUTON,
                                AUTON_DISTANCE_TOLERANCE,
                                AUTON_ANGULAR_TOLERANCE,
                                1.0))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    private static Command quattroPounder(String trajectoryName) {
        return Commands.sequence(
                        scoreOnReef(trajectoryName, 0, true, 2.5),
                        intakeFromCoralStation(trajectoryName, 1),
                        scoreOnReef(trajectoryName, 2, false, 2.3),
                        intakeFromCoralStation(trajectoryName, 3),
                        scoreOnReef(trajectoryName, 4, false, 2.3),
                        intakeFromCoralStation(trajectoryName, 5),
                        scoreOnReef(trajectoryName, 6, false, 2.3),
                        Commands.sequence(
                                        PIVOT.goToHorizontalCommand().until(PIVOT.onPositionTrigger()),
                                        ELEVATOR.goToStowedCommand().until(ELEVATOR.onPositionTrigger()))
                                .onlyIf(ELEVATOR.isElevatorUpTrigger()))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    @AutoRoutine(name = "feedforwardCharacterizationo")
    @Disabled
    public static Command feedforwardCharacterization() {
        return Commands.sequence(DRIVETRAIN.feedforwardCharacterization(0.1))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    @AutoRoutine(name = "wheelRadiusCharacterization")
    @Disabled
    public static Command wheelRadiusCharacterization() {
        return Commands.sequence(DRIVETRAIN.wheelRadiusCharacterization(
                        0.1, DRIVE_CONSTANTS.getDrivetrainConfig().maxAngularVelocity(), 0.359, 0.048768))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    @AutoRoutine(name = "SysId quasistatic forward")
    @Disabled
    public static Command sysIdQuasistaticForward() {
        return Commands.sequence(DRIVETRAIN.sysIdQuasistatic(0.1, SysIdRoutine.Direction.kForward))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    @AutoRoutine(name = "SysId quasistatic backward")
    @Disabled
    public static Command sysIdQuasistaticBackward() {
        return Commands.sequence(DRIVETRAIN.sysIdQuasistatic(0.1, SysIdRoutine.Direction.kReverse))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    @AutoRoutine(name = "SysId dynamic forward")
    @Disabled
    public static Command sysIdDynamicForward() {
        return Commands.sequence(DRIVETRAIN.sysIdDynamic(6, SysIdRoutine.Direction.kForward))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    @AutoRoutine(name = "SysId dynamic backward")
    @Disabled
    public static Command sysIdDynamicBackward() {
        return Commands.sequence(DRIVETRAIN.sysIdDynamic(6, SysIdRoutine.Direction.kReverse))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    @AutoRoutine(name = "SysId quasistatic forward elevator")
    @Disabled
    public static Command sysIdQuasistaticForwardElevator() {
        return ELEVATOR.sysIdQuasistatic(0.1, SysIdRoutine.Direction.kForward)
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    @AutoRoutine(name = "SysId quasistatic backward elevator")
    @Disabled
    public static Command sysIdQuasistaticBackwardElevator() {
        return ELEVATOR.sysIdQuasistatic(0.1, SysIdRoutine.Direction.kReverse)
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    @AutoRoutine(name = "SysId dynamic forward elevator")
    @Disabled
    public static Command sysIdDynamicForwardElevator() {
        return ELEVATOR.sysIdDynamic(3, SysIdRoutine.Direction.kForward)
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    @AutoRoutine(name = "SysId dynamic backward elevator")
    @Disabled
    public static Command sysIdDynamicBackwardElevator() {
        return ELEVATOR.sysIdDynamic(3, SysIdRoutine.Direction.kReverse)
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    @AutoRoutine(name = "feedforwardCharacterizationElevator")
    @Disabled
    public static Command feedforwardCharacterizationElevator() {
        return Commands.sequence(ELEVATOR.feedforwardCharacterization(0.1))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    /**
     * Command to warm up the Choreo trajectory follower. This will not control the robot, but just use the functions
     * involved in following a trajectory with dummy values. The idea is based on PathPlanner warmupCommand. <a
     * href="https://pathplanner.dev/pplib-pathfinding.html#java-warmup">See more information here</a>
     *
     * @return Command to warm up the Choreo trajectory follower
     */
    public static Command warmupChoreoTrajectoryFollower() {
        TrajectoryFollower follower = new TrajectoryFollower(
                () -> new ChoreoTrajectoryAdapter("warmup_trajectory"),
                Pose2d::new,
                ChassisSpeeds::new,
                0.1,
                0.1,
                1,
                1,
                1);
        return Commands.sequence(
                        Commands.runOnce(follower::start),
                        Commands.run(follower::getTargetState)
                                .until(new Trigger(follower::isFinished))
                                .withTimeout(5))
                .andThen(Commands.print("Finished Choreo trajectory follower warmup"))
                .ignoringDisable(true)
                .withName("Warmup Choreo Trajectory Follower");
    }

    private static Command intakeFromCoralStation(String trajectoryName, int splitIndex) {
        return Commands.sequence(Commands.parallel(
                Commands.sequence(
                        PIVOT.goToHorizontalCommand()
                                .until(SYSTEM.robotWithinReefContact().negate())
                                .onlyIf(ELEVATOR.isElevatorUpTrigger()),
                        FEED_THROUGH_GRIPPER.disableCommand().withTimeout(0.05),
                        PIVOT.goToCoralStationAutonomousCommand().until(PIVOT.onPositionTrigger()),
                        FEED_THROUGH_GRIPPER.inCommand().withTimeout(0.6)),
                ELEVATOR.goToStowedCommand().until(ELEVATOR.onPositionTrigger()),
                DRIVETRAIN.followTrajectory(
                        new ChoreoTrajectoryAdapter(trajectoryName, splitIndex),
                        TRAJECTORY_TRANSLATION_STATIC_KP_AUTON,
                        TRAJECTORY_TRANSLATION_DYNAMIC_KP_AUTON,
                        TRAJECTORY_ROTATION_KP_AUTON,
                        AUTON_DISTANCE_TOLERANCE,
                        AUTON_ANGULAR_TOLERANCE,
                        0.5)));
    }

    private static Command scoreOnReef(
            String trajectoryName, int splitIndex, boolean resetToStartPosition, double margin) {
        return Commands.sequence(
                        Commands.parallel(
                                DRIVETRAIN.followTrajectory(
                                        new ChoreoTrajectoryAdapter(trajectoryName, splitIndex),
                                        TRAJECTORY_TRANSLATION_STATIC_KP_AUTON,
                                        TRAJECTORY_TRANSLATION_DYNAMIC_KP_AUTON,
                                        TRAJECTORY_ROTATION_KP_AUTON,
                                        AUTON_DISTANCE_TOLERANCE,
                                        AUTON_ANGULAR_TOLERANCE,
                                        1.0,
                                        resetToStartPosition),
                                Commands.sequence(
                                        new WaitUntilCommand(FEED_THROUGH_GRIPPER.coralPresent()).withTimeout(0.8),
                                        PIVOT.goToL4Command()
                                                .onlyIf(FEED_THROUGH_GRIPPER
                                                        .coralPresent()
                                                        .debounce(0.1))
                                                .until(PIVOT.onPositionTrigger()),
                                        FEED_THROUGH_GRIPPER.disableCommand().withTimeout(0.05)),
                                Commands.sequence(
                                        Commands.waitUntil(nearReef(margin)),
                                        ELEVATOR.goToL4Command()
                                                .until(ELEVATOR.onPositionTrigger())
                                                .onlyIf(FEED_THROUGH_GRIPPER
                                                        .coralPresent()
                                                        .debounce(0.1)))),
                        FEED_THROUGH_GRIPPER
                                .outL4Command()
                                .until(FEED_THROUGH_GRIPPER.coralPresent().negate()))
                .finallyDo(() -> SYSTEM.setCurrentArmState(System.ArmState.L4));
    }

    private static Command pickAlgaeFromReef(
            String trajectoryName, int splitIndex, boolean resetToStartPosition, double margin, boolean L3Algae) {
        return Commands.parallel(
                DRIVETRAIN.followTrajectory(
                        new ChoreoTrajectoryAdapter(trajectoryName, splitIndex),
                        TRAJECTORY_TRANSLATION_STATIC_KP_AUTON,
                        TRAJECTORY_TRANSLATION_DYNAMIC_KP_AUTON,
                        TRAJECTORY_ROTATION_KP_AUTON,
                        AUTON_DISTANCE_TOLERANCE,
                        AUTON_ANGULAR_TOLERANCE,
                        1.0,
                        resetToStartPosition),
                Commands.sequence(
                        new WaitCommand(0.5),
                        new ConditionalCommand(
                                ComplexCommands.goToAlgaeL3CommandAuto(),
                                ComplexCommands.goToAlgaeL2CommandAuto(),
                                () -> L3Algae)),
                Commands.sequence(
                        Commands.waitUntil(nearReef(2.5)),
                        ALGAE_GRIPPER
                                .inCommand()
                                .until(ALGAE_GRIPPER.holdingGamePiece())
                                .withTimeout(2),
                        ALGAE_GRIPPER.holdCommand().withTimeout(0.3)));
    }

    private static Command scoreAlgaeInBarge(String trajectoryName, int splitIndex) {
        return Commands.sequence(
                Commands.parallel(
                        DRIVETRAIN.followTrajectory(
                                new ChoreoTrajectoryAdapter(trajectoryName, splitIndex),
                                TRAJECTORY_TRANSLATION_STATIC_KP_AUTON,
                                TRAJECTORY_TRANSLATION_DYNAMIC_KP_AUTON,
                                TRAJECTORY_ROTATION_KP_AUTON,
                                AUTON_DISTANCE_TOLERANCE,
                                AUTON_ANGULAR_TOLERANCE,
                                1.0),
                        Commands.sequence(
                                new WaitUntilCommand(nearReef(2).negate()),
                                ComplexCommands.prepPivotBargeCommand(),
                                ComplexCommands.prepElevatorBargeCommand())),
                ComplexCommands.scoreAlgaeInBargeCommand(),
                Commands.parallel(ELEVATOR.goToStowedCommand().until(ELEVATOR.onPositionTrigger())));
    }

    private static Trigger nearReef(double margin) {
        return DRIVETRAIN
                .onPosition(FieldConstants.REEF_RED_CENTER, margin)
                .or(DRIVETRAIN.onPosition(FieldConstants.REEF_BLUE_CENTER, margin));
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    private static boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red);
    }

    private static Pose2d[] getAllianceBasedClosestPose(Pose2d[] redPoses, Pose2d[] bluePoses) {
        return isRedAlliance() ? redPoses : bluePoses;
    }
}
