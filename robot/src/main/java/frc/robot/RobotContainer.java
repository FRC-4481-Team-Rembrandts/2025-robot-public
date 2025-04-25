/*
 * Copyright (c) 2024-2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.constants.FieldConstants.*;

import com.pathplanner.lib.path.PathConstraints;
import com.teamrembrandts.auto.AutoSelector;
import com.teamrembrandts.input.CommandNaconPS4Controller;
import com.teamrembrandts.input.CommandSwitchProController;
import com.teamrembrandts.input.NaconPS4Controller;
import com.teamrembrandts.math.trajectory.PathPlannerTrajectoryAdapter;
import com.teamrembrandts.math.trajectory.PathPlannerTrajectoryGenerator;
import com.teamrembrandts.subsystems.drive.*;
import com.teamrembrandts.subsystems.vision.Vision;
import com.teamrembrandts.util.GeometryUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.ComplexCommands;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.algaegripper.AlgaeGripper;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.feedthroughgripper.FeedThroughGripper;
import frc.robot.subsystems.led.LEDController;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.util.CommandUtils;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Complete system
    private final System system = System.getInstance();

    // Subsystems
    private final Drive drivetrain = system.getDrivetrain();
    private final Vision vision = system.getVision();
    private final Elevator elevator = system.getElevator();
    private final Pivot pivot = system.getPivot();
    private final AlgaeGripper algaeGripper = system.getAlgaeGripper();
    private final LEDController ledController = system.getLEDController();
    private final Climber climber = system.getClimber();
    private final FeedThroughGripper feedThroughGripper = system.getFeedThroughGripper();

    private final AutoSelector autoSelector;

    private boolean autoAlignReefFinished = false;

    // Controller
    private final CommandSwitchProController driverController = new CommandSwitchProController(0);
    private final CommandNaconPS4Controller operatorController = new CommandNaconPS4Controller(1);

    private final int filterLength = 10;
    private final LinearFilter poseXFilter = LinearFilter.movingAverage(filterLength);
    private final LinearFilter poseYFilter = LinearFilter.movingAverage(filterLength);
    private final LinearFilter poseRotationFilter = LinearFilter.movingAverage(filterLength);

    private final PathPlannerTrajectoryGenerator ppTrajectoryGenerator = new PathPlannerTrajectoryGenerator(
            DRIVE_CONSTANTS.getPathPlannerRobotConfig(),
            drivetrain.getRobotPoseSupplier(),
            drivetrain.getFieldRelativeSpeedsSupplier());

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        autoSelector = new AutoSelector(Autos.class);

        // Configure the trigger bindings
        configureBindings();

        // Configure the game visualization bindings
        // May be disabled for performance reasons
        configureVisualizationBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(BooleanSupplier)} constructor with an arbitrary predicate, or via the named factories in
     * {@link CommandGenericHID}'s subclasses for {@link CommandXboxController Xbox}/{@link CommandPS4Controller PS4}
     * controllers or {@link CommandJoystick Flight joysticks}.
     */
    private void configureBindings() {
        // Setup vision
        vision.setDefaultCommand(
                vision.processVision(drivetrain.getRobotPoseSupplier(), drivetrain.getVisionMeasurementConsumer())
                        .ignoringDisable(true));

        // Setup arm
        isDisabledTrigger().onTrue(pivot.disableCommand().ignoringDisable(true));
        isDisabledTrigger().onTrue(elevator.disableCommand().ignoringDisable(true));

        isTeleopTrigger()
                .and(isDisabledTrigger().negate())
                .onTrue(system.setCoralAutoAlignModeToReef().withName("isTeleopTrigger"));

        ledController.setDefaultCommand(Commands.parallel(
                        new ConditionalCommand(
                                ledController.setDeviationPatternCommand(this::onRotationDeviation),
                                updateArmStateLEDS(),
                                isDisabledTrigger()),
                        GameVisualizer.getInstance()
                                .setLEDColors(ledController::getLEDColorsBottom, ledController::getLEDColorsTop))
                .ignoringDisable(true));

        elevator.bottomLimitSwitchTrigger()
                .debounce(ElevatorConstants.LIMIT_SWITCH_DEBOUNCE_TIME)
                .onTrue(elevator.resetEncodersCommand().ignoringDisable(true));

        // Setup coral gripper
        feedThroughGripper.setDefaultCommand(feedThroughGripper.disableCommand());

        // Select coral auto align mode
        feedThroughGripper
                .coralPresent()
                .and(isDisabledTrigger().negate())
                .whileTrue(system.setCoralAutoAlignModeToReef().andThen((feedThroughGripper.disableCommand())));

        feedThroughGripper
                .coralPresent()
                .and(isDisabledTrigger().negate())
                .whileTrue(ComplexCommands.prepareGamepieceForScoring());

        // Select coral hold/intake mode
        system.isInCoralIntakeMode()
                .and(system.robotWithinReefContact().negate())
                .and(pivot.onPositionTrigger())
                .whileTrue((feedThroughGripper.inCommand().unless(feedThroughGripper.coralPresent())))
                .onFalse(feedThroughGripper.disableCommand());

        system.isInCoralIntakeMode()
                .onTrue(new ParallelCommandGroup(
                        ComplexCommands.goToStowed()
                                .andThen(new WaitUntilCommand(
                                                system.robotWithinReefContact().negate())
                                        .andThen(ComplexCommands.intakingCommand())),
                        ledController.disableManualOverrideOverlayCommand()));

        // Setup algae gripper
        algaeGripper.setDefaultCommand(new ConditionalCommand(
                algaeGripper.holdCommand().until(algaeGripper.holdingGamePiece().negate()),
                algaeGripper.disableCommand(),
                algaeGripper.holdingGamePiece()));

        // Select algae auto align mode
        algaeGripper
                .holdingGamePiece()
                .and(isDisabledTrigger().negate())
                .whileTrue(system.setAlgaeAutoAlignModeToBarge().andThen((algaeGripper.holdCommand())));

        algaeGripper
                .holdingGamePiece()
                .negate()
                .and(isDisabledTrigger().negate())
                .whileTrue(system.setAlgaeAutoAlignModeToIntake());

        // Select coral hold mode
        system.isInAlgaeIntakeMode()
                .onFalse(new ConditionalCommand(
                        algaeGripper
                                .holdCommand()
                                .until(algaeGripper.holdingGamePiece().negate()),
                        algaeGripper.disableCommand(),
                        algaeGripper.holdingGamePiece()));

        climber.setDefaultCommand(climber.disableCommand());

        autoAlignReefFinishedTrigger()
                .and(elevator.onPositionTrigger())
                .and(() -> system.getCurrentArmState() == System.ArmState.L1
                        || system.getCurrentArmState() == System.ArmState.L2
                        || system.getCurrentArmState() == System.ArmState.L3
                        || system.getCurrentArmState() == System.ArmState.L4)
                .onTrue(new WaitCommand(0.3)
                        .onlyIf(algaeGripper.holdingGamePiece())
                        .andThen(ComplexCommands.scoreGamePieceOrSuperCycleCommand())
                        .finallyDo(() -> autoAlignReefFinished = false));

        configureDriverBindings();
        configureOperatorBindings();
    }

    /** Configure the button bindings for the driver controller. */
    private void configureDriverBindings() {
        // Driving
        drivetrain.setDefaultCommand(drivetrain.joystickDrive(
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                Constants.CONTROLLER_DEADBAND));

        driverController
                .minus()
                .or(driverController.plus())
                .onTrue(drivetrain.setRobotPose(() ->
                        new Pose2d(drivetrain.getRobotPoseSupplier().get().getTranslation(), getForwardRotation())));

        driverController
                .minus()
                .and(driverController.plus())
                .debounce(0.5)
                .onTrue(Commands.runOnce(() -> {
                            try {
                                vision.removeDefaultCommand();
                                vision.getCurrentCommand().cancel();
                            } catch (Exception e) {
                                java.lang.System.err.println(e.getMessage());
                            }
                        })
                        .andThen(setDriverRumble(1, 1))
                        .withName("Disabling vision"));

        // Toggle REEF / Intake Modus
        driverController.l().onTrue(Commands.runOnce(system::toggleCoralAutoAlignMode));

        // Auto Align REEF (CORAL) / CORAL STATION
        driverController
                .zr()
                .and((driverController.zl()).negate())
                .whileTrue(new ConditionalCommand(
                        autoAlign(
                                FieldConstants.CORAL_STATION_RED_RIGHT,
                                FieldConstants.CORAL_STATION_BLUE_RIGHT,
                                PATH_CONSTRAINTS_CORAL_STATION_AUTO_ALIGN,
                                true,
                                AUTO_ALIGN_LOOK_AHEAD_TIME_CORAL),
                        autoAlignRight(
                                        FieldConstants.REEF_RED_LEFT,
                                        FieldConstants.REEF_BLUE_LEFT,
                                        system::getCoralArmState,
                                        PATH_CONSTRAINTS_REEF_AUTO_ALIGN,
                                        AUTO_ALIGN_LOOK_AHEAD_TIME_REEF)
                                .andThen(() -> autoAlignReefFinished = true),
                        system.isInCoralIntakeMode()));
        driverController
                .zl()
                .and((driverController.zr()).negate())
                .whileTrue(new ConditionalCommand(
                        autoAlign(
                                FieldConstants.CORAL_STATION_RED_LEFT,
                                FieldConstants.CORAL_STATION_BLUE_LEFT,
                                PATH_CONSTRAINTS_CORAL_STATION_AUTO_ALIGN,
                                true,
                                AUTO_ALIGN_LOOK_AHEAD_TIME_CORAL),
                        autoAlignLeft(
                                        FieldConstants.REEF_RED_RIGHT,
                                        FieldConstants.REEF_BLUE_RIGHT,
                                        system::getCoralArmState,
                                        PATH_CONSTRAINTS_REEF_AUTO_ALIGN,
                                        AUTO_ALIGN_LOOK_AHEAD_TIME_REEF)
                                .andThen(() -> autoAlignReefFinished = true),
                        system.isInCoralIntakeMode()));

        // Set pivot to desired position when within circle
        CommandUtils.xor(driverController.zl(), driverController.zr(), 0.2)
                .and(drivetrain
                        .onPosition(FieldConstants.REEF_RED_CENTER, Constants.CORAL_SCORING_CIRCLE_RADIUS)
                        .or(drivetrain.onPosition(
                                FieldConstants.REEF_BLUE_CENTER, Constants.CORAL_SCORING_CIRCLE_RADIUS)))
                .and(system.isInCoralIntakeMode().negate())
                .onTrue(new ConditionalCommand(
                                ComplexCommands.safeSetArmForReefActionWhenArmDown(),
                                ComplexCommands.safeSetArmForReefActionWhenArmUp(),
                                pivot.isPivotDown())
                        .withName("Safe reef action"));

        CommandUtils.xor(driverController.zl(), driverController.zr(), 0.2)
                .and(drivetrain
                        .onPosition(FieldConstants.REEF_RED_CENTER, CORAL_PREPARE_RADIUS)
                        .or(drivetrain.onPosition(FieldConstants.REEF_BLUE_CENTER, CORAL_PREPARE_RADIUS)))
                .and((drivetrain
                                .onPosition(FieldConstants.REEF_RED_CENTER, Constants.CORAL_SCORING_CIRCLE_RADIUS)
                                .or(drivetrain.onPosition(
                                        FieldConstants.REEF_BLUE_CENTER, Constants.CORAL_SCORING_CIRCLE_RADIUS)))
                        .negate())
                .and(system.isInCoralIntakeMode().negate())
                .onTrue(ComplexCommands.prepareGamepieceForScoring().withName("prep if within circle"));

        system.canSetScoringComplete()
                .and(system.robotWithinReefContact().negate())
                .onTrue(ComplexCommands.finishSetArmForReefActionWhenSafe());

        // Auto Align ALGAE scoring Barge and reef
        driverController
                .zl(0.8)
                .and(driverController.zr(0.8))
                .whileTrue(new ConditionalCommand(
                        system.setAlgaeIntakeActive(true)
                                .andThen(autoAlignPair(
                                                FieldConstants.ALGAE_BOTH_REEFS,
                                                FieldConstants.ALGAE_BOTH_REEFS,
                                                PATH_CONSTRAINTS_REEF_AUTO_ALIGN,
                                                AUTO_ALIGN_LOOK_AHEAD_TIME_REEF)
                                        .until(algaeGripper.holdingGamePiece())),
                        autoAlign(
                                FieldConstants.BARGE_RED,
                                FieldConstants.BARGE_BLUE,
                                PATH_CONSTRAINTS_BARGE_AUTO_ALIGN,
                                false,
                                AUTO_ALIGN_LOOK_AHEAD_TIME_BARGE),
                        system.isInAlgaeIntakeMode()))
                .whileFalse(system.setAlgaeIntakeActive(false));

        // elevator and pivot sequence ALGAE intake Reef
        driverController
                .zl(0.8)
                .and(driverController.zr(0.8))
                .onTrue(new ConditionalCommand(
                                new SelectCommand<>(
                                                Map.ofEntries(
                                                        Map.entry(
                                                                FieldConstants.AlgaePosition.HIGH,
                                                                ComplexCommands.requestAlgae3IntakeCommand()),
                                                        Map.entry(
                                                                FieldConstants.AlgaePosition.LOW,
                                                                ComplexCommands.requestAlgae2IntakeCommand())),
                                                () -> GeometryUtil.getClosestFuturePair(
                                                                drivetrain
                                                                        .getRobotPoseSupplier()
                                                                        .get(),
                                                                drivetrain
                                                                        .getRobotRelativeSpeedsSupplier()
                                                                        .get(),
                                                                AUTO_ALIGN_LOOK_AHEAD_TIME_REEF,
                                                                ALGAE_BOTH_REEFS)
                                                        .getSecond())
                                        .andThen(new ConditionalCommand(
                                                ComplexCommands.safeSetArmForReefActionWhenArmDown(),
                                                ComplexCommands.safeSetArmForReefActionWhenArmUp(),
                                                pivot.isPivotDown()))
                                        .alongWith(algaeGripper.inCommand()),
                                Commands.sequence(
                                        ComplexCommands.requestBargeCommand(),
                                        new WaitCommand(0.1),
                                        ComplexCommands.prepareGamepieceForScoring()),
                                system.isInAlgaeIntakeMode())
                        .withName("Algae"));

        isNearBarge()
                .and(system.isInAlgaeIntakeMode().negate())
                .and(driverController.zl())
                .and(driverController.zr())
                .onTrue(ComplexCommands.prepPivotBargeCommand()
                        .andThen(ComplexCommands.prepElevatorBargeCommand())
                        .withName("Near Barge"));

        system.isAlgaeIntakeActive()
                .whileFalse(new ConditionalCommand(
                        Commands.runOnce(() -> system.setRequestedArmState(system.getCoralArmState()))
                                .andThen(new WaitCommand(0.2)),
                        Commands.none(),
                        () -> ((system.getCurrentArmState() == System.ArmState.ALGAE2
                                        || system.getCurrentArmState() == System.ArmState.ALGAE3)
                                && (system.getRequestedArmState() == System.ArmState.ALGAE2
                                        || system.getRequestedArmState() == System.ArmState.ALGAE3))));

        driverController.b().whileTrue(algaeGripper.outCommand());

        // Manual Release
        driverController.r().onTrue(ComplexCommands.scoreGamePieceOrSuperCycleCommand());

        system.isInCoralIntakeMode().onTrue(ledController.setLEDPatternIntakeCommand());
        system.isInCoralIntakeMode().onFalse(ledController.disableLedPatternIntakeCommand());

        driverController.a().onTrue(feedThroughGripper.fixIntakeCommand());
        driverController.x().whileTrue(feedThroughGripper.slowIntakeCommand());
    }

    /** Configure the button bindings for the operator controller. */
    private void configureOperatorBindings() {
        operatorController
                .share()
                .and(isEndgameTrigger())
                .onTrue(ComplexCommands.goToClimbCommand()
                        .andThen(ComplexCommands.stayAtClimbCommand())
                        .withName("Arm go to climb position")
                        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));

        operatorController
                .share()
                .and(isEndgameTrigger())
                .onTrue(new WaitUntilCommand(() -> system.getCurrentArmState() == System.ArmState.CLIMBING)
                        .andThen(climber.readyClimbCommand())
                        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));

        operatorController
                .povDown()
                .or(operatorController.povDownRight())
                .or(operatorController.povDownLeft())
                .whileTrue(climber.climberInCommand());

        operatorController
                .povUp()
                .or(operatorController.povUpRight())
                .or(operatorController.povUpLeft())
                .whileTrue(climber.climberSlowlyOutCommand());

        operatorController.square().onTrue(ComplexCommands.requestL1Command());
        operatorController.cross().onTrue(ComplexCommands.requestL2Command());
        operatorController.circle().onTrue(ComplexCommands.requestL3Command());
        operatorController.triangle().onTrue(ComplexCommands.requestL4Command());
        operatorController.r3().onTrue(ComplexCommands.requestBargeCommand());

        getAnyButtonPressedByOperator()
                .and(system.robotWithinReefContact()
                        .negate()
                        .and(system.isInCoralIntakeMode().negate())
                        .and(system.isAlgaeIntakeActive().negate()))
                .onTrue(ComplexCommands.prepareGamepieceForScoring());
        getAnyButtonPressedByOperator().onTrue(ledController.disableManualOverrideOverlayCommand());

        operatorController.share().onTrue(recordAveragePose().ignoringDisable(true));

        operatorController.options().whileTrue(algaeGripper.outCommand());
        operatorController.l3().whileTrue(algaeGripper.inCommand().until(algaeGripper.holdingGamePiece()));

        // Manual goToStowed
        operatorController
                .axisGreaterThan(NaconPS4Controller.Axis.kL2.value, 0.4)
                .debounce(0.2)
                .and(system.isInCoralIntakeMode().negate())
                .onTrue(ComplexCommands.goToStowed());

        // Manual override
        operatorController
                .axisGreaterThan(NaconPS4Controller.Axis.kR2.value, 0.4)
                .debounce(0.2)
                .and(system.isInCoralIntakeMode().negate())
                .onTrue(ComplexCommands.prepareGamepieceForScoring()
                        .andThen(new ConditionalCommand(
                                ComplexCommands.prepPivotBargeCommand()
                                        .andThen(ComplexCommands.prepElevatorBargeCommand()),
                                new ConditionalCommand(
                                        ComplexCommands.safeSetArmForReefActionWhenArmDown(),
                                        ComplexCommands.safeSetArmForReefActionWhenArmUp(),
                                        pivot.isPivotDown()),
                                () -> system.getCurrentArmState() == System.ArmState.BARGE_PREP)));

        // Manual Override Lampies
        operatorController
                .axisGreaterThan(NaconPS4Controller.Axis.kL2.value, 0.4)
                .debounce(0.2)
                .and(system.isInCoralIntakeMode().negate())
                .onTrue(ledController.enableManualOverrideOverlayCommand());

        // Manual Algae remove
        operatorController
                .r1()
                .onTrue(ComplexCommands.requestAlgae2IntakeCommand()
                        .andThen(system.setAlgaeAutoAlignModeToIntake())
                        .andThen(new ConditionalCommand(
                                ComplexCommands.safeSetArmForReefActionWhenArmDown(),
                                ComplexCommands.safeSetArmForReefActionWhenArmUp(),
                                pivot.isPivotDown()))
                        .andThen(algaeGripper.inCommand())
                        .andThen(system.setAlgaeIntakeActive(true))
                        .until(algaeGripper.holdingGamePiece()));

        operatorController
                .r1()
                .onTrue(ledController.enableManualOverrideOverlayCommand().withName("KAAS"));

        operatorController
                .l1()
                .onTrue(new ParallelCommandGroup(
                        ComplexCommands.requestAlgae3IntakeCommand()
                                .andThen(system.setAlgaeAutoAlignModeToIntake())
                                .andThen(new ConditionalCommand(
                                        ComplexCommands.safeSetArmForReefActionWhenArmDown(),
                                        ComplexCommands.safeSetArmForReefActionWhenArmUp(),
                                        pivot.isPivotDown()))
                                .andThen(algaeGripper.inCommand())
                                .andThen(system.setAlgaeIntakeActive(true))
                                .until(algaeGripper.holdingGamePiece()),
                        ledController.enableManualOverrideOverlayCommand()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoSelector.getAutonomousCommand();
    }

    /**
     * Determines if the robot is on the red alliance.
     *
     * @return True if the robot is on the red alliance, false otherwise.
     */
    private boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red);
    }

    /**
     * Get the closest pose based on the alliance color.
     *
     * @param redPoses The poses for the red alliance.
     * @param bluePoses The poses for the blue alliance.
     * @return Return the pose corresponding to which alliance we are currently on
     */
    private Pose2d[] getAllianceBasedClosestPose(Pose2d[] redPoses, Pose2d[] bluePoses) {
        return isRedAlliance() ? redPoses : bluePoses;
    }

    /**
     * Get the closest Pair based on the alliance color.
     *
     * @param redPoses The Pairs for the red alliance.
     * @param bluePoses The Pairs for the blue alliance.
     * @return Return the Pair corresponding to which alliance we are currently on
     */
    private Pair<Pose2d, FieldConstants.AlgaePosition>[] getAllianceBasedClosestPose(
            Pair<Pose2d, FieldConstants.AlgaePosition>[] redPoses,
            Pair<Pose2d, FieldConstants.AlgaePosition>[] bluePoses) {
        return isRedAlliance() ? redPoses : bluePoses;
    }

    /**
     * Command to align on a given list of poses based on the alliance color and closet pose.
     *
     * @param targetRed The poses for the red alliance.
     * @param targetBlue The poses for the blue alliance.
     * @param constraints The path constraints.
     * @param endReverse Whether the robot should approach the end pose in reverse.
     * @param autoAlignLookAheadTime The time difference between now and the desired measurement point in seconds.
     * @return The command to align the robot to the closest pose.
     */
    private Command autoAlign(
            Pose2d[] targetRed,
            Pose2d[] targetBlue,
            PathConstraints constraints,
            boolean endReverse,
            double autoAlignLookAheadTime) {
        return drivetrain.followTrajectory(
                () -> new PathPlannerTrajectoryAdapter(ppTrajectoryGenerator.generateFromCurrentPose(
                        GeometryUtil.getClosestFuturePose(
                                drivetrain.getRobotPoseSupplier().get(),
                                drivetrain.getRobotRelativeSpeedsSupplier().get(),
                                autoAlignLookAheadTime,
                                getAllianceBasedClosestPose(targetRed, targetBlue)),
                        constraints,
                        endReverse)),
                TRAJECTORY_TRANSLATION_STATIC_KP_TELEOP,
                TRAJECTORY_TRANSLATION_DYNAMIC_KP_TELEOP,
                TRAJECTORY_ROTATION_KP_TELEOP,
                AUTO_ALIGN_DISTANCE_TOLERANCE,
                AUTO_ALIGN_ANGULAR_TOLERANCE);
    }

    /**
     * Command to align on a given list of poses based on the alliance color, closet pose and arm state. Takes into
     * account a custom offset for each scoring position
     *
     * @param targetRed The poses for the red alliance.
     * @param targetBlue The poses for the blue alliance.
     * @param armState The arm state.
     * @param constraints The path constraints.
     * @param autoAlignLookAheadTime The time difference between now and the desired measurement point in seconds.
     * @return The command to align the robot to the closest pose.
     */
    private Command autoAlignRight(
            Pose2d[] targetRed,
            Pose2d[] targetBlue,
            Supplier<System.ArmState> armState,
            PathConstraints constraints,
            double autoAlignLookAheadTime) {
        return drivetrain.followTrajectory(
                () -> new PathPlannerTrajectoryAdapter(ppTrajectoryGenerator.generateFromCurrentPose(
                        GeometryUtil.getClosestFuturePose(
                                        drivetrain.getRobotPoseSupplier().get(),
                                        drivetrain
                                                .getRobotRelativeSpeedsSupplier()
                                                .get(),
                                        autoAlignLookAheadTime,
                                        getAllianceBasedClosestPose(targetRed, targetBlue))
                                .transformBy(REEF_OFFSET_MAP_RIGHT.getOrDefault(armState.get(), new Transform2d())),
                        constraints)),
                TRAJECTORY_TRANSLATION_STATIC_KP_TELEOP,
                TRAJECTORY_TRANSLATION_DYNAMIC_KP_TELEOP,
                TRAJECTORY_ROTATION_KP_TELEOP,
                AUTO_ALIGN_DISTANCE_TOLERANCE,
                AUTO_ALIGN_ANGULAR_TOLERANCE);
    }

    private Command autoAlignLeft(
            Pose2d[] targetRed,
            Pose2d[] targetBlue,
            Supplier<System.ArmState> armState,
            PathConstraints constraints,
            double autoAlignLookAheadTime) {
        return drivetrain.followTrajectory(
                () -> new PathPlannerTrajectoryAdapter(ppTrajectoryGenerator.generateFromCurrentPose(
                        GeometryUtil.getClosestFuturePose(
                                        drivetrain.getRobotPoseSupplier().get(),
                                        drivetrain
                                                .getRobotRelativeSpeedsSupplier()
                                                .get(),
                                        autoAlignLookAheadTime,
                                        getAllianceBasedClosestPose(targetRed, targetBlue))
                                .transformBy(REEF_OFFSET_MAP_LEFT.getOrDefault(armState.get(), new Transform2d())),
                        constraints)),
                TRAJECTORY_TRANSLATION_STATIC_KP_TELEOP,
                TRAJECTORY_TRANSLATION_DYNAMIC_KP_TELEOP,
                TRAJECTORY_ROTATION_KP_TELEOP,
                AUTO_ALIGN_DISTANCE_TOLERANCE,
                AUTO_ALIGN_ANGULAR_TOLERANCE);
    }

    /**
     * Command to align on a given list of Pairs based on the alliance color and closest Pair.
     *
     * @param targetRed The Pairs for the red alliance.
     * @param targetBlue The Pairs for the blue alliance.
     * @param constraints The path constraints.
     * @param autoAlignLookAheadTime The time difference between now and the desired measurement point in seconds.
     * @return The command to align the robot to the closest Pairs.
     */
    private Command autoAlignPair(
            Pair<Pose2d, FieldConstants.AlgaePosition>[] targetRed,
            Pair<Pose2d, FieldConstants.AlgaePosition>[] targetBlue,
            PathConstraints constraints,
            double autoAlignLookAheadTime) {
        return drivetrain.followTrajectory(
                () -> new PathPlannerTrajectoryAdapter(ppTrajectoryGenerator.generateFromCurrentPose(
                        GeometryUtil.getClosestFuturePair(
                                        drivetrain.getRobotPoseSupplier().get(),
                                        drivetrain
                                                .getRobotRelativeSpeedsSupplier()
                                                .get(),
                                        autoAlignLookAheadTime,
                                        getAllianceBasedClosestPose(targetRed, targetBlue))
                                .getFirst(),
                        constraints)),
                TRAJECTORY_TRANSLATION_STATIC_KP_TELEOP,
                TRAJECTORY_TRANSLATION_DYNAMIC_KP_TELEOP,
                TRAJECTORY_ROTATION_KP_TELEOP,
                AUTO_ALIGN_DISTANCE_TOLERANCE,
                AUTO_ALIGN_ANGULAR_TOLERANCE);
    }

    /**
     * Command to align on a given list of Pairs<Pose2d, AlgaePositions> based on the alliance color and closet
     * Pair<Pose2d, AlgaePositions> with an x and y direction transform
     *
     * @param targetRed The Pairs<Pose2d, AlgaePositions> for the red alliance.
     * @param targetBlue The Pairs<Pose2d, AlgaePositions> for the blue alliance.
     * @param transformX The transform in the x direction.
     * @param transformY The transform in the y direction.
     * @param constraints The path constraints.
     * @return The command to align the robot to the closest Pairs<Pose2d, AlgaePositions>.
     */
    private Command autoAlignPairTransform(
            Pair<Pose2d, FieldConstants.AlgaePosition>[] targetRed,
            Pair<Pose2d, FieldConstants.AlgaePosition>[] targetBlue,
            double transformX,
            double transformY,
            PathConstraints constraints) {
        return drivetrain.followTrajectory(
                () -> new PathPlannerTrajectoryAdapter(ppTrajectoryGenerator.generateFromCurrentPose(
                        GeometryUtil.getClosestPair(
                                        drivetrain.getRobotPoseSupplier().get(),
                                        getAllianceBasedClosestPose(targetRed, targetBlue))
                                .getFirst()
                                .transformBy(new Transform2d(transformX, transformY, new Rotation2d(0))),
                        constraints)),
                TRAJECTORY_TRANSLATION_STATIC_KP_TELEOP,
                TRAJECTORY_TRANSLATION_DYNAMIC_KP_TELEOP,
                TRAJECTORY_ROTATION_KP_TELEOP,
                AUTO_ALIGN_DISTANCE_TOLERANCE,
                AUTO_ALIGN_ANGULAR_TOLERANCE);
    }

    private Command updateArmStateLEDS() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(System.ArmState.L1, ledController.setLEDPatternL1Command()),
                        Map.entry(System.ArmState.L2, ledController.setLEDPatternL2Command()),
                        Map.entry(System.ArmState.L3, ledController.setLEDPatternL3Command()),
                        Map.entry(System.ArmState.L4, ledController.setLEDPatternL4Command()),
                        Map.entry(System.ArmState.BARGE, ledController.setLEDPatternBargeCommand())),
                system::getRequestedArmState);
    }

    /**
     * Command that returns true if any button that changes the position of the elevator or pivot is pressed by the
     * operator
     *
     * @return command that returns true if any button that changes the position of the elevator or pivot is pressed by
     *     the operator
     */
    private Trigger getAnyButtonPressedByOperator() {
        return operatorController
                .cross()
                .or(operatorController.circle())
                .or(operatorController.triangle())
                .or(operatorController.square())
                .or(operatorController.r3());
    }

    private Trigger isDisabledTrigger() {
        return new Trigger(DriverStation::isDisabled);
    }

    private Trigger isTeleopTrigger() {
        return new Trigger(DriverStation::isTeleop);
    }

    private Trigger isEndgameTrigger() {
        return new Trigger(() -> DriverStation.getMatchTime() < 25);
    }

    private Trigger isNearBarge() {
        return new Trigger(() -> isRedAlliance()
                ? MathUtil.isNear(
                                PREP_LINE_BARGE,
                                drivetrain.getRobotPoseSupplier().get().getX(),
                                ELEVATOR_SAFEZONE_BARGE)
                        && drivetrain.getRobotPoseSupplier().get().getY() < (FieldConstants.FIELD_WIDTH / 2)
                : MathUtil.isNear(
                                PREP_LINE_BARGE,
                                drivetrain.getRobotPoseSupplier().get().getX(),
                                ELEVATOR_SAFEZONE_BARGE)
                        && drivetrain.getRobotPoseSupplier().get().getY() > (FieldConstants.FIELD_WIDTH / 2));
    }

    /**
     * Command that sets the rumble of the driver controller
     *
     * @param value the value of the rumble
     * @param duration the duration of the rumble
     * @return command that sets the rumble of the driver controller
     */
    private Command setDriverRumble(double value, double duration) {
        return Commands.run(() -> driverController.setRumble(GenericHID.RumbleType.kBothRumble, value))
                .withTimeout(duration)
                .finallyDo(() -> driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0))
                .withName("setDriverRumble");
    }

    /**
     * Get the rotation that the robot should have when driving forward with respect to the current alliance station
     *
     * @return The rotation that the robot should have when driving forward
     */
    private Rotation2d getForwardRotation() {
        return isRedAlliance() ? new Rotation2d(Math.PI) : new Rotation2d();
    }

    /**
     * Command that records the average pose of the robot. It is filtered using a moving average. This is useful for
     * recording the robot pose at auto align positions during calibration.
     *
     * @return command that records the average pose of the robot
     */
    private Command recordAveragePose() {
        return Commands.run(() -> {
                    Pose2d currentPose = drivetrain.getRobotPoseSupplier().get();
                    poseXFilter.calculate(currentPose.getTranslation().getX());
                    poseYFilter.calculate(currentPose.getTranslation().getY());
                    poseRotationFilter.calculate(currentPose.getRotation().getDegrees());
                })
                .withTimeout(LoggedRobot.defaultPeriodSecs * filterLength * 2)
                .beforeStarting(() -> {
                    poseXFilter.reset();
                    poseYFilter.reset();
                    poseRotationFilter.reset();
                })
                .finallyDo(() -> {
                    Logger.recordOutput("Pose snapshot/X", poseXFilter.lastValue());
                    Logger.recordOutput("Pose snapshot/Y", poseYFilter.lastValue());
                    Logger.recordOutput("Pose snapshot/Rotation (deg)", poseRotationFilter.lastValue());
                });
    }

    /**
     * Command that returns the deviation of the robot with a targetRotation of 45 degrees
     *
     * @return command that returns the deviation of the robot
     */
    private double onRotationDeviation() {
        double currentRotation =
                drivetrain.getRobotPoseSupplier().get().getRotation().getDegrees();

        return Math.abs((((currentRotation - AUTONOMOUS_PRESET_ANGLE.getDegrees()) % 90 + 90) % 90) - 45) / 45;
    }

    /** Configure the bindings for game visualization. */
    private void configureVisualizationBindings() {
        GameVisualizer visualizer = GameVisualizer.getInstance();

        // Visualize game pieces in the robot
        feedThroughGripper.coralPresent().onTrue(visualizer.setCoralPresent()).onFalse(visualizer.setCoralAbsent());
        algaeGripper.holdingGamePiece().onTrue(visualizer.setAlgaePresent()).onFalse(visualizer.setAlgaeAbsent());

        // Visualize scored coral
        feedThroughGripper
                .coralPresent()
                .onFalse(visualizer.scoreCoral(
                        () -> GeometryUtil.getClosestPose(
                                drivetrain.getRobotPoseSupplier().get(),
                                getAllianceBasedClosestPose(FieldConstants.REEF_RED, FieldConstants.REEF_BLUE)),
                        system::getCoralArmState));
    }

    private Trigger autoAlignReefFinishedTrigger() {
        return new Trigger(() -> autoAlignReefFinished);
    }
}
