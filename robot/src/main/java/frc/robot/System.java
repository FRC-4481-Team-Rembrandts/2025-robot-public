/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot;

import static frc.robot.Constants.DRIVE_CONSTANTS;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.teamrembrandts.simulation.VisionEnvironmentSimulator;
import com.teamrembrandts.subsystems.drive.*;
import com.teamrembrandts.subsystems.vision.RealPhotonVisionIO;
import com.teamrembrandts.subsystems.vision.ReplayVisionIO;
import com.teamrembrandts.subsystems.vision.SimulatedPhotonVisionIO;
import com.teamrembrandts.subsystems.vision.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.algaegripper.*;
import frc.robot.subsystems.climber.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.RealElevatorIO;
import frc.robot.subsystems.elevator.SimulatedElevatorIO;
import frc.robot.subsystems.feedthroughgripper.FeedThroughGripper;
import frc.robot.subsystems.feedthroughgripper.FeedThroughGripperIO;
import frc.robot.subsystems.feedthroughgripper.RealFeedThroughGripperIO;
import frc.robot.subsystems.feedthroughgripper.SimulatedFeedThroughGripperIO;
import frc.robot.subsystems.led.LEDController;
import frc.robot.subsystems.led.LEDControllerIO;
import frc.robot.subsystems.led.RealLEDControllerIO;
import frc.robot.subsystems.led.SimulatedLEDControllerIO;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.RealPivotIO;
import frc.robot.subsystems.pivot.SimulatedPivotIO;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Singleton that represents the entire robot system. Contains a single instance of each subsystem. This class should be
 * used to access subsystems instead of creating them directly.
 */
public final class System {

    private static System instance = null;

    // Subsystems
    private final Drive drivetrain;
    private final Vision vision;
    private final Elevator elevator;
    private final Pivot pivot;
    private final AlgaeGripper algaeGripper;
    private final LEDController ledController;
    private final FeedThroughGripper feedThroughGripper;
    private final Climber climber;

    @AutoLogOutput
    private boolean coralIntakeMode = false;

    private boolean algaeIntakeMode = true;
    private boolean algaeIntakingActive = false;

    private boolean canSetScoreComplete = false;

    @AutoLogOutput
    public ArmState requestedArmState = ArmState.L4;

    @AutoLogOutput
    public ArmState currentArmState = ArmState.STOWED;

    @AutoLogOutput
    public ArmState coralArmState = ArmState.L4;

    /** Creates a new instance of the System class. */
    private System() {
        drivetrain = createDrive();
        vision = createVision();

        // Switch to define all subsystems
        switch (Constants.MODE) {
            case REAL -> {
                elevator = new Elevator(new RealElevatorIO());
                pivot = new Pivot(new RealPivotIO());
                algaeGripper = new AlgaeGripper(new DeltaAlgaeGripperIO());
                ledController = new LEDController(new RealLEDControllerIO());
                feedThroughGripper = new FeedThroughGripper(new RealFeedThroughGripperIO());
                climber = new Climber(new RealClimberIO());
            }
            case SIM -> {
                elevator = new Elevator(new SimulatedElevatorIO());
                pivot = new Pivot(new SimulatedPivotIO());
                algaeGripper = new AlgaeGripper(new SimulatedAlgaeGripperIO());
                ledController = new LEDController(new SimulatedLEDControllerIO());
                feedThroughGripper = new FeedThroughGripper(new SimulatedFeedThroughGripperIO());
                climber = new Climber(new ClimberIO() {});
            }
            default -> {
                elevator = new Elevator(new ElevatorIO() {});
                pivot = new Pivot(new PivotIO() {});
                algaeGripper = new AlgaeGripper(new AlgaeGripperIO() {});
                ledController = new LEDController(new LEDControllerIO() {});
                feedThroughGripper = new FeedThroughGripper(new FeedThroughGripperIO() {});
                climber = new Climber(new ClimberIO() {});
            }
        }
    }

    public enum ArmState {
        L1,
        L2,
        L3,
        L4,
        L4_PREP,
        INTAKE,
        STOWED,
        ALGAE2,
        ALGAE3,
        PROCESSOR,
        BARGE,
        BARGE_PREP,
        CLIMBING
    }
    /**
     * Gets the single instance of the System class.
     *
     * @return The single instance of the System class.
     */
    public static System getInstance() {
        if (instance == null) {
            instance = new System();
        }
        return instance;
    }

    /**
     * get requested ArmState
     *
     * @return requested Armstate
     */
    public ArmState getRequestedArmState() {
        return requestedArmState;
    }

    /**
     * set requested ArmState
     *
     * @param requestedArmState requested ArmState
     */
    public void setRequestedArmState(ArmState requestedArmState) {
        this.requestedArmState = requestedArmState;
    }

    /**
     * get current ArmState
     *
     * @return current Armstate
     */
    public ArmState getCurrentArmState() {
        return currentArmState;
    }

    /**
     * set current ArmState
     *
     * @param currentArmState current ArmState
     */
    public void setCurrentArmState(ArmState currentArmState) {
        this.currentArmState = currentArmState;
    }

    /**
     * get previous armState
     *
     * @return previous ArmState
     */
    public ArmState getCoralArmState() {
        return coralArmState;
    }

    /**
     * set previous ArmState
     *
     * @param coralArmState previous ArmState
     */
    public void setCoralArmState(ArmState coralArmState) {
        this.coralArmState = coralArmState;
    }

    /**
     * Gets the drivetrain subsystem.
     *
     * @return The drivetrain subsystem.
     */
    public Drive getDrivetrain() {
        return drivetrain;
    }

    /**
     * Gets the vision subsystem.
     *
     * @return The vision subsystem.
     */
    public Vision getVision() {
        return vision;
    }

    /**
     * Gets the elevator subsystem.
     *
     * @return The elevator subsystem.
     */
    public Elevator getElevator() {
        return elevator;
    }

    /**
     * Gets the pivot subsystem.
     *
     * @return The pivot subsystem.
     */
    public Pivot getPivot() {
        return pivot;
    }

    /**
     * Gets the algae gripper subsystem.
     *
     * @return The algae gripper subsystem.
     */
    public AlgaeGripper getAlgaeGripper() {
        return algaeGripper;
    }

    /**
     * Gets the climber subsystem.
     *
     * @return The climber subsystem.
     */
    public Climber getClimber() {
        return climber;
    }

    /**
     * Gets the LED controller subsystem.
     *
     * @return The LED controller subsystem.
     */
    public LEDController getLEDController() {
        return ledController;
    }

    /**
     * Gets the feedthrough gripper subsystem.
     *
     * @return The feedthrough gripper subsystem.
     */
    public FeedThroughGripper getFeedThroughGripper() {
        return feedThroughGripper;
    }

    // true is coral, false is reef

    public void toggleCoralAutoAlignMode() {
        coralIntakeMode = !coralIntakeMode;
    }

    public Command setCoralAutoAlignModeToReef() {
        return Commands.runOnce(() -> coralIntakeMode = false);
    }

    public Trigger isInCoralIntakeMode() {
        return new Trigger(() -> coralIntakeMode);
    }

    public void toggleAlgaeAutoAlignMode() {
        algaeIntakeMode = !algaeIntakeMode;
    }

    public Command setAlgaeAutoAlignModeToBarge() {
        return Commands.runOnce(() -> algaeIntakeMode = false);
    }

    public Command setAlgaeAutoAlignModeToIntake() {
        return Commands.runOnce(() -> algaeIntakeMode = true);
    }

    public Trigger isInAlgaeIntakeMode() {
        return new Trigger(() -> algaeIntakeMode);
    }

    public Command setAlgaeIntakeActive(boolean active) {
        return Commands.runOnce(() -> algaeIntakingActive = active);
    }

    public Trigger isAlgaeIntakeActive() {
        return new Trigger(() -> algaeIntakingActive);
    }

    public Trigger robotWithinReefContact() {
        return new Trigger(drivetrain
                .onPosition(FieldConstants.REEF_RED_CENTER, Constants.SAFE_ARM_MOVEMENT_RADIUS)
                .or(drivetrain.onPosition(FieldConstants.REEF_BLUE_CENTER, Constants.SAFE_ARM_MOVEMENT_RADIUS)));
    }

    public void isSafeToFinishArmMovementForScoring(boolean canSetScoreComplete) {
        this.canSetScoreComplete = canSetScoreComplete;
    }

    public Trigger canSetScoringComplete() {
        return new Trigger(() -> canSetScoreComplete);
    }
    /**
     * Creates the drivetrain subsystem based on the current mode.
     *
     * @return The drivetrain subsystem.
     */
    private Drive createDrive() {
        DriveConstants.DrivetrainConfig drivetrainConfig = DRIVE_CONSTANTS.getDrivetrainConfig();
        DriveConstants.CanIdConfig canIdConfig = DRIVE_CONSTANTS.getCanIdConfig();

        return switch (Constants.MODE) {
            case REAL -> new SwerveDrive(
                    drivetrainConfig.trackWidth(),
                    drivetrainConfig.wheelBase(),
                    drivetrainConfig.maxLinearVelocity(),
                    drivetrainConfig.maxLinearAcceleration(),
                    drivetrainConfig.maxAngularVelocity(),
                    drivetrainConfig.odometryFrequency(),
                    new Pigeon2IMUIO(canIdConfig.imu()),
                    new SparkSwerveModuleIO(
                            new SparkFlex(canIdConfig.frontLeftDrive(), SparkLowLevel.MotorType.kBrushless),
                            new SparkFlex(canIdConfig.frontLeftTurn(), SparkLowLevel.MotorType.kBrushless),
                            DRIVE_CONSTANTS.getFrontLeftDriveSparkConfig(),
                            DRIVE_CONSTANTS.getTurnSparkConfig(),
                            DRIVE_CONSTANTS.getDriveFF(),
                            DRIVE_CONSTANTS.getTurnFF()),
                    new SparkSwerveModuleIO(
                            new SparkFlex(canIdConfig.frontRightDrive(), SparkLowLevel.MotorType.kBrushless),
                            new SparkFlex(canIdConfig.frontRightTurn(), SparkLowLevel.MotorType.kBrushless),
                            DRIVE_CONSTANTS.getFrontRightDriveSparkConfig(),
                            DRIVE_CONSTANTS.getTurnSparkConfig(),
                            DRIVE_CONSTANTS.getDriveFF(),
                            DRIVE_CONSTANTS.getTurnFF()),
                    new SparkSwerveModuleIO(
                            new SparkFlex(canIdConfig.backLeftDrive(), SparkLowLevel.MotorType.kBrushless),
                            new SparkFlex(canIdConfig.backLeftTurn(), SparkLowLevel.MotorType.kBrushless),
                            DRIVE_CONSTANTS.getBackLeftDriveSparkConfig(),
                            DRIVE_CONSTANTS.getTurnSparkConfig(),
                            DRIVE_CONSTANTS.getDriveFF(),
                            DRIVE_CONSTANTS.getTurnFF()),
                    new SparkSwerveModuleIO(
                            new SparkFlex(canIdConfig.backRightDrive(), SparkLowLevel.MotorType.kBrushless),
                            new SparkFlex(canIdConfig.backRightTurn(), SparkLowLevel.MotorType.kBrushless),
                            DRIVE_CONSTANTS.getBackRightDriveSparkConfig(),
                            DRIVE_CONSTANTS.getTurnSparkConfig(),
                            DRIVE_CONSTANTS.getDriveFF(),
                            DRIVE_CONSTANTS.getTurnFF()));
            case SIM -> new SwerveDrive(
                    drivetrainConfig.trackWidth(),
                    drivetrainConfig.wheelBase(),
                    drivetrainConfig.maxLinearVelocity(),
                    drivetrainConfig.maxLinearAcceleration(),
                    drivetrainConfig.maxAngularVelocity(),
                    drivetrainConfig.odometryFrequency(),
                    new IMUIO() {},
                    new SimulatedSwerveModuleIO(),
                    new SimulatedSwerveModuleIO(),
                    new SimulatedSwerveModuleIO(),
                    new SimulatedSwerveModuleIO());
            case REPLAY -> new SwerveDrive(
                    drivetrainConfig.trackWidth(),
                    drivetrainConfig.wheelBase(),
                    drivetrainConfig.maxLinearVelocity(),
                    drivetrainConfig.maxLinearAcceleration(),
                    drivetrainConfig.maxAngularVelocity(),
                    drivetrainConfig.odometryFrequency(),
                    new IMUIO() {},
                    new SwerveModuleIO() {},
                    new SwerveModuleIO() {},
                    new SwerveModuleIO() {},
                    new SwerveModuleIO() {});
            case XRP -> new DifferentialDrive(
                    0.155,
                    1,
                    1,
                    drivetrainConfig.odometryFrequency(),
                    new XRPIMUIO(),
                    new XRPDifferentialDriveIO(XRPDifferentialDriveIO.ModuleSide.LEFT),
                    new XRPDifferentialDriveIO(XRPDifferentialDriveIO.ModuleSide.RIGHT));
        };
    }

    /**
     * Creates the vision subsystem based on the current mode.
     *
     * @return The vision subsystem.
     */
    private Vision createVision() {
        switch (Constants.MODE) {
            case REAL:
                return new Vision(
                        VisionConstants.VISION_FILTER_PARAMETERS,
                        new RealPhotonVisionIO(
                                VisionConstants.CAMERA_FRONT_LEFT_NAME,
                                VisionConstants.ROBOT_TO_CAMERA_FRONT_LEFT,
                                VisionConstants.APRIL_TAG_FIELD_LAYOUT),
                        new RealPhotonVisionIO(
                                VisionConstants.CAMERA_FRONT_RIGHT_NAME,
                                VisionConstants.ROBOT_TO_CAMERA_FRONT_RIGHT,
                                VisionConstants.APRIL_TAG_FIELD_LAYOUT),
                        new RealPhotonVisionIO(
                                VisionConstants.CAMERA_BACK_LEFT_NAME,
                                VisionConstants.ROBOT_TO_CAMERA_BACK_LEFT,
                                VisionConstants.APRIL_TAG_FIELD_LAYOUT),
                        new RealPhotonVisionIO(
                                VisionConstants.CAMERA_BACK_RIGHT_NAME,
                                VisionConstants.ROBOT_TO_CAMERA_BACK_RIGHT,
                                VisionConstants.APRIL_TAG_FIELD_LAYOUT));
            case SIM:
                VisionEnvironmentSimulator.getInstance().addAprilTags(VisionConstants.APRIL_TAG_FIELD_LAYOUT);
                VisionEnvironmentSimulator.getInstance().addRobotPoseSupplier(drivetrain.getRobotPoseSupplier());

                return new Vision(
                        VisionConstants.VISION_FILTER_PARAMETERS,
                        new SimulatedPhotonVisionIO(
                                VisionConstants.CAMERA_FRONT_LEFT_NAME,
                                VisionConstants.ROBOT_TO_CAMERA_FRONT_LEFT,
                                VisionConstants.APRIL_TAG_FIELD_LAYOUT,
                                VisionConstants.CAMERA_SIM_PROPERTIES),
                        new SimulatedPhotonVisionIO(
                                VisionConstants.CAMERA_FRONT_RIGHT_NAME,
                                VisionConstants.ROBOT_TO_CAMERA_FRONT_RIGHT,
                                VisionConstants.APRIL_TAG_FIELD_LAYOUT,
                                VisionConstants.CAMERA_SIM_PROPERTIES),
                        new SimulatedPhotonVisionIO(
                                VisionConstants.CAMERA_BACK_LEFT_NAME,
                                VisionConstants.ROBOT_TO_CAMERA_BACK_LEFT,
                                VisionConstants.APRIL_TAG_FIELD_LAYOUT,
                                VisionConstants.CAMERA_SIM_PROPERTIES),
                        new SimulatedPhotonVisionIO(
                                VisionConstants.CAMERA_BACK_RIGHT_NAME,
                                VisionConstants.ROBOT_TO_CAMERA_BACK_RIGHT,
                                VisionConstants.APRIL_TAG_FIELD_LAYOUT,
                                VisionConstants.CAMERA_SIM_PROPERTIES));
            default:
                return new Vision(
                        VisionConstants.VISION_FILTER_PARAMETERS,
                        new ReplayVisionIO(VisionConstants.CAMERA_FRONT_LEFT_NAME),
                        new ReplayVisionIO(VisionConstants.CAMERA_FRONT_RIGHT_NAME),
                        new ReplayVisionIO(VisionConstants.CAMERA_BACK_LEFT_NAME),
                        new ReplayVisionIO(VisionConstants.CAMERA_BACK_RIGHT_NAME));
        }
    }
}
