/*
 * Copyright (c) 2024-2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.constants;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.teamrembrandts.control.FeedForwardGains;
import com.teamrembrandts.control.PIDGains;
import com.teamrembrandts.hardware.constants.GearRatios;
import com.teamrembrandts.hardware.constants.SwerveInversions;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;

public class REVSwerveDriveConstants implements DriveConstants {
    private static final double ODOMETRY_FREQUENCY = 100.0;

    private static final double TRACK_WIDTH = 0.3606;
    private static final double WHEEL_BASE = 0.3606;
    private static final double DRIVEBASE_RADIUS = Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);

    private static final double MAX_VELOCITY = 4.5;
    private static final double MAX_ACCELERATION = 3.0;
    private static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / DRIVEBASE_RADIUS;

    // PathPlanner robot config parameters
    private static final double MASS = 55;
    private static final double MOMENT_OF_INERTIA = 7;
    private static final double COEFFICIENT_OF_FRICTION = 1;

    // CAN IDs
    private static final int CAN_ID_IMU = 9;

    private static final int CAN_ID_FRONT_LEFT_DRIVE = 11;
    private static final int CAN_ID_FRONT_RIGHT_DRIVE = 13;
    private static final int CAN_ID_BACK_LEFT_DRIVE = 15;
    private static final int CAN_ID_BACK_RIGHT_DRIVE = 17;

    private static final int CAN_ID_FRONT_LEFT_TURN = 12;
    private static final int CAN_ID_FRONT_RIGHT_TURN = 14;
    private static final int CAN_ID_BACK_LEFT_TURN = 16;
    private static final int CAN_ID_BACK_RIGHT_TURN = 18;

    // Motor constants
    private static final boolean FRONT_LEFT_INVERTED = SwerveInversions.RevRobotics.MaxSwerve.FRONT_LEFT_INVERTED;
    private static final boolean FRONT_RIGHT_INVERTED = SwerveInversions.RevRobotics.MaxSwerve.FRONT_RIGHT_INVERTED;
    private static final boolean BACK_LEFT_INVERTED = SwerveInversions.RevRobotics.MaxSwerve.BACK_LEFT_INVERTED;
    private static final boolean BACK_RIGHT_INVERTED = SwerveInversions.RevRobotics.MaxSwerve.BACK_RIGHT_INVERTED;
    private static final double WHEEL_DIAMETER = Units.Meter.convertFrom(3, Units.Inch);
    private static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
    private static final double DRIVE_MOTOR_REDUCTION = GearRatios.RevRobotics.MaxSwerve.REDUCTION_HIGH_SPEED;

    private static final int TURN_MOTOR_CURRENT_LIMIT = 30;

    // Encoder constants
    private static final double DRIVE_ENCODER_POSITION_FACTOR = Math.PI / DRIVE_MOTOR_REDUCTION * WHEEL_DIAMETER;
    private static final double DRIVE_ENCODER_VELOCITY_FACTOR = Math.PI / 60.0 / DRIVE_MOTOR_REDUCTION * WHEEL_DIAMETER;

    // Turn encoder configuration
    private static final double TURN_ENCODER_POSITION_FACTOR = 2 * Math.PI; // Rotations -> Radians
    private static final double TURN_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Closed-loop constants
    private static final PIDGains DRIVE_PID = new PIDGains(0, 0, 0);
    private static final FeedForwardGains DRIVE_FF = new FeedForwardGains(0.12249, 0.09722, 0);
    private static final PIDGains TURN_PID = new PIDGains(2, 0, 0);
    private static final FeedForwardGains TURN_FF = new FeedForwardGains(0.2, 0.6, 0);

    // Configuration variables
    private final SparkFlexConfig driveConfigFrontLeft = new SparkFlexConfig();
    private final SparkFlexConfig driveConfigFrontRight = new SparkFlexConfig();
    private final SparkFlexConfig driveConfigBackLeft = new SparkFlexConfig();
    private final SparkFlexConfig driveConfigBackRight = new SparkFlexConfig();

    private final SparkMaxConfig turnConfig = new SparkMaxConfig();

    // Trajectory follower config
    private static final double TRAJECTORY_FOLLOWER_TRANSLATION_KP = 1.6;
    private static final double TRAJECTORY_FOLLOWER_ROTATION_KP = 3;

    private final DrivetrainConfig drivetrainConfig;
    private final CanIdConfig canIdConfig;
    private final TrajectoryFollowerConfig trajectoryFollowerConfig;
    private final RobotConfig ppRobotConfig;

    public REVSwerveDriveConstants() {
        SparkFlexConfig driveConfigBase = new SparkFlexConfig();
        driveConfigBase
                .encoder
                .positionConversionFactor(DRIVE_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(DRIVE_ENCODER_VELOCITY_FACTOR)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        driveConfigBase
                .closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .pidf(DRIVE_PID.kP(), DRIVE_PID.kI(), DRIVE_PID.kD(), 0.0);
        driveConfigBase
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT)
                .voltageCompensation(12.0);
        driveConfigBase
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        // Apply the separate motor inversion settings
        driveConfigFrontLeft.apply(driveConfigBase);
        driveConfigFrontLeft.inverted(FRONT_LEFT_INVERTED);
        driveConfigFrontRight.apply(driveConfigBase);
        driveConfigFrontRight.inverted(FRONT_RIGHT_INVERTED);
        driveConfigBackLeft.apply(driveConfigBase);
        driveConfigBackLeft.inverted(BACK_LEFT_INVERTED);
        driveConfigBackRight.apply(driveConfigBase);
        driveConfigBackRight.inverted(BACK_RIGHT_INVERTED);

        turnConfig
                .absoluteEncoder
                .inverted(SwerveInversions.RevRobotics.MaxSwerve.REV_THROUGH_BORE_ENCODER_INVERTED)
                .positionConversionFactor(TURN_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(TURN_ENCODER_VELOCITY_FACTOR)
                .averageDepth(2);
        turnConfig
                .closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI)
                .pidf(TURN_PID.kP(), TURN_PID.kI(), TURN_PID.kD(), 0.0);
        turnConfig
                .inverted(SwerveInversions.RevRobotics.MaxSwerve.TURN_MOTOR_INVERTED)
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(TURN_MOTOR_CURRENT_LIMIT)
                .voltageCompensation(12.0);
        turnConfig
                .signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        drivetrainConfig = new DrivetrainConfig(
                TRACK_WIDTH, WHEEL_BASE, MAX_VELOCITY, MAX_ACCELERATION, MAX_ANGULAR_VELOCITY, ODOMETRY_FREQUENCY);

        trajectoryFollowerConfig =
                new TrajectoryFollowerConfig(TRAJECTORY_FOLLOWER_TRANSLATION_KP, TRAJECTORY_FOLLOWER_ROTATION_KP);

        canIdConfig = new CanIdConfig(
                CAN_ID_IMU,
                CAN_ID_FRONT_LEFT_DRIVE,
                CAN_ID_FRONT_RIGHT_DRIVE,
                CAN_ID_BACK_LEFT_DRIVE,
                CAN_ID_BACK_RIGHT_DRIVE,
                CAN_ID_FRONT_LEFT_TURN,
                CAN_ID_FRONT_RIGHT_TURN,
                CAN_ID_BACK_LEFT_TURN,
                CAN_ID_BACK_RIGHT_TURN);

        ppRobotConfig = new RobotConfig(
                MASS,
                MOMENT_OF_INERTIA,
                new ModuleConfig(
                        WHEEL_DIAMETER / 2,
                        MAX_VELOCITY,
                        COEFFICIENT_OF_FRICTION,
                        DCMotor.getNeoVortex(1),
                        DRIVE_MOTOR_REDUCTION,
                        DRIVE_MOTOR_CURRENT_LIMIT,
                        1),
                new Translation2d(TRACK_WIDTH / 2, WHEEL_BASE / 2),
                new Translation2d(TRACK_WIDTH / 2, -WHEEL_BASE / 2),
                new Translation2d(-TRACK_WIDTH / 2, WHEEL_BASE / 2),
                new Translation2d(-TRACK_WIDTH / 2, -WHEEL_BASE / 2));
    }

    @Override
    public SparkBaseConfig getFrontLeftDriveSparkConfig() {
        return driveConfigFrontLeft;
    }

    @Override
    public SparkBaseConfig getFrontRightDriveSparkConfig() {
        return driveConfigFrontRight;
    }

    @Override
    public SparkBaseConfig getBackLeftDriveSparkConfig() {
        return driveConfigBackLeft;
    }

    @Override
    public SparkBaseConfig getBackRightDriveSparkConfig() {
        return driveConfigBackRight;
    }

    @Override
    public SparkBaseConfig getTurnSparkConfig() {
        return turnConfig;
    }

    @Override
    public FeedForwardGains getDriveFF() {
        return DRIVE_FF;
    }

    @Override
    public FeedForwardGains getTurnFF() {
        return TURN_FF;
    }

    @Override
    public DrivetrainConfig getDrivetrainConfig() {
        return drivetrainConfig;
    }

    @Override
    public TrajectoryFollowerConfig getTrajectoryFollowerConfig() {
        return trajectoryFollowerConfig;
    }

    @Override
    public CanIdConfig getCanIdConfig() {
        return canIdConfig;
    }

    @Override
    public RobotConfig getPathPlannerRobotConfig() {
        return ppRobotConfig;
    }
}
