/*
 * Copyright (c) 2024-2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.constants;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.teamrembrandts.control.FeedForwardGains;

/** Constants for the drivetrain */
public interface DriveConstants {

    /**
     * Get the configuration for a Spark motor controller for the front left drive motor.
     *
     * @return SparkBaseConfig the configuration for a Spark motor controller for the front left drive motor.
     */
    SparkBaseConfig getFrontLeftDriveSparkConfig();

    /**
     * Get the configuration for a Spark motor controller for the front right drive motor.
     *
     * @return SparkBaseConfig the configuration for a Spark motor controller for the front right drive motor.
     */
    SparkBaseConfig getFrontRightDriveSparkConfig();

    /**
     * Get the configuration for a Spark motor controller for the back left drive motor.
     *
     * @return SparkBaseConfig the configuration for a Spark motor controller for the back left drive motor.
     */
    SparkBaseConfig getBackLeftDriveSparkConfig();

    /**
     * Get the configuration for a Spark motor controller for the back right drive motor.
     *
     * @return SparkBaseConfig the configuration for a Spark motor controller for the back right drive motor.
     */
    SparkBaseConfig getBackRightDriveSparkConfig();

    /**
     * Get the configuration for a Spark motor controller for the turn motor.
     *
     * @return SparkBaseConfig the configuration for a Spark motor controller for the turn motor.
     */
    SparkBaseConfig getTurnSparkConfig();

    /**
     * Get the feedforward constants for the drive motors.
     *
     * @return FeedForwardGains the feedforward constants for the drive motors.
     */
    FeedForwardGains getDriveFF();

    /**
     * Get the feedforward constants for the turn motors.
     *
     * @return FeedForwardGains the feedforward constants for the turn motors.
     */
    FeedForwardGains getTurnFF();

    /**
     * Get the configuration parameters for the drivetrain.
     *
     * @return DrivetrainConfig the configuration parameters for the drivetrain.
     */
    DrivetrainConfig getDrivetrainConfig();

    /**
     * Get the CAN IDs for the motors and sensors.
     *
     * @return CanIdConfig the CAN IDs for the motors and sensors.
     */
    CanIdConfig getCanIdConfig();

    /**
     * Get the configuration for the trajectory follower.
     *
     * @return TrajectoryFollowerConfig the configuration for the trajectory follower.
     */
    TrajectoryFollowerConfig getTrajectoryFollowerConfig();

    /**
     * Get the robot config for path planner path generation
     *
     * @return Robotconfig
     */
    RobotConfig getPathPlannerRobotConfig();

    /**
     * Configuration parameters of the drivetrain
     *
     * @param trackWidth The distance between the left and right wheels.
     * @param wheelBase The distance between the front and back wheels.
     * @param maxLinearVelocity The maximum linear velocity of the robot in m/s.
     * @param maxLinearAcceleration The maximum linear acceleration of the robot in m/s.
     * @param maxAngularVelocity The maximum angular velocity of the robot in rad/s.
     * @param odometryFrequency The frequency at which the odometry is updated.
     */
    record DrivetrainConfig(
            double trackWidth,
            double wheelBase,
            double maxLinearVelocity,
            double maxLinearAcceleration,
            double maxAngularVelocity,
            double odometryFrequency) {}

    /**
     * Configuration parameters for the trajectory follower.
     *
     * @param translationKp The proportional gain for the translation controller.
     * @param rotationKp The proportional gain for the rotation controller.
     */
    record TrajectoryFollowerConfig(double translationKp, double rotationKp) {}

    /**
     * CAN IDs for the motors and sensors of the drivetrain.
     *
     * @param imu The CAN ID of the IMU.
     * @param frontLeftDrive The CAN ID of the front left drive motor.
     * @param frontRightDrive The CAN ID of the front right drive motor.
     * @param backLeftDrive The CAN ID of the back left drive motor.
     * @param backRightDrive The CAN ID of the back right drive motor.
     * @param frontLeftTurn The CAN ID of the front left turn motor.
     * @param frontRightTurn The CAN ID of the front right turn motor.
     * @param backLeftTurn The CAN ID of the back left turn motor.
     * @param backRightTurn The CAN ID of the back right turn motor.
     */
    record CanIdConfig(
            int imu,
            int frontLeftDrive,
            int frontRightDrive,
            int backLeftDrive,
            int backRightDrive,
            int frontLeftTurn,
            int frontRightTurn,
            int backLeftTurn,
            int backRightTurn) {}
}
