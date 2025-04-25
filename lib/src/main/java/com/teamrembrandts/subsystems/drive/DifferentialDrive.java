/*
 * Copyright (c) 2024-2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.subsystems.drive;

import com.teamrembrandts.math.kinematics.ChassisState;
import com.teamrembrandts.subsystems.vision.VisionMeasurement;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

public class DifferentialDrive extends Drive {

    private final double trackWidthMeters;
    private final double maxVelocity;
    private final double maxAcceleration;

    private final DifferentialDriveIO leftModuleIO;
    private final DifferentialDriveInputsAutoLogged leftInputs = new DifferentialDriveInputsAutoLogged();
    private double leftPreviousPosition;
    private final DifferentialDriveIO rightModuleIO;
    private final DifferentialDriveInputsAutoLogged rightInputs = new DifferentialDriveInputsAutoLogged();
    private double rightPreviousPosition;

    private Rotation2d robotAngle = new Rotation2d();

    private final DifferentialDriveKinematics kinematics;

    private final DifferentialDrivePoseEstimator poseEstimator;

    public final Supplier<Pose2d> robotPoseSupplier;
    public final Consumer<VisionMeasurement> visionMeasurementConsumer;
    private double lastResetTime = -1.0;

    /**
     * Creates a new differential drive (tank drive) subsystem.
     *
     * @param trackWidthMeters The vertical distance between the wheels.
     * @param maxVelocity The maximum linear velocity of the drivetrain.
     * @param maxAcceleration The maximum linear acceleration of the drivetrain.
     * @param odometryThreadFrequency The frequency of the odometry thread.
     * @param imuIO The input/output that measures the robot angle.
     * @param leftModuleIO The IO for the left part of the drivetrain
     * @param rightModuleIO The IO for the right part of the drivetrain
     */
    public DifferentialDrive(
            double trackWidthMeters,
            double maxVelocity,
            double maxAcceleration,
            double odometryThreadFrequency,
            IMUIO imuIO,
            DifferentialDriveIO leftModuleIO,
            DifferentialDriveIO rightModuleIO) {
        super(imuIO, odometryThreadFrequency);

        this.trackWidthMeters = trackWidthMeters;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;

        this.leftModuleIO = leftModuleIO;
        this.rightModuleIO = rightModuleIO;

        leftPreviousPosition = leftInputs.position;
        rightPreviousPosition = rightInputs.position;

        kinematics = new DifferentialDriveKinematics(this.trackWidthMeters);
        poseEstimator = new DifferentialDrivePoseEstimator(
                kinematics, robotAngle, leftInputs.position, rightInputs.position, new Pose2d());

        // Define the consumers and suppliers
        robotPoseSupplier = poseEstimator::getEstimatedPosition;
        visionMeasurementConsumer = (measurement) -> poseEstimator.addVisionMeasurement(
                measurement.robotPose(), measurement.timestamp(), measurement.stdDevs());
    }

    @Override
    public void periodic() {
        ODOMETRY_LOCK.lock();
        super.periodic();
        ODOMETRY_LOCK.unlock();

        Logger.processInputs("DifferentialDrive/IMU", imuInputs);
        leftModuleIO.updateInputs(leftInputs);
        Logger.processInputs("DifferentialDrive/Left", leftInputs);
        rightModuleIO.updateInputs(rightInputs);
        Logger.processInputs("DifferentialDrive/Right", rightInputs);

        if (imuInputs.connected) {
            robotAngle = imuInputs.yawPosition;
        } else {
            // estimation, but better than no data at all
            double leftDeltaPosition = leftInputs.position - leftPreviousPosition;
            double rightDeltaPosition = rightInputs.position - rightPreviousPosition;

            Twist2d twist = kinematics.toTwist2d(leftDeltaPosition, rightDeltaPosition);
            robotAngle = robotAngle.plus(new Rotation2d(twist.dtheta));
        }

        poseEstimator.update(robotAngle, leftInputs.position, rightInputs.position);
        leftPreviousPosition = leftInputs.position;
        rightPreviousPosition = rightInputs.position;
    }

    /**
     * Sets the target speeds for the drivetrain. Speeds should be robot relative.
     *
     * @param targetSpeeds The target speeds for the drivetrain.
     */
    @Override
    protected void setTargetSpeeds(ChassisSpeeds targetSpeeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(targetSpeeds);
        wheelSpeeds.desaturate(getMaxVelocity());
        leftModuleIO.setDriveVelocity(wheelSpeeds.leftMetersPerSecond);
        rightModuleIO.setDriveVelocity(wheelSpeeds.rightMetersPerSecond);
    }

    /** {@inheritDoc} */
    @Override
    protected void setTargetState(ChassisState targetState) {
        setTargetSpeeds(targetState.getSpeeds());
    }

    /**
     * Gets the robot relative speeds of the drivetrain.
     *
     * @return The robot relative speeds of the drivetrain.
     */
    @Override
    protected ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftInputs.velocity, rightInputs.velocity));
    }

    /**
     * Gets the field relative speeds of the drivetrain.
     *
     * @return The field relative speeds of the drivetrain.
     */
    @Override
    protected ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                getRobotRelativeSpeeds(), poseEstimator.getEstimatedPosition().getRotation());
    }

    /**
     * Gets the pose of the drivetrain.
     *
     * @return The pose of the drivetrain.
     */
    @AutoLogOutput(key = "Odometry/Robot")
    @Override
    protected Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Gets the maximum velocity of the drivetrain.
     *
     * @return The maximum velocity of the drivetrain.
     */
    @Override
    protected double getMaxVelocity() {
        return maxVelocity;
    }

    /**
     * Gets the maximum angular velocity of the drivetrain.
     *
     * @return The maximum angular velocity of the drivetrain.
     */
    @Override
    protected double getMaxAngularVelocity() {
        return getMaxVelocity() / (trackWidthMeters / 2);
    }

    /**
     * Gets the maximum acceleration of the drivetrain.
     *
     * @return The maximum acceleration of the drivetrain.
     */
    @Override
    protected double getMaxAcceleration() {
        return maxAcceleration;
    }

    /**
     * Gets the maximum angular acceleration of the drivetrain.
     *
     * @return The maximum angular acceleration of the drivetrain.
     */
    @Override
    protected double getMaxAngularAcceleration() {
        return getMaxAcceleration() / (trackWidthMeters / 2);
    }

    /**
     * Get the supplier for the current robot pose
     *
     * @return The supplier for the current robot pose
     */
    @Override
    public Supplier<Pose2d> getRobotPoseSupplier() {
        return this::getPose;
    }

    /**
     * Get the consumer for the vision measurements, this is used by the vision subsystem to pass measurements to the
     * drive subsystem
     *
     * @return The consumer for the vision measurements
     */
    @Override
    public Consumer<VisionMeasurement> getVisionMeasurementConsumer() {
        return (measurement) -> poseEstimator.addVisionMeasurement(
                measurement.robotPose(), measurement.timestamp(), measurement.stdDevs());
    }

    /**
     * Get the supplier for the field relative speeds
     *
     * @return The supplier for the field relative speeds
     */
    @Override
    public Supplier<ChassisSpeeds> getFieldRelativeSpeedsSupplier() {
        return this::getFieldRelativeSpeeds;
    }

    /**
     * Get the supplier for the robot relative speeds
     *
     * @return The supplier for the robot relative speeds
     */
    @Override
    public Supplier<ChassisSpeeds> getRobotRelativeSpeedsSupplier() {
        return ChassisSpeeds::new;
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     *
     * @param xSupplier The supplier for the x-axis of the robot relative velocity.
     * @param ySupplier The supplier for the y-axis of the robot relative velocity.
     * @param omegaSupplier The supplier for the angular velocity.
     * @param deadband The deadband for the joysticks.
     */
    @Override
    public Command joystickDrive(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, double deadband) {
        return Commands.run(
                () -> {
                    double xSpeed = MathUtil.applyDeadband(xSupplier.getAsDouble(), deadband);
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), deadband);

                    xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
                    omega = Math.copySign(omega * omega, omega);

                    this.setTargetSpeeds(
                            new ChassisSpeeds(xSpeed * this.getMaxVelocity(), 0, omega * this.getMaxAngularVelocity()));
                },
                this);
    }

    /**
     * Sets the robot to a specified pose.
     *
     * @param poseSupplier The pose to reset to.
     */
    @Override
    public Command setRobotPose(Supplier<Pose2d> poseSupplier) {
        return Commands.runOnce(() -> {
            // Wait 6 code cycles before accepting poses again, this has been verified by means of simulation
            lastResetTime = Logger.getTimestamp() / 1e6 + LoggedRobot.defaultPeriodSecs * 6;
            poseEstimator.resetPosition(robotAngle, leftInputs.position, rightInputs.position, poseSupplier.get());
        });
    }

    /** {@inheritDoc} */
    @Override
    protected void runCharacterization(double voltage) {
        leftModuleIO.setDriveVoltage(voltage);
        rightModuleIO.setDriveVoltage(voltage);
    }

    /** {@inheritDoc} */
    @Override
    protected double getFFCharacterizationVelocity() {
        return (leftInputs.velocity + rightInputs.velocity) / 2;
    }

    /** {@inheritDoc} */
    @Override
    protected List<Double> getWheelRadiusCharacterizationPositions() {
        List<Double> positions = new LinkedList<>();
        positions.add(leftInputs.position);
        // For wheel radius characterization the robots turns in a circle, so one side turns backwards
        positions.add(-rightInputs.position);
        return positions;
    }
}
