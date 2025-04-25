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

import static org.littletonrobotics.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.teamrembrandts.control.FeedForwardGains;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Swerve Module IO implementation for REV Spark motor controllers. Feedback sensors are assumed to be connected to the
 * Spark motor controllers. Absolute encoder offsets should be configured in the REV Hardware Client.
 */
public class SparkSwerveModuleIO implements SwerveModuleIO {

    // Hardware objects
    private final SparkBase driveMotor;
    private final SparkBase turnMotor;

    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnAbsoluteEncoder;
    private final RelativeEncoder turnRelativeEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    private final FeedForwardGains driveFFGains;
    private final FeedForwardGains turnFFGains;

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

    /**
     * Creates a new {@code SparkSwerveModule}.
     *
     * @param driveMotor The Spark motor controller for the drive motor.
     * @param turnMotor The Spark motor controller for the turn motor.
     * @param driveConfig The configuration for the drive motor controller.
     * @param turnConfig The configuration for the turn motor controller.
     * @param driveFFGains The feedforward gains for the drive motor controller.
     * @param turnFFGains The feedforward gains for the turn motor controller.
     */
    public SparkSwerveModuleIO(
            SparkBase driveMotor,
            SparkBase turnMotor,
            SparkBaseConfig driveConfig,
            SparkBaseConfig turnConfig,
            FeedForwardGains driveFFGains,
            FeedForwardGains turnFFGains) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;

        this.driveFFGains = driveFFGains;
        this.turnFFGains = turnFFGains;

        driveEncoder = driveMotor.getEncoder();
        turnAbsoluteEncoder = turnMotor.getAbsoluteEncoder();
        turnRelativeEncoder = turnMotor.getEncoder();
        driveController = driveMotor.getClosedLoopController();
        turnController = turnMotor.getClosedLoopController();

        // Configure the drive and turn motors
        tryUntilOk(
                driveMotor,
                5,
                () -> driveMotor.configure(
                        driveConfig,
                        SparkBase.ResetMode.kResetSafeParameters,
                        SparkBase.PersistMode.kPersistParameters));
        tryUntilOk(driveMotor, 5, () -> driveEncoder.setPosition(0.0));

        tryUntilOk(
                turnMotor,
                5,
                () -> turnMotor.configure(
                        turnConfig,
                        SparkBase.ResetMode.kResetSafeParameters,
                        SparkBase.PersistMode.kPersistParameters));
        // Create odometry queues
        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveMotor, driveEncoder::getPosition);
        turnPositionQueue =
                SparkOdometryThread.getInstance().registerSignal(turnMotor, turnAbsoluteEncoder::getPosition);
    }

    /** {@inheritDoc} */
    public void updateInputs(SwerveModuleInputs inputs) {
        // Update drive inputs
        sparkStickyFault = false;
        ifOk(driveMotor, driveEncoder::getPosition, (value) -> inputs.drivePosition = value);
        ifOk(driveMotor, driveEncoder::getVelocity, (value) -> inputs.driveVelocity = value);
        ifOk(
                driveMotor,
                new DoubleSupplier[] {driveMotor::getAppliedOutput, driveMotor::getBusVoltage},
                (values) -> inputs.driveAppliedVoltage = values[0] * values[1]);
        ifOk(driveMotor, driveMotor::getOutputCurrent, (value) -> inputs.driveCurrent = value);
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

        // Update turn inputs
        sparkStickyFault = false;
        ifOk(
                turnMotor,
                turnAbsoluteEncoder::getPosition,
                (value) -> inputs.turnAbsoluteRotation = new Rotation2d(value));
        ifOk(turnMotor, turnRelativeEncoder::getVelocity, (value) -> inputs.turnAngularVelocity = value);
        ifOk(
                turnMotor,
                new DoubleSupplier[] {turnMotor::getAppliedOutput, turnMotor::getBusVoltage},
                (values) -> inputs.turnAppliedVoltage = values[0] * values[1]);
        ifOk(turnMotor, turnMotor::getOutputCurrent, (value) -> inputs.turnCurrent = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        // Update odometry inputs
        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositions =
                drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions =
                turnPositionQueue.stream().map(Rotation2d::new).toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    /** {@inheritDoc} */
    @Override
    public void setDriveState(double velocity, double acceleration) {
        // If acceleration is in same direction as velocity we are 'accelerating' (velocity is increasing in magnitude)
        // Otherwise, we are decelerating. For both cases, a different kA can be used.
        double kA = (Math.signum(velocity) == Math.signum(acceleration))
                ? driveFFGains.kAAcceleration()
                : driveFFGains.kADeceleration();
        double ff = driveFFGains.kS() * Math.signum(velocity) + driveFFGains.kV() * velocity + kA * acceleration;
        driveController.setReference(
                velocity,
                SparkBase.ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                ff,
                SparkClosedLoopController.ArbFFUnits.kVoltage);
    }

    /** {@inheritDoc} */
    @Override
    public void setTurnState(double rotation, double angularVelocity) {
        double direction = Math.abs(angularVelocity) > 1e-5 ? Math.signum(angularVelocity) : 0;
        double ff = turnFFGains.kS() * direction + turnFFGains.kV() * angularVelocity;
        turnController.setReference(
                rotation,
                SparkBase.ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                ff,
                SparkClosedLoopController.ArbFFUnits.kVoltage);
    }

    /** {@inheritDoc} */
    @Override
    public void setDriveOpenLoop(double voltage) {
        driveMotor.setVoltage(voltage);
    }

    /** {@inheritDoc} */
    @Override
    public void setTurnOpenLoop(double voltage) {
        turnMotor.setVoltage(voltage);
    }
}
