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

import com.teamrembrandts.math.kinematics.NthOrderSwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

/** Class to represent an individual swerve module. */
public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();
    private final int id;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;

    private final double epsilon = 1e-5;

    /**
     * Creates a new swerve module.
     *
     * @param io The IO for the swerve module.
     * @param id The ID of the swerve module.
     */
    public SwerveModule(SwerveModuleIO io, int id) {
        this.io = io;
        this.id = id;

        driveDisconnectedAlert = new Alert(
                "Alerts/Swerve/Module" + id,
                "[Swerve Module " + id + "] Drive motor disconnected",
                Alert.AlertType.kError);
        turnDisconnectedAlert = new Alert(
                "Alerts/Swerve/Module" + id,
                "[Swerve Module " + id + "] Turn motor disconnected",
                Alert.AlertType.kError);
    }

    /** This method should be called every cycle */
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/Module" + id, inputs);

        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
        SmartDashboard.putBoolean("Connection Status/SwerveModule " + id + " Drive", inputs.driveConnected);
        SmartDashboard.putBoolean("Connection Status/SwerveModule " + id + " Turn", inputs.turnConnected);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositions[i];
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
    }

    /**
     * Sets the target state for the swerve module. It does not set the target state for the rotation if there is no
     * desired speed.
     *
     * @param state The target state for the swerve module.
     */
    public void setTargetState(NthOrderSwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) > epsilon) {
            io.setTurnState(state.angle.getRadians(), state.angularVelocity.in(Units.RadiansPerSecond));
        } else {
            io.setTurnOpenLoop(0.0);
        }
        state.cosineScale(getTurnAngle()); // Slow down the module if the error is too large
        io.setDriveState(state.speedMetersPerSecond, state.acceleration.in(Units.MetersPerSecondPerSecond));
    }

    /**
     * Gets the current state of the swerve module.
     *
     * @return The current state of the swerve module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getTurnAngle());
    }

    /**
     * Gets the position of the swerve module.
     *
     * @return The position of the swerve module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistance(), getTurnAngle());
    }

    /**
     * Get all the position of the swerve module retrieved by the odometry thread this cycle
     *
     * @return Array of the swerve module positions
     */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /**
     * Returns the timestamps of the samples received this cycle.
     *
     * @return the timestamps of the samples received this cycle
     */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    /**
     * Gets the absolute rotation angle of the module
     *
     * @return the absolute rotation angle from inputs
     */
    public Rotation2d getTurnAngle() {
        return inputs.turnAbsoluteRotation;
    }

    /**
     * Gets the angular velocity of the module in rad/s
     *
     * @return the angular velocity from inputs
     */
    public double getTurnAngularVelocity() {
        return inputs.turnAngularVelocity;
    }

    /**
     * Gets the drive distance of the module
     *
     * @return the drive position from inputs
     */
    public double getDriveDistance() {
        return inputs.drivePosition;
    }

    /**
     * Gets the drive velocity of the module
     *
     * @return the drive velocity from inputs
     */
    public double getDriveVelocity() {
        return inputs.driveVelocity;
    }

    /**
     * Set the voltage for the drive motor
     *
     * @param voltage the voltage to set
     */
    public void setDriveOpenLoop(double voltage) {
        io.setDriveOpenLoop(voltage);
    }

    /**
     * Set the voltage for the turn motor
     *
     * @param voltage the voltage to set
     */
    public void setTurnOpenLoop(double voltage) {
        io.setTurnOpenLoop(voltage);
    }

    /**
     * Set the target angle for the turn motor
     *
     * @param angle the angle to set
     */
    public void setTurnAngle(Rotation2d angle) {
        io.setTurnState(angle.getRadians(), 0);
    }
}
