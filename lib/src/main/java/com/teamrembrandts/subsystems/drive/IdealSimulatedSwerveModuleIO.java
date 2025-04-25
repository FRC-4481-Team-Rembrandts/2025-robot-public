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

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

/**
 * An ideal simulated swerve module IO. The IO is ideal, because all setpoints are instantaneously followed. This is not
 * meant to accurately represent a real swerve module, but to test out the swerve kinematics
 */
public class IdealSimulatedSwerveModuleIO implements SwerveModuleIO {
    double drivePosition = 0; // Drive wheel position in meters covered
    double driveVelocity = 0; // Desired velocity in m/s
    double driveAcceleration = 0; // Desired acceleration in m/s^2 of the wheel
    double turnPosition = 0; // Desired rotation of the wheel in rad
    double turnVelocity = 0; // Desired angular velocity in rad/s of the wheel
    double driveLastUpdated; // Timestamp that the drive setpoints were last updated in seconds
    double turnLastUpdated; // Timestamp that the turn setpoints were last updated in seconds

    /** Creates a new {@code IdealSimulatedSwerveModuleIO}. */
    public IdealSimulatedSwerveModuleIO() {
        driveLastUpdated = Logger.getTimestamp() / 1e6;
        turnLastUpdated = Logger.getTimestamp() / 1e6;
    }

    /**
     * Updates the system with new inputs from the module.
     *
     * @param inputs The new inputs.
     */
    @Override
    public void updateInputs(SwerveModuleInputs inputs) {

        updateStates();
        inputs.drivePosition = drivePosition;
        inputs.driveVelocity = driveVelocity;

        inputs.turnAbsoluteRotation = new Rotation2d(turnPosition);
        inputs.turnAngularVelocity = turnVelocity;

        // No high speed odometry thread is running in the sim, so just fill the arrays with the
        // current value
        inputs.odometryTimestamps = new double[] {Logger.getTimestamp() / 1e6};
        inputs.odometryDrivePositions = new double[] {inputs.drivePosition};
        inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnAbsoluteRotation};
    }

    private void updateStates() {
        // Assume wheel linear acceleration and angular velocity to be constant over the loop
        double dTdrive = Logger.getTimestamp() / 1e6 - driveLastUpdated;
        driveLastUpdated = Logger.getTimestamp() / 1e6;
        double dTturn = Logger.getTimestamp() / 1e6 - turnLastUpdated;
        turnLastUpdated = Logger.getTimestamp() / 1e6;

        drivePosition += driveVelocity * dTdrive + 0.5 * driveAcceleration * dTdrive * dTdrive;
        driveVelocity += driveAcceleration * dTdrive;
        turnPosition += turnVelocity * dTturn;
    }

    /** {@inheritDoc} */
    @Override
    public void setDriveState(double velocity, double acceleration) {
        driveVelocity = velocity;
        driveAcceleration = acceleration;
        driveLastUpdated = Logger.getTimestamp() / 1e6;
    }

    /** {@inheritDoc} */
    @Override
    public void setTurnState(double rotation, double angularVelocity) {
        turnPosition = rotation;
        turnVelocity = angularVelocity;
        turnLastUpdated = Logger.getTimestamp() / 1e6;
    }
}
