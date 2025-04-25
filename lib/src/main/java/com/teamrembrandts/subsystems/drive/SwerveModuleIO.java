/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface to define common swerve module input/output functionality. */
public interface SwerveModuleIO {

    @AutoLog
    class SwerveModuleInputs {
        public double drivePosition = 0;
        public double driveVelocity = 0;
        public double driveAppliedVoltage = 0;
        public double driveCurrent = 0;
        public double driveTemperature = 0;
        public boolean driveConnected = false;

        public Rotation2d turnAbsoluteRotation = new Rotation2d();
        public double turnAngularVelocity = 0;
        public double turnAppliedVoltage = 0;
        public double turnCurrent = 0;

        public double turnTemperature = 0;
        public boolean turnConnected = false;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositions = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    /**
     * Updates the system with new inputs from the module.
     *
     * @param inputs The new inputs.
     */
    default void updateInputs(SwerveModuleInputs inputs) {}

    /**
     * Sets the target drive velocity and acceleration for the module.
     *
     * @param velocity target drive velocity in m/s.
     * @param acceleration target drive acceleration in m/s^2.
     */
    default void setDriveState(double velocity, double acceleration) {}

    /**
     * Sets the target turn position and velocity for the module.
     *
     * @param rotation target turn position in radians.
     * @param angularVelocity target turn velocity in radians per second.
     */
    default void setTurnState(double rotation, double angularVelocity) {}

    /**
     * Sets the drive output to a voltage.
     *
     * @param voltage The output voltage.
     */
    default void setDriveOpenLoop(double voltage) {}

    /**
     * Sets the turn output to a voltage.
     *
     * @param voltage The output voltage.
     */
    default void setTurnOpenLoop(double voltage) {}
}
