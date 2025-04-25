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
import org.littletonrobotics.junction.AutoLog;

/** Interface to define common IMU input/output functionality. */
public interface IMUIO {
    @AutoLog
    class IMUInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawVelocity = 0.0;
        public Rotation2d pitchPosition = new Rotation2d();
        public Rotation2d rollPosition = new Rotation2d();
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    /**
     * Updates the system with new inputs from the IMU.
     *
     * @param inputs Inputs from the IMU.
     */
    default void updateInputs(IMUInputs inputs) {}
}
