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

import org.littletonrobotics.junction.AutoLog;

public interface DifferentialDriveIO {

    @AutoLog
    class DifferentialDriveInputs {
        public double position = 0;
        public double velocity = 0;
        public double[] appliedVoltages = {};
        public double[] currentsDrawn = {};
        public double[] temperatures = {};
    }

    /**
     * Updates the system with new inputs.
     *
     * @param inputs The new inputs.
     */
    default void updateInputs(DifferentialDriveInputs inputs) {}

    /**
     * Sets the target drive velocity for the module.
     *
     * @param velocity target drive velocity in m/s.
     */
    default void setDriveVelocity(double velocity) {}

    /**
     * Sets the target voltage for the drive motors.
     *
     * @param voltage target drive voltage.
     */
    default void setDriveVoltage(double voltage) {}
}
