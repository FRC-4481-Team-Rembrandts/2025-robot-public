/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.math.kinematics;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * A class representing the state of a swerve module. The state is represented as the velocity and the angle of the
 * wheel.
 */
public class FirstOrderSwerveModuleState extends NthOrderSwerveModuleState {
    /**
     * Construct a first order swerve module state.
     *
     * @param speedMetersPerSecond The speed of the wheel of the module
     * @param angleRadians The angle of the module
     */
    public FirstOrderSwerveModuleState(double speedMetersPerSecond, Rotation2d angleRadians) {
        this.speedMetersPerSecond = speedMetersPerSecond;
        this.angle = angleRadians;
    }

    /**
     * Construct a first order swerve module state from a regular SwerveModuleState.
     *
     * @param swerveModuleState The SwerveModuleState to construct the first order state from
     */
    public FirstOrderSwerveModuleState(SwerveModuleState swerveModuleState) {
        this(swerveModuleState.speedMetersPerSecond, swerveModuleState.angle);
    }

    /** Construct an empty first order swerve module state. */
    public FirstOrderSwerveModuleState() {
        this(0.0, new Rotation2d());
    }
}
