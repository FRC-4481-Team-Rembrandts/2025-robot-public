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
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;

/**
 * Class representing the state of a swerve module. The state is represented as the linear velocity and acceleration of
 * the wheel in combination with the angle and angular velocity.
 */
public class SecondOrderSwerveModuleState extends NthOrderSwerveModuleState {
    /**
     * Construct a second order swerve module state.
     *
     * @param speedMetersPerSecond The speed of the wheel of the module
     * @param angle The angle of the module
     * @param acceleration The acceleration of the wheel of the module
     * @param angularVelocity The angular velocity of the rotation of the module
     */
    public SecondOrderSwerveModuleState(
            double speedMetersPerSecond,
            Rotation2d angle,
            LinearAcceleration acceleration,
            AngularVelocity angularVelocity) {
        this.speedMetersPerSecond = speedMetersPerSecond;
        this.angle = angle;
        this.acceleration = acceleration;
        this.angularVelocity = angularVelocity;
    }

    /**
     * Construct a second order swerve module state from a regular SwerveModuleState.
     *
     * @param swerveModuleState The SwerveModuleState to construct the second order state from
     * @param acceleration The acceleration of the wheel of the module
     * @param angularVelocity The angular velocity of the rotation of the module
     */
    public SecondOrderSwerveModuleState(
            SwerveModuleState swerveModuleState, LinearAcceleration acceleration, AngularVelocity angularVelocity) {
        this(swerveModuleState.speedMetersPerSecond, swerveModuleState.angle, acceleration, angularVelocity);
    }

    /** Construct an empty second order swerve module state. */
    public SecondOrderSwerveModuleState() {
        this(0.0, new Rotation2d(), Units.MetersPerSecondPerSecond.zero(), Units.RadiansPerSecond.zero());
    }
}
