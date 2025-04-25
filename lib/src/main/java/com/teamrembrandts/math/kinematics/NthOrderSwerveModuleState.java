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

/** Abstract class representing an Nth order swerve module state. */
public abstract class NthOrderSwerveModuleState extends SwerveModuleState {
    /** The acceleration of the wheel of the module. */
    public LinearAcceleration acceleration = Units.MetersPerSecondPerSecond.zero();
    /** The angular velocity of the rotation of the module. */
    public AngularVelocity angularVelocity = Units.RadiansPerSecond.zero();

    /**
     * {@inheritDoc} The functionality of this method is enhanced to also flip the acceleration of the module when this
     * is given. Angular velocity is not influenced by optimization.
     */
    @Override
    public void optimize(Rotation2d currentAngle) {
        Rotation2d delta = this.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 90.0) {
            angle = angle.rotateBy(Rotation2d.kPi);
            speedMetersPerSecond *= -1;
            this.acceleration = this.acceleration.unaryMinus();
        }
    }

    /**
     * {@inheritDoc} The functionality of this method is enhanced to also scale the acceleration of the module when this
     * is given.
     */
    @Override
    public void cosineScale(Rotation2d currentAngle) {
        super.cosineScale(currentAngle);

        acceleration.times(angle.minus(currentAngle).getCos());
    }

    /**
     * Return a regular SwerveModuleState representing the wheel velocity and angle of this NOrderSwerveModuleState.
     *
     * @return A SwerveModuleState based on this NOrderSwerveModuleState.
     */
    public SwerveModuleState toSwerveModuleState() {
        return new SwerveModuleState(speedMetersPerSecond, angle);
    }
}
