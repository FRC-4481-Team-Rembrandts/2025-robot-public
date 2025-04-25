/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.math.kinematics;

import com.teamrembrandts.math.kinematics.struct.ChassisStateStruct;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.struct.StructSerializable;
import java.util.Objects;

/** Represents the state of a chassis (speeds and accelerations) in a two-dimensional plane. */
public class ChassisState implements StructSerializable {
    ChassisSpeeds speeds;
    ChassisAccelerations accelerations;

    public static final ChassisStateStruct struct = new ChassisStateStruct();

    /** Constructs a ChassisState. */
    public ChassisState() {
        this(new ChassisSpeeds(), new ChassisAccelerations());
    }

    /**
     * Constructs a ChassisState.
     *
     * @param speeds The speeds of the chassis.
     * @param accelerations The accelerations of the chassis.
     */
    public ChassisState(ChassisSpeeds speeds, ChassisAccelerations accelerations) {
        this.speeds = speeds;
        this.accelerations = accelerations;
    }

    /**
     * Constructs a ChassisState.
     *
     * @param vx The x component of the velocity.
     * @param vy The y component of the velocity.
     * @param omega The angular velocity.
     * @param ax The x component of the acceleration.
     * @param ay The y component of the acceleration.
     * @param alpha The angular acceleration.
     */
    public ChassisState(double vx, double vy, double omega, double ax, double ay, double alpha) {
        this(new ChassisSpeeds(vx, vy, omega), new ChassisAccelerations(ax, ay, alpha));
    }

    /**
     * Gets the speeds from the current state.
     *
     * @return The speeds of the chassis.
     */
    public ChassisSpeeds getSpeeds() {
        return speeds;
    }

    /**
     * Gets the accelerations from the current state.
     *
     * @return The accelerations of the chassis.
     */
    public ChassisAccelerations getAccelerations() {
        return accelerations;
    }

    /**
     * Converts a ChassisState from a field-relative frame to a robot-relative frame.
     *
     * @param fieldRelativeState The field-relative state.
     * @param robotHeading The heading of the robot.
     * @return The robot-relative state.
     */
    public static ChassisState fromFieldRelativeState(ChassisState fieldRelativeState, Rotation2d robotHeading) {
        return new ChassisState(
                ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeState.speeds, robotHeading),
                ChassisAccelerations.fromFieldRelativeAccelerations(fieldRelativeState.accelerations, robotHeading));
    }

    /**
     * Converts a ChassisState from a robot-relative frame to a field-relative frame.
     *
     * @param robotRelativeState The robot-relative state.
     * @param robotHeading The heading of the robot.
     * @return The field-relative state.
     */
    public static ChassisState fromRobotRelativeState(ChassisState robotRelativeState, Rotation2d robotHeading) {
        return new ChassisState(
                ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeState.speeds, robotHeading),
                ChassisAccelerations.fromRobotRelativeAccelerations(robotRelativeState.accelerations, robotHeading));
    }

    /**
     * Adds two ChassisStates together.
     *
     * @param other The other ChassisState.
     * @return The sum of the two ChassisStates.
     */
    public ChassisState plus(ChassisState other) {
        return new ChassisState(speeds.plus(other.speeds), accelerations.plus(other.accelerations));
    }

    /**
     * Subtracts one ChassisState from another.
     *
     * @param other The other ChassisState.
     * @return The difference of the two ChassisStates.
     */
    public ChassisState minus(ChassisState other) {
        return new ChassisState(speeds.minus(other.speeds), accelerations.minus(other.accelerations));
    }

    /**
     * Negates a ChassisState.
     *
     * @return The negated ChassisState.
     */
    public ChassisState unaryMinus() {
        return new ChassisState(speeds.unaryMinus(), accelerations.unaryMinus());
    }

    /**
     * Multiplies a ChassisState by a scalar.
     *
     * @param scalar The scalar to multiply by.
     * @return The scaled ChassisState.
     */
    public ChassisState times(double scalar) {
        return new ChassisState(speeds.times(scalar), accelerations.times(scalar));
    }

    /**
     * Divides a ChassisState by a scalar.
     *
     * @param scalar The scalar to divide by.
     * @return The divided ChassisState.
     */
    public ChassisState div(double scalar) {
        return new ChassisState(speeds.div(scalar), accelerations.div(scalar));
    }

    @Override
    public final int hashCode() {
        return Objects.hash(speeds, accelerations);
    }

    @Override
    public boolean equals(Object o) {
        return o == this
                || o instanceof ChassisState c && speeds.equals(c.speeds) && accelerations.equals(c.accelerations);
    }
}
