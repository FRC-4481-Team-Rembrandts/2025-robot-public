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

import static edu.wpi.first.units.Units.*;

import com.teamrembrandts.math.kinematics.struct.ChassisAccelerationsStruct;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.util.struct.StructSerializable;
import java.util.Objects;

public class ChassisAccelerations implements StructSerializable {
    /** Acceleration along the x-axis. (Fwd is +) */
    public double ax;

    /** Acceleration along the y-axis. (Left is +) */
    public double ay;

    /** Represents the angular acceleration of the robot frame. (CCW is +) */
    public double alpha;

    /** ChassisSpeeds struct for serialization. */
    public static final ChassisAccelerationsStruct struct = new ChassisAccelerationsStruct();

    /** Constructs a ChassisAccelerations with zeros for dx, dy, and theta. */
    public ChassisAccelerations() {}

    /**
     * Constructs a ChassisAccelerations object.
     *
     * @param ax Forward acceleration.
     * @param ay Sideways acceleration.
     * @param alpha Angular acceleration.
     */
    public ChassisAccelerations(double ax, double ay, double alpha) {
        this.ax = ax;
        this.ay = ay;
        this.alpha = alpha;
    }

    /**
     * Constructs a ChassisAccelerations object.
     *
     * @param ax Forward acceleration.
     * @param ay Sideways acceleration.
     * @param alpha Angular acceleration.
     */
    public ChassisAccelerations(LinearAcceleration ax, LinearAcceleration ay, AngularAcceleration alpha) {
        this(ax.in(MetersPerSecondPerSecond), ay.in(MetersPerSecondPerSecond), alpha.in(RadiansPerSecondPerSecond));
    }

    /**
     * Converts a user provided field-relative set of accelerations into a robot-relative ChassisAccelerations object.
     *
     * @param ax The component of speed in the x direction relative to the field. Positive x is away from your alliance
     *     wall.
     * @param ay The component of speed in the y direction relative to the field. Positive y is to your left when
     *     standing behind the alliance wall.
     * @param alpha The angular rate of the robot.
     * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is considered to be zero
     *     when it is facing directly away from your alliance station wall. Remember that this should be CCW positive.
     * @return ChassisAccelerations object representing the speeds in the robot's frame of reference.
     */
    public static ChassisAccelerations fromFieldRelativeAccelerations(
            double ax, double ay, double alpha, Rotation2d robotAngle) {
        // CW rotation into chassis frame
        var rotated = new Translation2d(ax, ay).rotateBy(robotAngle.unaryMinus());
        return new ChassisAccelerations(rotated.getX(), rotated.getY(), alpha);
    }

    /**
     * Converts a user provided field-relative set of speeds into a robot-relative ChassisAccelerations object.
     *
     * @param ax The component of speed in the x direction relative to the field. Positive x is away from your alliance
     *     wall.
     * @param ay The component of speed in the y direction relative to the field. Positive y is to your left when
     *     standing behind the alliance wall.
     * @param alpha The angular rate of the robot.
     * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is considered to be zero
     *     when it is facing directly away from your alliance station wall. Remember that this should be CCW positive.
     * @return ChassisAccelerations object representing the speeds in the robot's frame of reference.
     */
    public static ChassisAccelerations fromFieldRelativeAccelerations(
            LinearAcceleration ax, LinearAcceleration ay, AngularAcceleration alpha, Rotation2d robotAngle) {
        return fromFieldRelativeAccelerations(
                ax.in(MetersPerSecondPerSecond),
                ay.in(MetersPerSecondPerSecond),
                alpha.in(RadiansPerSecondPerSecond),
                robotAngle);
    }

    /**
     * Converts a user provided field-relative ChassisAccelerations object into a robot-relative ChassisSpeeds object.
     *
     * @param fieldRelativeAccelerations The ChassisAccelerations object representing the speeds in the field frame of
     *     reference. Positive x is away from your alliance wall. Positive y is to your left when standing behind the
     *     alliance wall.
     * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is considered to be zero
     *     when it is facing directly away from your alliance station wall. Remember that this should be CCW positive.
     * @return ChassisAccelerations object representing the speeds in the robot's frame of reference.
     */
    public static ChassisAccelerations fromFieldRelativeAccelerations(
            ChassisAccelerations fieldRelativeAccelerations, Rotation2d robotAngle) {
        return fromFieldRelativeAccelerations(
                fieldRelativeAccelerations.ax,
                fieldRelativeAccelerations.ay,
                fieldRelativeAccelerations.alpha,
                robotAngle);
    }

    /**
     * Converts a user provided robot-relative set of speeds into a field-relative ChassisAccelerations object.
     *
     * @param ax The component of speed in the x direction relative to the robot. Positive x is towards the robot's
     *     front.
     * @param ay The component of speed in the y direction relative to the robot. Positive y is towards the robot's
     *     left.
     * @param alpha The angular rate of the robot.
     * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is considered to be zero
     *     when it is facing directly away from your alliance station wall. Remember that this should be CCW positive.
     * @return ChassisAccelerations object representing the speeds in the field's frame of reference.
     */
    public static ChassisAccelerations fromRobotRelativeAccelerations(
            double ax, double ay, double alpha, Rotation2d robotAngle) {
        // CCW rotation out of chassis frame
        var rotated = new Translation2d(ax, ay).rotateBy(robotAngle);
        return new ChassisAccelerations(rotated.getX(), rotated.getY(), alpha);
    }

    /**
     * Converts a user provided robot-relative set of speeds into a field-relative ChassisAccelerations object.
     *
     * @param ax The component of speed in the x direction relative to the robot. Positive x is towards the robot's
     *     front.
     * @param ay The component of speed in the y direction relative to the robot. Positive y is towards the robot's
     *     left.
     * @param alpha The angular rate of the robot.
     * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is considered to be zero
     *     when it is facing directly away from your alliance station wall. Remember that this should be CCW positive.
     * @return ChassisAccelerations object representing the speeds in the field's frame of reference.
     */
    public static ChassisAccelerations fromRobotRelativeAccelerations(
            LinearAcceleration ax, LinearAcceleration ay, AngularAcceleration alpha, Rotation2d robotAngle) {
        return fromRobotRelativeAccelerations(
                ax.in(MetersPerSecondPerSecond),
                ay.in(MetersPerSecondPerSecond),
                alpha.in(RadiansPerSecondPerSecond),
                robotAngle);
    }

    /**
     * Converts a user provided robot-relative ChassisAccelerations object into a field-relative ChassisAccelerations
     * object.
     *
     * @param robotRelativeAccelerations The ChassisAccelerations object representing the speeds in the robot frame of
     *     reference. Positive x is towards the robot's front. Positive y is towards the robot's left.
     * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is considered to be zero
     *     when it is facing directly away from your alliance station wall. Remember that this should be CCW positive.
     * @return ChassisAccelerations object representing the speeds in the field's frame of reference.
     */
    public static ChassisAccelerations fromRobotRelativeAccelerations(
            ChassisAccelerations robotRelativeAccelerations, Rotation2d robotAngle) {
        return fromRobotRelativeAccelerations(
                robotRelativeAccelerations.ax,
                robotRelativeAccelerations.ay,
                robotRelativeAccelerations.alpha,
                robotAngle);
    }

    /**
     * Adds two ChassisAccelerations and returns the sum.
     *
     * <p>For example, ChassisAccelerations{1.0, 0.5, 0.75} + ChassisAccelerations{2.0, 1.5, 0.25} =
     * ChassisAccelerations{3.0, 2.0, 1.0}
     *
     * @param other The ChassisAccelerations to add.
     * @return The sum of the ChassisAccelerations.
     */
    public ChassisAccelerations plus(ChassisAccelerations other) {
        return new ChassisAccelerations(ax + other.ax, ay + other.ay, alpha + other.alpha);
    }

    /**
     * Subtracts the other ChassisAccelerations from the current ChassisAccelerations and returns the difference.
     *
     * <p>For example, ChassisSpeeds{5.0, 4.0, 2.0} - ChassisSpeeds{1.0, 2.0, 1.0} = ChassisSpeeds{4.0, 2.0, 1.0}
     *
     * @param other The ChassisSpeeds to subtract.
     * @return The difference between the two ChassisSpeeds.
     */
    public ChassisAccelerations minus(ChassisAccelerations other) {
        return new ChassisAccelerations(ax - other.ax, ay - other.ay, alpha - other.alpha);
    }

    /**
     * Returns the inverse of the current ChassisSpeeds. This is equivalent to negating all components of the
     * ChassisSpeeds.
     *
     * @return The inverse of the current ChassisSpeeds.
     */
    public ChassisAccelerations unaryMinus() {
        return new ChassisAccelerations(-ax, -ay, -alpha);
    }

    /**
     * Multiplies the ChassisSpeeds by a scalar and returns the new ChassisSpeeds.
     *
     * <p>For example, ChassisSpeeds{2.0, 2.5, 1.0} * 2 = ChassisSpeeds{4.0, 5.0, 1.0}
     *
     * @param scalar The scalar to multiply by.
     * @return The scaled ChassisSpeeds.
     */
    public ChassisAccelerations times(double scalar) {
        return new ChassisAccelerations(ax * scalar, ay * scalar, scalar * scalar);
    }

    /**
     * Divides the ChassisSpeeds by a scalar and returns the new ChassisSpeeds.
     *
     * <p>For example, ChassisSpeeds{2.0, 2.5, 1.0} / 2 = ChassisSpeeds{1.0, 1.25, 0.5}
     *
     * @param scalar The scalar to divide by.
     * @return The scaled ChassisSpeeds.
     */
    public ChassisAccelerations div(double scalar) {
        return new ChassisAccelerations(ax / scalar, ay / scalar, alpha / scalar);
    }

    @Override
    public final int hashCode() {
        return Objects.hash(ax, ay, alpha);
    }

    @Override
    public boolean equals(Object o) {
        return o == this || o instanceof ChassisAccelerations c && ax == c.ax && ay == c.ay && alpha == c.alpha;
    }

    @Override
    public String toString() {
        return String.format(
                "ChassisAccelerations(Vx: %.2f m/s^2, Vy: %.2f m/s^2, Omega: %.2f rad/s^2)", ax, ay, alpha);
    }
}
