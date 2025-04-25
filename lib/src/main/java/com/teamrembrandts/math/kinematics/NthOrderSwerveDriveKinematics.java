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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Abstract class for an Nth order swerve drive kinematics. */
public abstract class NthOrderSwerveDriveKinematics extends SwerveDriveKinematics {

    /**
     * Create a new Nth order swerve drive kinematics object.
     *
     * @param moduleLocationsMeters The location of the modules relative to the physical center of the robot
     */
    public NthOrderSwerveDriveKinematics(Translation2d... moduleLocationsMeters) {
        super(moduleLocationsMeters);
    }

    /**
     * Performs inverse kinematics to return the module states from a desired ChassisState.
     *
     * @param chassisState The desired robot relative ChassisState.
     * @param robotAngle The angle of the robot with respect to the field.
     * @return An array containing the module states.
     */
    public abstract NthOrderSwerveModuleState[] toSwerveModuleStates(ChassisState chassisState, Rotation2d robotAngle);

    public abstract NthOrderSwerveModuleState[] toDesaturatedSwerveModuleStates(
            ChassisState chassisState, Rotation2d robotAngle, double attainableMaxSpeedMetersPerSecond);

    /**
     * Check whether the measured swerve module states are consistent which each other. This means that the wheels are
     * not 'fighting' each other. This is determined by converting the measured SwerveModule states to ChassisSpeeds.
     * The ChassisSpeed is then converted back to SwerveModule states and then the difference is taken with the measured
     * states.
     *
     * @param measuredSwerveStates The measured swerve module states.
     * @return The error in velocity and angle of each module
     */
    public SwerveModuleState[] getKinematicError(SwerveModuleState[] measuredSwerveStates) {
        ChassisSpeeds chassisSpeeds = toChassisSpeeds(measuredSwerveStates);
        SwerveModuleState[] idealStates = toSwerveModuleStates(chassisSpeeds);
        SwerveModuleState[] error = new SwerveModuleState[idealStates.length];
        for (int i = 0; i < idealStates.length; i++) {
            error[i] = new SwerveModuleState();
            idealStates[i].optimize(measuredSwerveStates[i].angle);
            error[i].speedMetersPerSecond =
                    measuredSwerveStates[i].speedMetersPerSecond - idealStates[i].speedMetersPerSecond;
            error[i].angle = measuredSwerveStates[i].angle.minus(idealStates[i].angle);
        }
        return error;
    }
}
