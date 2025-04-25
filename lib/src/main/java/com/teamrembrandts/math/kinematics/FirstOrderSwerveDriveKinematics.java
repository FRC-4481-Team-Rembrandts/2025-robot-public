/*
 * Copyright (c) 2024-2025 FRC 4481 - Team Rembrandts.
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
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Class that uses the velocity elements of the ChassisState to determine individual module states. First order here
 * means that only velocities and positions are considered. The linear and rotational velocity of the drivetrain is
 * converted to an angle and position of the module.
 */
public class FirstOrderSwerveDriveKinematics extends NthOrderSwerveDriveKinematics {

    /**
     * Create a first order swerve drive kinematics object. This takes in a variable number of module locations as
     * Translation2d objects. The order in which you pass in the module locations is the same order that you will
     * receive the module states when performing inverse kinematics. It is also expected that you pass in the module
     * states in the same order when calling the forward kinematics methods.
     *
     * @param moduleLocationsMeters The location of the modules relative to the physical center of the robot
     */
    public FirstOrderSwerveDriveKinematics(Translation2d... moduleLocationsMeters) {
        super(moduleLocationsMeters);
    }

    /**
     * Performs inverse kinematics to return the module states from a desired chassis velocity. Because this is first
     * order, the linear and angular acceleration of the ChassisState are not considered. This method is often used to
     * convert joystick values into module speeds and angles.
     *
     * <p>In the case that the desired chassis speeds are zero (i.e. the robot will be stationary), the previously
     * calculated module angle will be maintained.
     *
     * @param chassisState The desired first order ChassisState.
     * @param robotAngle The current angle of the robot with respect to the field
     * @return An array containing the module states. Use caution because these module states are not normalized.
     *     Sometimes, a user input may cause one of the module speeds to go above the attainable max velocity. Use the
     *     {@link #desaturateWheelSpeeds(SwerveModuleState[], double) DesaturateWheelSpeeds} function to rectify this
     *     issue.
     */
    @Override
    public FirstOrderSwerveModuleState[] toSwerveModuleStates(ChassisState chassisState, Rotation2d robotAngle) {
        SwerveModuleState[] moduleStates = super.toSwerveModuleStates(chassisState.getSpeeds());
        int nModules = moduleStates.length;
        FirstOrderSwerveModuleState[] firstOrderResults = new FirstOrderSwerveModuleState[nModules];
        for (int i = 0; i < nModules; i++) {
            firstOrderResults[i] = new FirstOrderSwerveModuleState(moduleStates[i]);
        }
        return firstOrderResults;
    }

    /**
     * Performs inverse kinematics to return the module states from a desired chassis velocity. Because this is first
     * order, the linear and angular acceleration of the ChassisState are not considered. This method is often used to
     * convert joystick values into module speeds and angles. In addition to inverse kinematics, the module states are
     * normalized, to make sure that no module speed exceeds the attainable max velocity.
     *
     * <p>In the case that the desired chassis speeds are zero (i.e. the robot will be stationary), the previously
     * calculated module angle will be maintained.
     *
     * @param chassisState The desired first order ChassisState.
     * @param robotAngle The current angle of the robot with respect to the field
     * @param attainableMaxSpeedMetersPerSecond The maximum speed that a module can attain
     * @return An array containing the module states.
     */
    @Override
    public NthOrderSwerveModuleState[] toDesaturatedSwerveModuleStates(
            ChassisState chassisState, Rotation2d robotAngle, double attainableMaxSpeedMetersPerSecond) {
        FirstOrderSwerveModuleState[] moduleStates = toSwerveModuleStates(chassisState, robotAngle);
        desaturateWheelSpeeds(moduleStates, attainableMaxSpeedMetersPerSecond);
        return moduleStates;
    }
}
