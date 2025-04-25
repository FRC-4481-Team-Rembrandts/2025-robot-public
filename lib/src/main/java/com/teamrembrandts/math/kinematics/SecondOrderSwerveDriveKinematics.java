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
import edu.wpi.first.units.Units;
import java.util.Arrays;

/**
 * Class that uses a ChassisState to determine individual module states. Second order here means that positions,
 * velocities and accelerations are considered. The velocity and acceleration of the drivetrain are converted to an
 * angle, angular velocity, linear velocity and acceleration of the modules.
 */
public class SecondOrderSwerveDriveKinematics extends NthOrderSwerveDriveKinematics {
    private final Translation2d[] moduleTranslations;
    private final int nModules;
    /**
     * Create a first order swerve drive kinematics object. This takes in a variable number of module locations as
     * Translation2d objects. The order in which you pass in the module locations is the same order that you will
     * receive the module states when performing inverse kinematics. It is also expected that you pass in the module
     * states in the same order when calling the forward kinematics methods.
     *
     * @param moduleLocationsMeters The location of the modules relative to the physical center of the robot
     */
    public SecondOrderSwerveDriveKinematics(Translation2d... moduleLocationsMeters) {
        super(moduleLocationsMeters);
        nModules = moduleLocationsMeters.length;
        moduleTranslations = Arrays.copyOf(moduleLocationsMeters, nModules);
    }

    /**
     * Performs inverse kinematics to return the second order module states from a desired chassis velocity and
     * acceleration.
     *
     * <p>In the case that the desired chassis speeds are zero (i.e. the robot will be stationary), the previously
     * calculated module angle will be maintained.
     *
     * @param chassisState The desired robot relative ChassisState.
     * @param robotAngle The current angle of the robot with respect to the field.
     * @return An array containing the second order module states. This includes acceleration of the drive motor and
     *     velocity of the turn motor. Use caution because these module states are not normalized. Sometimes, a user
     *     input may cause one of the module speeds to go above the attainable max velocity. Use the
     *     {@link #desaturateWheelSpeeds(SwerveModuleState[], double) DesaturateWheelSpeeds} function to rectify this
     *     issue.
     */
    @Override
    public SecondOrderSwerveModuleState[] toSwerveModuleStates(ChassisState chassisState, Rotation2d robotAngle) {
        ChassisState fieldRelativeChassisState = ChassisState.fromRobotRelativeState(chassisState, robotAngle);

        SwerveModuleState[] firstOrderResults = super.toSwerveModuleStates(chassisState.getSpeeds());
        double[] accel = new double[nModules];
        double[] angularVel = new double[nModules];
        SecondOrderSwerveModuleState[] secondOrderStates = new SecondOrderSwerveModuleState[nModules];
        for (int i = 0; i < nModules; i++) {
            Translation2d moduleLoc = moduleTranslations[i].rotateBy(robotAngle);
            double a_mx = fieldRelativeChassisState.getAccelerations().ax
                    - moduleLoc.getX()
                            * chassisState.getSpeeds().omegaRadiansPerSecond
                            * chassisState.getSpeeds().omegaRadiansPerSecond
                    - moduleLoc.getY() * chassisState.getAccelerations().alpha;
            double a_my = fieldRelativeChassisState.getAccelerations().ay
                    - moduleLoc.getY()
                            * chassisState.getSpeeds().omegaRadiansPerSecond
                            * chassisState.getSpeeds().omegaRadiansPerSecond
                    + moduleLoc.getX() * chassisState.getAccelerations().alpha;
            Translation2d accelVector = new Translation2d(a_mx, a_my);

            // Rotate the acceleration vector to get a component in the module's direction and perpendicular to it
            Rotation2d fieldToModuleRotation = firstOrderResults[i].angle.plus(robotAngle);
            Translation2d moduleAccelVec = accelVector.rotateBy(fieldToModuleRotation.unaryMinus());
            accel[i] = moduleAccelVec.getX();
            angularVel[i] = Math.abs(firstOrderResults[i].speedMetersPerSecond) > 0
                    ? moduleAccelVec.getY() / firstOrderResults[i].speedMetersPerSecond
                            - chassisState.getSpeeds().omegaRadiansPerSecond
                    : 0;
            secondOrderStates[i] = new SecondOrderSwerveModuleState(
                    firstOrderResults[i],
                    Units.MetersPerSecondPerSecond.of(accel[i]),
                    Units.RadiansPerSecond.of(angularVel[i]));
        }
        return secondOrderStates;
    }

    /**
     * Performs inverse kinematics to return the second order module states from a desired chassis velocity and
     * acceleration. In addition, desaturate the wheels speeds to make sure no module exceeds its velocity limit.
     *
     * <p>In the case that the desired chassis speeds are zero (i.e. the robot will be stationary), the previously
     * calculated module angle will be maintained.
     *
     * @param chassisState The desired robot relative ChassisState.
     * @param robotAngle The current angle of the robot with respect to the field.
     * @param attainableMaxSpeedMetersPerSecond The maximum speed that a module can reach in m/s.
     * @return An array containing the second order module states. This includes acceleration of the drive motor and
     *     velocity of the turn motor.
     */
    @Override
    public SecondOrderSwerveModuleState[] toDesaturatedSwerveModuleStates(
            ChassisState chassisState, Rotation2d robotAngle, double attainableMaxSpeedMetersPerSecond) {
        SecondOrderSwerveModuleState[] moduleStates = toSwerveModuleStates(chassisState, robotAngle);
        double realMaxSpeed = 0;
        for (SwerveModuleState moduleState : moduleStates) {
            realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
        }
        if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
            double speedScale = attainableMaxSpeedMetersPerSecond / realMaxSpeed;

            moduleStates = toSwerveModuleStates(chassisState.times(speedScale), robotAngle);
        }
        return moduleStates;
    }
}
