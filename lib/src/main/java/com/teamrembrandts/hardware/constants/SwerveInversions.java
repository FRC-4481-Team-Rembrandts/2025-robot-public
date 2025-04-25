/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.hardware.constants;

/**
 * The SwerveInversions class contains whether the motors and encoders used in common COTS swerve modules should be
 * inverted. This assumes 4 modules are used, one in each corner of the robot.
 */
public final class SwerveInversions {

    /**
     * The RevRobotics class provides the inversions for the motors and encoders used in REV Robotics swerve modules.
     */
    public static class RevRobotics {

        /**
         * The MaxSwerve class provides the inversions for the motors and encoders used in the REV Robotics MAXSwerve
         * system.
         */
        public static class MaxSwerve {
            /** The inversion of the front left drive motor for the MAXSwerve module. */
            public static final boolean FRONT_LEFT_INVERTED = true;

            /** The inversion of the front right drive motor for the MAXSwerve module. */
            public static final boolean FRONT_RIGHT_INVERTED = false;

            /** The inversion of the back left drive motor for the MAXSwerve module. */
            public static final boolean BACK_LEFT_INVERTED = true;

            /** The inversion of the back right drive motor for the MAXSwerve module. */
            public static final boolean BACK_RIGHT_INVERTED = false;

            /** The inversion of the turn motor for the MAXSwerve module. */
            public static final boolean TURN_MOTOR_INVERTED = false;

            /**
             * The inversion of the turn encoder when a REV Through Bore Absolute Encoder is used for the MAXSwerve
             * module.
             */
            public static final boolean REV_THROUGH_BORE_ENCODER_INVERTED = true;
        }
    }

    /**
     * The SDS class provides the inversions for the motors and encoders used in SwerveDriveSpecialties Swerve modules.
     */
    public static class SDS {
        /**
         * The MK4n class provides the inversions for the motors and encoders used in the SDS Swerve module, this
         * assumes each module is calibrated with the bevel gear facing inward.
         */
        public static class MK4n {
            /** The inversion of the front left drive motor for the SDS MK4n Swerve module. */
            public static final boolean FRONT_LEFT_INVERTED = false;

            /** The inversion of the front right drive motor for the SDS MK4n Swerve module. */
            public static final boolean FRONT_RIGHT_INVERTED = true;

            /** The inversion of the back left drive motor for the SDS MK4n Swerve module. */
            public static final boolean BACK_LEFT_INVERTED = false;

            /** The inversion of the back right drive motor for the SDS MK4n Swerve module. */
            public static final boolean BACK_RIGHT_INVERTED = true;

            /** The inversion of the turn motor for the SDS MK4n Swerve module. */
            public static final boolean TURN_MOTOR_INVERTED = true;

            /**
             * The inversion of the turn encoder when a REV Through Bore Absolute Encoder is used for the SDS MK4n
             * Swerve module.
             */
            public static final boolean REV_THROUGH_BORE_ENCODER_INVERTED = false;
        }
    }
}
