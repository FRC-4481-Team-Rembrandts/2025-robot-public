/*
 * Copyright (c) 2024-2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.path.PathConstraints;
import com.teamrembrandts.util.RunMode;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.SDSFlexDriveConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to
 * reduce verbosity.
 */
public final class Constants {

    public static final RunMode MODE = RunMode.REAL;
    public static final double CONTROLLER_DEADBAND = 0.04481;
    public static final int PDH_ID = 1;

    /** Distance from the center of the robot to the center of the reef within which the arm can move. */
    public static final double CORAL_SCORING_CIRCLE_RADIUS = 1.9;

    public static final double CORAL_PREPARE_RADIUS = 4.0;
    public static final double SAFE_ARM_MOVEMENT_RADIUS = 1.7;

    public static final Rotation2d AUTONOMOUS_PRESET_ANGLE = Rotation2d.fromDegrees(45);

    // Initialize correct type of drivetrain constants
    public static final DriveConstants DRIVE_CONSTANTS = new SDSFlexDriveConstants();

    public static final PathConstraints PATH_CONSTRAINTS_REEF_AUTO_ALIGN = new PathConstraints(3, 3, 10, 15, 12, false);
    public static final PathConstraints PATH_CONSTRAINTS_CORAL_STATION_AUTO_ALIGN =
            new PathConstraints(3.5, 3.5, 10, 15, 12, false);
    public static final PathConstraints PATH_CONSTRAINTS_BARGE_AUTO_ALIGN =
            new PathConstraints(3.5, 3.5, 10, 15, 12, false);
    public static final double AUTO_ALIGN_LOOK_AHEAD_TIME_REEF = 0.4;
    public static final double AUTO_ALIGN_LOOK_AHEAD_TIME_CORAL = 0.4;
    public static final double AUTO_ALIGN_LOOK_AHEAD_TIME_BARGE = 0.4;

    public static final double AUTO_ALIGN_DISTANCE_TOLERANCE = 0.02;
    public static final double AUTO_ALIGN_ANGULAR_TOLERANCE = Radians.convertFrom(1, Degrees);
    public static final double AUTON_DISTANCE_TOLERANCE = 0.045;
    public static final double AUTON_ANGULAR_TOLERANCE = Radians.convertFrom(6, Degrees);

    public static final double TRAJECTORY_TRANSLATION_STATIC_KP_AUTON = 8;
    public static final double TRAJECTORY_ROTATION_KP_AUTON = 6;
    public static final double TRAJECTORY_TRANSLATION_DYNAMIC_KP_AUTON = 3;
    public static final double TRAJECTORY_TRANSLATION_STATIC_KP_TELEOP = 6.0;
    public static final double TRAJECTORY_ROTATION_KP_TELEOP = 6;
    public static final double TRAJECTORY_TRANSLATION_DYNAMIC_KP_TELEOP = 1.0;
}
