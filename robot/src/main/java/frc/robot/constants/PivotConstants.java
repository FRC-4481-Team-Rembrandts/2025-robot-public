/*
 * Copyright (c) 2024-2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.teamrembrandts.control.PIDGains;
import edu.wpi.first.math.geometry.Rotation2d;

public class PivotConstants {
    public static final int CAN_ID = 31;
    public static final Rotation2d TARGET_ANGLE_L1 = Rotation2d.fromDegrees(-90);
    public static final Rotation2d TARGET_ANGLE_L2 = Rotation2d.fromDegrees(-83);
    public static final Rotation2d ALGAE_L2 = Rotation2d.fromDegrees(-66.3);
    public static final Rotation2d ALGAE_L3 = Rotation2d.fromDegrees(-60);
    public static final Rotation2d TARGET_ANGLE_L3 = Rotation2d.fromDegrees(-78); // -83
    public static final Rotation2d TARGET_ANGLE_L4 = Rotation2d.fromDegrees(28.4);
    public static final Rotation2d TARGET_ANGLE_PROCESSOR = Rotation2d.fromDegrees(-90);
    public static final Rotation2d STOWED_ANGLE = Rotation2d.fromDegrees(-83);
    public static final Rotation2d TARGET_ANGLE_CORAL_INTAKE = Rotation2d.fromDegrees(-96);
    public static final Rotation2d TARGET_ANGLE_CORAL_INTAKE_AUTON = Rotation2d.fromDegrees(-97.5); //

    public static final Rotation2d TARGET_CLIMBER = Rotation2d.fromDegrees(100);

    public static final Rotation2d TARGET_ANGLE_HORIZONTAL_CORAL = Rotation2d.fromDegrees(56);
    public static final Rotation2d TARGET_ANGLE_VERTICAL_CORAL = Rotation2d.fromDegrees(0.0);

    public static final Rotation2d TARGET_ANGLE_BARGE = Rotation2d.fromDegrees(20); // 40
    public static final Rotation2d TARGET_ANGLE_BARGE_PREP = Rotation2d.fromDegrees(-50); // -50
    public static final double PIVOT_READY_TO_SCORE_IN_BARGE = -42; // -42

    public static final double PIVOT_IS_DOWN = 0;

    public static final double GEAR_REDUCTION = 72.0;
    /** UNITS: radians per rotation */
    public static final double MOTOR_ENCODER_POSITION_FACTOR = (2 * Math.PI) / GEAR_REDUCTION;
    /** UNITS: (radians per second) per RPM */
    public static final double MOTOR_ENCODER_VELOCITY_FACTOR = MOTOR_ENCODER_POSITION_FACTOR / 60;

    public static final boolean MOTOR_INVERTED = false;

    /** UNITS: radians per rotation */
    public static final double ABSOLUTE_ENCODER_POSITION_FACTOR = 2 * Math.PI;
    /** UNITS: (radians per second) per RPM */
    public static final double ABSOLUTE_ENCODER_VELOCITY_FACTOR = ABSOLUTE_ENCODER_POSITION_FACTOR / 60;

    public static final boolean ABSOLUTE_ENCODER_INVERTED = true;

    public static final double KG = 0.23 * 1.2;
    public static final double KV = 1.13;
    public static final double KA = 0;
    public static final double KS = 0.08;
    public static final PIDGains PID_GAINS = new PIDGains(0.3, 0, 0);
    public static final int CURRENT_LIMIT = 67;
    /** UNITS: radians per second */
    public static final double M_PROFILE_MAX_VEL = 8;
    /** UNITS: radians per second squared */
    public static final double M_PROFILE_MAX_ACCEL = 70;

    public static final double POSITION_MARGIN_DEGREES = 5;

    public static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
}
