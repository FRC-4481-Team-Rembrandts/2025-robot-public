/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.teamrembrandts.control.PIDGains;

public class ElevatorConstants {
    public static final int CAN_ID_LEFT = 21;
    public static final int CAN_ID_RIGHT = 22;

    public static final double GROUND_TO_PIVOT_AXLE_DISTANCE_LOWEST_POSITION = 0.991;
    public static final double STOWED_HEIGHT = GROUND_TO_PIVOT_AXLE_DISTANCE_LOWEST_POSITION;
    public static final double DISABLE_HEIGHT = STOWED_HEIGHT + 0.03;
    public static final double TARGET_HEIGHT_L1 = STOWED_HEIGHT;
    public static final double TARGET_HEIGHT_L2 = 1.225;
    public static final double TARGET_HEIGHT_L3 = 1.55;
    public static final double TARGET_HEIGHT_L4 = 1.804;
    public static final double TARGET_HEIGHT_ALGAE_L2 = STOWED_HEIGHT;
    public static final double TARGET_HEIGHT_ALGAE_L3 = 1.3;
    public static final double TARGET_HEIGHT_PROCESSOR = STOWED_HEIGHT;
    public static final double TARGET_HEIGHT_BARGE = TARGET_HEIGHT_L4;
    public static final double TARGET_HEIGHT_CORAL_INTAKE = STOWED_HEIGHT;
    public static final double TARGET_HEIGHT_CORAL_INTAKE_AUTON = STOWED_HEIGHT;

    public static final double TARGET_HEIGHT_CLIMB = STOWED_HEIGHT + 0.04;

    public static final double ELEVATOR_TRIGGER_UP = (TARGET_HEIGHT_L3 + TARGET_HEIGHT_L4) / 2;

    public static final double SPROCKET_DIAMETER = Inches.of(2.55).in(Meters);
    public static final double GEAR_REDUCTION = 5.8286;
    /** UNITS: meter per rotation */
    public static final double MOTOR_ENCODER_POSITION_FACTOR = 1.0 / GEAR_REDUCTION * SPROCKET_DIAMETER * Math.PI;
    /** UNITS: (meter per second) per RPM */
    public static final double MOTOR_ENCODER_VELOCITY_FACTOR = MOTOR_ENCODER_POSITION_FACTOR / 60;

    public static final boolean LEADER_MOTOR_INVERTED = false;
    public static final boolean FOLLOWER_MOTOR_INVERTED = true;

    /** UNITS: meter per rotation */
    public static final double ABSOLUTE_ENCODER_POSITION_FACTOR = SPROCKET_DIAMETER * Math.PI;
    /** UNITS: (meter per second) per RPM */
    public static final double ABSOLUTE_ENCODER_VELOCITY_FACTOR = ABSOLUTE_ENCODER_POSITION_FACTOR / 60;

    public static final boolean ABSOLUTE_ENCODER_INVERTED = true;
    public static final double KG = 0.36;
    public static final double KV = 3.6;
    public static final double KA = 0.131;
    public static final double KS = 0.09;
    public static final PIDGains PID_GAINS = new PIDGains(3.5, 0, 0);
    public static final int CURRENT_LIMIT = 105;
    /** UNITS: meters per second */
    public static final double MOTION_PROFILE_MAX_VELOCITY = 3.3;
    /** UNITS: meters per second squared */
    public static final double M_PROFILE_MAX_ACCEL = 15;

    public static final double POSITION_MARGIN_METERS = 0.025;

    /** UNITS: RPM */
    public static final double MAX_RPM = 6400;

    /** UNITS: meters per second */
    public static final double MAX_VELOCITY = MAX_RPM * MOTOR_ENCODER_VELOCITY_FACTOR;

    public static final double LIMIT_SWITCH_DEBOUNCE_TIME = 3.0;
    public static final double POWER_CALIBRATE = -0.05;
    /** At which factor of the current limit the current limit detection should be triggered */
    public static final double CURRENT_LIMIT_DETECTION_FACTOR = 0.8;

    public static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
}
