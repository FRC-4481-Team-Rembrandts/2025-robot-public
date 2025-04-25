/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig;

public class FeedThroughGripperConstants {
    public static final int CORAL_GRIPPER_CAN_ID = 41;
    public static final boolean MOTOR_INVERTED = false;
    public static final int CURRENT_LIMIT = 80;
    public static final double INTAKE_POWER = 0.4;
    public static final double L1_OUTTAKE_POWER = 0.23;
    public static final double L2_OUTTAKE_POWER = 0.38;
    public static final double L3_OUTTAKE_POWER = 0.38;
    public static final double L4_OUTTAKE_POWER = -0.5;

    public static final double FEED_THROUGH_GRIPPER_ACCELERATION_TIME = 0.25;
    public static final double FEED_THROUGH_GRIPPER_RATE_LIMIT = INTAKE_POWER
            / FEED_THROUGH_GRIPPER_ACCELERATION_TIME; // Speed which the gripper speed increases in units per second

    public static final double OUTTAKE_TIME = 0.35;

    public static final double SENSOR_DEBOUNCE_TIME = 0;

    public static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
}
