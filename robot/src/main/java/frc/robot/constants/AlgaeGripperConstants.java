/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig;

public class AlgaeGripperConstants {
    public static final int ALGAE_GRIPPER_CAN_ID = 42;
    public static final boolean MOTOR_INVERTED = false;
    public static final int CURRENT_LIMIT = 60;

    public static final double ALGAE_INTAKE_POWER = -1;
    public static final double ALGAE_OUTTAKE_POWER = 1; // 0.4
    public static final double ALGAE_HOLD_POWER = -0.3;
    public static final double ALGAE_DISABLE_POWER = 0;

    public static final double SENSOR_THRESHOLD = 2.5;
    public static final double SENSOR_DEBOUNCE_TIME = 0.1;

    public static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
}
