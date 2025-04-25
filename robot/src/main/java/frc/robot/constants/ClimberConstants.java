/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.teamrembrandts.control.PIDGains;
import com.teamrembrandts.hardware.constants.GearRatios;
import edu.wpi.first.math.geometry.Rotation2d;

public class ClimberConstants {
    public static final int CLIMBER_CAN_ID = 51;
    public static final double REDUCTION = 2
            * GearRatios.RevRobotics.MaxPlanetary.REDUCTION_5_TO_1_CARDRIDGE
            * GearRatios.RevRobotics.MaxPlanetary.REDUCTION_5_TO_1_CARDRIDGE
            * GearRatios.RevRobotics.MaxPlanetary.REDUCTION_5_TO_1_CARDRIDGE;

    public static final PIDGains PID_GAINS = new PIDGains(3.5, 0, 0);

    public static final Rotation2d CLIMBER_READY_POSITION = Rotation2d.fromDegrees(6);
    public static final Rotation2d CLIMBED_POSITION = Rotation2d.fromDegrees(111);
    public static final double POWER_FOLD_OUT = -0.99;
    public static final double POWER_REVERT_CLIMB = -0.15;
    public static final double POWER_CLIMB = 0.40;
    public static final double TOLERANCE_POSITION_DEGREES = 2.5;
    public static final double TOLERANCE_VELOCITY = 1.5;
    public static final double ACCELERATION_WAIT_TIME = 0.9;
    public static final int CLIMB_CURRENT_LIMIT = 110;
    public static final int FOLD_CURRENT_LIMIT = 30;
    public static final int FOLD_TWO_CURRENT_LIMIT = 60;
    public static final boolean INVERTED = false;
    public static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;

    public static final double MOTOR_ENCODER_POSITION_FACTOR = (2 * Math.PI) / REDUCTION;
    public static final double MOTOR_ENCODER_VELOCITY_FACTOR = MOTOR_ENCODER_POSITION_FACTOR / 60;
}
