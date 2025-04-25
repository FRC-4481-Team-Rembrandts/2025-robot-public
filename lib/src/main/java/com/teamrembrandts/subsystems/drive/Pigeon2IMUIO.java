/*
 * Copyright (c) 2024-2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;

/** Implementation for Pigeon2 IMU IO */
public class Pigeon2IMUIO implements IMUIO {
    private final Pigeon2 pigeon;
    private final StatusSignal<Angle> yaw;
    private final StatusSignal<Angle> pitch;
    private final StatusSignal<Angle> roll;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    private final StatusSignal<AngularVelocity> yawVelocity;

    /**
     * Constructs a new Pigeon2IMUIO.
     *
     * @param id The CAN ID of the Pigeon2.
     */
    public Pigeon2IMUIO(int id) {
        pigeon = new Pigeon2(id);
        yaw = pigeon.getYaw();
        yawVelocity = pigeon.getAngularVelocityZWorld();
        pitch = pigeon.getPitch();
        roll = pigeon.getRoll();

        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(100.0);
        yawVelocity.setUpdateFrequency(100.0);
        pitch.setUpdateFrequency(50.0);
        roll.setUpdateFrequency(50.0);
        pigeon.optimizeBusUtilization();
        yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(yaw::getValueAsDouble);
    }

    /**
     * Updates the system with new inputs from the IMU.
     *
     * @param inputs The IMU inputs.
     */
    @Override
    public void updateInputs(IMUInputs inputs) {
        inputs.connected =
                BaseStatusSignal.refreshAll(yaw, yawVelocity, pitch, roll).equals(StatusCode.OK);
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocity = Units.degreesToRadians(yawVelocity.getValueAsDouble());
        inputs.pitchPosition = Rotation2d.fromDegrees(pitch.getValueAsDouble()); // Pitch is about +Y
        inputs.rollPosition = Rotation2d.fromDegrees(roll.getValueAsDouble()); // Roll is about +X

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
