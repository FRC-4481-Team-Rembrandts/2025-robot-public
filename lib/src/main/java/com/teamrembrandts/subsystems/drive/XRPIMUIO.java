/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.xrp.XRPGyro;

/** IMU IO implementation that represents the onboard gyroscope of a default XRP robot. */
public class XRPIMUIO implements IMUIO {
    private final XRPGyro gyro;

    /** Creates a new XRP IMU IO. */
    public XRPIMUIO() {
        gyro = new XRPGyro();
        gyro.reset();
    }

    /**
     * Updates the system with new inputs from the IMU.
     *
     * @param inputs The IMU inputs.
     */
    @Override
    public void updateInputs(IMUInputs inputs) {
        inputs.connected = true; // Integrated, no way for it to disconnect
        inputs.yawPosition = Rotation2d.fromDegrees(gyro.getAngleZ());
        inputs.yawVelocity = Units.degreesToRadians(gyro.getRateZ());
    }
}
