/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.math.kinematics.struct;

import com.teamrembrandts.math.kinematics.ChassisAccelerations;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class ChassisAccelerationsStruct implements Struct<ChassisAccelerations> {
    @Override
    public Class<ChassisAccelerations> getTypeClass() {
        return ChassisAccelerations.class;
    }

    @Override
    public String getTypeName() {
        return "ChassisAccelerations";
    }

    @Override
    public int getSize() {
        return kSizeDouble * 3;
    }

    @Override
    public String getSchema() {
        return "double ax;double ay;double alpha";
    }

    @Override
    public ChassisAccelerations unpack(ByteBuffer byteBuffer) {
        double ax = byteBuffer.getDouble();
        double ay = byteBuffer.getDouble();
        double alpha = byteBuffer.getDouble();

        return new ChassisAccelerations(ax, ay, alpha);
    }

    @Override
    public void pack(ByteBuffer byteBuffer, ChassisAccelerations chassisAccelerations) {
        byteBuffer.putDouble(chassisAccelerations.ax);
        byteBuffer.putDouble(chassisAccelerations.ay);
        byteBuffer.putDouble(chassisAccelerations.alpha);
    }
}
