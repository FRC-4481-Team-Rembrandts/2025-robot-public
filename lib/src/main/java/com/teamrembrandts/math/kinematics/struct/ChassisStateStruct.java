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
import com.teamrembrandts.math.kinematics.ChassisState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class ChassisStateStruct implements Struct<ChassisState> {
    @Override
    public Class<ChassisState> getTypeClass() {
        return ChassisState.class;
    }

    @Override
    public String getTypeName() {
        return "ChassisState";
    }

    @Override
    public int getSize() {
        return ChassisSpeeds.struct.getSize() + ChassisAccelerations.struct.getSize();
    }

    @Override
    public String getSchema() {
        return "ChassisSpeeds speeds; ChassisAccelerations accelerations";
    }

    @Override
    public ChassisState unpack(ByteBuffer byteBuffer) {
        ChassisSpeeds speeds = ChassisSpeeds.struct.unpack(byteBuffer);
        ChassisAccelerations accelerations = ChassisAccelerations.struct.unpack(byteBuffer);
        return new ChassisState(speeds, accelerations);
    }

    @Override
    public void pack(ByteBuffer byteBuffer, ChassisState chassisState) {
        ChassisSpeeds.struct.pack(byteBuffer, chassisState.getSpeeds());
        ChassisAccelerations.struct.pack(byteBuffer, chassisState.getAccelerations());
    }
}
