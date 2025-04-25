/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.math.trajectory.struct;

import com.teamrembrandts.math.kinematics.ChassisState;
import com.teamrembrandts.math.trajectory.HolonomicTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class HolonomicTrajectorySampleStruct implements Struct<HolonomicTrajectory.Sample> {
    @Override
    public Class<HolonomicTrajectory.Sample> getTypeClass() {
        return HolonomicTrajectory.Sample.class;
    }

    @Override
    public String getTypeName() {
        return "HolonomicTrajectory.Sample";
    }

    @Override
    public int getSize() {
        return kSizeDouble + Pose2d.struct.getSize() + ChassisState.struct.getSize();
    }

    @Override
    public String getSchema() {
        return "double timestamp; Pose2d pose; ChassisState chassisState";
    }

    @Override
    public HolonomicTrajectory.Sample unpack(ByteBuffer byteBuffer) {
        double timestamp = byteBuffer.getDouble();
        Pose2d pose = Pose2d.struct.unpack(byteBuffer);
        ChassisState chassisState = ChassisState.struct.unpack(byteBuffer);
        return new HolonomicTrajectory.Sample(timestamp, pose, chassisState);
    }

    @Override
    public void pack(ByteBuffer byteBuffer, HolonomicTrajectory.Sample sample) {
        byteBuffer.putDouble(sample.timestamp());
        Pose2d.struct.pack(byteBuffer, sample.pose());
        ChassisState.struct.pack(byteBuffer, sample.chassisState());
    }
}
