/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.math.trajectory;

import com.teamrembrandts.math.kinematics.ChassisState;
import com.teamrembrandts.math.trajectory.struct.HolonomicTrajectorySampleStruct;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.util.List;

/** Interface for a holonomic trajectory. */
public interface HolonomicTrajectory {

    /**
     * A single sample of a holonomic trajectory.
     *
     * @param timestamp The timestamp of the sample.
     * @param pose The pose of the sample.
     * @param chassisState The chassis state of the sample.
     */
    record Sample(double timestamp, Pose2d pose, ChassisState chassisState) implements StructSerializable {
        public static final Struct<Sample> struct = new HolonomicTrajectorySampleStruct();
    }

    /**
     * Get the initial pose of the trajectory.
     *
     * @return the initial pose of the trajectory.
     */
    Pose2d getInitialPose();

    /**
     * Get the final pose of the trajectory.
     *
     * @return the final pose of the trajectory.
     */
    Pose2d getFinalPose();

    /**
     * Get the total time of the trajectory.
     *
     * @return the total duration of the motion profile of the trajectory.
     */
    double getTotalTime();

    /**
     * Get the samples of the trajectory.
     *
     * @return the samples of the trajectory.
     */
    List<Sample> getSamples();

    /**
     * Get the sample at a specific timestamp.
     *
     * @param timestamp The timestamp of the sample.
     * @return the sample at the given timestamp.
     */
    Sample sample(double timestamp);
}
