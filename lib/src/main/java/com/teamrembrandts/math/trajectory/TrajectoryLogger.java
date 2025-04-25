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

import com.teamrembrandts.math.kinematics.ChassisAccelerations;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Utility class for logging {@code HolonomicTrajectory} objects to the {@code Logger}.
 *
 * <p>This class provides a static method to log a {@code HolonomicTrajectory} object, converting it to a
 * {@code Trajectory} object in the process.
 *
 * <p>This is a band-aid solution to the fact that {@code HolonomicTrajectory} objects can be logged, but not natively
 * visualized in AdvantageScope. While the data that is logged here is directly sourced from an actual
 * {@code HolonomicTrajectory}, some data is lost in the process. If you need direct access to the actual fields of the
 * {@code HolonomicTrajectory}, you should log an array of {@code HolonomicTrajectory.Sample} objects instead.
 *
 * <p>Because of the implicit conversion to WPILib trajectories done by this class, it is not advised to use its methods
 * periodically.
 *
 * @see HolonomicTrajectory
 * @see Trajectory
 */
public class TrajectoryLogger {
    private TrajectoryLogger() {
        // Private constructor to prevent instantiation
    }

    /**
     * Helper function to log an otherwise unloggable {@code HolonomicTrajectory}.
     *
     * <p>Be aware that some of the original data is lost in the conversion to a {@code Trajectory} object. If you need
     * direct access to the fields in the {@code HolonomicTrajectory}, you should log an array of
     * {@code HolonomicTrajectory.Sample} objects instead.
     *
     * @param key The key to log the trajectory under.
     * @param trajectory The trajectory to log.
     */
    public static void log(String key, HolonomicTrajectory trajectory) {
        Logger.recordOutput(key, toWPITrajectory(trajectory));
    }

    /**
     * Helper function to convert a {@code HolonomicTrajectory} to a {@code Trajectory}.
     *
     * @param trajectory The trajectory to convert.
     * @return The converted trajectory.
     */
    private static Trajectory toWPITrajectory(HolonomicTrajectory trajectory) {
        List<HolonomicTrajectory.Sample> samples = trajectory.getSamples();
        List<Trajectory.State> states =
                samples.stream().map(TrajectoryLogger::toWPITrajectoryState).toList();

        return new Trajectory(states);
    }

    /**
     * Helper function to convert a {@code HolonomicTrajectory.Sample} to a {@code Trajectory.State}.
     *
     * @param sample The sample to convert.
     * @return The converted sample.
     */
    private static Trajectory.State toWPITrajectoryState(HolonomicTrajectory.Sample sample) {
        ChassisSpeeds speeds = sample.chassisState().getSpeeds();
        ChassisAccelerations accelerations = sample.chassisState().getAccelerations();
        double curvature = speeds.omegaRadiansPerSecond / speeds.vxMetersPerSecond;

        return new Trajectory.State(
                sample.timestamp(), speeds.vxMetersPerSecond, accelerations.ax, sample.pose(), curvature);
    }
}
