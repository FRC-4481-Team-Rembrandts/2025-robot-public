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

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.teamrembrandts.math.kinematics.ChassisState;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.List;

/** Adapter for a Choreo trajectory to a HolonomicTrajectory. */
public class ChoreoTrajectoryAdapter implements HolonomicTrajectory {
    Trajectory<SwerveSample> trajectory;

    /**
     * Creates a new HolonomicTrajectory from a stored Choreo trajectory.
     *
     * @param name The filename of the trajectory.
     * @param splitIndex The index of the split to load.
     */
    public ChoreoTrajectoryAdapter(String name, int splitIndex) {
        var optionalTrajectory = Choreo.loadTrajectory(name);

        try {
            trajectory = (Trajectory<SwerveSample>)
                    optionalTrajectory.get().getSplit(splitIndex).get();
        } catch (Exception e) {
            System.err.println(
                    "Error: Trajectory with name: " + name + " and split index: " + splitIndex + " does not exist.");
        }
    }

    /**
     * Creates a new HolonomicTrajectory from a stored Choreo trajectory.
     *
     * @param name The filename of the trajectory.
     */
    public ChoreoTrajectoryAdapter(String name) {
        var optionalTrajectory = Choreo.loadTrajectory(name);

        try {
            trajectory = (Trajectory<SwerveSample>) optionalTrajectory.get();
        } catch (Exception e) {
            System.err.println("Error: Trajectory with name: " + name + " does not exist.");
        }
    }

    @Override
    public Pose2d getInitialPose() {
        var optionalPose = trajectory.getInitialPose(false);
        return optionalPose.orElseGet(Pose2d::new);
    }

    @Override
    public Pose2d getFinalPose() {
        var optionalPose = trajectory.getFinalPose(false);
        return optionalPose.orElseGet(Pose2d::new);
    }

    @Override
    public double getTotalTime() {
        return trajectory.getTotalTime();
    }

    /**
     * Get the samples of the trajectory. The list of samples is converted only when this method is called. This makes
     * it quite an expensive operation. Refrain from using it when the robot is enabled.
     *
     * @return the samples of the trajectory.
     */
    @Override
    public List<Sample> getSamples() {
        var samples = trajectory.samples();
        ArrayList<Sample> holonomicSamples = new ArrayList<>();

        for (SwerveSample sample : samples) {
            holonomicSamples.add(toHolonomicSample(sample));
        }

        return holonomicSamples;
    }

    @Override
    public Sample sample(double timestamp) {
        var optionalSample = trajectory.sampleAt(timestamp, false);

        return toHolonomicSample(optionalSample.orElseGet(
                () -> new SwerveSample(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, new double[] {}, new double[] {})));
    }

    /**
     * Converts a Choreo {@code SwerveSample} to a {@code HolonomicTrajectory.Sample}.
     *
     * @param sample the {@code SwerveSample} to convert.
     * @return the converted {@code HolonomicTrajectory.Sample}.
     */
    private HolonomicTrajectory.Sample toHolonomicSample(SwerveSample sample) {
        ChassisState state = new ChassisState(sample.vx, sample.vy, sample.omega, sample.ax, sample.ay, sample.alpha);

        return new HolonomicTrajectory.Sample(sample.getTimestamp(), sample.getPose(), state);
    }
}
