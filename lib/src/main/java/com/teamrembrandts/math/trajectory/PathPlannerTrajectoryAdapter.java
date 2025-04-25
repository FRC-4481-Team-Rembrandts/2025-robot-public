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

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.teamrembrandts.math.kinematics.ChassisAccelerations;
import com.teamrembrandts.math.kinematics.ChassisState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LoggedRobot;

public class PathPlannerTrajectoryAdapter implements HolonomicTrajectory {
    private final PathPlannerTrajectory trajectory;

    public PathPlannerTrajectoryAdapter(PathPlannerTrajectory trajectory) {
        this.trajectory = trajectory;
    }

    @Override
    public Pose2d getInitialPose() {
        return trajectory.getInitialPose();
    }

    @Override
    public Pose2d getFinalPose() {
        return trajectory.getEndState().pose;
    }

    @Override
    public double getTotalTime() {
        return trajectory.getTotalTimeSeconds();
    }

    @Override
    public List<Sample> getSamples() {
        var states = trajectory.getStates();
        ArrayList<Sample> holonomicSamples = new ArrayList<>();

        for (PathPlannerTrajectoryState state : states) {
            holonomicSamples.add(toHolonomicSample(state));
        }

        return holonomicSamples;
    }

    @Override
    public Sample sample(double timestamp) {
        return toHolonomicSample(trajectory.sample(timestamp));
    }

    /**
     * Converts a {@code PathPlannerTrajectoryState} to a {@code HolonomicTrajectory.Sample}.
     *
     * @param state The state to convert.
     * @return the HolonomicTrajectory.Sample equivalent of the given {@code State}.
     */
    private Sample toHolonomicSample(PathPlannerTrajectoryState state) {
        double deltaTime = LoggedRobot.defaultPeriodSecs;
        double timestamp = state.timeSeconds;

        Pose2d pose = state.pose;

        ChassisSpeeds chassisSpeeds = state.fieldSpeeds;

        ChassisSpeeds nextSpeeds = trajectory.sample(timestamp + deltaTime).fieldSpeeds;
        // To account for strange start behaviour, fade in acceleration over a couple cycles
        int accelerationFadeIn = 4;
        double accelFadeFactor = Math.min(1, timestamp / (deltaTime * accelerationFadeIn));
        ChassisSpeeds deltaSpeeds =
                nextSpeeds.minus(chassisSpeeds).div(deltaTime).times(accelFadeFactor);

        ChassisAccelerations chassisAccelerations = new ChassisAccelerations(
                deltaSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, deltaSpeeds.omegaRadiansPerSecond);

        ChassisState chassisState = new ChassisState(chassisSpeeds, chassisAccelerations);
        return new Sample(timestamp, pose, chassisState);
    }
}
