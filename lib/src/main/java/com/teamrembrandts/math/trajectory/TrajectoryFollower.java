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

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.teamrembrandts.math.kinematics.ChassisAccelerations;
import com.teamrembrandts.math.kinematics.ChassisState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TrajectoryFollower {

    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;
    private final Supplier<HolonomicTrajectory> trajectorySupplier;
    private HolonomicTrajectory trajectory;
    private final double toleranceDistance;
    private final double toleranceRadians;
    private final double translationStaticKp;
    private final double translationDynamicKp;
    private final double rotationKp;

    private double startTime = 0;
    private boolean isFinished = false;
    private boolean isProfileFinished = false;

    /**
     * Create a trajectory follower to follow the given trajectory
     *
     * @param trajectorySupplier Supplier for the trajectory to follow
     * @param robotPoseSupplier Supplier of the current robot position
     * @param chassisSpeedsSupplier Supplier of the robot velocity, field relative
     * @param toleranceDistanceMeters Translational tolerance within which the robot should end
     * @param toleranceAngleRadians Rotational tolerance within the robot should end the trajectory
     * @param translationStaticKp Static translational kP constant to compensate for error
     * @param translationDynamicKp Dynamic translational kp constant to compensate for error when close to the reef
     * @param rotationKp Rotational kP constant to compensate for error
     */
    public TrajectoryFollower(
            Supplier<HolonomicTrajectory> trajectorySupplier,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> chassisSpeedsSupplier,
            double toleranceDistanceMeters,
            double toleranceAngleRadians,
            double translationStaticKp,
            double translationDynamicKp,
            double rotationKp) {
        this.trajectorySupplier = trajectorySupplier;
        this.robotPoseSupplier = robotPoseSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
        this.toleranceDistance = toleranceDistanceMeters;
        this.toleranceRadians = toleranceAngleRadians;
        this.translationStaticKp = translationStaticKp;
        this.translationDynamicKp = translationDynamicKp;
        this.rotationKp = rotationKp;
    }

    /**
     * Create a trajectory follower to follow the given trajectory
     *
     * @param trajectorySupplier Supplier for the trajectory to follow
     * @param robotPoseSupplier Supplier of the current robot position
     * @param chassisSpeedsSupplier Supplier of the Chassis State of the robot
     * @param tolerance Translational tolerance within which the robot should end
     * @param angularTolerance Rotational tolerance within the robot should end the trajectory
     * @param translationStaticKp Translational kP constant to compensate for error
     * @param translationDynamicKp Dynamic translational kp constant to compensate for error when close to the reef
     * @param rotationKp Rotational kP constant to compensate for error
     */
    public TrajectoryFollower(
            Supplier<HolonomicTrajectory> trajectorySupplier,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> chassisSpeedsSupplier,
            Distance tolerance,
            Angle angularTolerance,
            double translationStaticKp,
            double translationDynamicKp,
            double rotationKp) {
        this(
                trajectorySupplier,
                robotPoseSupplier,
                chassisSpeedsSupplier,
                tolerance.in(Meters),
                angularTolerance.in(Radians),
                translationStaticKp,
                translationDynamicKp,
                rotationKp);
    }

    /** Start the trajectory follower. This will start the timer and get the trajectory from the supplier. */
    public void start() {
        startTime = Logger.getTimestamp() / 1e6;
        trajectory = trajectorySupplier.get();
    }

    /**
     * Get the initial pose of the trajectory. This is the pose at which the trajectory starts.
     *
     * @return The initial pose of the trajectory.
     */
    public Pose2d getInitialPose() {
        return trajectorySupplier.get().getInitialPose();
    }

    /**
     * Gets the target state of the trajectory.
     *
     * @return The target state of the trajectory.
     */
    public ChassisState getTargetState() {
        Pose2d currentPose = robotPoseSupplier.get();
        double timestamp = Logger.getTimestamp() / 1e6;

        HolonomicTrajectory.Sample sample = trajectory.sample(timestamp - startTime);
        Pose2d samplePose = sample.pose();

        double staticTime = 0.15; // Time before the end of the trajectory which already counts as static
        boolean enableStaticKp = timestamp - startTime + staticTime > trajectory.getTotalTime();
        double translationKp = enableStaticKp ? translationStaticKp : translationDynamicKp;

        Translation2d translationFeedback =
                samplePose.getTranslation().minus(currentPose.getTranslation()).times(translationKp);

        double rotationFeedback =
                samplePose.getRotation().minus(currentPose.getRotation()).getRadians() * rotationKp;

        ChassisSpeeds feedbackSpeeds =
                new ChassisSpeeds(translationFeedback.getX(), translationFeedback.getY(), rotationFeedback);

        boolean stationary = isStationary(chassisSpeedsSupplier.get());

        boolean withinDistanceMargin = MathUtil.isNear(
                0,
                currentPose
                        .getTranslation()
                        .getDistance(trajectory.getFinalPose().getTranslation()),
                toleranceDistance);
        boolean withinAngleMargin = MathUtil.isNear(
                trajectory.getFinalPose().getRotation().getRadians(),
                currentPose.getRotation().getRadians(),
                toleranceRadians,
                -Rotation2d.k180deg.getRadians(),
                Rotation2d.k180deg.getRadians());
        isProfileFinished = timestamp - startTime > trajectory.getTotalTime();
        isFinished = withinDistanceMargin && withinAngleMargin && isProfileFinished && stationary;
        Logger.recordOutput(
                "Trajectory/distanceToTarget",
                currentPose
                        .getTranslation()
                        .getDistance(trajectory.getFinalPose().getTranslation()));
        Logger.recordOutput("Trajectory/targetPose", samplePose);
        Logger.recordOutput("Trajectory/withinDistance", withinDistanceMargin);
        Logger.recordOutput("Trajectory/withinAngleMargin", withinAngleMargin);
        Logger.recordOutput("Trajectory/profileFinished", isProfileFinished);
        Logger.recordOutput("Trajectory/stationary", stationary);
        Logger.recordOutput(
                "Trajectory/profileVelocity",
                Math.hypot(
                        sample.chassisState().getSpeeds().vxMetersPerSecond,
                        sample.chassisState().getSpeeds().vyMetersPerSecond));

        if (isProfileFinished) {
            return ChassisState.fromFieldRelativeState(
                    new ChassisState(
                            sample.chassisState().getSpeeds().plus(feedbackSpeeds), new ChassisAccelerations()),
                    currentPose.getRotation());
        }
        // compute chassis speeds
        return ChassisState.fromFieldRelativeState(
                new ChassisState(
                        sample.chassisState().getSpeeds().plus(feedbackSpeeds),
                        sample.chassisState().getAccelerations()),
                currentPose.getRotation());
    }

    /**
     * Returns whether the trajectory is finished. The trajectory is finished when the robot is within tolerance in both
     * time, distance and rotation.
     *
     * @return Whether the trajectory is finished.
     */
    public boolean isFinished() {
        return isFinished;
    }

    /**
     * Returns whether the motion profile is finished.
     *
     * @return Whether the motion profile is finished.
     */
    public boolean isProfileFinished() {
        return isProfileFinished;
    }

    private boolean isStationary(ChassisSpeeds speeds) {
        double maxStationarySpeeds = 0.25;
        return Math.abs(speeds.vxMetersPerSecond) <= maxStationarySpeeds
                && Math.abs(speeds.vyMetersPerSecond) <= maxStationarySpeeds;
    }
}
