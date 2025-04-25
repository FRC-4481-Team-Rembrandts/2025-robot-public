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

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.LoggedRobot;

public class PathPlannerTrajectoryGenerator {
    private final RobotConfig config;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedSupplier;
    private static final double MIN_VELOCITY = 0.5;
    private static final double CONTROL_POINT_FACTOR = 3.0;
    private static final double END_CONTROL_POINT_LENGTH = 0.1;
    private static final double BEGIN_POSE_LOOK_AHEAD_TIME = 5 * LoggedRobot.defaultPeriodSecs;

    /**
     * Create a new PathPlannerTrajectoryGenerator
     *
     * @param config The robot configuration
     * @param robotPoseSupplier Supplier for the current robot pose
     * @param chassisSpeedSupplier Supplier for the current chassis speeds
     */
    public PathPlannerTrajectoryGenerator(
            RobotConfig config, Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedSupplier) {
        this.config = config;
        this.robotPoseSupplier = robotPoseSupplier;
        this.chassisSpeedSupplier = chassisSpeedSupplier;
    }

    /**
     * Generate a path planner trajectory from a starting to a goal
     *
     * @param beginPose Begin position of the robot, the rotation component is where the robot is facing
     * @param endPose End position of the robot, the rotation component is where the robot is facing
     * @param beginTravelDirection In which direction the is moving at the start
     * @param endTravelDirection In which direction the robot should be moving at the end
     * @param constraints The constraints for the path
     * @return A PathPlannerTrajectory to get from the begin pose to the end pose
     */
    public PathPlannerTrajectory generate(
            Pose2d beginPose,
            Pose2d endPose,
            Rotation2d beginTravelDirection,
            Rotation2d endTravelDirection,
            PathConstraints constraints) {

        double beginVelocity =
                Math.hypot(chassisSpeedSupplier.get().vxMetersPerSecond, chassisSpeedSupplier.get().vyMetersPerSecond);

        // When the path starts from standstill, the direction can be directly towards the goal
        // The path should also start at this minimum velocity to avoid some weird PathPlanner behaviour
        // where the path is extremely slow
        if (beginVelocity < MIN_VELOCITY) {
            Translation2d beginToGoal = endPose.getTranslation().minus(beginPose.getTranslation());
            beginTravelDirection = beginToGoal.getAngle();
            beginVelocity = MIN_VELOCITY;
        }

        Translation2d beginControl = beginPose
                .getTranslation()
                .plus(new Translation2d(beginVelocity / CONTROL_POINT_FACTOR, beginTravelDirection));
        Waypoint beginWaypoint = new Waypoint(null, beginPose.getTranslation(), beginControl);

        Translation2d endControl =
                endPose.getTranslation().minus(new Translation2d(END_CONTROL_POINT_LENGTH, endTravelDirection));
        Waypoint endWaypoint = new Waypoint(endControl, endPose.getTranslation(), null);

        // Make a list of the waypoints
        List<Waypoint> waypoints = List.of(beginWaypoint, endWaypoint);

        // Making the path
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                new IdealStartingState(beginVelocity, beginPose.getRotation()),
                new GoalEndState(0, endPose.getRotation()));

        ChassisSpeeds robotRelativeSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeedSupplier.get(), beginPose.getRotation());
        return path.generateTrajectory(robotRelativeSpeeds, beginPose.getRotation(), config);
    }

    /**
     * Generate a path planner trajectory from the current robot pose and velocity to a goal.
     *
     * @param endPose End position of the robot, the rotation component is where the robot is facing.
     * @param endTravelDirection In which direction the robot should be moving at the end.
     * @param constraints The constraints for the path.
     * @return A PathPlannerTrajectory to get from the current robot pose and velocity to the end pose.
     */
    public PathPlannerTrajectory generateFromCurrentPose(
            Pose2d endPose, Rotation2d endTravelDirection, PathConstraints constraints) {
        Pose2d beginPose = robotPoseSupplier.get();
        ChassisSpeeds beginSpeeds = chassisSpeedSupplier.get();
        Twist2d twist = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeedSupplier.get(), beginPose.getRotation())
                .toTwist2d(BEGIN_POSE_LOOK_AHEAD_TIME);

        Pose2d futurePose = beginPose.exp(twist);
        Rotation2d movementDirection =
                new Rotation2d(Math.atan2(beginSpeeds.vyMetersPerSecond, beginSpeeds.vxMetersPerSecond));

        return generate(futurePose, endPose, movementDirection, endTravelDirection, constraints);
    }

    /**
     * Generate a path planner trajectory from the current robot pose and velocity to a goal. The goal is assumed to be
     * tangent to the direction of travel at the end of the path
     *
     * @param endPose End position of the robot, the rotation component is where the robot is facing.
     * @param constraints The constraints for the path.
     * @return A PathPlannerTrajectory to get from the current robot pose and velocity to the end pose.
     */
    public PathPlannerTrajectory generateFromCurrentPose(Pose2d endPose, PathConstraints constraints) {
        return generateFromCurrentPose(endPose, endPose.getRotation(), constraints);
    }

    /**
     * Generate a path planner trajectory from the current robot pose and velocity to a goal. The travel direction at
     * the end of the path is assumed to be in the direction where the robot is facing. If endReverse is set to true,
     * the robot will approach the end pose in reverse (driving opposite to the direction it is facing).
     *
     * @param endPose End position of the robot, the rotation component is where the robot is facing.
     * @param constraints The constraints for the path.
     * @param endReverse Whether the robot should approach the end pose in reverse.
     * @return A PathPlannerTrajectory to get from the current robot pose and velocity to the end pose.
     */
    public PathPlannerTrajectory generateFromCurrentPose(
            Pose2d endPose, PathConstraints constraints, boolean endReverse) {
        Rotation2d endTravelDirection =
                endReverse ? endPose.getRotation().rotateBy(Rotation2d.k180deg) : endPose.getRotation();
        return generateFromCurrentPose(endPose, endTravelDirection, constraints);
    }
}
