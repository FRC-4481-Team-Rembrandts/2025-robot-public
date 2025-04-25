/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

/** Utility class for calculations */
public class GeometryUtil {
    /**
     * Returns the closest pose from a given list of target poses to the source translation.
     *
     * @param currentTranslation2d The source translation.
     * @param targetPoses Array of scoring poses.
     * @return The scoring position which is closest to the source translation.
     */
    public static Pose2d getClosestPose(Translation2d currentTranslation2d, Pose2d[] targetPoses) {
        double poseDistance = Double.POSITIVE_INFINITY;
        Pose2d closestPose = targetPoses[0];
        for (Pose2d targetPose : targetPoses) {
            if (currentTranslation2d.getDistance(targetPose.getTranslation()) < poseDistance) {
                poseDistance = currentTranslation2d.getDistance(targetPose.getTranslation());
                closestPose = targetPose;
            }
        }

        Logger.recordOutput("Trajectory/FinalTargetPose", closestPose);

        return closestPose;
    }

    /**
     * Returns the closest pose from a given list of target poses to the source pose.
     *
     * @param currentPose2d The source Pose.
     * @param targetPoses Array of scoring poses.
     * @return The scoring position which is closest to the source pose.
     */
    public static Pose2d getClosestPose(Pose2d currentPose2d, Pose2d[] targetPoses) {
        return getClosestPose(currentPose2d.getTranslation(), targetPoses);
    }

    /**
     * Returns the closest pose from a given set, based on an extrapolated source pose.
     *
     * @param currentPose The current pose of the source.
     * @param sourceRelativeSpeeds The current velocity of the source.
     * @param deltaTime The time difference between now and the desired measurement point in seconds.
     * @param targetPoses The list of target poses.
     * @return The closest pose from a given set, based on the extrapolated source pose.
     */
    public static Pose2d getClosestFuturePose(
            Pose2d currentPose, ChassisSpeeds sourceRelativeSpeeds, double deltaTime, Pose2d[] targetPoses) {
        // The translational velocity is important to know in which direction the mass of the robot is moving
        // Additionally, the translational velocity indicates in which direction the driver wants to move the robot
        // Therefore, we neglect the rotational velocity
        sourceRelativeSpeeds.omegaRadiansPerSecond = 0;
        Twist2d twist = sourceRelativeSpeeds.toTwist2d(deltaTime);

        Pose2d futurePose = currentPose.exp(twist);
        Logger.recordOutput("Trajectory/FutureSourcePose", futurePose);

        return getClosestPose(futurePose, targetPoses);
    }

    /**
     * Returns the closest Translation2d from a given list of target Translation2d objects to the source translation2d.
     *
     * @param currentLocation2d The source translation2d
     * @param targetTranslations Array of scoring Translation2d objects
     * @return The scoring position which is closest to the source Translation2d
     */
    public static Translation2d getClosestTranslation(
            Translation2d currentLocation2d, Translation2d[] targetTranslations) {
        double poseDistance = Double.POSITIVE_INFINITY;
        Translation2d closestTranslation2d = targetTranslations[0];
        for (Translation2d scoringPose : targetTranslations) {
            if (currentLocation2d.getDistance(scoringPose) < poseDistance) {
                poseDistance = currentLocation2d.getDistance(scoringPose);
                closestTranslation2d = scoringPose;
            }
        }

        return closestTranslation2d;
    }

    /**
     * Returns the closest translation from a given set, based on an extrapolated source translation.
     *
     * @param currentTranslation The current translation of the source.
     * @param sourceRelativeSpeeds The current velocity of the source.
     * @param deltaTime The time difference between now and the desired measurement point in seconds.
     * @param targetTranslations The list of target translations.
     * @return The closest translation from a given set, based on the extrapolated source translation.
     */
    public static Translation2d getClosestFutureTranslation(
            Translation2d currentTranslation,
            ChassisSpeeds sourceRelativeSpeeds,
            double deltaTime,
            Translation2d[] targetTranslations) {
        // The translational velocity is important to know in which direction the mass of the robot is moving
        // Additionally, the translational velocity indicates in which direction the driver wants to move the robot
        // Therefore, we neglect the rotational velocity
        sourceRelativeSpeeds.omegaRadiansPerSecond = 0;
        Twist2d twist = sourceRelativeSpeeds.toTwist2d(deltaTime);

        Pose2d futurePose = new Pose2d(currentTranslation, new Rotation2d()).exp(twist);
        Translation2d futureTranslation = futurePose.getTranslation();

        return getClosestTranslation(futureTranslation, targetTranslations);
    }

    /**
     * Returns the closest Pair from a given list of target Pairs to the source Pose2d.
     *
     * @param currentLocation2d The source Pose2d
     * @param targetPairs Array of scoring Pair objects
     * @return The closest Pair which is closest to the source Pose2d.
     * @param <T> Generic Type
     */
    public static <T> Pair<Pose2d, T> getClosestPair(Pose2d currentLocation2d, Pair<Pose2d, T>[] targetPairs) {
        double poseDistance = Double.POSITIVE_INFINITY;
        Pair<Pose2d, T> closestPair = targetPairs[0];
        for (Pair<Pose2d, T> targetPair : targetPairs) {
            if (currentLocation2d
                            .getTranslation()
                            .getDistance(targetPair.getFirst().getTranslation())
                    < poseDistance) {
                poseDistance = currentLocation2d
                        .getTranslation()
                        .getDistance(targetPair.getFirst().getTranslation());
                closestPair = targetPair;
            }
        }

        return closestPair;
    }

    /**
     * Returns the closest pose from a given set, based on an extrapolated source pose.
     *
     * @param currentPose The current pose of the source.
     * @param sourceRelativeSpeeds The current velocity of the source.
     * @param deltaTime The time difference between now and the desired measurement point in seconds.
     * @param targetPairs The list of target poses.
     * @param <T> Generic Type.
     * @return The closest pose from a given set, based on the extrapolated source pose.
     */
    public static <T> Pair<Pose2d, T> getClosestFuturePair(
            Pose2d currentPose, ChassisSpeeds sourceRelativeSpeeds, double deltaTime, Pair<Pose2d, T>[] targetPairs) {
        // The translational velocity is important to know in which direction the mass of the robot is moving
        // Additionally, the translational velocity indicates in which direction the driver wants to move the robot
        // Therefore, we neglect the rotational velocity
        sourceRelativeSpeeds.omegaRadiansPerSecond = 0;
        Twist2d twist = sourceRelativeSpeeds.toTwist2d(deltaTime);

        Pose2d futurePose = currentPose.exp(twist);

        return getClosestPair(futurePose, targetPairs);
    }

    /**
     * Returns the closest Pair from a given list of target Pair objects to the source Translation2d.
     *
     * @param currentLocation2d The source Translation2d
     * @param targetPairs Array of scoring Pair objects
     * @return The closest Pair which is closest to the source Translation2d.
     * @param <T> Generic Type
     */
    public static <T> Pair<Translation2d, T> getClosestPose(
            Translation2d currentLocation2d, Pair<Translation2d, T>[] targetPairs) {
        double poseDistance = Double.POSITIVE_INFINITY;
        Pair<Translation2d, T> closestPair = targetPairs[0];
        for (Pair<Translation2d, T> targetPair : targetPairs) {
            if (currentLocation2d.getDistance(targetPair.getFirst()) < poseDistance) {
                poseDistance = currentLocation2d.getDistance(targetPair.getFirst());
                closestPair = targetPair;
            }
        }

        return closestPair;
    }

    /**
     * Calculates the pose at 50% of a linear interpolation between two poses.
     *
     * @param pose1 The first pose.
     * @param pose2 The second pose.
     * @return The pose at 50% of the linear interpolation between the two poses.
     */
    public static Pose2d averagePose(Pose2d pose1, Pose2d pose2) {
        return pose1.interpolate(pose2, 0.5);
    }

    /**
     * Calculates the translation at 50% of a linear interpolation between two translations.
     *
     * @param translation1 The first translation.
     * @param translation2 The second translation.
     * @return The translation at 50% of the linear interpolation between the two translations.
     */
    public static Translation2d averageTranslation(Translation2d translation1, Translation2d translation2) {
        return translation1.interpolate(translation2, 0.5);
    }
}
