/*
 * Copyright (c) 2024-2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

/**
 * Parameters to filter vision measurements.
 *
 * @param xyStandardDevBase The base value for the translational standard deviation.
 * @param rotStandardDevBase The base value for the rotational standard deviation.
 * @param aprilTagWidth The width of the AprilTag.
 * @param maxAmbiguityRatio The maximum ambiguity ratio for a target to be considered valid.
 * @param maxAprilTagDistance The maximum distance to an AprilTag for which a measurement will be considered in meters
 * @param estimatedFOV Estimated horizontal field of view of camera as {@code Rotation2d}.
 * @param zMargin Margin in Z direction (height) outside which selected poses are not considered valid.
 * @param aprilTagFieldLayout Layout of the AprilTags around the field.
 */
public record VisionFilterParameters(
        double xyStandardDevBase,
        double rotStandardDevBase,
        Distance aprilTagWidth,
        double maxAmbiguityRatio,
        double maxAprilTagDistance,
        Rotation2d estimatedFOV,
        Distance zMargin,
        AprilTagFieldLayout aprilTagFieldLayout) {}
