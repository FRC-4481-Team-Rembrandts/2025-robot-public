/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * A vision measurement
 *
 * @param robotPose The robot's pose.
 * @param timestamp The timestamp of the measurement.
 * @param stdDevs The standard deviations of the measurement.
 */
public record VisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs) {}
