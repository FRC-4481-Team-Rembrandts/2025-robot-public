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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for a vision IO */
public interface VisionIO {
    /** The inputs for a vision system */
    @AutoLog
    class VisionInputs {
        boolean connected = false;
        boolean hasResults = false;

        /** Ambiguity ration of the target, 0 is no ambiguity (good), 1 is maximum ambiguity (bad) */
        double ambiguityRatio = 0;

        int[] visibleTagIDs = {};
        Pose3d[] fieldSpaceRobotPoses = {new Pose3d(), new Pose3d()};
        double timeStamp = 0;

        /** Area of the tags as a percentage of the screen taken up by the tag (0 - 100) */
        double[] tagAreas = {};

        String cameraName = "";
    }

    /**
     * Update the inputs of a vision system
     *
     * @param inputs the inputs to update
     */
    void updateInputs(VisionInputs inputs);

    /**
     * Pass the robot rotation that is measured with the IMU to the vision system This should be updated every loop
     *
     * @param robotRotation Actual rotation of the robot
     */
    default void setRobotRotation(Rotation2d robotRotation) {}
    ;
}
