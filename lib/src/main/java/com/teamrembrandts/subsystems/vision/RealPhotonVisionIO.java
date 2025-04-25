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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;

/** A class used to interface with a PhotonVision camera */
public class RealPhotonVisionIO extends PhotonVisionIO {

    /**
     * Create a new PhotonVisionIO object for a real robot
     *
     * @param cameraName Name of the camera in networktables
     * @param robotToCameraTransform Transform that represents the translation and rotation from robot centre to the
     *     camera
     * @param aprilTagFieldLayout Layout of the april tags around the field
     */
    public RealPhotonVisionIO(
            String cameraName, Transform3d robotToCameraTransform, AprilTagFieldLayout aprilTagFieldLayout) {
        super(cameraName, robotToCameraTransform, aprilTagFieldLayout);
    }
}
