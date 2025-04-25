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

import com.teamrembrandts.simulation.VisionEnvironmentSimulator;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

/** A class used to interface with a PhotonVision camera */
public class SimulatedPhotonVisionIO extends PhotonVisionIO {

    /**
     * Create a new PhotonVisionIO object for a simulated robot
     *
     * @param cameraName Name of the camera in networktables
     * @param robotToCameraTransform Transform that represents the translation and rotation from robot centre to the
     *     camera
     * @param aprilTagFieldLayout Layout of the april tags around the field
     * @param simCameraProperties Properties of the simulated PhotonVision camera
     */
    public SimulatedPhotonVisionIO(
            String cameraName,
            Transform3d robotToCameraTransform,
            AprilTagFieldLayout aprilTagFieldLayout,
            SimCameraProperties simCameraProperties) {
        super(cameraName, robotToCameraTransform, aprilTagFieldLayout);
        PhotonCameraSim camSim = new PhotonCameraSim(camera, simCameraProperties);
        // Enable the processed streams to localhost:1182.
        camSim.enableRawStream(true);
        camSim.enableProcessedStream(true);
        // Enable drawing a wireframe visualization of the field to the camera streams.
        // This is extremely resource-intensive and is disabled by default.
        camSim.enableDrawWireframe(true);
        VisionEnvironmentSimulator.getInstance().addCamera(camSim, robotToCameraTransform);
    }
}
