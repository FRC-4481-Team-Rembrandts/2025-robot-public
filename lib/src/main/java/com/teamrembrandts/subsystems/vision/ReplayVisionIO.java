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

/** A class used to replay vision inputs */
public class ReplayVisionIO implements VisionIO {
    private final String cameraName;

    /**
     * Create a new PhotonVisionIO object for a replay This effectively only updates the camera name to retrieve the
     * right logs
     *
     * @param cameraName Name of the camera in networktables
     */
    public ReplayVisionIO(String cameraName) {
        this.cameraName = cameraName;
    }

    /** {@inheritDoc} */
    public void updateInputs(VisionInputs inputs) {
        inputs.cameraName = cameraName;
    }
}
