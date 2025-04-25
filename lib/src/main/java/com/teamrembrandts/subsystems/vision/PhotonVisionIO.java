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
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** A class used to interface with a PhotonVision camera */
public abstract class PhotonVisionIO implements VisionIO {
    PhotonCamera camera;
    AprilTagFieldLayout aprilTagFieldLayout;
    Transform3d cameraToRobotTransform;
    String cameraName;
    final double rotationLoopsDelay = 2; // Fraction of looper cycles that the robot rotation is delayed

    final Alert nullPointerAlert;

    /**
     * Buffer which keeps track of the robot rotation over the past few seconds This allows us to match a vision
     * estimate (which are determined with some delay) with a robot rotation
     */
    TimeInterpolatableBuffer<Rotation2d> rotationBuffer = TimeInterpolatableBuffer.createBuffer(1.5);

    /**
     * Create a new PhotonVisionIO object for either a real or simulated robot
     *
     * @param cameraName Name of the camera in networktables
     * @param robotToCameraTransform Transform that represents the translation and rotation from robot centre to the
     *     camera
     * @param aprilTagFieldLayout Layout of the april tags around the field
     */
    public PhotonVisionIO(
            String cameraName, Transform3d robotToCameraTransform, AprilTagFieldLayout aprilTagFieldLayout) {
        this.cameraName = cameraName;
        camera = new PhotonCamera(cameraName);
        this.cameraToRobotTransform = robotToCameraTransform.inverse();
        this.aprilTagFieldLayout = aprilTagFieldLayout;

        nullPointerAlert = new Alert(
                "Alerts/Vision",
                "[Camera " + cameraName + "] has caused null pointer exceptions",
                Alert.AlertType.kError);
        nullPointerAlert.set(false);
    }

    /**
     * Update the inputs of a photon vision system Depending on whether one or multiple tags are visible, the robot pose
     * is estimated differently Multiple alternatives are found for the robot pose, which are stored in the robot pose
     * array
     *
     * @param inputs the inputs to update
     */
    @Override
    public void updateInputs(VisionInputs inputs) {
        // Default has results and is connected to false in case errors occur
        inputs.hasResults = false;
        inputs.connected = false;
        try {
            // Pass the camera name to the inputs
            inputs.cameraName = cameraName;

            // Check if the camera is connected, otherwise exit the update
            boolean cameraConnected = camera.isConnected();
            if (!cameraConnected) {
                return;
            }
            inputs.connected = true;

            // Check the results we have not processed yet
            List<PhotonPipelineResult> resultList = camera.getAllUnreadResults();
            // Prevent errors when there are no results
            if (resultList == null
                    || resultList.isEmpty()
                    || !resultList.get(resultList.size() - 1).hasTargets()) {
                return;
            }
            inputs.hasResults = true;

            // Retrieve the most recent result from the total list
            PhotonPipelineResult latestResult = resultList.get(resultList.size() - 1);
            List<PhotonTrackedTarget> visibleTags = latestResult.getTargets();

            inputs.ambiguityRatio = latestResult.getBestTarget().getPoseAmbiguity();

            // Fill in the tag area and visible ID arrays
            double[] areas = new double[visibleTags.size()];
            int[] id = new int[visibleTags.size()];
            for (int i = 0; i < visibleTags.size(); i++) {
                areas[i] = visibleTags.get(i).getArea();
                id[i] = visibleTags.get(i).getFiducialId();
            }
            inputs.tagAreas = areas;
            inputs.visibleTagIDs = id;

            // Retrieve in the timestamp of the latest measurement
            inputs.timeStamp = latestResult.getTimestampSeconds();

            // Determine the possible robot pose results the camera has determined
            if (visibleTags.size() == 1) {
                inputs.fieldSpaceRobotPoses = retrieveSingleTagEstimates(latestResult);
            } else {
                inputs.fieldSpaceRobotPoses = retrieveMultiTagEstimates(latestResult);
            }
        } catch (NullPointerException e) {
            nullPointerAlert.set(true);
        }
    }

    /**
     * Pass the robot rotation that is measured with the IMU to the vision system This should be updated every loop
     *
     * @param robotRotation Actual rotation of the robot
     */
    @Override
    public void setRobotRotation(Rotation2d robotRotation) {
        // Put the rotation in a buffer
        rotationBuffer.addSample(
                RobotController.getFPGATime() / 1e6 - rotationLoopsDelay * LoggedRobot.defaultPeriodSecs,
                robotRotation);
    }

    /**
     * Determine several options for the robot pose in field space when only 1 tag is visible This algorithm will
     * discard the rotation of the tag and use the robot rotation instead for improved accuracy
     *
     * @param latestResult latest PhotonPipelineResult
     * @return Array with possible robot poses in field space
     */
    private Pose3d[] retrieveSingleTagEstimates(PhotonPipelineResult latestResult) {
        Pose3d[] possibleRobotposes = new Pose3d[] {new Pose3d(), new Pose3d()};
        PhotonTrackedTarget target = latestResult.getBestTarget();

        // Only proceed if the target can be found in the april tag field layout
        // and the robot rotation can be retrieved at the time that the result was determined
        Optional<Pose3d> tagPoseOptional = aprilTagFieldLayout.getTagPose(target.getFiducialId());
        Optional<Rotation2d> robotRotationOptional = rotationBuffer.getSample(latestResult.getTimestampSeconds());
        if (tagPoseOptional.isPresent() && robotRotationOptional.isPresent()) {
            Pose3d tagPose = tagPoseOptional.get();
            Rotation3d tagRotation = tagPose.getRotation();
            Rotation2d robotRotation = robotRotationOptional.get();

            // Now convert robot rotation to Rotation3d and subtract it from the tag pose to get the
            // relative rotation between robot and target
            Rotation3d robotToTargetRot = tagRotation.minus(new Rotation3d(0, 0, robotRotation.getRadians()));
            // Now we can include the rotation between the camera and the robot
            Rotation3d cameraToTargetRot = robotToTargetRot.plus(cameraToRobotTransform.getRotation());

            // Now we can combine the rotation of the robot with the translation determined by the
            // camera
            Transform3d[] camToTargetOptions = new Transform3d[] {new Transform3d(), new Transform3d()};
            if (target.getBestCameraToTarget().getTranslation() != null
                    && target.getAlternateCameraToTarget().getTranslation() != null) {
                camToTargetOptions = new Transform3d[] {
                    new Transform3d(target.getBestCameraToTarget().getTranslation(), cameraToTargetRot),
                    new Transform3d(target.getAlternateCameraToTarget().getTranslation(), cameraToTargetRot)
                };
            }

            for (int i = 0; i < camToTargetOptions.length; i++) {
                Transform3d camToTarget = camToTargetOptions[i];
                possibleRobotposes[i] =
                        PhotonUtils.estimateFieldToRobotAprilTag(camToTarget, tagPose, cameraToRobotTransform);
            }
            logRotationDiff(tagPose.plus(target.getBestCameraToTarget().inverse()));
        }
        return possibleRobotposes;
    }

    /**
     * Determine several options for the robot pose in field space when multiple tags are visible
     *
     * @param latestResult latest PhotonPipelineResult
     * @return Array with possible robot poses in field space
     */
    private Pose3d[] retrieveMultiTagEstimates(PhotonPipelineResult latestResult) {
        // Retrieve the camera pose in field space represented as transform
        Transform3d bestMultiPose = new Transform3d(new Translation3d(), new Rotation3d());
        Transform3d alternateMultiPose = new Transform3d(new Translation3d(), new Rotation3d());

        if (latestResult.getMultiTagResult().isPresent()) {
            bestMultiPose = latestResult.getMultiTagResult().get().estimatedPose.best;
            alternateMultiPose = latestResult.getMultiTagResult().get().estimatedPose.alt;
        }
        logRotationDiff(new Pose3d().transformBy(bestMultiPose));

        // Convert to field space robot pose and return
        return new Pose3d[] {
            new Pose3d().transformBy(bestMultiPose).transformBy(cameraToRobotTransform),
            new Pose3d().transformBy(alternateMultiPose).transformBy(cameraToRobotTransform)
        };
    }

    /**
     * Log the difference between the camera rotation entered in constants and the camera rotation measured by the
     * vision system. A positive error means that the camera rotation needs to be reduced in constants.
     *
     * @param fieldSpaceCameraPose The detected pose of the camera in field space
     */
    private void logRotationDiff(Pose3d fieldSpaceCameraPose) {
        Rotation3d rotation = fieldSpaceCameraPose.getRotation();
        Rotation3d cameraRotation = cameraToRobotTransform.getRotation().unaryMinus();
        Logger.recordOutput(
                "Vision/Camera Rotation Error/" + cameraName + "/x (roll, deg)",
                cameraRotation.getMeasureX().minus(rotation.getMeasureX()).in(Units.Degree));
        Logger.recordOutput(
                "Vision/Camera Rotation Error/" + cameraName + "/y (pitch, deg)",
                cameraRotation.getMeasureY().minus(rotation.getMeasureY()).in(Units.Degree));
    }
}
