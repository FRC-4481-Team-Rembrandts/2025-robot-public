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

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** A subsystem used to gather visual data using cameras and process it to determine the robot's pose */
public class Vision extends SubsystemBase {

    VisionIO[] visionIOs; // Array of vision IOs representing the different cameras
    VisionInputsAutoLogged[] inputs;
    Alert[] connectionAlerts;
    VisionFilterParameters filterParameters;
    Pose2d actualRobotPose = new Pose2d();
    int[] disconnectCount;
    final int maxDisconnectCount = 5;
    final Pose3d defaultLogPos = new Pose3d(0, 0, -5, new Rotation3d());

    /**
     * Creates a new Vision subsystem
     *
     * @param filterParameters The parameters to filter the vision measurements.
     * @param ios The vision IOs representing the different cameras.
     */
    public Vision(VisionFilterParameters filterParameters, VisionIO... ios) {
        super("Vision");

        this.filterParameters = filterParameters;

        visionIOs = ios;
        // Create the array of vision inputs
        inputs = new VisionInputsAutoLogged[visionIOs.length];
        connectionAlerts = new Alert[visionIOs.length];

        for (int i = 0; i < visionIOs.length; i++) {
            inputs[i] = new VisionInputsAutoLogged();
            connectionAlerts[i] =
                    new Alert("Alerts/Vision/" + i, "[Camera " + i + "] disconnected", Alert.AlertType.kError);
        }
        disconnectCount = new int[] {0, 0, 0, 0};
    }

    @Override
    public void periodic() {
        // Update each io, which represents the different cameras
        for (int i = 0; i < visionIOs.length; i++) {
            // Update the inputs
            visionIOs[i].updateInputs(inputs[i]);
            visionIOs[i].setRobotRotation(actualRobotPose.getRotation());

            // Error handling for the vision IO
            connectionAlerts[i].set(!inputs[i].connected);
            SmartDashboard.putBoolean("Connection Status/Camera " + inputs[i].cameraName, inputs[i].connected);

            Logger.processInputs("Vision/" + inputs[i].cameraName + " (" + i + ")", inputs[i]);
        }
    }

    /**
     * Processes the vision measurements and adds them to the measurement queue
     *
     * @param robotPoseSupplier A supplier for the robot's pose
     * @param measurementConsumer A consumer for the vision measurements
     * @return A command that processes the vision measurements
     */
    public Command processVision(Supplier<Pose2d> robotPoseSupplier, Consumer<VisionMeasurement> measurementConsumer) {
        return Commands.run(
                () -> {
                    // First update the actual robot pose
                    actualRobotPose = robotPoseSupplier.get();

                    // Cycle through all cameras
                    for (int i = 0; i < visionIOs.length; i++) {
                        // Check for results, the hasResults is also false if there is something
                        // wrong with the camera object
                        if (!inputs[i].connected || !inputs[i].hasResults) {
                            disconnectCount[i] += 1;
                            // If the camera has no results for several cycles, log some default values
                            if (disconnectCount[i] > maxDisconnectCount) {
                                Logger.recordOutput("Vision/Accepted Poses/" + inputs[i].cameraName, defaultLogPos);
                            }
                            continue;
                        }
                        disconnectCount[i] = 0;

                        // Log all tag poses that are seen
                        List<Pose3d> tagPoses = new LinkedList<>();
                        for (int j = 0; j < inputs[i].visibleTagIDs.length; j++) {
                            Optional<Pose3d> tagPose =
                                    filterParameters.aprilTagFieldLayout().getTagPose(inputs[i].visibleTagIDs[j]);
                            tagPose.ifPresent(tagPoses::add);
                        }
                        Logger.recordOutput(
                                "Vision/Tag Poses/" + inputs[i].cameraName,
                                tagPoses.toArray(new Pose3d[tagPoses.size()]));

                        // If ambiguity ratio is too high, continue to the next IO
                        if (inputs[i].ambiguityRatio > filterParameters.maxAmbiguityRatio()) {
                            continue;
                        }

                        // Select the pose closest to the actual robot pose
                        Pose3d selectedPose;
                        if (inputs[i].ambiguityRatio < filterParameters.maxAmbiguityRatio() / 2) {
                            selectedPose = inputs[i].fieldSpaceRobotPoses[0];
                        } else {
                            selectedPose = selectClosestPose(inputs[i].fieldSpaceRobotPoses, actualRobotPose);
                        }
                        if (selectedPose == null || outsideFieldBounds(selectedPose)) {
                            continue;
                        }

                        // Determine the standard deviation of the measurement using the distance
                        // The distance is estimated based on the tag area in percent (0-100)
                        double tagDistance = calculateTagDistance(inputs[i].tagAreas);
                        Logger.recordOutput("Vision/Estimated Distance/" + inputs[i].cameraName, tagDistance);
                        if (tagDistance > filterParameters.maxAprilTagDistance()) {
                            continue;
                        }
                        Matrix<N3, N1> stdDevMat = determineStandardDeviation(
                                tagDistance, inputs[i].visibleTagIDs.length > 1, inputs[i].visibleTagIDs.length);

                        // Retrieve the timestamp of the measurement
                        double timestamp = inputs[i].timeStamp;

                        // Log the accepted pose
                        Logger.recordOutput("Vision/Accepted Poses/" + inputs[i].cameraName, selectedPose);
                        Logger.recordOutput("Vision/Standard Deviations/" + inputs[i].cameraName, stdDevMat.getData());
                        // Add the measurement to the queue
                        measurementConsumer.accept(
                                new VisionMeasurement(selectedPose.toPose2d(), timestamp, stdDevMat));
                    }
                },
                this);
    }

    /**
     * selects the closest pose to the actual robot pose.
     *
     * @param fieldSpaceRobotPoses The measured poses of the robot in field space.
     * @param actualRobotPose The actual robot pose.
     * @return The closest measured pose to the current robot pose.
     */
    private Pose3d selectClosestPose(Pose3d[] fieldSpaceRobotPoses, Pose2d actualRobotPose) {
        // Get the distance closest to the current robot pose if multiple poses are determined for a
        // camera
        double minDistance = Double.POSITIVE_INFINITY;
        Pose3d selectedPose = null;
        Pose3d actualRobotPose3d = new Pose3d(actualRobotPose);

        for (Pose3d fieldSpaceRobotPose : fieldSpaceRobotPoses) {
            double distance = fieldSpaceRobotPose.getTranslation().getDistance(actualRobotPose3d.getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
                selectedPose = fieldSpaceRobotPose;
            }
        }
        return selectedPose;
    }

    /**
     * Checks if the selected pose is outside the field bounds.
     *
     * @param selectedPose The selected pose.
     * @return True if the selected pose is outside the field bounds, false otherwise.
     */
    private boolean outsideFieldBounds(Pose3d selectedPose) {
        return selectedPose.getTranslation().getX() < 0
                || selectedPose.getTranslation().getX()
                        > filterParameters.aprilTagFieldLayout().getFieldLength()
                || selectedPose.getTranslation().getY() < 0
                || selectedPose.getTranslation().getY()
                        > filterParameters.aprilTagFieldLayout().getFieldWidth()
                || selectedPose.getTranslation().getZ()
                        < -filterParameters.zMargin().in(Units.Meters)
                || selectedPose.getTranslation().getZ()
                        > filterParameters.zMargin().in(Units.Meters);
    }

    /**
     * Calculates the minimum tag distance based on the tag areas.
     *
     * @param tagAreas The areas of the tags.
     * @return The minimum tag distance.
     */
    private double calculateTagDistance(double[] tagAreas) {
        // Get the largest area, this will be the closest tag
        double largestArea = tagAreas[0];
        for (double area : tagAreas) {
            if (area > largestArea) {
                largestArea = area;
            }
        }
        // Estimate of distance when tag fills up 100 percent of the screen
        double minTagDistance = filterParameters.aprilTagWidth().in(Units.Meters)
                / (2 * Math.tan(filterParameters.estimatedFOV().getRadians() / 2));
        // Estimate the tag distance in meters
        return (1 / Math.sqrt(largestArea / 100)) * minTagDistance;
    }

    /**
     * Determines the standard deviation of the measurement.
     *
     * @param tagDistance The distance to the tag.
     * @param isMultiPose True if multiple poses are determined for a camera, false otherwise.
     * @param tagCount The number of tags visible.
     * @return The standard deviation of the measurement.
     */
    private Matrix<N3, N1> determineStandardDeviation(double tagDistance, boolean isMultiPose, int tagCount) {
        // Determine the standard deviation of the measurement
        double rotStdDev;
        double stdDevFactor = Math.pow(tagDistance, 4.0) / tagCount;
        double xyStdDev = filterParameters.xyStandardDevBase() * stdDevFactor;

        if (isMultiPose) {
            // Multi tag result
            rotStdDev = filterParameters.rotStandardDevBase() * stdDevFactor;
        } else {
            // Single tag result
            // Determine the corresponding standard deviation for this result
            // When the distance is 1 meter, the standard deviation is the base value
            rotStdDev = 1e4; // Don't use rotation result when only one tag is visible
        }
        return MatBuilder.fill(Nat.N3(), Nat.N1(), xyStdDev, xyStdDev, rotStdDev);
    }
}
