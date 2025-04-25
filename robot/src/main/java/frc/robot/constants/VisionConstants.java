/*
 * Copyright (c) 2024-2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.constants;

import com.teamrembrandts.subsystems.vision.VisionFilterParameters;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import org.photonvision.simulation.SimCameraProperties;

public class VisionConstants {
    /** Location of all the AprilTags on the field */
    private static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT_2025_WELDED =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final String CUSTOM_APRIL_TAG_JSON = "2025-reefscape.json";
    private static final File APRIL_TAG_DIR = new File(Filesystem.getDeployDirectory(), "apriltag");
    private static final File LAYOUT_FILE = new File(APRIL_TAG_DIR, CUSTOM_APRIL_TAG_JSON);

    private static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT_CUSTOM;

    public static final double XY_STANDARD_DEV_BASE = 0.0060;
    public static final double ROTATION_STANDARD_DEV_BASE = 0.040;
    public static final Distance APRILTAG_WIDTH = Units.Millimeters.of(165);
    public static final double MAX_AMBIGUITY_RATIO = 0.5;
    public static final double MAX_APRILTAG_DISTANCE = 6;
    public static final Distance Z_HEIGHT_MARGIN = Units.Centimeter.of(15);
    /** This estimated FOV is used only for estimating the distance to the AprilTag */
    public static final Rotation2d ESTIMATED_FOV = Rotation2d.fromDegrees(60);

    static {
        AprilTagFieldLayout aprilTagFieldLayoutCustom1;
        try {
            aprilTagFieldLayoutCustom1 = new AprilTagFieldLayout(LAYOUT_FILE.toPath());
        } catch (IOException e) {
            // Default to 2025 field layout
            aprilTagFieldLayoutCustom1 = APRIL_TAG_FIELD_LAYOUT_2025_WELDED;
            System.err.println("Error loading custom AprilTag field layout: " + e.getMessage());
            System.err.println("Defaulting to 2025 field layout");
        }
        APRIL_TAG_FIELD_LAYOUT_CUSTOM = aprilTagFieldLayoutCustom1;
    }

    // Choose which april tag layout to use
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = APRIL_TAG_FIELD_LAYOUT_2025_WELDED;

    /** Transform3d object that represents the translation and rotation from the robot centre to the camera */
    public static final Transform3d ROBOT_TO_CAMERA_FRONT_LEFT = new Transform3d(
            new Translation3d(0.198, 0.221, 0.21386),
            new Rotation3d(Math.toRadians(0), Math.toRadians(-20), Math.toRadians(-14)));

    public static final Transform3d ROBOT_TO_CAMERA_FRONT_RIGHT = new Transform3d(
            new Translation3d(0.198, -0.221, 0.21386),
            new Rotation3d(Math.toRadians(0), Math.toRadians(-20), Math.toRadians(14)));

    public static final Transform3d ROBOT_TO_CAMERA_BACK_RIGHT = new Transform3d(
            new Translation3d(-0.06175, -0.1732, 0.97234),
            new Rotation3d(Math.toRadians(-178), Math.toRadians(-35.5), Math.toRadians(150)));

    public static final Transform3d ROBOT_TO_CAMERA_BACK_LEFT = new Transform3d(
            new Translation3d(-0.11175, 0.1932, 0.97234),
            new Rotation3d(Math.toRadians(-180), Math.toRadians(-36), Math.toRadians(-150)));

    public static final String CAMERA_FRONT_LEFT_NAME = "front_left_camera";
    public static final String CAMERA_FRONT_RIGHT_NAME = "front_right_camera";
    public static final String CAMERA_BACK_LEFT_NAME = "back_left_camera";
    public static final String CAMERA_BACK_RIGHT_NAME = "back_right_camera";

    public static final SimCameraProperties CAMERA_SIM_PROPERTIES = new SimCameraProperties();

    // Setup the camera sim properties
    static {
        CAMERA_SIM_PROPERTIES.setCalibration(800, 600, Rotation2d.fromDegrees(100));
        CAMERA_SIM_PROPERTIES.setCalibError(0.25, 0.08);
        CAMERA_SIM_PROPERTIES.setFPS(20);
        CAMERA_SIM_PROPERTIES.setAvgLatencyMs(35);
        CAMERA_SIM_PROPERTIES.setLatencyStdDevMs(5);
    }

    public static final VisionFilterParameters VISION_FILTER_PARAMETERS = new VisionFilterParameters(
            XY_STANDARD_DEV_BASE,
            ROTATION_STANDARD_DEV_BASE,
            APRILTAG_WIDTH,
            MAX_AMBIGUITY_RATIO,
            MAX_APRILTAG_DISTANCE,
            ESTIMATED_FOV,
            Z_HEIGHT_MARGIN,
            APRIL_TAG_FIELD_LAYOUT);
}
