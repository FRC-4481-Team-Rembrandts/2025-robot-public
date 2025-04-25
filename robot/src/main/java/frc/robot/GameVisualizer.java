/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Singleton responsible for visualizing the mechanisms and subsystems of the robot and game pieces in a dashboard. */
public class GameVisualizer {

    private static GameVisualizer instance = null;

    @AutoLogOutput(key = "RobotVisualizer/Mechanism")
    private final LoggedMechanism2d mechanism;

    private final LoggedMechanismLigament2d elevator;
    private final LoggedMechanismLigament2d pivot;

    @AutoLogOutput(key = "RobotVisualizer/LEDStrip")
    private final LoggedMechanism2d ledMechanism;

    LoggedMechanismLigament2d bottomStrip;
    LoggedMechanismLigament2d topStrip;

    private Pose3d elevatorPose = new Pose3d();
    private Pose3d pivotPose = new Pose3d();

    private boolean algaeInRobot = false;
    private boolean coralInRobot = false;

    @AutoLogOutput(key = "RobotVisualizer/GamePieces/RobotAlgae")
    private Pose3d[] algaeRobotPose = new Pose3d[] {};

    @AutoLogOutput(key = "RobotVisualizer/GamePieces/RobotCoral")
    private Pose3d[] coralRobotPose = new Pose3d[] {};

    @AutoLogOutput(key = "RobotVisualizer/GamePieces/FieldCoral")
    private Pose3d[] coralFieldPoses = new Pose3d[] {};

    /** Creates a new GameVisualizer. */
    private GameVisualizer() {
        mechanism = new LoggedMechanism2d(3, 3);
        LoggedMechanismRoot2d root = mechanism.getRoot("Arm", 1.5, 0);

        elevator = root.append(new LoggedMechanismLigament2d("Elevator", DesignConstants.BASE_ELEVATOR_HEIGHT, 90));
        pivot = elevator.append(
                new LoggedMechanismLigament2d("Pivot", DesignConstants.PIVOT_LENGTH, DesignConstants.BASE_PIVOT_ANGLE));

        // D'n sims maar dan met LEDS
        ledMechanism = new LoggedMechanism2d(3, 3);
        LoggedMechanismRoot2d ledRoot = ledMechanism.getRoot("LED Strip", 1.5, 0);
        LoggedMechanismLigament2d offset = ledRoot.append(new LoggedMechanismLigament2d(
                "Offset", DesignConstants.LED_STRIP_OFFSET.getY(), 90, 0, new Color8Bit()));
        bottomStrip = offset.append(new LoggedMechanismLigament2d(
                "Bottom Strip", DesignConstants.LED_STRIP_HEIGHT / 2, 0, 5, new Color8Bit()));
        topStrip = bottomStrip.append(new LoggedMechanismLigament2d(
                "Top Strip", DesignConstants.LED_STRIP_HEIGHT / 2, 0, 5, new Color8Bit()));
    }

    /**
     * Gets the instance of the GameVisualizer.
     *
     * @return the instance of the GameVisualizer
     */
    public static GameVisualizer getInstance() {
        if (instance == null) {
            instance = new GameVisualizer();
        }
        return instance;
    }

    /**
     * Adds a coral to the reef based on a given anchor pose and a level. If the level is invalid, the coral will not be
     * added.
     *
     * @param anchor the anchor pose of the reef pole. In practice, this is always the auto align target.
     * @param level The level to place the coral at (L1-4).
     * @return A command that logs a {@code Pose3d} that represents the coral.
     */
    public Command scoreCoral(Supplier<Pose2d> anchor, Supplier<System.ArmState> level) {
        return Commands.runOnce(() -> {
            Pose3d anchorPose3d = new Pose3d(anchor.get());
            Pose3d newCoralPose = null;

            switch (level.get()) {
                case L1 -> newCoralPose = anchorPose3d.transformBy(DesignConstants.CORAL_L1_OFFSET);
                case L2 -> newCoralPose = anchorPose3d.transformBy(DesignConstants.CORAL_L2_OFFSET);
                case L3 -> newCoralPose = anchorPose3d.transformBy(DesignConstants.CORAL_L3_OFFSET);
                case L4 -> newCoralPose = anchorPose3d.transformBy(DesignConstants.CORAL_L4_OFFSET);
                default -> {}
            }

            if (newCoralPose != null) {
                Pose3d[] newCoralFieldPoses = new Pose3d[coralFieldPoses.length + 1];

                java.lang.System.arraycopy(coralFieldPoses, 0, newCoralFieldPoses, 0, coralFieldPoses.length);

                newCoralFieldPoses[coralFieldPoses.length] = newCoralPose;

                coralFieldPoses = newCoralFieldPoses;
            }
        });
    }

    /**
     * Adds an algae to the algae gripper in the visualization.
     *
     * @return A command that logs a {@code Pose3d} that represents the algae in the robot.
     */
    public Command setAlgaePresent() {
        return Commands.runOnce(() -> algaeInRobot = true);
    }

    /**
     * Adds a coral to the coral gripper in the visualization.
     *
     * @return A command that logs a {@code Pose3d} that represents the coral in the robot.
     */
    public Command setCoralPresent() {
        return Commands.runOnce(() -> coralInRobot = true);
    }

    /**
     * Removes the algae from the algae gripper in the visualization.
     *
     * @return A command that logs an empty array of {@code Pose3d} that represents the algae in the robot.
     */
    public Command setAlgaeAbsent() {
        return Commands.runOnce(() -> algaeInRobot = false);
    }

    /**
     * Removes the coral from the coral gripper in the visualization.
     *
     * @return A command that logs an empty array of {@code Pose3d} that represents the coral in the robot.
     */
    public Command setCoralAbsent() {
        return Commands.runOnce(() -> coralInRobot = false);
    }

    /**
     * Sets the color of the LED strip in the visualization.
     *
     * @param colorsBottom the colors of the bottom LED strip.
     * @param colorsTop the colors of the top LED strip.
     * @return A command that logs a {@code Color8Bit} that represents the LED strip in the robot.
     */
    public Command setLEDColors(Supplier<Color8Bit[]> colorsBottom, Supplier<Color8Bit[]> colorsTop) {
        return Commands.runOnce(() -> {
            bottomStrip.setColor(increaseBrightness(colorsBottom.get()[0]));
            topStrip.setColor(increaseBrightness(colorsTop.get()[0]));
        });
    }

    /**
     * Sets the position of the elevator.
     *
     * @param position the position of the elevator
     */
    public void setElevatorPosition(double position) {
        elevator.setLength(position);

        Translation3d translation = new Translation3d(0, 0, position);

        elevatorPose = new Pose3d(translation, new Rotation3d());
        pivotPose = new Pose3d(translation, pivotPose.getRotation());

        updateRobotAlgaePose();
        updateRobotCoralPose();

        Logger.recordOutput("RobotVisualizer/ModelPoses", elevatorPose, pivotPose);
    }

    /**
     * Sets the angle of the pivot.
     *
     * @param angle the angle of the pivot
     */
    public void setPivotAngle(Rotation2d angle) {
        pivot.setAngle(angle.getDegrees() + DesignConstants.BASE_PIVOT_ANGLE);

        Translation3d translation = pivotPose.getTranslation();
        Rotation3d rotation = new Rotation3d(0, -angle.getRadians(), 0);

        pivotPose = new Pose3d(translation, rotation);

        updateRobotAlgaePose();
        updateRobotCoralPose();

        Logger.recordOutput("RobotVisualizer/Mechanism", mechanism);
        Logger.recordOutput("RobotVisualizer/ModelPoses", elevatorPose, pivotPose);
    }

    /** Updates the pose of the algae in the robot. */
    private void updateRobotAlgaePose() {
        if (!algaeInRobot) {
            algaeRobotPose = new Pose3d[] {};
            return;
        }

        Pose3d gamePiecePoseRobotRelative = elevatorPose
                .transformBy(DesignConstants.ALGAE_CENTER_OFFSET)
                .rotateAround(elevatorPose.getTranslation(), pivotPose.getRotation());

        Transform3d gamePieceTransform =
                new Transform3d(gamePiecePoseRobotRelative.getTranslation(), gamePiecePoseRobotRelative.getRotation());

        Pose3d robotAlgaePose = new Pose3d(System.getInstance()
                        .getDrivetrain()
                        .getRobotPoseSupplier()
                        .get())
                .transformBy(gamePieceTransform);

        algaeRobotPose = new Pose3d[] {robotAlgaePose};
    }

    /** Updates the pose of the coral in the robot. */
    private void updateRobotCoralPose() {
        if (!coralInRobot) {
            coralRobotPose = new Pose3d[] {};
            return;
        }

        Pose3d gamePiecePoseRobotRelative = elevatorPose
                .transformBy(DesignConstants.CORAL_CENTER_OFFSET)
                .rotateAround(elevatorPose.getTranslation(), pivotPose.getRotation());

        Transform3d gamePieceTransform =
                new Transform3d(gamePiecePoseRobotRelative.getTranslation(), gamePiecePoseRobotRelative.getRotation());

        Pose3d robotCoralPose = new Pose3d(System.getInstance()
                        .getDrivetrain()
                        .getRobotPoseSupplier()
                        .get())
                .transformBy(gamePieceTransform);

        coralRobotPose = new Pose3d[] {robotCoralPose};
    }

    /**
     * Increases the brightness of a color.
     *
     * @param color the color to increase the brightness of
     * @return the new color with increased brightness
     */
    private Color8Bit increaseBrightness(Color8Bit color) {
        return new Color8Bit(
                (int) Math.min(color.red * DesignConstants.LED_BRIGHTNESS_MULTIPLIER, 255),
                (int) Math.min(color.green * DesignConstants.LED_BRIGHTNESS_MULTIPLIER, 255),
                (int) Math.min(color.blue * DesignConstants.LED_BRIGHTNESS_MULTIPLIER, 255));
    }

    /**
     * Geometric constants from the robot design needed for accurate visualization. Only includes measurements that
     * should not be accessible anywhere by the robot.
     */
    private static class DesignConstants {
        public static final double BASE_ELEVATOR_HEIGHT = 1.0;
        public static final double PIVOT_LENGTH = 0.5;
        public static final double BASE_PIVOT_ANGLE = -90;

        public static final double LED_STRIP_HEIGHT = 1;
        public static final Translation2d LED_STRIP_OFFSET = new Translation2d(0, 2.5);
        public static final double LED_BRIGHTNESS_MULTIPLIER = 4;

        /** The offset of the center of the algae from the center of the pivot. */
        public static final Transform3d ALGAE_CENTER_OFFSET = new Transform3d(0.17, 0, 0.32, new Rotation3d());
        /** The offset of the center of the coral from the center of the pivot. */
        public static final Transform3d CORAL_CENTER_OFFSET =
                new Transform3d(0.425, 0, 0.1, new Rotation3d(0, Math.PI / 2, 0));

        public static final Transform3d CORAL_L1_OFFSET =
                new Transform3d(0.55, 0, 0.5, new Rotation3d(0, Radians.convertFrom(-25, Degrees), 0));
        public static final Transform3d CORAL_L2_OFFSET =
                new Transform3d(0.53, 0, 0.7, new Rotation3d(0, Radians.convertFrom(35, Degrees), 0));
        public static final Transform3d CORAL_L3_OFFSET =
                new Transform3d(0.53, 0, 1.11, new Rotation3d(0, Radians.convertFrom(35, Degrees), 0));
        public static final Transform3d CORAL_L4_OFFSET =
                new Transform3d(0.47, 0, 1.68, new Rotation3d(0, Radians.convertFrom(75, Degrees), 0));
    }
}
