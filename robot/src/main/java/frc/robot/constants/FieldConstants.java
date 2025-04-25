/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.constants;

import static com.teamrembrandts.util.GeometryUtil.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.System;
import java.util.Map;

public class FieldConstants {
    public static final double FIELD_LENGTH = 17.548;
    public static final double FIELD_WIDTH = 8.052;

    public static final Pose2d REEF_BLUE_A = new Pose2d(3.2106, 4.1904, new Rotation2d(6.2832))
            .transformBy(new Transform2d(0, 0.014, new Rotation2d(0)));
    public static final Pose2d REEF_BLUE_B =
            new Pose2d(3.2106, 3.8614, new Rotation2d(6.2832)).transformBy(new Transform2d(0, 0.01, new Rotation2d(0)));
    public static final Pose2d REEF_BLUE_C =
            new Pose2d(3.7080, 3.0014, new Rotation2d(1.0472)).transformBy(new Transform2d(0, 0, new Rotation2d(0)));
    public static final Pose2d REEF_BLUE_D =
            new Pose2d(3.9929, 2.8370, new Rotation2d(1.0472)).transformBy(new Transform2d(0, 0, new Rotation2d(0)));
    public static final Pose2d REEF_BLUE_E =
            new Pose2d(4.9858, 2.8370, new Rotation2d(2.0944)).transformBy(new Transform2d(0, 0, new Rotation2d(0)));
    public static final Pose2d REEF_BLUE_F = new Pose2d(5.2707, 3.0014, new Rotation2d(2.0944))
            .transformBy(new Transform2d(0, -0.01, new Rotation2d(0)));
    public static final Pose2d REEF_BLUE_G =
            new Pose2d(5.7680, 3.8614, new Rotation2d(3.1416)).transformBy(new Transform2d(0, 0, new Rotation2d(0)));
    public static final Pose2d REEF_BLUE_H =
            new Pose2d(5.7680, 4.1904, new Rotation2d(3.1416)).transformBy(new Transform2d(0, 0, new Rotation2d(0)));
    public static final Pose2d REEF_BLUE_I =
            new Pose2d(5.2707, 5.0504, new Rotation2d(4.1888)).transformBy(new Transform2d(0, 0.01, new Rotation2d(0)));
    public static final Pose2d REEF_BLUE_J =
            new Pose2d(4.9858, 5.2148, new Rotation2d(4.1888)).transformBy(new Transform2d(0, 0, new Rotation2d(0)));
    public static final Pose2d REEF_BLUE_K = new Pose2d(3.9929, 5.2148, new Rotation2d(5.2360))
            .transformBy(new Transform2d(0, 0.017, new Rotation2d(0)));
    public static final Pose2d REEF_BLUE_L =
            new Pose2d(3.7080, 5.0504, new Rotation2d(5.2360)).transformBy(new Transform2d(0, 0.02, new Rotation2d(0)));
    public static final Pose2d REEF_RED_A =
            new Pose2d(14.3375, 3.8614, new Rotation2d(3.1416)).transformBy(new Transform2d(0, 0, new Rotation2d(0)));
    public static final Pose2d REEF_RED_B =
            new Pose2d(14.3375, 4.1904, new Rotation2d(3.1416)).transformBy(new Transform2d(0, 0, new Rotation2d(0)));
    public static final Pose2d REEF_RED_C =
            new Pose2d(13.8404, 5.0504, new Rotation2d(4.1888)).transformBy(new Transform2d(0, 0, new Rotation2d(0)));
    public static final Pose2d REEF_RED_D =
            new Pose2d(13.5555, 5.2148, new Rotation2d(4.1888)).transformBy(new Transform2d(0, 0, new Rotation2d(0)));
    public static final Pose2d REEF_RED_E =
            new Pose2d(12.5623, 5.2148, new Rotation2d(5.2360)).transformBy(new Transform2d(0, 0, new Rotation2d(0)));
    public static final Pose2d REEF_RED_F =
            new Pose2d(12.2774, 5.0504, new Rotation2d(5.2360)).transformBy(new Transform2d(0, 0, new Rotation2d(0)));
    public static final Pose2d REEF_RED_G = new Pose2d(11.7803, 4.1904, new Rotation2d(6.2832))
            .transformBy(new Transform2d(0, 0.007, new Rotation2d(0))); // y=0.014
    public static final Pose2d REEF_RED_H = new Pose2d(11.7803, 3.8614, new Rotation2d(6.2832))
            .transformBy(new Transform2d(0, 0.01, new Rotation2d(0)));
    public static final Pose2d REEF_RED_I =
            new Pose2d(12.2774, 3.0014, new Rotation2d(1.0472)).transformBy(new Transform2d(0, 0, new Rotation2d(0)));
    public static final Pose2d REEF_RED_J = new Pose2d(12.5623, 2.8370, new Rotation2d(1.0472))
            .transformBy(new Transform2d(0, 0.014, new Rotation2d(0)));
    public static final Pose2d REEF_RED_K =
            new Pose2d(13.5555, 2.8370, new Rotation2d(2.0944)).transformBy(new Transform2d(0, 0, new Rotation2d(0)));
    public static final Pose2d REEF_RED_L =
            new Pose2d(13.8404, 3.0014, new Rotation2d(2.0944)).transformBy(new Transform2d(0, 0, new Rotation2d(0)));

    public static final Pose2d REEF_BLUE_MIDDLE_AB = averagePose(REEF_BLUE_A, REEF_BLUE_B);
    public static final Pose2d REEF_BLUE_MIDDLE_CD = averagePose(REEF_BLUE_C, REEF_BLUE_D);
    public static final Pose2d REEF_BLUE_MIDDLE_EF = averagePose(REEF_BLUE_E, REEF_BLUE_F);
    public static final Pose2d REEF_BLUE_MIDDLE_GH = averagePose(REEF_BLUE_G, REEF_BLUE_H);
    public static final Pose2d REEF_BLUE_MIDDLE_IJ = averagePose(REEF_BLUE_I, REEF_BLUE_J);
    public static final Pose2d REEF_BLUE_MIDDLE_KL = averagePose(REEF_BLUE_K, REEF_BLUE_L);

    public static final Pose2d REEF_RED_MIDDLE_AB = averagePose(REEF_RED_A, REEF_RED_B);
    public static final Pose2d REEF_RED_MIDDLE_CD = averagePose(REEF_RED_C, REEF_RED_D);
    public static final Pose2d REEF_RED_MIDDLE_EF = averagePose(REEF_RED_E, REEF_RED_F);
    public static final Pose2d REEF_RED_MIDDLE_GH = averagePose(REEF_RED_G, REEF_RED_H);
    public static final Pose2d REEF_RED_MIDDLE_IJ = averagePose(REEF_RED_I, REEF_RED_J);
    public static final Pose2d REEF_RED_MIDDLE_KL = averagePose(REEF_RED_K, REEF_RED_L);

    public static final Pose2d[] REEF_RED_RIGHT = {
        REEF_RED_A, REEF_RED_C, REEF_RED_E, REEF_RED_G, REEF_RED_I, REEF_RED_K
    };
    public static final Pose2d[] REEF_RED_LEFT = {REEF_RED_B, REEF_RED_D, REEF_RED_F, REEF_RED_H, REEF_RED_J, REEF_RED_L
    };
    public static final Pose2d[] REEF_RED_MIDDLE = {
        REEF_RED_MIDDLE_AB,
        REEF_RED_MIDDLE_CD,
        REEF_RED_MIDDLE_EF,
        REEF_RED_MIDDLE_GH,
        REEF_RED_MIDDLE_IJ,
        REEF_RED_MIDDLE_KL
    };

    public static final Pose2d[] REEF_RED_MIDDLE_REVERSE = {
        REEF_RED_MIDDLE_AB.plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180))),
        REEF_RED_MIDDLE_CD.plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180))),
        REEF_RED_MIDDLE_EF.plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180))),
        REEF_RED_MIDDLE_GH.plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180))),
        REEF_RED_MIDDLE_IJ.plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180))),
        REEF_RED_MIDDLE_KL.plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180)))
    };

    public static final Pose2d[] REEF_RED = {
        REEF_RED_A,
        REEF_RED_C,
        REEF_RED_E,
        REEF_RED_G,
        REEF_RED_I,
        REEF_RED_K,
        REEF_RED_B,
        REEF_RED_D,
        REEF_RED_F,
        REEF_RED_H,
        REEF_RED_J,
        REEF_RED_L
    };

    public static final Pose2d[] REEF_BLUE_RIGHT = {
        REEF_BLUE_A, REEF_BLUE_C, REEF_BLUE_E, REEF_BLUE_G, REEF_BLUE_I, REEF_BLUE_K
    };

    public static final Pose2d[] REEF_BLUE_LEFT = {
        REEF_BLUE_B, REEF_BLUE_D, REEF_BLUE_F, REEF_BLUE_H, REEF_BLUE_J, REEF_BLUE_L
    };

    public static final Pose2d[] REEF_BLUE_MIDDLE = {
        REEF_BLUE_MIDDLE_AB,
        REEF_BLUE_MIDDLE_CD,
        REEF_BLUE_MIDDLE_EF,
        REEF_BLUE_MIDDLE_GH,
        REEF_BLUE_MIDDLE_IJ,
        REEF_BLUE_MIDDLE_KL
    };

    public static final Pose2d[] REEF_BLUE_MIDDLE_REVERSE = {
        REEF_BLUE_MIDDLE_AB.plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180))),
        REEF_BLUE_MIDDLE_CD.plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180))),
        REEF_BLUE_MIDDLE_EF.plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180))),
        REEF_BLUE_MIDDLE_GH.plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180))),
        REEF_BLUE_MIDDLE_IJ.plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180))),
        REEF_BLUE_MIDDLE_KL.plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180)))
    };

    public static final Pose2d[] REEF_BLUE = {
        REEF_BLUE_A,
        REEF_BLUE_C,
        REEF_BLUE_E,
        REEF_BLUE_G,
        REEF_BLUE_I,
        REEF_BLUE_K,
        REEF_BLUE_B,
        REEF_BLUE_D,
        REEF_BLUE_F,
        REEF_BLUE_H,
        REEF_BLUE_J,
        REEF_BLUE_L
    };

    public static final Map<System.ArmState, Transform2d> REEF_OFFSET_MAP_RIGHT = Map.ofEntries(
            Map.entry(System.ArmState.L4, new Transform2d(0, 0, new Rotation2d(0))),
            Map.entry(System.ArmState.L2, new Transform2d(-0.095, 0, new Rotation2d(0))),
            Map.entry(System.ArmState.L3, new Transform2d(-0.1, 0, new Rotation2d(0))),
            Map.entry(System.ArmState.L1, new Transform2d(0.0, -0.30, Rotation2d.fromDegrees(0))));

    public static final Map<System.ArmState, Transform2d> REEF_OFFSET_MAP_LEFT = Map.ofEntries(
            Map.entry(System.ArmState.L4, new Transform2d(0, 0, new Rotation2d(0))),
            Map.entry(System.ArmState.L2, new Transform2d(-0.095, 0, new Rotation2d(0))),
            Map.entry(System.ArmState.L3, new Transform2d(-0.1, 0, new Rotation2d(0))),
            Map.entry(System.ArmState.L1, new Transform2d(-0, 0.30, Rotation2d.fromDegrees(0))));
    // scoring L2

    // Right/left is to indicate on which side of the AprilTag should align to a coral station
    // Here, right means on the right side of the AprilTag when viewing the coral station from the field
    // and left is the other side.
    // Barge/processor is the side at which the specific coral station is
    public static final Pose2d CORAL_STATION_BLUE_PROCESSOR_RIGHT = new Pose2d(0.880, 1.29, Rotation2d.fromDegrees(54))
            .transformBy(new Transform2d(0.0, 0.1, new Rotation2d()));
    public static final Pose2d CORAL_STATION_BLUE_PROCESSOR_LEFT =
            new Pose2d(1.60, 0.749, Rotation2d.fromDegrees(54)).transformBy(new Transform2d(0.0, 0, new Rotation2d()));
    public static final Pose2d CORAL_STATION_BLUE_BARGE_RIGHT = new Pose2d(1.60, 7.30, Rotation2d.fromDegrees(-54))
            .transformBy(new Transform2d(0.0, -0.11, new Rotation2d()));
    public static final Pose2d CORAL_STATION_BLUE_BARGE_LEFT =
            new Pose2d(0.880, 6.76, Rotation2d.fromDegrees(-54)).transformBy(new Transform2d(0.0, 0, new Rotation2d()));

    public static final Pose2d CORAL_STATION_RED_PROCESSOR_RIGHT = new Pose2d(16.67, 6.76, Rotation2d.fromDegrees(-126))
            .transformBy(new Transform2d(0.0, 0.1, new Rotation2d()));
    public static final Pose2d CORAL_STATION_RED_PROCESSOR_LEFT = new Pose2d(15.95, 7.30, Rotation2d.fromDegrees(-126))
            .transformBy(new Transform2d(0.0, 0, new Rotation2d()));
    public static final Pose2d CORAL_STATION_RED_BARGE_RIGHT = new Pose2d(15.95, 0.749, Rotation2d.fromDegrees(126))
            .transformBy(new Transform2d(0.0, 0.1, new Rotation2d()));
    public static final Pose2d CORAL_STATION_RED_BARGE_LEFT = new Pose2d(16.67, 1.290, Rotation2d.fromDegrees(126))
            .transformBy(new Transform2d(0.0, 0, new Rotation2d()));

    public static final Pose2d[] CORAL_STATION_BLUE_RIGHT = {
        CORAL_STATION_BLUE_PROCESSOR_RIGHT, CORAL_STATION_BLUE_BARGE_RIGHT
    };
    public static final Pose2d[] CORAL_STATION_BLUE_LEFT = {
        CORAL_STATION_BLUE_PROCESSOR_LEFT, CORAL_STATION_BLUE_BARGE_LEFT
    };
    public static final Pose2d[] CORAL_STATION_RED_RIGHT = {
        CORAL_STATION_RED_PROCESSOR_RIGHT, CORAL_STATION_RED_BARGE_RIGHT
    };
    public static final Pose2d[] CORAL_STATION_RED_LEFT = {
        CORAL_STATION_RED_BARGE_LEFT, CORAL_STATION_RED_PROCESSOR_LEFT
    };

    public static final Pair<Pose2d, AlgaePosition> ALGAE_BLUE_AB = new Pair<>(REEF_BLUE_MIDDLE_AB, AlgaePosition.HIGH);
    public static final Pair<Pose2d, AlgaePosition> ALGAE_BLUE_CD = new Pair<>(REEF_BLUE_MIDDLE_CD, AlgaePosition.LOW);
    public static final Pair<Pose2d, AlgaePosition> ALGAE_BLUE_EF = new Pair<>(REEF_BLUE_MIDDLE_EF, AlgaePosition.HIGH);
    public static final Pair<Pose2d, AlgaePosition> ALGAE_BLUE_GH = new Pair<>(REEF_BLUE_MIDDLE_GH, AlgaePosition.LOW);
    public static final Pair<Pose2d, AlgaePosition> ALGAE_BLUE_IJ = new Pair<>(REEF_BLUE_MIDDLE_IJ, AlgaePosition.HIGH);
    public static final Pair<Pose2d, AlgaePosition> ALGAE_BLUE_KL = new Pair<>(REEF_BLUE_MIDDLE_KL, AlgaePosition.LOW);

    public static final Pair<Pose2d, AlgaePosition> ALGAE_RED_AB = new Pair<>(REEF_RED_MIDDLE_AB, AlgaePosition.HIGH);
    public static final Pair<Pose2d, AlgaePosition> ALGAE_RED_CD = new Pair<>(REEF_RED_MIDDLE_CD, AlgaePosition.LOW);
    public static final Pair<Pose2d, AlgaePosition> ALGAE_RED_EF = new Pair<>(REEF_RED_MIDDLE_EF, AlgaePosition.HIGH);
    public static final Pair<Pose2d, AlgaePosition> ALGAE_RED_GH = new Pair<>(REEF_RED_MIDDLE_GH, AlgaePosition.LOW);
    public static final Pair<Pose2d, AlgaePosition> ALGAE_RED_IJ = new Pair<>(REEF_RED_MIDDLE_IJ, AlgaePosition.HIGH);
    public static final Pair<Pose2d, AlgaePosition> ALGAE_RED_KL = new Pair<>(REEF_RED_MIDDLE_KL, AlgaePosition.LOW);

    public static final Pair<Pose2d, AlgaePosition>[] ALGAE_BLUE_REEF =
            new Pair[] {ALGAE_BLUE_AB, ALGAE_BLUE_CD, ALGAE_BLUE_EF, ALGAE_BLUE_GH, ALGAE_BLUE_IJ, ALGAE_BLUE_KL};

    public static final Pair<Pose2d, AlgaePosition>[] ALGAE_RED_REEF =
            new Pair[] {ALGAE_RED_AB, ALGAE_RED_CD, ALGAE_RED_EF, ALGAE_RED_GH, ALGAE_RED_IJ, ALGAE_RED_KL};

    public static final Pair<Pose2d, AlgaePosition>[] ALGAE_BOTH_REEFS = new Pair[] {
        ALGAE_BLUE_AB,
        ALGAE_BLUE_CD,
        ALGAE_BLUE_EF,
        ALGAE_BLUE_GH,
        ALGAE_BLUE_IJ,
        ALGAE_BLUE_KL,
        ALGAE_RED_AB,
        ALGAE_RED_CD,
        ALGAE_RED_EF,
        ALGAE_RED_GH,
        ALGAE_RED_IJ,
        ALGAE_RED_KL
    };

    // Intuitively, the center of the reef is exactly in the middle between two opposing sides. The sides are chosen
    // arbitrarily
    public static final Translation2d REEF_BLUE_CENTER = averageTranslation(
            ALGAE_BLUE_AB.getFirst().getTranslation(), ALGAE_BLUE_GH.getFirst().getTranslation());

    public static final Translation2d REEF_RED_CENTER = averageTranslation(
            ALGAE_RED_AB.getFirst().getTranslation(), ALGAE_RED_GH.getFirst().getTranslation());

    /** The position of the ALGAE on the REEF. */
    public enum AlgaePosition {
        HIGH,
        LOW
    }

    public static final double RED_SCORING_LINE_BARGE = 10.0 - 0.26;
    public static final double BLUE_SCORING_LINE_BARGE = 7.5 + 0.26;
    public static final double ELEVATOR_SAFEZONE_BARGE = 1.75;
    public static final double PREP_LINE_BARGE = 8.75;

    /** Starting from left to right from driverstation PoV */
    public static final Pose2d BARGE_RED_1 =
            new Pose2d(RED_SCORING_LINE_BARGE, 0.7, Rotation2d.fromRadians(0)); // -2.7  + PI

    public static final Pose2d BARGE_RED_2 = new Pose2d(RED_SCORING_LINE_BARGE, 1, Rotation2d.fromRadians(0));
    public static final Pose2d BARGE_RED_3 = new Pose2d(RED_SCORING_LINE_BARGE, 1.3, Rotation2d.fromRadians(0));
    public static final Pose2d BARGE_RED_4 = new Pose2d(RED_SCORING_LINE_BARGE, 1.6, Rotation2d.fromRadians(0));
    public static final Pose2d BARGE_RED_5 = new Pose2d(RED_SCORING_LINE_BARGE, 1.9, Rotation2d.fromRadians(0));
    public static final Pose2d BARGE_RED_6 = new Pose2d(RED_SCORING_LINE_BARGE, 2.2, Rotation2d.fromRadians(0));
    public static final Pose2d BARGE_RED_7 = new Pose2d(RED_SCORING_LINE_BARGE, 2.5, Rotation2d.fromRadians(0));
    public static final Pose2d BARGE_RED_8 = new Pose2d(RED_SCORING_LINE_BARGE, 2.8, Rotation2d.fromRadians(0));
    public static final Pose2d BARGE_RED_9 = new Pose2d(RED_SCORING_LINE_BARGE, 3.1, Rotation2d.fromRadians(0));
    public static final Pose2d BARGE_RED_10 = new Pose2d(RED_SCORING_LINE_BARGE, 3.4, Rotation2d.fromRadians(0));

    // scoring on the blue side
    public static final Pose2d BARGE_RED_1_B =
            new Pose2d(BLUE_SCORING_LINE_BARGE, 0.7, Rotation2d.fromRadians(Math.PI)); // 2.7
    public static final Pose2d BARGE_RED_2_B = new Pose2d(BLUE_SCORING_LINE_BARGE, 1, Rotation2d.fromRadians(Math.PI));
    public static final Pose2d BARGE_RED_3_B =
            new Pose2d(BLUE_SCORING_LINE_BARGE, 1.3, Rotation2d.fromRadians(Math.PI));
    public static final Pose2d BARGE_RED_4_B =
            new Pose2d(BLUE_SCORING_LINE_BARGE, 1.6, Rotation2d.fromRadians(Math.PI));
    public static final Pose2d BARGE_RED_5_B =
            new Pose2d(BLUE_SCORING_LINE_BARGE, 1.9, Rotation2d.fromRadians(Math.PI));
    public static final Pose2d BARGE_RED_6_B =
            new Pose2d(BLUE_SCORING_LINE_BARGE, 2.2, Rotation2d.fromRadians(Math.PI));
    public static final Pose2d BARGE_RED_7_B =
            new Pose2d(BLUE_SCORING_LINE_BARGE, 2.5, Rotation2d.fromRadians(Math.PI));
    public static final Pose2d BARGE_RED_8_B =
            new Pose2d(BLUE_SCORING_LINE_BARGE, 2.8, Rotation2d.fromRadians(Math.PI));
    public static final Pose2d BARGE_RED_9_B =
            new Pose2d(BLUE_SCORING_LINE_BARGE, 3.1, Rotation2d.fromRadians(Math.PI));
    public static final Pose2d BARGE_RED_10_B =
            new Pose2d(BLUE_SCORING_LINE_BARGE, 3.4, Rotation2d.fromRadians(Math.PI));

    /** Starting from left to right from driverstation PoV */
    public static final Pose2d BARGE_BLUE_1 =
            new Pose2d(BLUE_SCORING_LINE_BARGE, 4.6, Rotation2d.fromRadians(Math.PI)); // 0.44

    public static final Pose2d BARGE_BLUE_2 = new Pose2d(BLUE_SCORING_LINE_BARGE, 4.9, Rotation2d.fromRadians(Math.PI));
    public static final Pose2d BARGE_BLUE_3 = new Pose2d(BLUE_SCORING_LINE_BARGE, 5.2, Rotation2d.fromRadians(Math.PI));
    public static final Pose2d BARGE_BLUE_4 = new Pose2d(BLUE_SCORING_LINE_BARGE, 5.5, Rotation2d.fromRadians(Math.PI));
    public static final Pose2d BARGE_BLUE_5 = new Pose2d(BLUE_SCORING_LINE_BARGE, 5.8, Rotation2d.fromRadians(Math.PI));
    public static final Pose2d BARGE_BLUE_6 = new Pose2d(BLUE_SCORING_LINE_BARGE, 6.1, Rotation2d.fromRadians(Math.PI));
    public static final Pose2d BARGE_BLUE_7 = new Pose2d(BLUE_SCORING_LINE_BARGE, 6.4, Rotation2d.fromRadians(Math.PI));
    public static final Pose2d BARGE_BLUE_8 = new Pose2d(BLUE_SCORING_LINE_BARGE, 6.7, Rotation2d.fromRadians(Math.PI));
    public static final Pose2d BARGE_BLUE_9 = new Pose2d(BLUE_SCORING_LINE_BARGE, 7, Rotation2d.fromRadians(Math.PI));
    public static final Pose2d BARGE_BLUE_10 =
            new Pose2d(BLUE_SCORING_LINE_BARGE, 7.3, Rotation2d.fromRadians(Math.PI));

    // scoring add red side
    public static final Pose2d BARGE_BLUE_1_R =
            new Pose2d(RED_SCORING_LINE_BARGE, 4.6, Rotation2d.fromRadians(0)); // 0.44
    public static final Pose2d BARGE_BLUE_2_R = new Pose2d(RED_SCORING_LINE_BARGE, 4.9, Rotation2d.fromRadians(0));
    public static final Pose2d BARGE_BLUE_3_R = new Pose2d(RED_SCORING_LINE_BARGE, 5.2, Rotation2d.fromRadians(0));
    public static final Pose2d BARGE_BLUE_4_R = new Pose2d(RED_SCORING_LINE_BARGE, 5.5, Rotation2d.fromRadians(0));
    public static final Pose2d BARGE_BLUE_5_R = new Pose2d(RED_SCORING_LINE_BARGE, 5.8, Rotation2d.fromRadians(0));
    public static final Pose2d BARGE_BLUE_6_R = new Pose2d(RED_SCORING_LINE_BARGE, 6.1, Rotation2d.fromRadians(0));
    public static final Pose2d BARGE_BLUE_7_R = new Pose2d(RED_SCORING_LINE_BARGE, 6.4, Rotation2d.fromRadians(0));
    public static final Pose2d BARGE_BLUE_8_R = new Pose2d(RED_SCORING_LINE_BARGE, 6.7, Rotation2d.fromRadians(0));
    public static final Pose2d BARGE_BLUE_9_R = new Pose2d(RED_SCORING_LINE_BARGE, 7, Rotation2d.fromRadians(0));
    public static final Pose2d BARGE_BLUE_10_R = new Pose2d(RED_SCORING_LINE_BARGE, 7.3, Rotation2d.fromRadians(0));

    public static final Pose2d[] BARGE_BLUE = {
        BARGE_BLUE_1,
        BARGE_BLUE_2,
        BARGE_BLUE_3,
        BARGE_BLUE_4,
        BARGE_BLUE_5,
        BARGE_BLUE_6,
        BARGE_BLUE_7,
        BARGE_BLUE_8,
        BARGE_BLUE_9,
        BARGE_BLUE_10,
        BARGE_BLUE_1_R,
        BARGE_BLUE_2_R,
        BARGE_BLUE_3_R,
        BARGE_BLUE_4_R,
        BARGE_BLUE_5_R,
        BARGE_BLUE_6_R,
        BARGE_BLUE_7_R,
        BARGE_BLUE_8_R,
        BARGE_BLUE_9_R,
        BARGE_BLUE_10_R
    };

    public static final Pose2d[] BARGE_RED = {
        BARGE_RED_1,
        BARGE_RED_2,
        BARGE_RED_3,
        BARGE_RED_4,
        BARGE_RED_5,
        BARGE_RED_6,
        BARGE_RED_7,
        BARGE_RED_8,
        BARGE_RED_9,
        BARGE_RED_10,
        BARGE_RED_1_B,
        BARGE_RED_2_B,
        BARGE_RED_3_B,
        BARGE_RED_4_B,
        BARGE_RED_5_B,
        BARGE_RED_6_B,
        BARGE_RED_7_B,
        BARGE_RED_8_B,
        BARGE_RED_9_B,
        BARGE_RED_10_B
    };
}
