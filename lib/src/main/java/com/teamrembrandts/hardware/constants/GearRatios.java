/*
 * Copyright (c) 2024-2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.hardware.constants;

/**
 * The GearRatios class provides easy access to gear ratios of common COTS gearboxes. Some vendors such as REV Robotics
 * sell gearboxes with multiple reduction options. The ratios posted on these products might differ from the actual
 * ratios. This class provides the actual ratios for these gearboxes.
 */
public final class GearRatios {

    /** The RevRobotics class provides the actual reductions for gearboxes and swerve modules sold by REV Robotics. */
    public static class RevRobotics {

        /**
         * The UltraPlanetary class provides the actual reductions for the REV Robotics UltraPlanetary system.
         * UltraPlanetary gearboxes come with 3, 4, and 5 to 1 reduction cartridges. The UltraPlanetary system is
         * compatible with the NEO 550 brushless motor.
         */
        public static class UltraPlanetary {

            /** The actual gear reduction for the REV UltraPlanetary 3:1 cartridge. */
            public static final double REDUCTION_3_TO_1_CARDRIDGE = 2.89;

            /** The actual gear reduction for the REV UltraPlanetary 4:1 cartridge. */
            public static final double REDUCTION_4_TO_1_CARDRIDGE = 3.61;

            /** The actual gear reduction for the REV UltraPlanetary 5:1 cartridge. */
            public static final double REDUCTION_5_TO_1_CARDRIDGE = 5.23;
        }

        /**
         * The MaxPlanetary class provides the actual reductions for the REV Robotics MAXPlanetary system. MAXPlanetary
         * gearboxes come with 3, 4, 5 and 9 to 1 reduction cartridges. The MAXPlanetary system is compatible with the
         * NEO and NEO Vortex, Falcon 500 and Kraken X60 brushless motors. Unlike the UltraPlanetary system, the
         * MAXPlanetary system has exact gear ratios as advertised. Therefore, this class only exists for clarity
         * purposes.
         */
        public static class MaxPlanetary {

            /** The exact gear reduction for the REV MAXPlanetary 3:1 cartridge. */
            public static final double REDUCTION_3_TO_1_CARDRIDGE = 3.0;

            /** The exact gear reduction for the REV MAXPlanetary 4:1 cartridge. */
            public static final double REDUCTION_4_TO_1_CARDRIDGE = 4.0;

            /** The exact gear reduction for the REV MAXPlanetary 5:1 cartridge. */
            public static final double REDUCTION_5_TO_1_CARDRIDGE = 5.0;

            /** The exact gear reduction for the REV MAXPlanetary 9:1 cartridge. */
            public static final double REDUCTION_9_TO_1_CARDRIDGE = 9.0;
        }

        /** The MaxSwerve class provides the actual reductions for the REV Robotics MAXSwerve system. */
        public static class MaxSwerve {

            /** The exact gear reduction for the MAXSwerve 'Low' speed option. */
            public static final double REDUCTION_LOW_SPEED = 5.50;

            /** The exact gear reduction for the MAXSwerve 'Medium' speed option. */
            public static final double REDUCTION_MEDIUM_SPEED = 5.08;

            /** The exact gear reduction for the MAXSwerve 'High' speed option. */
            public static final double REDUCTION_HIGH_SPEED = 4.71;

            /** The exact gear reduction for the MAXSwerve 'Extra High 1' speed option. */
            public static final double REDUCTION_EXTRA_HIGH_SPEED_1 = 4.50;

            /** The exact gear reduction for the MAXSwerve 'Extra High 2' speed option. */
            public static final double REDUCTION_EXTRA_HIGH_SPEED_2 = 4.29;

            /** The exact gear reduction for the MAXSwerve 'Extra High 3' speed option. */
            public static final double REDUCTION_EXTRA_HIGH_SPEED_3 = 4.00;

            /** The exact gear reduction for the MAXSwerve 'Extra High 4' speed option. */
            public static final double REDUCTION_EXTRA_HIGH_SPEED_4 = 3.75;

            /** The exact gear reduction for the MAXSwerve 'Extra High 5' speed option. */
            public static final double REDUCTION_EXTRA_HIGH_SPEED_5 = 3.56;

            /**
             * The exact gear reduction of the NEO 550 turn motor on the MAXSwerve. The turn rotation of the wheel is
             * also called azimuth.
             */
            public static final double REDUCTION_AZIMUTH = 9424.0 / 203;
        }
    }

    /** The SwerveDriveSpecialties class provides the gear reductions for swerve modules sold by SDS. */
    public static class SwerveDriveSpecialties {

        /** The Mk4 class provides the gear reductions for the SDS Mk4 swerve module. */
        public static class Mk4 {

            /** The exact gear reduction for the SDS Mk4 'L1' speed option. */
            public static final double REDUCTION_L1 = 8.14;

            /** The exact gear reduction for the SDS Mk4 'L2' speed option. */
            public static final double REDUCTION_L2 = 6.75;

            /** The exact gear reduction for the SDS Mk4 'L3' speed option. */
            public static final double REDUCTION_L3 = 6.12;

            /** The exact gear reduction for the SDS Mk4 'L4' speed option. */
            public static final double REDUCTION_L4 = 5.14;

            /** The exact gear reduction for the steering motor in an SDS Mk4 module. */
            public static final double STEER_REDUCTION = 12.8;
        }

        /** The Mk4i class provides the gear reductions for the SDS Mk4i swerve module. */
        public static class Mk4i {

            /** The exact gear reduction for the SDS Mk4i 'L1' speed option. */
            public static final double REDUCTION_L1 = 8.14;

            /** The exact gear reduction for the SDS Mk4i 'L2' speed option. */
            public static final double REDUCTION_L2 = 6.75;

            /** The exact gear reduction for the SDS Mk4i 'L3' speed option. */
            public static final double REDUCTION_L3 = 6.12;

            /** The exact gear reduction for the steering motor in an SDS Mk4i module. */
            public static final double STEER_REDUCTION = 150 / 7.0;
        }

        /** The Mk4n class provides the gear reductions for the SDS Mk4n swerve module. */
        public static class Mk4n {

            /** The exact gear reduction for the SDS Mk4n 'L1' speed option. */
            public static final double REDUCTION_L1 = 7.13;

            /** The exact gear reduction for the SDS Mk4n 'L2' speed option. */
            public static final double REDUCTION_L2 = 5.9;

            /** The exact gear reduction for the SDS Mk4n 'L3' speed option. */
            public static final double REDUCTION_L3 = 5.36;

            /** The exact gear reduction for the steering motor in an SDS Mk4n module. */
            public static final double STEER_REDUCTION = 18.75;
        }
    }
}
