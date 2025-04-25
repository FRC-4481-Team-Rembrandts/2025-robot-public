/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.LEDPattern;
import org.littletonrobotics.junction.AutoLog;

/** Interface for the LED controller subsystem. */
public interface LEDControllerIO {

    /** Class that stores the inputs of the LED controller subsystem. */
    @AutoLog
    class LEDControllerInputs {
        public int[][] topColors;
        public int[][] bottomColors;
    }

    /**
     * Updates the inputs of the subsystem.
     *
     * @param inputs The inputs object that stores the subsystem info.
     */
    default void updateInputs(LEDControllerInputs inputs) {}

    /**
     * Sets the pattern of the bottom LED strip on the robot.
     *
     * @param pattern The input pattern of the LED strip.
     */
    default void setPatternBottom(LEDPattern pattern) {}

    /**
     * Sets the pattern of the top LED strip on the robot.
     *
     * @param pattern The input pattern of the LED strip.
     */
    default void setPatternTop(LEDPattern pattern) {}

    /**
     * Sets the pattern of the LED strip on the robot.
     *
     * @param pattern The input pattern of the LED strip.
     */
    default void setPatternWhole(LEDPattern pattern) {}
}
