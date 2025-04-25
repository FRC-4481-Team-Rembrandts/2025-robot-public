/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.util;

import org.littletonrobotics.junction.Logger;

public class TimeUtil {
    private static double previousTime = 0;
    private static double deltaTime = 0;

    /** calculate the deltaTime since the last call to this method. */
    public static void updateDeltaTime() {
        double currentTime = Logger.getTimestamp() / 1e6;
        deltaTime = currentTime - previousTime;
        previousTime = currentTime;
    }

    /**
     * Get the time since the last call to this method.
     *
     * @return The time since the last call to this method in seconds.
     */
    public static double getDeltaTime() {
        return deltaTime;
    }
}
