/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandUtils {

    /**
     * Returns a trigger that is the exclusive or of two triggers.
     *
     * @param a The first trigger
     * @param b The second trigger
     * @return The exclusive or of the two triggers
     */
    public static Trigger xor(Trigger a, Trigger b) {
        return (a.and(b.negate().debounce(0.1))).or(a.negate().debounce(0.1).and(b));
    }

    /**
     * Returns a trigger that is the exclusive or of two triggers.
     *
     * @param a The first trigger
     * @param b The second trigger
     * @param debounceTime The debounce time for the triggers
     * @return The exclusive or of the two triggers
     */
    public static Trigger xor(Trigger a, Trigger b, double debounceTime) {
        return (a.and(b.negate().debounce(debounceTime)))
                .or(a.negate().debounce(debounceTime).and(b));
    }
}
