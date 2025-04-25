/*
 * Copyright (c) 2024-2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** A utility class for selecting autonomous routines from the dashboard. */
public class AutoSelector {

    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Routine");
    /**
     * Creates a new AutoSelector.
     *
     * @param autoRoutineClass The class containing the autonomous routines.
     */
    public AutoSelector(Class autoRoutineClass) {
        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

        for (AutoOption option : fetchAutoOptions(autoRoutineClass)) {
            if (option.autoRoutine.getInterruptionBehavior() != Command.InterruptionBehavior.kCancelIncoming) {
                System.err.println("Error: Auto routine " + option.name
                        + " has an invalid interruption behavior. It should be kCancelIncoming.");
            }
            autoChooser.addOption(option.name(), option.autoRoutine());
        }
    }

    /**
     * Gets the selected autonomous routine.
     *
     * @return The selected autonomous routine.
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    /**
     * Fetches the autonomous routine options from the given class.
     *
     * @param autoRoutineClass The class containing the autonomous routines.
     * @return The autonomous routine options.
     */
    private AutoOption[] fetchAutoOptions(Class autoRoutineClass) {
        List<AutoOption> options = new ArrayList<>();

        for (Method method : autoRoutineClass.getDeclaredMethods()) {
            if (method.isAnnotationPresent(AutoRoutine.class) && !method.isAnnotationPresent(Disabled.class)) {
                if (doesAllianceMatch(
                        method.getAnnotation(AutoRoutine.class).alliance(), DriverStation.getAlliance())) {
                    try {
                        options.add(new AutoOption(
                                method.getAnnotation(AutoRoutine.class).name(), (Command) method.invoke(null)));
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            }
        }

        return options.toArray(new AutoOption[0]);
    }

    /**
     * Checks if the current alliance is in the provided list of alliances
     *
     * @param allianceList list of alliances
     * @param currentAlliance alliance to check for
     * @return whether the alliance is in the list
     */
    private boolean doesAllianceMatch(
            DriverStation.Alliance[] allianceList, Optional<DriverStation.Alliance> currentAlliance) {
        // When the current alliance list is empty, return true as failsafe
        return currentAlliance.isEmpty() || List.of(allianceList).contains(currentAlliance.get());
    }

    /**
     * A record representing an autonomous routine option.
     *
     * @param name The name of the autonomous routine.
     * @param autoRoutine The autonomous routine command.
     */
    record AutoOption(String name, Command autoRoutine) {}
}
