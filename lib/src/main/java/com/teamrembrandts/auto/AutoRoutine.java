/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.auto;

import edu.wpi.first.wpilibj.DriverStation;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.Target;

/** Indicates that a method is an autonomous routine that can be selected from the dashboard. */
@Retention(java.lang.annotation.RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
public @interface AutoRoutine {
    DriverStation.Alliance[] alliance() default {DriverStation.Alliance.Red, DriverStation.Alliance.Blue};

    String name();
}
