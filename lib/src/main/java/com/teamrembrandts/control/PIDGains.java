/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.control;

/**
 * Container class for Proportional, Integral, and Derivative gains.
 *
 * @param kP Proportional gain.
 * @param kI Integral gain.
 * @param kD Derivative gain.
 */
public record PIDGains(double kP, double kI, double kD) {

    /** Creates an empty {@code PIDGains} object without gains. */
    public PIDGains() {
        this(0, 0, 0);
    }
}
