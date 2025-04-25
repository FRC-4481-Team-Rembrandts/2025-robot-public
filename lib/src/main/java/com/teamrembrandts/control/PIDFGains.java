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
 * Container class for Proportional, Integral, Derivative and FeedForward gains.
 *
 * @param pid Proportional, Integral, and Derivative gains.
 * @param ff FeedForward gains.
 */
public record PIDFGains(PIDGains pid, FeedForwardGains ff) {

    /** Creates an empty {@code PIDFGains} object without gains. */
    public PIDFGains() {
        this(new PIDGains(), new FeedForwardGains());
    }
}
