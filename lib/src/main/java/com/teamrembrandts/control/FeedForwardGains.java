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
 * Container class for FeedForward gains with separate kA for accelerating and decelerating.
 *
 * @param kS Static gain.
 * @param kV Velocity gain.
 * @param kAAcceleration Acceleration gain when magnitude of velocity is increasing.
 * @param kADeceleration Acceleration gain when magnitude of velocity is decreasing.
 */
public record FeedForwardGains(double kS, double kV, double kAAcceleration, double kADeceleration) {

    /** Creates an empty {@code FeedForwardGains} object without gains. */
    public FeedForwardGains() {
        this(0, 0, 0, 0);
    }

    /**
     * Create a new {@code FeedForwardGains} object with the same acceleration gain for both acceleration and
     * deceleration.
     */
    public FeedForwardGains(double kS, double kV, double kA) {
        this(kS, kV, kA, 0);
    }
}
