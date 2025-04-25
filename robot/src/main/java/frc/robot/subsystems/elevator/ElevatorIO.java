/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** The interface for the elevator I/O. */
public interface ElevatorIO {

    /** Container that stores the inputs from the elevator hardware. */
    @AutoLog
    class ElevatorInputs {
        public double leaderVelocity;
        public double leaderTemperature;
        public double leaderPosition;
        public double leaderAppliedVoltage;
        public double leaderCurrentDraw;

        public double followerVelocity;
        public double followerTemperature;
        public double followerPosition;
        public double followerAppliedVoltage;
        public double followerCurrentDraw;

        public double absoluteEncoderPosition;

        public boolean topLimitSwitchTriggered;
        public boolean bottomLimitSwitchTriggered;
    }

    /**
     * Updates the inputs of the subsystem.
     *
     * @param inputs The inputs object that stores the subsystem info.
     */
    default void updateInputs(ElevatorInputs inputs) {}

    /**
     * Sets the position of the elevator in meters from the bottom position of the elevator.
     *
     * @param position The position of the elevator in meters from the bottom position of the elevator.
     */
    default void setTarget(double position) {}

    /**
     * Sets the velocity and position of the elevator in meters per second and meters from the bottom position of the
     * elevator.
     *
     * @param position The position of the elevator in meters from the bottom position of the elevator.
     * @param velocity The velocity of the elevator in meters per second.
     * @param acceleration the acceleration of the elevator in meters per second squared.
     */
    default void setTarget(double position, double velocity, double acceleration) {}

    /**
     * sets the power that will be applied to the motor
     *
     * @param power the power applied to the motor between -1 and 1.
     */
    default void setPower(double power) {}

    /**
     * Reset the relative encoder position to a new value
     *
     * @param newPosition the new position of the encoder
     */
    default void setRelativeEncoderPosition(double newPosition) {}
}
