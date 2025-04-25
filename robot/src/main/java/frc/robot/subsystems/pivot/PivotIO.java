/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

    @AutoLog
    class PivotInputs {
        public Rotation2d angle = new Rotation2d();
        public double velocityRadPerSec;
        public double appliedVoltage;
        public double currentDraw;
        public double temperature;
        public boolean forwardLimitSwitchTriggered;
        public boolean reverseLimitSwitchTriggered;
    }

    /**
     * Updates the inputs of the pivot.
     *
     * @param inputs The new inputs.
     */
    default void updateInputs(PivotInputs inputs) {}

    /**
     * Sets the angles for the pivot.
     *
     * @param angle The angle setpoint.
     */
    default void setTarget(Rotation2d angle) {}

    /**
     * sets the power that will be applied to the motor
     *
     * @param power the power applied to the motor between -1 and 1.
     */
    default void setPower(double power) {}

    /**
     * Sets the angle and velocity of the pivot in radians per second.
     *
     * @param angle The angle setpoint.
     * @param angularVelocity The velocity of the pivot in radians per second.
     * @param angularAcceleration the acceleration of the pivot in radians per seconds^2.
     */
    default void setTarget(Rotation2d angle, double angularVelocity, double angularAcceleration) {}
}
