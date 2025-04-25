/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.algaegripper;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeGripperIO {

    @AutoLog
    class AlgaeGripperInputs {
        public double velocity;
        public double temperature;
        public double position;
        public double appliedVoltage;
        public double currentDraw;
        public double sensorValue;
        public boolean sensorConnected;
    }

    /**
     * Updates the inputs of the subsystem
     *
     * @param inputs The inputs object that stores the subsystem info.
     */
    default void updateInputs(AlgaeGripperInputs inputs) {}

    /**
     * sets the power that will be applied to the motor
     *
     * @param power the power applied to the motor between -1 and 1.
     */
    default void setPower(double power) {}
}
