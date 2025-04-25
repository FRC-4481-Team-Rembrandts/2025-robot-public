/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

    @AutoLog
    class ClimberInputs {
        public Rotation2d position = new Rotation2d();
        public double velocity;
        public double temperature;
        public double appliedVoltage;
        public double currentDraw;
    }

    default void setAngle(Rotation2d angle) {}

    default void updateInputs(ClimberInputs inputs) {}

    default void setPower(double power) {}

    default void setCurrentLimit(int limit) {}
}
