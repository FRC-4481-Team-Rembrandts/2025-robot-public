/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.pivot;

import static frc.robot.constants.PivotConstants.*;

import com.teamrembrandts.simulation.SimpleMotorSim;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;

public class SimulatedPivotIO implements PivotIO {
    private final SimpleMotorSim motorSim;

    public SimulatedPivotIO() {
        motorSim = new SimpleMotorSim(
                Units.Second.of(0.5),
                1 / MOTOR_ENCODER_POSITION_FACTOR,
                1 / MOTOR_ENCODER_VELOCITY_FACTOR,
                Units.RPM.of(6000));
    }

    @Override
    public void updateInputs(PivotInputs inputs) {
        inputs.angle = Rotation2d.fromRadians(motorSim.getCurrentPosition());
        inputs.velocityRadPerSec = motorSim.getCurrentVelocity();
    }

    @Override
    public void setTarget(Rotation2d angle) {
        motorSim.setTargetPosition(angle.getRadians());
    }

    @Override
    public void setPower(double power) {
        motorSim.setTargetPower(power);
    }

    @Override
    public void setTarget(Rotation2d angle, double angularVelocity, double angularAcceleration) {
        motorSim.setTargetPosition(angle.getRadians());
    }
}
