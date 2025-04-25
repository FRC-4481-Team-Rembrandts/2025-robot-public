/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.climber;

import static frc.robot.constants.ClimberConstants.*;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;

public class RealClimberIO implements ClimberIO {
    private final SparkFlex climberMotor;
    private final SparkFlexConfig config;

    public RealClimberIO() {
        climberMotor = new SparkFlex(CLIMBER_CAN_ID, SparkLowLevel.MotorType.kBrushless);

        config = new SparkFlexConfig();
        config.inverted(INVERTED)
                .idleMode(IDLE_MODE)
                .smartCurrentLimit(FOLD_CURRENT_LIMIT)
                .voltageCompensation(12.0);
        config.closedLoop
                .pidf(PID_GAINS.kP(), PID_GAINS.kI(), PID_GAINS.kD(), 0.0)
                .positionWrappingEnabled(false);
        config.encoder
                .positionConversionFactor(MOTOR_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(MOTOR_ENCODER_VELOCITY_FACTOR);

        climberMotor.configure(
                config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        climberMotor.getClosedLoopController().setReference(angle.getRadians(), SparkBase.ControlType.kPosition);
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.position = Rotation2d.fromRadians(climberMotor.getEncoder().getPosition());
        inputs.appliedVoltage = climberMotor.getAppliedOutput() * 12;
        inputs.velocity = climberMotor.getEncoder().getVelocity();
        inputs.currentDraw = climberMotor.getOutputCurrent();
        inputs.temperature = climberMotor.getMotorTemperature();
    }

    @Override
    public void setPower(double Power) {
        climberMotor.set(Power);
    }

    public void setCurrentLimit(int currentLimit) {
        config.smartCurrentLimit(currentLimit);
        climberMotor.configureAsync(
                config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }
}
