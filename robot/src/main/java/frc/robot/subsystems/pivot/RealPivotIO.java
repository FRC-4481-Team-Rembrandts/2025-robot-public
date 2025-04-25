/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.pivot;

import static frc.robot.constants.PivotConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;

public class RealPivotIO implements PivotIO {
    private final SparkFlex motor;
    private final AbsoluteEncoder absoluteEncoder;
    private final RelativeEncoder relativeEncoder;
    private final SparkClosedLoopController controller;

    public RealPivotIO() {
        motor = new SparkFlex(CAN_ID, SparkLowLevel.MotorType.kBrushless);
        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig
                .encoder
                .positionConversionFactor(MOTOR_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(MOTOR_ENCODER_VELOCITY_FACTOR)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        motorConfig
                .closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
                .pidf(PID_GAINS.kP(), PID_GAINS.kI(), PID_GAINS.kD(), 0.0)
                .positionWrappingEnabled(false);
        motorConfig
                .inverted(MOTOR_INVERTED)
                .idleMode(IDLE_MODE)
                .smartCurrentLimit(CURRENT_LIMIT)
                .voltageCompensation(12.0);
        motorConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        motorConfig
                .absoluteEncoder
                .inverted(ABSOLUTE_ENCODER_INVERTED)
                .positionConversionFactor(ABSOLUTE_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(ABSOLUTE_ENCODER_VELOCITY_FACTOR)
                .zeroCentered(true)
                .averageDepth(2);
        motor.configure(
                motorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        absoluteEncoder = motor.getAbsoluteEncoder();
        relativeEncoder = motor.getEncoder();
        controller = motor.getClosedLoopController();
    }

    @Override
    public void updateInputs(PivotIO.PivotInputs inputs) {
        inputs.angle = Rotation2d.fromRadians(absoluteEncoder.getPosition());
        inputs.velocityRadPerSec = relativeEncoder.getVelocity();
        inputs.appliedVoltage = motor.getAppliedOutput() * 12;
        inputs.currentDraw = motor.getOutputCurrent();
        inputs.temperature = motor.getMotorTemperature();
        inputs.forwardLimitSwitchTriggered = motor.getForwardLimitSwitch().isPressed();
        inputs.reverseLimitSwitchTriggered = motor.getReverseLimitSwitch().isPressed();
    }

    @Override
    public void setTarget(Rotation2d angle) {
        setTarget(angle, 0, 0);
    }

    @Override
    public void setTarget(Rotation2d angle, double angularVelocity, double angularAcceleration) {
        Rotation2d error = angle.minus(Rotation2d.fromRadians(absoluteEncoder.getPosition()));
        controller.setReference(
                angle.getRadians(),
                SparkBase.ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                calculateFeedforward(angle, angularVelocity, angularAcceleration, error));
    }

    @Override
    public void setPower(double power) {
        motor.set(power);
    }

    private double calculateFeedforward(
            Rotation2d angle, double angularVelocity, double angularAcceleration, Rotation2d error) {
        return KG * angle.getCos()
                + KV * angularVelocity
                + KA * angularAcceleration
                + Math.tanh(150 * error.getRadians()) * KS;
    }
}
