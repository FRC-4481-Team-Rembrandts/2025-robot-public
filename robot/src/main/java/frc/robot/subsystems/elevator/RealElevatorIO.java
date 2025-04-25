/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.elevator;

import static frc.robot.constants.ElevatorConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkFlexConfig;

/** Represents the physical elevator hardware on a robot. */
public class RealElevatorIO implements ElevatorIO {
    private final SparkFlex leaderMotor;
    private final SparkFlex followerMotor;
    private final RelativeEncoder leaderEncoder;
    private final RelativeEncoder followerEncoder;
    private final SparkClosedLoopController closedLoopController;
    private final AbsoluteEncoder absoluteEncoder;

    public static final SparkFlexConfig MOTOR_CONFIG = new SparkFlexConfig();
    public static final SparkFlexConfig MOTOR_CONFIG_LEADER = new SparkFlexConfig();
    public static final SparkFlexConfig MOTOR_CONFIG_FOLLOWER = new SparkFlexConfig();

    /** Creates a new Elevator I/O that represents a physical elevator on a robot. */
    public RealElevatorIO() {
        MOTOR_CONFIG
                .encoder
                .positionConversionFactor(MOTOR_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(MOTOR_ENCODER_VELOCITY_FACTOR)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        MOTOR_CONFIG.closedLoop.pidf(PID_GAINS.kP(), PID_GAINS.kI(), PID_GAINS.kD(), 0.0);
        MOTOR_CONFIG.idleMode(IDLE_MODE).smartCurrentLimit(CURRENT_LIMIT).voltageCompensation(12.0);
        MOTOR_CONFIG
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        MOTOR_CONFIG
                .absoluteEncoder
                .inverted(ABSOLUTE_ENCODER_INVERTED)
                .positionConversionFactor(ABSOLUTE_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(ABSOLUTE_ENCODER_VELOCITY_FACTOR)
                .zeroCentered(true)
                .averageDepth(2);

        MOTOR_CONFIG_LEADER.apply(MOTOR_CONFIG);
        MOTOR_CONFIG_LEADER.inverted(LEADER_MOTOR_INVERTED);

        leaderMotor = new SparkFlex(CAN_ID_RIGHT, SparkLowLevel.MotorType.kBrushless);
        leaderEncoder = leaderMotor.getEncoder();
        leaderMotor.configure(
                MOTOR_CONFIG_LEADER,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        MOTOR_CONFIG_FOLLOWER.apply(MOTOR_CONFIG);
        MOTOR_CONFIG_FOLLOWER.follow(leaderMotor, FOLLOWER_MOTOR_INVERTED);

        followerMotor = new SparkFlex(CAN_ID_LEFT, SparkLowLevel.MotorType.kBrushless);
        followerEncoder = followerMotor.getEncoder();
        followerMotor.configure(
                MOTOR_CONFIG_FOLLOWER,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        // Reset the encoder to the lowest position as an estimate of where they will be
        // Calibration is in place to reset them to actual position
        setRelativeEncoderPosition(GROUND_TO_PIVOT_AXLE_DISTANCE_LOWEST_POSITION);

        closedLoopController = leaderMotor.getClosedLoopController();
        absoluteEncoder = leaderMotor.getAbsoluteEncoder();
    }

    @Override
    public void updateInputs(ElevatorIO.ElevatorInputs inputs) {
        inputs.leaderPosition = leaderEncoder.getPosition();
        inputs.leaderVelocity = leaderEncoder.getVelocity();
        inputs.leaderAppliedVoltage = leaderMotor.getAppliedOutput() * 12;
        inputs.leaderCurrentDraw = leaderMotor.getOutputCurrent();
        inputs.leaderTemperature = leaderMotor.getMotorTemperature();

        inputs.followerPosition = followerEncoder.getPosition();
        inputs.followerVelocity = followerEncoder.getVelocity();
        inputs.followerAppliedVoltage = followerMotor.getAppliedOutput() * 12;
        inputs.followerCurrentDraw = followerMotor.getOutputCurrent();
        inputs.followerTemperature = followerMotor.getMotorTemperature();

        inputs.absoluteEncoderPosition = absoluteEncoder.getPosition() + GROUND_TO_PIVOT_AXLE_DISTANCE_LOWEST_POSITION;

        inputs.topLimitSwitchTriggered = leaderMotor.getForwardLimitSwitch().isPressed();
        inputs.bottomLimitSwitchTriggered = leaderMotor.getReverseLimitSwitch().isPressed();
    }

    @Override
    public void setTarget(double position) {
        setTarget(position, 0, 0);
    }

    @Override
    public void setTarget(double position, double velocity, double acceleration) {
        double error = position - leaderEncoder.getPosition();
        closedLoopController.setReference(
                position,
                SparkBase.ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                calculateFeedforward(velocity, acceleration, error));
    }

    private double calculateFeedforward(double velocity, double acceleration, double error) {
        return KG + KV * velocity + KA * acceleration + Math.tanh(150 * error) * KS;
    }

    @Override
    public void setPower(double power) {
        leaderMotor.set(power);
    }

    @Override
    public void setRelativeEncoderPosition(double newPosition) {
        leaderEncoder.setPosition(newPosition);
        followerEncoder.setPosition(newPosition);
    }
}
