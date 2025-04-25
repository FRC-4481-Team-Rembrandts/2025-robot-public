/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.feedthroughgripper;

import static frc.robot.constants.FeedThroughGripperConstants.*;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

public class RealFeedThroughGripperIO implements FeedThroughGripperIO {
    private final SparkFlex motor;
    private final SparkFlexConfig motorConfig;

    public RealFeedThroughGripperIO() {
        motor = new SparkFlex(CORAL_GRIPPER_CAN_ID, SparkLowLevel.MotorType.kBrushless);

        motorConfig = new SparkFlexConfig();
        motorConfig
                .inverted(MOTOR_INVERTED)
                .idleMode(IDLE_MODE)
                .smartCurrentLimit(CURRENT_LIMIT)
                .voltageCompensation(12.0);
        motorConfig
                .limitSwitch
                .forwardLimitSwitchEnabled(false)
                .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);

        motor.configure(
                motorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(FeedThroughGripperInputs inputs) {
        inputs.position = motor.getEncoder().getPosition();
        inputs.velocity = motor.getEncoder().getVelocity();
        inputs.appliedVoltage = motor.getAppliedOutput() * 12;
        inputs.currentDraw = motor.getOutputCurrent();
        inputs.temperature = motor.getMotorTemperature();

        inputs.forwardLimitSwitchIsPressed = motor.getForwardLimitSwitch().isPressed();
    }

    @Override
    public void setPower(double Power) {
        motor.set(Power);
    }

    @Override
    public void enableLimitSwitch(boolean enableLimitSwitch) {
        motorConfig.limitSwitch.forwardLimitSwitchEnabled(enableLimitSwitch);
        motor.configureAsync(
                motorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }
}
