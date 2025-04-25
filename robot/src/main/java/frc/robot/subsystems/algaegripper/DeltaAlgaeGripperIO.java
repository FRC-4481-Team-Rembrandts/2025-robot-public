/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.algaegripper;

import static frc.robot.constants.AlgaeGripperConstants.*;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;

public class DeltaAlgaeGripperIO implements AlgaeGripperIO {
    private final SparkFlex motor;

    public DeltaAlgaeGripperIO() {
        motor = new SparkFlex(ALGAE_GRIPPER_CAN_ID, SparkLowLevel.MotorType.kBrushless);

        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig
                .inverted(MOTOR_INVERTED)
                .idleMode(IDLE_MODE)
                .smartCurrentLimit(CURRENT_LIMIT)
                .voltageCompensation(12.0);

        motor.configure(
                motorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(AlgaeGripperInputs inputs) {
        inputs.position = motor.getEncoder().getPosition();
        inputs.velocity = motor.getEncoder().getVelocity();
        inputs.appliedVoltage = motor.getAppliedOutput() * 12;
        inputs.currentDraw = motor.getOutputCurrent();
        inputs.temperature = motor.getMotorTemperature();

        inputs.sensorConnected = true;

        try {
            inputs.sensorValue = motor.getAnalog().getVoltage();
        } catch (IllegalStateException e) {
            inputs.sensorConnected = false;
            inputs.sensorValue = 0;
        }
    }

    @Override
    public void setPower(double Power) {
        motor.set(Power);
    }
}
