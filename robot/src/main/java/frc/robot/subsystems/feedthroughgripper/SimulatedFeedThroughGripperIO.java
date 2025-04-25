/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.feedthroughgripper;

import com.teamrembrandts.simulation.SimpleMotorSim;
import edu.wpi.first.units.Units;

public class SimulatedFeedThroughGripperIO implements FeedThroughGripperIO {
    private final SimpleMotorSim motorSim;

    public SimulatedFeedThroughGripperIO() {
        motorSim = new SimpleMotorSim(Units.Second.of(0.5), 1, 1, Units.RPM.of(6000));
    }

    /**
     * Updates the inputs of the subsystem
     *
     * @param inputs The inputs object that stores the subsystem info.
     */
    @Override
    public void updateInputs(FeedThroughGripperInputs inputs) {
        inputs.velocity = motorSim.getCurrentVelocity();
        inputs.position = motorSim.getCurrentPosition();
        inputs.forwardLimitSwitchIsPressed = false;
    }

    /**
     * sets the power that will be applied to the motor
     *
     * @param power the power applied to the motor between -1 and 1.
     */
    @Override
    public void setPower(double power) {
        motorSim.setTargetPower(power);
    }
}
