/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.elevator;

import static frc.robot.constants.ElevatorConstants.GROUND_TO_PIVOT_AXLE_DISTANCE_LOWEST_POSITION;

import com.teamrembrandts.simulation.SimpleMotorSim;
import edu.wpi.first.units.Units;
import frc.robot.constants.ElevatorConstants;

public class SimulatedElevatorIO implements ElevatorIO {
    private final SimpleMotorSim vortexSim;

    public SimulatedElevatorIO() {
        vortexSim = new SimpleMotorSim(
                Units.Second.of(0.5),
                1 / ElevatorConstants.MOTOR_ENCODER_POSITION_FACTOR,
                1 / ElevatorConstants.MOTOR_ENCODER_VELOCITY_FACTOR,
                Units.RPM.of(6000));
    }

    @Override
    public void updateInputs(ElevatorIO.ElevatorInputs inputs) {
        inputs.leaderPosition = vortexSim.getCurrentPosition() + GROUND_TO_PIVOT_AXLE_DISTANCE_LOWEST_POSITION;
        inputs.leaderVelocity = vortexSim.getCurrentVelocity() + GROUND_TO_PIVOT_AXLE_DISTANCE_LOWEST_POSITION;

        inputs.followerPosition = vortexSim.getCurrentPosition() + GROUND_TO_PIVOT_AXLE_DISTANCE_LOWEST_POSITION;
        inputs.followerVelocity = vortexSim.getCurrentVelocity() + GROUND_TO_PIVOT_AXLE_DISTANCE_LOWEST_POSITION;

        inputs.bottomLimitSwitchTriggered = inputs.leaderPosition <= GROUND_TO_PIVOT_AXLE_DISTANCE_LOWEST_POSITION;

        inputs.absoluteEncoderPosition = inputs.leaderPosition;
    }

    @Override
    public void setTarget(double position) {
        setTarget(position, 0, 0);
    }

    @Override
    public void setTarget(double position, double velocity, double acceleration) {
        vortexSim.setTargetPosition(position - GROUND_TO_PIVOT_AXLE_DISTANCE_LOWEST_POSITION);
    }

    @Override
    public void setPower(double power) {
        vortexSim.setTargetPower(power);
    }

    @Override
    public void setRelativeEncoderPosition(double newPosition) {}
}
