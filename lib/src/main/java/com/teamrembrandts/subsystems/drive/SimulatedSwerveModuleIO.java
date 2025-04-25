/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.subsystems.drive;

import com.teamrembrandts.hardware.constants.GearRatios;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

/**
 * A simulated swerve module for use in physics simulation. The model simulates the physical properties of a NEO Vortex
 * for the drive motor and those of a NEO 550 for the turn motor.
 *
 * <p>This class is not meant to accurately represent a real swerve module. Yet it should be a good enough approximation
 * for testing purposes.
 *
 * <p>The setpoints for the simulated module are derived using computed torque control.
 *
 * @see <a href="https://en.wikipedia.org/wiki/Computed_torque_control">Computed Torque Control</a>
 */
public class SimulatedSwerveModuleIO implements SwerveModuleIO {

    private static final double LOOP_DELTA_TIME = 0.02;
    private static final double WHEEL_RADIUS = Units.Meter.convertFrom(1.5, Units.Inch);

    private static final double DRIVE_INERTIA = 0.15;
    private static final double TURN_INERTIA = 0.04;

    private static final double DRIVE_RATIO = GearRatios.RevRobotics.MaxSwerve.REDUCTION_HIGH_SPEED;
    private static final double TURN_RATIO = GearRatios.RevRobotics.MaxSwerve.REDUCTION_AZIMUTH;

    private static final double DRIVE_NAT_FREQUENCY = 3;
    private static final double TURN_NAT_FREQUENCY = 15;

    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;
    private final DCMotor driveMotor = DCMotor.getNeoVortex(1);
    private final DCMotor turnMotor = DCMotor.getNeo550(1);

    double driveDesiredVelocity = 0; // Desired velocity in rad/s of the wheel
    double turnDesiredPosition = 0; // Desired rotation of the wheel in rad

    /** Creates a new {@code SimulatedSwerveModuleIO}. */
    public SimulatedSwerveModuleIO() {
        // The inertia that DCMotorSim takes as input is most likely the inertia felt by the motor
        // Therefore the gear ratio should be taken into account
        driveSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(driveMotor, DRIVE_INERTIA / DRIVE_RATIO, DRIVE_RATIO), driveMotor);
        turnSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(turnMotor, TURN_INERTIA / TURN_RATIO, TURN_RATIO), turnMotor);

        // Random offset when the robot starts to make the simulation more interesting
        Rotation2d arbitraryTurnOffset = new Rotation2d(Math.random() * 2 * Math.PI);
        turnSim.setState(arbitraryTurnOffset.getRadians(), 0);
    }

    /**
     * Updates the system with new inputs from the module.
     *
     * @param inputs The new inputs.
     */
    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        driveSim.update(LOOP_DELTA_TIME);
        turnSim.update(LOOP_DELTA_TIME);

        // Determine the voltage that needs to be sent to the sim objects using Inverse Dynamics,
        // also known as computed torque control.
        // The desired acceleration is determined based on the error in velocity and position
        // This acceleration is then used to compute the voltage that needs to be sent to the motors
        double driveAppliedVoltage = calculateDriveVoltage();
        double turnAppliedVoltage = calculateTurnVoltage();

        inputs.drivePosition = driveSim.getAngularPositionRad() * WHEEL_RADIUS;
        inputs.driveVelocity = driveSim.getAngularVelocityRadPerSec() * WHEEL_RADIUS;
        inputs.driveAppliedVoltage = driveAppliedVoltage;
        inputs.driveCurrent = driveSim.getCurrentDrawAmps();
        inputs.driveTemperature = 0;

        inputs.turnAbsoluteRotation = new Rotation2d(MathUtil.angleModulus(turnSim.getAngularPositionRad()));
        inputs.turnAngularVelocity = turnSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVoltage = turnAppliedVoltage;
        inputs.turnCurrent = turnSim.getCurrentDrawAmps();
        inputs.turnTemperature = 0;

        // No high speed odometry thread is running in the sim, so just fill the arrays with the
        // current value
        inputs.odometryTimestamps = new double[] {Logger.getTimestamp() / 1e6};
        inputs.odometryDrivePositions = new double[] {inputs.drivePosition};
        inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnAbsoluteRotation};

        driveSim.setInputVoltage(driveAppliedVoltage);
        turnSim.setInputVoltage(turnAppliedVoltage);

        if (DriverStation.isDisabled()) {
            setDriveState(0, 0);
            setTurnState(0, 0);
        }
    }

    /** {@inheritDoc} */
    @Override
    public void setDriveState(double velocity, double acceleration) {
        driveDesiredVelocity = velocity / WHEEL_RADIUS; // Convert m/s to rad/s
    }

    /** {@inheritDoc} */
    @Override
    public void setTurnState(double rotation, double angularVelocity) {
        turnDesiredPosition = rotation;
    }

    /**
     * Calculates the voltage to be applied to the drive motor.
     *
     * @return The voltage to be applied to the drive motor.
     */
    private double calculateDriveVoltage() {
        double driveAccel = 2 * DRIVE_NAT_FREQUENCY * (driveDesiredVelocity - driveSim.getAngularVelocityRadPerSec());

        double driveAppliedVoltage = driveMotor.rOhms
                / (DRIVE_RATIO * driveMotor.KtNMPerAmp)
                * (DRIVE_INERTIA * driveAccel
                        + (DRIVE_RATIO * DRIVE_RATIO * driveMotor.KtNMPerAmp)
                                / (driveMotor.rOhms * driveMotor.KvRadPerSecPerVolt)
                                * driveSim.getAngularVelocityRadPerSec());

        driveAppliedVoltage = MathUtil.clamp(
                driveAppliedVoltage, -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());

        return driveAppliedVoltage;
    }

    /**
     * Calculates the voltage to be applied to the turn motor.
     *
     * @return The voltage to be applied to the turn motor.
     */
    private double calculateTurnVoltage() {
        double turnError = MathUtil.angleModulus(turnDesiredPosition - turnSim.getAngularPositionRad());
        double turnAccel = Math.pow(TURN_NAT_FREQUENCY, 2) * turnError
                + 2 * TURN_NAT_FREQUENCY * -turnSim.getAngularVelocityRadPerSec();

        double turnAppliedVoltage = turnMotor.rOhms
                / (TURN_RATIO * turnMotor.KtNMPerAmp)
                * (TURN_INERTIA * turnAccel
                        + (TURN_RATIO * TURN_RATIO * turnMotor.KtNMPerAmp)
                                / (turnMotor.rOhms * turnMotor.KvRadPerSecPerVolt)
                                * turnSim.getAngularVelocityRadPerSec());

        turnAppliedVoltage = MathUtil.clamp(
                turnAppliedVoltage, -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());

        return turnAppliedVoltage;
    }
}
