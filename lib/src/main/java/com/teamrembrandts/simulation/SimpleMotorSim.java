/*
 * Copyright (c) 2024-2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.simulation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Create a very basic motor simulation which can take in several setpoints and take some time to reach them This is
 * physically not very accurate but can be useful for testing
 */
public class SimpleMotorSim {
    private final double timeConstant;
    private final double positionToVelocityRatio;
    private final double maxVelocity;
    private double lastUpdateTime;

    private enum RunMode {
        VELOCITY,
        POSITION
    }

    public enum MechanismType {
        /** A linear mechanism 2d which changes length depending on position (such as an elevator) */
        LINEAR,
        /** A rotational mechanism 2d which changes angle depending on position */
        PIVOT,
        /**
         * A simple flywheel mechanism 2d which changes color based on velocity (white = 0, green = max positive, blue =
         * max negative
         */
        FLYWHEEL
    }

    RunMode currentMode = RunMode.VELOCITY;

    private double setPoint;
    private double currentVelocity;
    private double currentPosition;

    private String mechName;
    private LoggedMechanismLigament2d motorMech;
    private MechanismType mechType;
    private LoggedMechanism2d mech;
    private double maxLength;
    private double maxPosition;
    private double angleOffset;

    /**
     * Create a new SimpleMotorSim
     *
     * @param timeConstant Indication for delay to reach setpoint, larger time constant means slower response
     * @param positionRatio Gear ratio from motor revolution to desired output unit, > 1 means gear reduction
     * @param velocityRatio Gear ratio from motor RPM to desired output unit, > 1 means gear reduction
     * @param maxMotorVelocity Maximum angular velocity of the motor itself
     */
    public SimpleMotorSim(
            Time timeConstant, double positionRatio, double velocityRatio, AngularVelocity maxMotorVelocity) {
        this.timeConstant = timeConstant.in(Units.Second);
        maxVelocity = maxMotorVelocity.in(Units.RPM) / velocityRatio;
        positionToVelocityRatio = (positionRatio * 60) / velocityRatio;
        lastUpdateTime = Logger.getTimestamp() / 1e6;
    }

    private void createMechanism2d(
            String name, MechanismType mechanismType, double x, double y, double length, Rotation2d zeroAngle) {
        mechType = mechanismType;
        mechName = name;
        maxLength = length;
        angleOffset = zeroAngle.getDegrees();
        mech = new LoggedMechanism2d(1, 1);
        LoggedMechanismRoot2d root = mech.getRoot("root", x, y);
        motorMech = root.append(new LoggedMechanismLigament2d("motor", length, zeroAngle.getDegrees()));
        Logger.recordOutput(name, mech);
    }

    /**
     * Construct a mechanism 2d for this motor simulation that resembles a flywheel. The flywheel will change color
     * based on the velocity of the motor.
     *
     * @param name Name that will be used to publish it ot the Networktables
     * @param x The x position of the mechanism relative to the robot if the mechanism is used in the 3D field mode in
     *     AdvantageScope
     * @param y The y position of the mechanism relative to the robot if the mechanism is used in the 3D field mode in
     *     AdvantageScope
     */
    public void createFlywheelMechanism2d(String name, double x, double y) {
        createMechanism2d(name, MechanismType.FLYWHEEL, x, y, 0.25, Rotation2d.fromDegrees(0));
    }

    /**
     * Construct a mechanism 2d for this motor simulation that resembles a linear mechanism such as an elevator.
     *
     * @param name Name that will be used to publish it ot the Networktables
     * @param x The x position of the mechanism relative to the robot if the mechanism is used in the 3D field mode in
     *     AdvantageScope
     * @param y The y position of the mechanism relative to the robot if the mechanism is used in the 3D field mode in
     *     AdvantageScope
     * @param maxPosition The maximum position that the mechanism can reach (that is targeted using .setTargetPosition)
     */
    public void createLinearMechanism2d(String name, double x, double y, double maxPosition) {
        this.maxPosition = maxPosition;
        createMechanism2d(name, MechanismType.LINEAR, x, y, 2, Rotation2d.fromDegrees(90));
    }

    /**
     * Construct a mechanism 2d for this motor simulation that resembles a pivoting mechanism such as an arm. It is
     * important to use degrees for the setTargetPosition method to ensure that the mechanism is set to the correct
     * angle.
     *
     * @param name Name that will be used to publish it ot the Networktables
     * @param x The x position of the mechanism relative to the robot if the mechanism is used in the 3D field mode in
     *     AdvantageScope
     * @param y The y position of the mechanism relative to the robot if the mechanism is used in the 3D field mode in
     *     AdvantageScope
     * @param zeroAngle The zero offset of the arm mechanism
     */
    public void createPivotMechanism2d(String name, double x, double y, Rotation2d zeroAngle) {
        createMechanism2d(name, MechanismType.PIVOT, x, y, 1, zeroAngle);
    }

    /**
     * Create a new SimpleMotorSim
     *
     * @param timeConstant Indication for delay to reach setpoint, larger time constant means slower response
     * @param positionRatio Gear ratio from motor revolution to desired output unit, > 1 means gear reduction
     * @param velocityRatio Gear ratio from motor RPM to desired output unit, > 1 means gear reduction
     */
    public SimpleMotorSim(Time timeConstant, double positionRatio, double velocityRatio) {
        this(timeConstant, positionRatio, velocityRatio, Units.RPM.of(6000));
    }

    /**
     * Create a new SimpleMotorSim
     *
     * @param timeConstant Indication for delay to reach setpoint, larger time constant means slower response
     */
    public SimpleMotorSim(Time timeConstant) {
        this(timeConstant, 1, 1);
    }

    /**
     * Set the target velocity, default units are RPM or have been defined by the velocity ratio
     *
     * @param velocity The target velocity of the output
     */
    public void setTargetVelocity(double velocity) {
        setPoint = velocity;
        currentMode = RunMode.VELOCITY;
    }

    /**
     * Set the target power of the motor
     *
     * @param power Target power of the motor from -1 to 1
     */
    public void setTargetPower(double power) {
        setPoint = power * maxVelocity;
        currentMode = RunMode.VELOCITY;
    }

    /**
     * Set the target position, default units are revolutions or have been defined by the position ratio
     *
     * @param position The target position of the output
     */
    public void setTargetPosition(double position) {
        setPoint = position;
        currentMode = RunMode.POSITION;
    }

    /**
     * Get the current system velocity
     *
     * @return The current velocity of the system
     */
    public double getCurrentVelocity() {
        update();
        return currentVelocity;
    }

    /**
     * Get the current system position
     *
     * @return The current position of the system
     */
    public double getCurrentPosition() {
        update();
        return currentPosition;
    }

    private void update() {
        double dT = (Logger.getTimestamp() / 1e6) - lastUpdateTime;
        if (dT < 1e-4) {
            return;
        }
        lastUpdateTime = Logger.getTimestamp() / 1e6;

        if (DriverStation.isDisabled()) {
            currentVelocity = 0;
            return;
        }
        if (currentMode == RunMode.VELOCITY) {
            double velError = setPoint - currentVelocity;
            double velocityStep = Math.min(maxVelocity * dT / timeConstant, Math.abs(velError));
            currentVelocity += Math.copySign(velocityStep, velError);
        } else {
            double posError = setPoint - currentPosition;
            if (Math.abs(posError) < 1e-4) {
                currentVelocity = 0;
                currentPosition = setPoint;
            }
            currentVelocity = (posError * positionToVelocityRatio) / (timeConstant * 0.5);
        }
        currentVelocity = MathUtil.clamp(currentVelocity, -maxVelocity, maxVelocity);
        currentPosition += (currentVelocity / positionToVelocityRatio) * dT;

        // Update the mechanism 2d if present
        if (motorMech != null) {
            switch (mechType) {
                case LINEAR:
                    double length = Math.max(maxLength * Math.abs(currentPosition) / maxPosition, 0.01 * maxLength);
                    motorMech.setLength(Math.copySign(length, currentPosition));
                    break;
                case PIVOT:
                    motorMech.setAngle(currentPosition + angleOffset);
                    break;
                case FLYWHEEL:
                    motorMech.setColor(velocityToColor(currentVelocity, maxVelocity));
                    break;
            }
            Logger.recordOutput(mechName, mech);
        }
    }

    /**
     * Convert velocity to a color. Zero velocity is white, max velocity in positive direction is green, max velocity in
     * negative direction is blue
     *
     * @param velocity Current velocity
     * @param maxVelocity Maximal velocity
     * @return Color8Bit representing the velocity
     */
    private static Color8Bit velocityToColor(double velocity, double maxVelocity) {
        double ratio = velocity / maxVelocity;
        int r = (int) (255 * (1 - Math.abs(ratio)));
        int g = (int) (255 * (1 + Math.min(ratio, 0)));
        int b = (int) (255 * (1 - Math.max(ratio, 0)));
        return new Color8Bit(r, g, b);
    }
}
