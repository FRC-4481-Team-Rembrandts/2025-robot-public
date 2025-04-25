/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.input;

import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class NaconPS4Controller extends GenericHID implements Sendable {
    public enum Button {
        /** Square button. */
        kSquare(3),
        /** Cross button. */
        kCross(1),
        /** Circle button. */
        kCircle(2),
        /** Triangle button. */
        kTriangle(4),
        /** Left trigger 1 button. */
        kL1(5),
        /** Right trigger 1 button. */
        kR1(6),
        /** Share button. */
        kShare(7),
        /** Options button. */
        kOptions(8),
        /** L3 (left stick) button. */
        kL3(9),
        /** R3 (right stick) button. */
        kR3(10);

        /** Button value. */
        public final int value;

        Button(int value) {
            this.value = value;
        }

        /**
         * Get the human-friendly name of the button, matching the relevant methods. This is done by stripping the
         * leading `k`, and appending `Button`.
         *
         * <p>Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the button.
         */
        @Override
        public String toString() {
            // Remove leading `k`
            return this.name().substring(1) + "Button";
        }
    }

    /** Represents an axis on an NaconPS4Controller. */
    public enum Axis {
        /** Left X axis. */
        kLeftX(0),
        /** Left Y axis. */
        kLeftY(1),
        /** Right X axis. */
        kRightX(4),
        /** Right Y axis. */
        kRightY(5),
        /** Left trigger 2. */
        kL2(2),
        /** Right trigger 2. */
        kR2(3);

        /** Axis value. */
        public final int value;

        Axis(int value) {
            this.value = value;
        }

        /**
         * Get the human-friendly name of the axis, matching the relevant methods. This is done by stripping the leading
         * `k`, and appending `Axis` if the name ends with `2`.
         *
         * <p>Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the axis.
         */
        @Override
        public String toString() {
            var name = this.name().substring(1); // Remove leading `k`
            if (name.endsWith("2")) {
                return name + "Axis";
            }
            return name;
        }
    }

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into (0-5).
     */
    public NaconPS4Controller(final int port) {
        super(port);
        HAL.report(FRCNetComm.tResourceType.kResourceType_PS4Controller, port + 1);
    }

    /**
     * Get the X axis value of left side of the controller.
     *
     * @return The axis value.
     */
    public double getLeftX() {
        return getRawAxis(NaconPS4Controller.Axis.kLeftX.value);
    }

    /**
     * Get the Y axis value of left side of the controller.
     *
     * @return The axis value.
     */
    public double getLeftY() {
        return getRawAxis(NaconPS4Controller.Axis.kLeftY.value);
    }

    /**
     * Get the X axis value of right side of the controller.
     *
     * @return The axis value.
     */
    public double getRightX() {
        return getRawAxis(NaconPS4Controller.Axis.kRightX.value);
    }

    /**
     * Get the Y axis value of right side of the controller.
     *
     * @return The axis value.
     */
    public double getRightY() {
        return getRawAxis(NaconPS4Controller.Axis.kRightY.value);
    }

    /**
     * Get the left trigger 2 axis value of the controller. Note that this axis is bound to the range of [0, 1] as
     * opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getL2Axis() {
        return getRawAxis(NaconPS4Controller.Axis.kL2.value);
    }

    /**
     * Get the right trigger 2 axis value of the controller. Note that this axis is bound to the range of [0, 1] as
     * opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getR2Axis() {
        return getRawAxis(NaconPS4Controller.Axis.kR2.value);
    }

    /**
     * Read the value of the square button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getSquareButton() {
        return getRawButton(NaconPS4Controller.Button.kSquare.value);
    }

    /**
     * Whether the square button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getSquareButtonPressed() {
        return getRawButtonPressed(NaconPS4Controller.Button.kSquare.value);
    }

    /**
     * Whether the square button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getSquareButtonReleased() {
        return getRawButtonReleased(NaconPS4Controller.Button.kSquare.value);
    }

    /**
     * Constructs an event instance around the square button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the square button's digital signal attached to the given loop.
     */
    public BooleanEvent square(EventLoop loop) {
        return button(NaconPS4Controller.Button.kSquare.value, loop);
    }

    /**
     * Read the value of the cross button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getCrossButton() {
        return getRawButton(NaconPS4Controller.Button.kCross.value);
    }

    /**
     * Whether the cross button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getCrossButtonPressed() {
        return getRawButtonPressed(NaconPS4Controller.Button.kCross.value);
    }

    /**
     * Whether the cross button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getCrossButtonReleased() {
        return getRawButtonReleased(NaconPS4Controller.Button.kCross.value);
    }

    /**
     * Constructs an event instance around the cross button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the cross button's digital signal attached to the given loop.
     */
    public BooleanEvent cross(EventLoop loop) {
        return button(NaconPS4Controller.Button.kCross.value, loop);
    }

    /**
     * Read the value of the circle button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getCircleButton() {
        return getRawButton(NaconPS4Controller.Button.kCircle.value);
    }

    /**
     * Whether the circle button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getCircleButtonPressed() {
        return getRawButtonPressed(NaconPS4Controller.Button.kCircle.value);
    }

    /**
     * Whether the circle button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getCircleButtonReleased() {
        return getRawButtonReleased(NaconPS4Controller.Button.kCircle.value);
    }

    /**
     * Constructs an event instance around the circle button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the circle button's digital signal attached to the given loop.
     */
    public BooleanEvent circle(EventLoop loop) {
        return button(NaconPS4Controller.Button.kCircle.value, loop);
    }

    /**
     * Read the value of the triangle button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getTriangleButton() {
        return getRawButton(NaconPS4Controller.Button.kTriangle.value);
    }

    /**
     * Whether the triangle button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getTriangleButtonPressed() {
        return getRawButtonPressed(NaconPS4Controller.Button.kTriangle.value);
    }

    /**
     * Whether the triangle button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getTriangleButtonReleased() {
        return getRawButtonReleased(NaconPS4Controller.Button.kTriangle.value);
    }

    /**
     * Constructs an event instance around the triangle button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the triangle button's digital signal attached to the given loop.
     */
    public BooleanEvent triangle(EventLoop loop) {
        return button(NaconPS4Controller.Button.kTriangle.value, loop);
    }

    /**
     * Read the value of the left trigger 1 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getL1Button() {
        return getRawButton(NaconPS4Controller.Button.kL1.value);
    }

    /**
     * Whether the left trigger 1 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getL1ButtonPressed() {
        return getRawButtonPressed(NaconPS4Controller.Button.kL1.value);
    }

    /**
     * Whether the left trigger 1 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getL1ButtonReleased() {
        return getRawButtonReleased(NaconPS4Controller.Button.kL1.value);
    }

    /**
     * Constructs an event instance around the left trigger 1 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left trigger 1 button's digital signal attached to the given loop.
     */
    public BooleanEvent l1(EventLoop loop) {
        return button(NaconPS4Controller.Button.kL1.value, loop);
    }

    /**
     * Read the value of the right trigger 1 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getR1Button() {
        return getRawButton(NaconPS4Controller.Button.kR1.value);
    }

    /**
     * Whether the right trigger 1 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getR1ButtonPressed() {
        return getRawButtonPressed(NaconPS4Controller.Button.kR1.value);
    }

    /**
     * Whether the right trigger 1 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getR1ButtonReleased() {
        return getRawButtonReleased(NaconPS4Controller.Button.kR1.value);
    }

    /**
     * Constructs an event instance around the right trigger 1 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right trigger 1 button's digital signal attached to the given loop.
     */
    public BooleanEvent r1(EventLoop loop) {
        return button(NaconPS4Controller.Button.kR1.value, loop);
    }

    /**
     * Read the value of the share button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getShareButton() {
        return getRawButton(NaconPS4Controller.Button.kShare.value);
    }

    /**
     * Whether the share button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getShareButtonPressed() {
        return getRawButtonPressed(NaconPS4Controller.Button.kShare.value);
    }

    /**
     * Whether the share button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getShareButtonReleased() {
        return getRawButtonReleased(NaconPS4Controller.Button.kShare.value);
    }

    /**
     * Constructs an event instance around the share button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the share button's digital signal attached to the given loop.
     */
    public BooleanEvent share(EventLoop loop) {
        return button(NaconPS4Controller.Button.kShare.value, loop);
    }

    /**
     * Read the value of the options button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getOptionsButton() {
        return getRawButton(NaconPS4Controller.Button.kOptions.value);
    }

    /**
     * Whether the options button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getOptionsButtonPressed() {
        return getRawButtonPressed(NaconPS4Controller.Button.kOptions.value);
    }

    /**
     * Whether the options button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getOptionsButtonReleased() {
        return getRawButtonReleased(NaconPS4Controller.Button.kOptions.value);
    }

    /**
     * Constructs an event instance around the options button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the options button's digital signal attached to the given loop.
     */
    public BooleanEvent options(EventLoop loop) {
        return button(NaconPS4Controller.Button.kOptions.value, loop);
    }

    /**
     * Read the value of the L3 (left stick) button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getL3Button() {
        return getRawButton(NaconPS4Controller.Button.kL3.value);
    }

    /**
     * Whether the L3 (left stick) button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getL3ButtonPressed() {
        return getRawButtonPressed(NaconPS4Controller.Button.kL3.value);
    }

    /**
     * Whether the L3 (left stick) button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getL3ButtonReleased() {
        return getRawButtonReleased(NaconPS4Controller.Button.kL3.value);
    }

    /**
     * Constructs an event instance around the L3 (left stick) button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the L3 (left stick) button's digital signal attached to the given loop.
     */
    public BooleanEvent l3(EventLoop loop) {
        return button(NaconPS4Controller.Button.kL3.value, loop);
    }

    /**
     * Read the value of the R3 (right stick) button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getR3Button() {
        return getRawButton(NaconPS4Controller.Button.kR3.value);
    }

    /**
     * Whether the R3 (right stick) button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getR3ButtonPressed() {
        return getRawButtonPressed(NaconPS4Controller.Button.kR3.value);
    }

    /**
     * Whether the R3 (right stick) button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getR3ButtonReleased() {
        return getRawButtonReleased(NaconPS4Controller.Button.kR3.value);
    }

    /**
     * Constructs an event instance around the R3 (right stick) button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the R3 (right stick) button's digital signal attached to the given loop.
     */
    public BooleanEvent r3(EventLoop loop) {
        return button(NaconPS4Controller.Button.kR3.value, loop);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("HID");
        builder.publishConstString("ControllerType", "NaconPS4");
        builder.addDoubleProperty("L2", this::getL2Axis, null);
        builder.addDoubleProperty("R2", this::getR2Axis, null);
        builder.addDoubleProperty("LeftX", this::getLeftX, null);
        builder.addDoubleProperty("LeftY", this::getLeftY, null);
        builder.addDoubleProperty("RightX", this::getRightX, null);
        builder.addDoubleProperty("RightY", this::getRightY, null);
        builder.addBooleanProperty("Square", this::getSquareButton, null);
        builder.addBooleanProperty("Cross", this::getCrossButton, null);
        builder.addBooleanProperty("Circle", this::getCircleButton, null);
        builder.addBooleanProperty("Triangle", this::getTriangleButton, null);
        builder.addBooleanProperty("L1", this::getL1Button, null);
        builder.addBooleanProperty("R1", this::getR1Button, null);
        builder.addBooleanProperty("Share", this::getShareButton, null);
        builder.addBooleanProperty("Options", this::getOptionsButton, null);
        builder.addBooleanProperty("L3", this::getL3Button, null);
        builder.addBooleanProperty("R3", this::getR3Button, null);
    }
}
