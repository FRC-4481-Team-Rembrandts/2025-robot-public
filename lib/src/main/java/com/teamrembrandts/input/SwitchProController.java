/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.input;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class SwitchProController extends GenericHID {

    public static enum Button {
        /** L. */
        L(5),
        /** R. */
        R(6),
        /** Left stick. */
        LeftStick(9),
        /** Right stick. */
        RightStick(10),
        /** B. */
        B(1),
        /** A. */
        A(2),
        /** Y. */
        Y(3),
        /** X. */
        X(4),
        /** Minus. */
        Minus(7),
        /** Plus. */
        Plus(8);

        public final int value;

        private Button(int value) {
            this.value = value;
        }

        public String toString() {
            return name() + "Button";
        }
    }

    public static enum Axis {
        /** Left X. */
        LeftX(0),
        /** Right X. */
        RightX(4),
        /** Left Y. */
        LeftY(1),
        /** Right Y. */
        RightY(5),
        /** ZL. */
        ZL(2),
        /** ZR. */
        ZR(3);

        public final int value;

        private Axis(int value) {
            this.value = value;
        }

        public String toString() {
            return name();
        }
    }

    /**
     * Construct an instance of a device.
     *
     * @param port The port index on the Driver Station that the device is plugged into.
     */
    public SwitchProController(int port) {
        super(port);
    }

    /**
     * Get the X axis value of left side of the controller.
     *
     * @return The axis value.
     */
    public double getLeftX() {
        return getRawAxis(SwitchProController.Axis.LeftX.value);
    }

    /**
     * Get the X axis value of right side of the controller.
     *
     * @return The axis value.
     */
    public double getRightX() {
        return getRawAxis(SwitchProController.Axis.RightX.value);
    }

    /**
     * Get the Y axis value of left side of the controller.
     *
     * @return The axis value.
     */
    public double getLeftY() {
        return getRawAxis(SwitchProController.Axis.LeftY.value);
    }

    /**
     * Get the Y axis value of right side of the controller.
     *
     * @return The axis value.
     */
    public double getRightY() {
        return getRawAxis(SwitchProController.Axis.RightY.value);
    }

    /**
     * Get the left trigger (LT) axis value of the controller. Note that this axis is bound to the range of [0, 1] as
     * opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getZLAxis() {
        return getRawAxis(Axis.ZL.value);
    }

    /**
     * Get the right trigger (RT) axis value of the controller. Note that this axis is bound to the range of [0, 1] as
     * opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getZRAxis() {
        return getRawAxis(Axis.ZR.value);
    }

    /**
     * Read the value of the left bumper (LB) button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getL() {
        return getRawButton(Button.L.value);
    }

    /**
     * Read the value of the right bumper (RB) button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getR() {
        return getRawButton(Button.R.value);
    }

    /**
     * Whether the left bumper (LB) was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getLPressed() {
        return getRawButtonPressed(Button.L.value);
    }

    /**
     * Whether the right bumper (RB) was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getRPressed() {
        return getRawButtonPressed(Button.R.value);
    }

    /**
     * Whether the left bumper (LB) was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getLReleased() {
        return getRawButtonReleased(Button.L.value);
    }

    /**
     * Whether the right bumper (RB) was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getRReleased() {
        return getRawButtonReleased(Button.R.value);
    }

    /**
     * Constructs an event instance around the right bumper's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right bumper's digital signal attached to the given loop.
     */
    public BooleanEvent l(EventLoop loop) {
        return new BooleanEvent(loop, this::getL);
    }

    /**
     * Constructs an event instance around the left bumper's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left bumper's digital signal attached to the given loop.
     */
    public BooleanEvent r(EventLoop loop) {
        return new BooleanEvent(loop, this::getR);
    }

    /**
     * Read the value of the left stick button (LSB) on the controller.
     *
     * @return The state of the button.
     */
    public boolean getLeftStickButton() {
        return getRawButton(SwitchProController.Button.LeftStick.value);
    }

    /**
     * Read the value of the right stick button (RSB) on the controller.
     *
     * @return The state of the button.
     */
    public boolean getRightStickButton() {
        return getRawButton(SwitchProController.Button.RightStick.value);
    }

    /**
     * Whether the left stick button (LSB) was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getLeftStickButtonPressed() {
        return getRawButtonPressed(SwitchProController.Button.LeftStick.value);
    }

    /**
     * Whether the right stick button (RSB) was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getRightStickButtonPressed() {
        return getRawButtonPressed(SwitchProController.Button.RightStick.value);
    }

    /**
     * Whether the left stick button (LSB) was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getLeftStickButtonReleased() {
        return getRawButtonReleased(SwitchProController.Button.LeftStick.value);
    }

    /**
     * Whether the right stick (RSB) button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getRightStickButtonReleased() {
        return getRawButtonReleased(SwitchProController.Button.RightStick.value);
    }

    /**
     * Constructs an event instance around the left stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left stick button's digital signal attached to the given loop.
     */
    public BooleanEvent leftStick(EventLoop loop) {
        return new BooleanEvent(loop, this::getLeftStickButton);
    }

    /**
     * Constructs an event instance around the right stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right stick button's digital signal attached to the given loop.
     */
    public BooleanEvent rightStick(EventLoop loop) {
        return new BooleanEvent(loop, this::getRightStickButton);
    }

    /**
     * Read the value of the A button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getAButton() {
        return getRawButton(SwitchProController.Button.A.value);
    }

    /**
     * Whether the A button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getAButtonPressed() {
        return getRawButtonPressed(SwitchProController.Button.A.value);
    }

    /**
     * Whether the A button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getAButtonReleased() {
        return getRawButtonReleased(SwitchProController.Button.A.value);
    }

    /**
     * Constructs an event instance around the A button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the A button's digital signal attached to the given loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent a(EventLoop loop) {
        return new BooleanEvent(loop, this::getAButton);
    }

    /**
     * Read the value of the B button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getBButton() {
        return getRawButton(SwitchProController.Button.B.value);
    }

    /**
     * Whether the B button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getBButtonPressed() {
        return getRawButtonPressed(SwitchProController.Button.B.value);
    }

    /**
     * Whether the B button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getBButtonReleased() {
        return getRawButtonReleased(SwitchProController.Button.B.value);
    }

    /**
     * Constructs an event instance around the B button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the B button's digital signal attached to the given loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent b(EventLoop loop) {
        return new BooleanEvent(loop, this::getBButton);
    }

    /**
     * Read the value of the X button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getXButton() {
        return getRawButton(SwitchProController.Button.X.value);
    }

    /**
     * Whether the X button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getXButtonPressed() {
        return getRawButtonPressed(SwitchProController.Button.X.value);
    }

    /**
     * Whether the X button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getXButtonReleased() {
        return getRawButtonReleased(SwitchProController.Button.X.value);
    }

    /**
     * Constructs an event instance around the X button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the X button's digital signal attached to the given loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent x(EventLoop loop) {
        return new BooleanEvent(loop, this::getXButton);
    }

    /**
     * Read the value of the Y button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getYButton() {
        return getRawButton(SwitchProController.Button.Y.value);
    }

    /**
     * Whether the Y button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getYButtonPressed() {
        return getRawButtonPressed(SwitchProController.Button.Y.value);
    }

    /**
     * Whether the Y button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getYButtonReleased() {
        return getRawButtonReleased(SwitchProController.Button.Y.value);
    }

    /**
     * Constructs an event instance around the Y button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the Y button's digital signal attached to the given loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent y(EventLoop loop) {
        return new BooleanEvent(loop, this::getYButton);
    }

    /**
     * Read the value of the back button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getMinusButton() {
        return getRawButton(Button.Minus.value);
    }

    /**
     * Whether the back button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getMinusButtonPressed() {
        return getRawButtonPressed(Button.Minus.value);
    }

    /**
     * Whether the back button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getMinusButtonReleased() {
        return getRawButtonReleased(Button.Minus.value);
    }

    /**
     * Constructs an event instance around the back button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the back button's digital signal attached to the given loop.
     */
    public BooleanEvent minus(EventLoop loop) {
        return new BooleanEvent(loop, this::getMinusButton);
    }

    /**
     * Read the value of the start button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getPlusButton() {
        return getRawButton(Button.Plus.value);
    }

    /**
     * Whether the start button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getPlusButtonPressed() {
        return getRawButtonPressed(Button.Plus.value);
    }

    /**
     * Whether the start button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getPlusButtonReleased() {
        return getRawButtonReleased(Button.Plus.value);
    }

    /**
     * Constructs an event instance around the start button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the start button's digital signal attached to the given loop.
     */
    public BooleanEvent plus(EventLoop loop) {
        return new BooleanEvent(loop, this::getPlusButton);
    }

    /**
     * Constructs an event instance around the axis value of the left trigger. The returned trigger will be true when
     * the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link BooleanEvent} to be true. This value should be in
     *     the range [0, 1] where 0 is the unpressed state of the axis.
     * @param loop the event loop instance to attach the event to.
     * @return an event instance that is true when the left trigger's axis exceeds the provided threshold, attached to
     *     the given event loop
     */
    public BooleanEvent zl(double threshold, EventLoop loop) {
        return new BooleanEvent(loop, () -> getZLAxis() > threshold);
    }

    /**
     * Constructs an event instance around the axis value of the left trigger. The returned trigger will be true when
     * the axis value is greater than 0.5.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance that is true when the left trigger's axis exceeds the provided threshold, attached to
     *     the given event loop
     */
    public BooleanEvent zl(EventLoop loop) {
        return zl(0.5, loop);
    }

    /**
     * Constructs an event instance around the axis value of the right trigger. The returned trigger will be true when
     * the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link BooleanEvent} to be true. This value should be in
     *     the range [0, 1] where 0 is the unpressed state of the axis.
     * @param loop the event loop instance to attach the event to.
     * @return an event instance that is true when the right trigger's axis exceeds the provided threshold, attached to
     *     the given event loop
     */
    public BooleanEvent zr(double threshold, EventLoop loop) {
        return new BooleanEvent(loop, () -> getZRAxis() > threshold);
    }

    /**
     * Constructs an event instance around the axis value of the right trigger. The returned trigger will be true when
     * the axis value is greater than 0.5.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance that is true when the right trigger's axis exceeds the provided threshold, attached to
     *     the given event loop
     */
    public BooleanEvent zr(EventLoop loop) {
        return zr(0.5, loop);
    }
}
