/*
 * Copyright (c) 2024-2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.input;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandSwitchProController extends CommandGenericHID {
    private final SwitchProController hid;

    /**
     * Construct an instance of a device.
     *
     * @param port The port index on the Driver Station that the device is plugged into.
     */
    public CommandSwitchProController(int port) {

        super(port);
        hid = new SwitchProController(port);
    }

    /**
     * Get the underlying GenericHID object.
     *
     * @return the wrapped GenericHID object
     */
    @Override
    public SwitchProController getHID() {
        return hid;
    }

    /**
     * Constructs an event instance around the left bumper's digital signal.
     *
     * @return an event instance representing the left bumper's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #l(EventLoop)
     */
    public Trigger l() {
        return l(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the left bumper's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right bumper's digital signal attached to the given loop.
     */
    public Trigger l(EventLoop loop) {
        return hid.l(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the right bumper's digital signal.
     *
     * @return an event instance representing the right bumper's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #r(EventLoop)
     */
    public Trigger r() {
        return r(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the right bumper's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left bumper's digital signal attached to the given loop.
     */
    public Trigger r(EventLoop loop) {
        return hid.r(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the left stick button's digital signal.
     *
     * @return an event instance representing the left stick button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #leftStick(EventLoop)
     */
    public Trigger leftStick() {
        return leftStick(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the left stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left stick button's digital signal attached to the given loop.
     */
    public Trigger leftStick(EventLoop loop) {
        return hid.leftStick(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the right stick button's digital signal.
     *
     * @return an event instance representing the right stick button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #rightStick(EventLoop)
     */
    public Trigger rightStick() {
        return rightStick(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the right stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right stick button's digital signal attached to the given loop.
     */
    public Trigger rightStick(EventLoop loop) {
        return hid.rightStick(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the A button's digital signal.
     *
     * @return an event instance representing the A button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #a(EventLoop)
     */
    public Trigger a() {
        return a(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the A button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the A button's digital signal attached to the given loop.
     */
    public Trigger a(EventLoop loop) {
        return hid.a(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the B button's digital signal.
     *
     * @return an event instance representing the B button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #b(EventLoop)
     */
    public Trigger b() {
        return b(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the B button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the B button's digital signal attached to the given loop.
     */
    public Trigger b(EventLoop loop) {
        return hid.b(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the X button's digital signal.
     *
     * @return an event instance representing the X button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #x(EventLoop)
     */
    public Trigger x() {
        return x(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the X button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the X button's digital signal attached to the given loop.
     */
    public Trigger x(EventLoop loop) {
        return hid.x(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the Y button's digital signal.
     *
     * @return an event instance representing the Y button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #y(EventLoop)
     */
    public Trigger y() {
        return y(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the Y button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the Y button's digital signal attached to the given loop.
     */
    public Trigger y(EventLoop loop) {
        return hid.y(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the start button's digital signal.
     *
     * @return an event instance representing the start button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #plus(EventLoop)
     */
    public Trigger plus() {
        return plus(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the start button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the start button's digital signal attached to the given loop.
     */
    public Trigger plus(EventLoop loop) {
        return hid.plus(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the back button's digital signal.
     *
     * @return an event instance representing the back button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #minus(EventLoop)
     */
    public Trigger minus() {
        return minus(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the back button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the back button's digital signal attached to the given loop.
     */
    public Trigger minus(EventLoop loop) {
        return hid.minus(loop).castTo(Trigger::new);
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger will be true when
     * the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value should be in the
     *     range [0, 1] where 0 is the unpressed state of the axis.
     * @param loop the event loop instance to attach the Trigger to.
     * @return a Trigger instance that is true when the left trigger's axis exceeds the provided threshold, attached to
     *     the given event loop
     */
    public Trigger zl(double threshold, EventLoop loop) {
        return hid.zl(threshold, loop).castTo(Trigger::new);
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger will be true when
     * the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value should be in the
     *     range [0, 1] where 0 is the unpressed state of the axis.
     * @return a Trigger instance that is true when the left trigger's axis exceeds the provided threshold, attached to
     *     the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger zl(double threshold) {
        return zl(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger will be true when
     * the axis value is greater than 0.5.
     *
     * @return a Trigger instance that is true when the left trigger's axis exceeds 0.5, attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger zl() {
        return zl(0.5);
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger will be true when
     * the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value should be in the
     *     range [0, 1] where 0 is the unpressed state of the axis.
     * @param loop the event loop instance to attach the Trigger to.
     * @return a Trigger instance that is true when the right trigger's axis exceeds the provided threshold, attached to
     *     the given event loop
     */
    public Trigger zr(double threshold, EventLoop loop) {
        return hid.zr(threshold, loop).castTo(Trigger::new);
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger will be true when
     * the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value should be in the
     *     range [0, 1] where 0 is the unpressed state of the axis.
     * @return a Trigger instance that is true when the right trigger's axis exceeds the provided threshold, attached to
     *     the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger zr(double threshold) {
        return zr(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger will be true when
     * the axis value is greater than 0.5.
     *
     * @return a Trigger instance that is true when the right trigger's axis exceeds 0.5, attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger zr() {
        return zr(0.5);
    }

    /**
     * Get the X axis value of left side of the controller.
     *
     * @return The axis value.
     */
    public double getLeftX() {
        return hid.getLeftX();
    }

    /**
     * Get the X axis value of right side of the controller.
     *
     * @return The axis value.
     */
    public double getRightX() {
        return hid.getRightX();
    }

    /**
     * Get the Y axis value of left side of the controller.
     *
     * @return The axis value.
     */
    public double getLeftY() {
        return hid.getLeftY();
    }

    /**
     * Get the Y axis value of right side of the controller.
     *
     * @return The axis value.
     */
    public double getRightY() {
        return hid.getRightY();
    }

    /**
     * Get the left trigger (LT) axis value of the controller. Note that this axis is bound to the range of [0, 1] as
     * opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getZLAxis() {
        return hid.getZLAxis();
    }

    /**
     * Get the right trigger (RT) axis value of the controller. Note that this axis is bound to the range of [0, 1] as
     * opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getZRAxis() {
        return hid.getZRAxis();
    }
}
