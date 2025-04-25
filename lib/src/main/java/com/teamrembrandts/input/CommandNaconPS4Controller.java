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

public class CommandNaconPS4Controller extends CommandGenericHID {
    private final NaconPS4Controller hid;

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public CommandNaconPS4Controller(int port) {
        super(port);
        hid = new NaconPS4Controller(port);
    }

    /**
     * Get the underlying GenericHID object.
     *
     * @return the wrapped GenericHID object
     */
    @Override
    public NaconPS4Controller getHID() {
        return hid;
    }

    /**
     * Constructs a Trigger instance around the square button's digital signal.
     *
     * @return a Trigger instance representing the square button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #square(EventLoop)
     */
    public Trigger square() {
        return square(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the square button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the square button's digital signal attached to the given loop.
     */
    public Trigger square(EventLoop loop) {
        return button(NaconPS4Controller.Button.kSquare.value, loop);
    }

    /**
     * Constructs a Trigger instance around the cross button's digital signal.
     *
     * @return a Trigger instance representing the cross button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #cross(EventLoop)
     */
    public Trigger cross() {
        return cross(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the cross button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the cross button's digital signal attached to the given loop.
     */
    public Trigger cross(EventLoop loop) {
        return button(NaconPS4Controller.Button.kCross.value, loop);
    }

    /**
     * Constructs a Trigger instance around the circle button's digital signal.
     *
     * @return a Trigger instance representing the circle button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #circle(EventLoop)
     */
    public Trigger circle() {
        return circle(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the circle button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the circle button's digital signal attached to the given loop.
     */
    public Trigger circle(EventLoop loop) {
        return button(NaconPS4Controller.Button.kCircle.value, loop);
    }

    /**
     * Constructs a Trigger instance around the triangle button's digital signal.
     *
     * @return a Trigger instance representing the triangle button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #triangle(EventLoop)
     */
    public Trigger triangle() {
        return triangle(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the triangle button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the triangle button's digital signal attached to the given loop.
     */
    public Trigger triangle(EventLoop loop) {
        return button(NaconPS4Controller.Button.kTriangle.value, loop);
    }

    /**
     * Constructs a Trigger instance around the left trigger 1 button's digital signal.
     *
     * @return a Trigger instance representing the left trigger 1 button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #l1(EventLoop)
     */
    public Trigger l1() {
        return l1(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the left trigger 1 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the left trigger 1 button's digital signal attached to the given loop.
     */
    public Trigger l1(EventLoop loop) {
        return button(NaconPS4Controller.Button.kL1.value, loop);
    }

    /**
     * Constructs a Trigger instance around the right trigger 1 button's digital signal.
     *
     * @return a Trigger instance representing the right trigger 1 button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #r1(EventLoop)
     */
    public Trigger r1() {
        return r1(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the right trigger 1 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the right trigger 1 button's digital signal attached to the given loop.
     */
    public Trigger r1(EventLoop loop) {
        return button(NaconPS4Controller.Button.kR1.value, loop);
    }

    /**
     * Constructs a Trigger instance around the share button's digital signal.
     *
     * @return a Trigger instance representing the share button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #share(EventLoop)
     */
    public Trigger share() {
        return share(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the share button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the share button's digital signal attached to the given loop.
     */
    public Trigger share(EventLoop loop) {
        return button(NaconPS4Controller.Button.kShare.value, loop);
    }

    /**
     * Constructs a Trigger instance around the options button's digital signal.
     *
     * @return a Trigger instance representing the options button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #options(EventLoop)
     */
    public Trigger options() {
        return options(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the options button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the options button's digital signal attached to the given loop.
     */
    public Trigger options(EventLoop loop) {
        return button(NaconPS4Controller.Button.kOptions.value, loop);
    }

    /**
     * Constructs a Trigger instance around the L3 (left stick) button's digital signal.
     *
     * @return a Trigger instance representing the L3 (left stick) button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #l3(EventLoop)
     */
    public Trigger l3() {
        return l3(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the L3 (left stick) button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the L3 (left stick) button's digital signal attached to the given loop.
     */
    public Trigger l3(EventLoop loop) {
        return button(NaconPS4Controller.Button.kL3.value, loop);
    }

    /**
     * Constructs a Trigger instance around the R3 (right stick) button's digital signal.
     *
     * @return a Trigger instance representing the R3 (right stick) button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #r3(EventLoop)
     */
    public Trigger r3() {
        return r3(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the R3 (right stick) button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the R3 (right stick) button's digital signal attached to the given loop.
     */
    public Trigger r3(EventLoop loop) {
        return button(NaconPS4Controller.Button.kR3.value, loop);
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
     * Get the Y axis value of left side of the controller.
     *
     * @return The axis value.
     */
    public double getLeftY() {
        return hid.getLeftY();
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
     * Get the Y axis value of right side of the controller.
     *
     * @return The axis value.
     */
    public double getRightY() {
        return hid.getRightY();
    }

    /**
     * Get the left trigger 2 axis value of the controller. Note that this axis is bound to the range of [0, 1] as
     * opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getL2Axis() {
        return hid.getL2Axis();
    }

    /**
     * Get the right trigger 2 axis value of the controller. Note that this axis is bound to the range of [0, 1] as
     * opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getR2Axis() {
        return hid.getR2Axis();
    }
}
