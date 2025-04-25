/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.feedthroughgripper;

import static frc.robot.constants.FeedThroughGripperConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class FeedThroughGripper extends SubsystemBase {
    private final FeedThroughGripperIO io;
    private final FeedThroughGripperInputsAutoLogged inputs = new FeedThroughGripperInputsAutoLogged();
    SlewRateLimiter feedtroughGripperSlewRateLimiter = new SlewRateLimiter(FEED_THROUGH_GRIPPER_RATE_LIMIT);

    public FeedThroughGripper(FeedThroughGripperIO io) {
        this.io = io;
    }

    /**
     * Command to disable the gripper
     *
     * @return Command to disable the gripper.
     */
    public Command disableCommand() {
        return Commands.run(() -> io.setPower(0.0), this).withName("Disable gripper");
    }

    /**
     * Command for enabling the limitswitch
     *
     * @return Command to enable the limitswitch.
     */
    private Command enableLimitSwitch() {
        return Commands.runOnce(() -> io.enableLimitSwitch(true), this).withName("Enable Switch");
    }

    /**
     * Command for Disableing the limitswitch
     *
     * @return Command to disable the limitswitch.
     */
    private Command disableLimitSwitch() {
        return Commands.runOnce(() -> io.enableLimitSwitch(false), this).withName("Disable Switch");
    }

    /**
     * Command for intaking gamepiece
     *
     * @return Command to intake a game piece.
     */
    public Command inCommand() {
        return Commands.sequence(
                        enableLimitSwitch(),
                        Commands.run(() -> io.setPower(feedtroughGripperSlewRateLimiter.calculate(INTAKE_POWER)), this)
                                .beforeStarting(() -> feedtroughGripperSlewRateLimiter.reset(0.0))
                                .until(coralPresent()),
                        holdCommand(),
                        disableLimitSwitch()
                                .until(coralPresent())
                                .andThen(disableCommand().withTimeout(0.01)))
                .withName("Intake Coral");
    }

    private Command outCommand(double outtake_power) {
        return Commands.sequence(disableLimitSwitch(), Commands.run(() -> io.setPower(outtake_power), this))
                .withName("Outtake Coral");
    }

    /**
     * Command for outtaking L1
     *
     * @return Command to outtake a game piece.
     */
    public Command outL1Command() {
        return outCommand(L1_OUTTAKE_POWER);
    }

    /**
     * Command for outtaking L2
     *
     * @return Command to outtake a game piece.
     */
    public Command outL2Command() {
        return outCommand(L2_OUTTAKE_POWER);
    }

    /**
     * Command for outtaking L3
     *
     * @return Command to outtake a game piece.
     */
    public Command outL3Command() {
        return outCommand(L3_OUTTAKE_POWER);
    }

    /**
     * Command for outtaking L4
     *
     * @return Command to outtake a game piece.
     */
    public Command outL4Command() {
        return outCommand(L4_OUTTAKE_POWER);
    }

    /**
     * Command to hold onto a game piece
     *
     * @return Command to hold onto a game piece.
     */
    public Command holdCommand() {
        return Commands.runOnce(() -> io.setPower(0.0), this).withName("Hold");
    }

    public Command fixIntakeCommand() {
        return Commands.run(() -> io.setPower(-0.5 * INTAKE_POWER), this)
                .withTimeout(0.5)
                .andThen(Commands.run(() -> io.setPower(INTAKE_POWER), this).until(coralPresent()));
    }

    public Command slowIntakeCommand() {
        return Commands.sequence(disableLimitSwitch(), Commands.run(() -> io.setPower(0.2 * INTAKE_POWER), this));
    }

    /**
     * Command to detect the holding of a game piece
     *
     * @return Command to detect if a game piece is being hold.
     */
    public Trigger coralPresent() {
        return new Trigger(() -> inputs.forwardLimitSwitchIsPressed)
                .debounce(SENSOR_DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("FeedThroughGripper", inputs);
        Logger.recordOutput("FeedThroughGripper/coralPresent", coralPresent().getAsBoolean());

        Logger.recordOutput("FeedThroughGripper/powerDraw", inputs.appliedVoltage * inputs.currentDraw);
    }
}
