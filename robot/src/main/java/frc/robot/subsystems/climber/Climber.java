/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.climber;

import static frc.robot.constants.ClimberConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ClimberConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

    private final ClimberIO io;
    private final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

    public Climber(ClimberIO io) {
        this.io = io;
    }

    public Trigger latched() {
        return new Trigger(() -> MathUtil.isNear(0, inputs.velocity, ClimberConstants.TOLERANCE_VELOCITY));
    }

    private Trigger onPosition(DoubleSupplier positionDeg) {
        return new Trigger(() -> MathUtil.isNear(
                positionDeg.getAsDouble(), inputs.position.getDegrees(), ClimberConstants.TOLERANCE_POSITION_DEGREES));
    }

    public Command readyClimbCommand() {
        return Commands.sequence(
                foldOutCommand(),
                new WaitCommand(ACCELERATION_WAIT_TIME),
                Commands.waitUntil(latched()),
                disableCommand().withTimeout(0.01),
                goToClimbPositionCommand().until(onPosition(CLIMBER_READY_POSITION::getDegrees)));
    }

    private Command foldOutCommand() {
        return Commands.runOnce(
                () -> {
                    io.setCurrentLimit(ClimberConstants.FOLD_CURRENT_LIMIT);
                    io.setPower(ClimberConstants.POWER_FOLD_OUT);
                },
                this);
    }

    private Command goToClimbPositionCommand() {
        return Commands.runOnce(() -> {
                    io.setCurrentLimit(ClimberConstants.FOLD_TWO_CURRENT_LIMIT);
                })
                .andThen(Commands.run(
                        () -> {
                            io.setAngle(ClimberConstants.CLIMBER_READY_POSITION);
                        },
                        this));
    }

    public Command climberInCommand() {
        return Commands.run(
                        () -> {
                            io.setPower(POWER_CLIMB);
                        },
                        this)
                .until(onPosition(CLIMBED_POSITION::getDegrees))
                .beforeStarting(() -> io.setCurrentLimit(CLIMB_CURRENT_LIMIT));
    }

    public Command climberSlowlyOutCommand() {
        return Commands.run(
                () -> {
                    io.setPower(ClimberConstants.POWER_REVERT_CLIMB);
                },
                this);
    }

    public Command disableCommand() {
        return Commands.run(
                () -> {
                    io.setPower(0);
                },
                this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }
}
