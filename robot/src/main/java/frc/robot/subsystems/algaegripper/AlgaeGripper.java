/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.algaegripper;

import static frc.robot.constants.AlgaeGripperConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class AlgaeGripper extends SubsystemBase {
    private final AlgaeGripperIO io;
    private final AlgaeGripperInputsAutoLogged inputs = new AlgaeGripperInputsAutoLogged();

    /**
     * Creates a new AlgaeGripper subsystem
     *
     * @param io The IO of the AlgaeGripper.
     */
    public AlgaeGripper(AlgaeGripperIO io) {
        this.io = io;
    }

    /**
     * Command to take in a game piece
     *
     * @return Command to intake a game piece.
     */
    public Command inCommand() {
        return Commands.run(() -> io.setPower(ALGAE_INTAKE_POWER), this).withName("Intake Coral");
    }

    /**
     * Command to take out a game piece
     *
     * @return Command to outtake a game piece.
     */
    public Command outCommand() {
        return Commands.run(() -> io.setPower(ALGAE_OUTTAKE_POWER), this).withName("Outtake Coral");
    }

    /**
     * Command to hold onto a game piece
     *
     * @return Command to hold onto a game piece.
     */
    public Command holdCommand() {
        return Commands.run(() -> io.setPower(ALGAE_HOLD_POWER), this).withName("Hold");
    }

    /**
     * Command to disable the gripper
     *
     * @return Command to disable the gripper
     */
    public Command disableCommand() {
        return Commands.runOnce(() -> io.setPower(ALGAE_DISABLE_POWER), this).withName("Disabled");
    }

    /**
     * Trigger holding game piece
     *
     * @return A trigger that signifies whether algae is located in the gripper
     */
    public Trigger holdingGamePiece() {
        return new Trigger(() -> inputs.sensorValue < SENSOR_THRESHOLD)
                .debounce(SENSOR_DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AlgaeGripper", inputs);

        Logger.recordOutput("AlgaeGripper/HoldingAlgae", holdingGamePiece().getAsBoolean());
    }
}
