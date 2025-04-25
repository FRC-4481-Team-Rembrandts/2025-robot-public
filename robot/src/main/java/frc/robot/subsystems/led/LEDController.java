/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.led;

import static frc.robot.constants.LEDConstants.*;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** The LED Controller subsystem. */
public class LEDController extends SubsystemBase {

    LEDControllerIO io;
    LEDControllerInputsAutoLogged inputs = new LEDControllerInputsAutoLogged();
    private LEDPattern currentScoringledPattern = PATTERN_L4;
    private boolean intakePatternEnabled = false;
    private boolean manualOverrideEnabled = false;

    /**
     * Creates a new LED Controller subsystem.
     *
     * @param io The IO of the LED Controller.
     */
    public LEDController(LEDControllerIO io) {
        this.io = io;
    }

    /**
     * Sets the pattern of the LED strips on the robot to the L1 pattern.
     *
     * @return Command to set the pattern of the LED strips to the L1 pattern.
     */
    public Command setLEDPatternL1Command() {
        return setPatternCommand(() -> PATTERN_L1, false);
    }

    /**
     * Sets the pattern of the LED strips on the robot to the L2 pattern.
     *
     * @return Command to set the pattern of the LED strips to the L2 pattern.
     */
    public Command setLEDPatternL2Command() {
        return setPatternCommand(() -> PATTERN_L2, false);
    }

    /**
     * Sets the pattern of the LED strips on the robot to the L3 pattern.
     *
     * @return Command to set the pattern of the LED strips to the L3 pattern.
     */
    public Command setLEDPatternL3Command() {
        return setPatternCommand(() -> PATTERN_L3, false);
    }

    /**
     * Sets the pattern of the LED strips on the robot to the L4 pattern.
     *
     * @return Command to set the pattern of the LED strips to the L4 pattern.
     */
    public Command setLEDPatternL4Command() {
        return setPatternCommand(() -> PATTERN_L4, false);
    }

    /**
     * Sets the pattern of the LED strips on the robot to the barge pattern.
     *
     * @return Command to set the pattern of the LED strips to the barge pattern.
     */
    public Command setLEDPatternBargeCommand() {
        return setPatternCommand(() -> PATTERN_BARGE, false);
    }

    /**
     * Sets the pattern of the LED strips on the robot to the intake pattern.
     *
     * @return Command to set the pattern of the LED strips to the intake pattern.
     */
    public Command setLEDPatternIntakeCommand() {
        return setPatternCommand(() -> currentScoringledPattern, false)
                .beforeStarting(() -> intakePatternEnabled = true);
    }

    /**
     * Sets the pattern of the LED strips on the robot to the deviation pattern. LOW deviation = Green, HIGH deviation =
     * Red. LOW deviation = all leds, HIGH deviation = 0 leds.
     *
     * @param deviation The input deviation of the LED strips [0, 1].
     * @return Command to set the pattern of the LED strips to the deviation pattern.
     */
    public Command setDeviationPatternCommand(DoubleSupplier deviation) {
        LEDPattern progressPattern = DEVIATION_GRADIENT_PATTERN.mask(LEDPattern.progressMaskLayer(deviation));

        return setPatternCommand(() -> progressPattern, true);
    }

    /**
     * Disables the intake pattern of the bottom LED strips on the robot.
     *
     * @return Command to disable the intake pattern of the bottom LED strips.
     */
    public Command disableLedPatternIntakeCommand() {
        return setPatternCommand(() -> currentScoringledPattern, false)
                .beforeStarting(() -> intakePatternEnabled = false);
    }

    /**
     * Enables the manual override overlay on the arm state pattern.
     *
     * @return Command to set the pattern of the LED strips to the manual override pattern.
     */
    public Command enableManualOverrideOverlayCommand() {
        return Commands.runOnce(() -> manualOverrideEnabled = true);
    }

    /**
     * Disables the manual override overlay on the arm state pattern.
     *
     * @return Command to set the pattern of the LED strips to the manual override pattern.
     */
    public Command disableManualOverrideOverlayCommand() {
        return Commands.runOnce(() -> manualOverrideEnabled = false);
    }

    /**
     * Gets the colors of the LED strip at the top on the robot.
     *
     * @return The colors of the LED strip at the top on the robot.
     */
    public Color8Bit[] getLEDColorsTop() {
        int[][] colorsDeconstructed = inputs.topColors;

        Color8Bit[] colors = new Color8Bit[colorsDeconstructed.length];

        for (int i = 0; i < colorsDeconstructed.length; i++) {
            colors[i] = new Color8Bit(colorsDeconstructed[i][0], colorsDeconstructed[i][1], colorsDeconstructed[i][2]);
        }

        return colors;
    }

    /**
     * Gets the colors of the LED strip at the bottom on the robot.
     *
     * @return The colors of the LED strip at the bottom on the robot.
     */
    public Color8Bit[] getLEDColorsBottom() {
        int[][] colorsDeconstructed = inputs.bottomColors;

        Color8Bit[] colors = new Color8Bit[colorsDeconstructed.length];

        for (int i = 0; i < colorsDeconstructed.length; i++) {
            colors[i] = new Color8Bit(colorsDeconstructed[i][0], colorsDeconstructed[i][1], colorsDeconstructed[i][2]);
        }

        return colors;
    }

    /**
     * Sets the pattern of the LED strips on the robot.
     *
     * @param pattern The input pattern of the LED strips.
     * @param completeStrip Whether the whole LED strip should be set.
     * @return Command to set the pattern of the LED strips.
     */
    private Command setPatternCommand(Supplier<LEDPattern> pattern, boolean completeStrip) {
        return runOnce(() -> {
            if (completeStrip) {
                io.setPatternWhole(pattern.get());
            } else {
                currentScoringledPattern = pattern.get();

                if (manualOverrideEnabled) {
                    currentScoringledPattern = currentScoringledPattern.blink(MANUAL_OVERRIDE_BLINK_TIME);
                }

                if (intakePatternEnabled) {
                    io.setPatternBottom(PATTERN_INTAKE);
                } else {
                    io.setPatternBottom(currentScoringledPattern);
                }
                io.setPatternTop(currentScoringledPattern);
            }
        });
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LEDController", inputs);
    }
}
