/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.led;

import static frc.robot.constants.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;

public class SimulatedLEDControllerIO implements LEDControllerIO {
    private final AddressableLEDBuffer ledBuffer;
    public final AddressableLEDBufferView ledBufferBottom;
    public final AddressableLEDBufferView ledBufferTop;

    /** Constructs a new Real LED Controller IO. */
    public SimulatedLEDControllerIO() {
        ledBuffer = new AddressableLEDBuffer(LED_LENGTH_BOTTOM + LED_LENGTH_TOP);

        ledBufferBottom = ledBuffer.createView(0, LED_LENGTH_BOTTOM - 1);
        ledBufferTop = ledBuffer.createView(LED_LENGTH_BOTTOM, ledBuffer.getLength() - 1);
    }

    @Override
    public void updateInputs(LEDControllerInputs inputs) {
        inputs.bottomColors = getStripColors(ledBufferBottom, LED_LENGTH_BOTTOM);
        inputs.topColors = getStripColors(ledBufferTop, LED_LENGTH_TOP);
    }

    @Override
    public void setPatternBottom(LEDPattern pattern) {
        pattern.applyTo(ledBufferBottom);
    }

    @Override
    public void setPatternTop(LEDPattern pattern) {
        pattern.applyTo(ledBufferTop);
    }

    @Override
    public void setPatternWhole(LEDPattern pattern) {
        pattern.applyTo(ledBuffer);
    }

    /**
     * Gets the color of the LED strip.
     *
     * @param bufferView The buffer view of the LED strip.
     * @param length The length of the LED strip.
     * @return The color of the LED strip.
     */
    private int[][] getStripColors(AddressableLEDBufferView bufferView, int length) {
        int[][] colors = new int[length][3];

        for (int i = 0; i < length; i++) {
            colors[i][0] = bufferView.getLED8Bit(i).red;
            colors[i][1] = bufferView.getLED8Bit(i).green;
            colors[i][2] = bufferView.getLED8Bit(i).blue;
        }
        return colors;
    }
}
