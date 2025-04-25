/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.constants;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {
    public static final int LED_CONTROLLER_PORT = 0;
    public static final int LED_LENGTH_BOTTOM = 27;
    public static final int LED_LENGTH_TOP = 28;
    private static final int LED_BRIGHTNESS = 140;
    public static final Time MANUAL_OVERRIDE_BLINK_TIME = Seconds.of(0.25);
    public static final LEDPattern PATTERN_L1 = LEDPattern.solid(Color.fromHSV(110, 240, 120));
    public static final LEDPattern PATTERN_L2 = LEDPattern.solid(Color.fromHSV(148, 255, LED_BRIGHTNESS));
    public static final LEDPattern PATTERN_L3 = LEDPattern.solid(Color.fromHSV(13, 255, LED_BRIGHTNESS));
    public static final LEDPattern PATTERN_L4 = LEDPattern.solid(Color.fromHSV(58, 255, LED_BRIGHTNESS));
    public static final LEDPattern PATTERN_BARGE = LEDPattern.solid(Color.fromHSV(0, 255, LED_BRIGHTNESS));
    public static final LEDPattern PATTERN_INTAKE = LEDPattern.solid(Color.fromHSV(255, 0, LED_BRIGHTNESS));
    public static final LEDPattern DEVIATION_GRADIENT_PATTERN =
            LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kGreen);
}
