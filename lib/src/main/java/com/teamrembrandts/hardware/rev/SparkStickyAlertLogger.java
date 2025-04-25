/*
 * Copyright (c) 2024-2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.hardware.rev;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Alert;

/**
 * Class to log persistent alerts based on the sticky faults and warnings of a SPARK MAX or SPARK Flex motor controller.
 * For full functionality, ensure that the update method is called periodically.
 */
public class SparkStickyAlertLogger extends SparkAlertLogger {
    private final Alert canFaultAlert;
    private final Alert sparkEepromFaultAlert;
    private final Alert firmwareFaultAlert;
    private final Alert gateDriverFaultAlert;
    private final Alert motorTypeFaultAlert;
    private final Alert otherFaultAlert;
    private final Alert sensorFaultAlert;
    private final Alert temperatureFaultAlert;

    private final Alert brownoutWarningAlert;
    private final Alert sparkEepromWarningAlert;
    private final Alert externalEepromWarningAlert;
    private final Alert hasResetWarningAlert;
    private final Alert otherWarningAlert;
    private final Alert overcurrentWarningAlert;
    private final Alert sensorWarningAlert;
    private final Alert stallWarningAlert;

    /**
     * Creates a new SparkStickyAlertLogger for the specified SPARK MAX or SPARK Flex motor controller. The alerts will
     * be logged to the specified path. Messages will be prefixed with the specified identifier.
     *
     * @param spark The SPARK MAX or SPARK Flex motor controller to log alerts for.
     * @param logPath The path to log the alerts to.
     * @param identifier The identifier to prefix the alert messages with.
     */
    public SparkStickyAlertLogger(SparkBase spark, String logPath, String identifier) {
        super(spark, logPath, identifier);

        canFaultAlert = new Alert(logPath, identifier + " Sticky CAN Fault", Alert.AlertType.kError);
        sparkEepromFaultAlert = new Alert(logPath, identifier + " Sticky SPARK EEPROM Fault", Alert.AlertType.kError);
        firmwareFaultAlert = new Alert(logPath, identifier + " Sticky Firmware Fault", Alert.AlertType.kError);
        gateDriverFaultAlert = new Alert(logPath, identifier + " Sticky Gate Driver Fault", Alert.AlertType.kError);
        motorTypeFaultAlert = new Alert(logPath, identifier + " Sticky Motor Type Fault", Alert.AlertType.kError);
        otherFaultAlert = new Alert(logPath, identifier + " Sticky Other Fault", Alert.AlertType.kError);
        sensorFaultAlert = new Alert(logPath, identifier + " Sticky Sensor Fault", Alert.AlertType.kError);
        temperatureFaultAlert = new Alert(logPath, identifier + " Sticky Temperature Fault", Alert.AlertType.kError);

        brownoutWarningAlert = new Alert(logPath, identifier + " Sticky Brownout Warning", Alert.AlertType.kWarning);
        sparkEepromWarningAlert =
                new Alert(logPath, identifier + " Sticky SPARK EEPROM Warning", Alert.AlertType.kWarning);
        externalEepromWarningAlert =
                new Alert(logPath, identifier + " Sticky External EEPROM Warning", Alert.AlertType.kWarning);
        hasResetWarningAlert = new Alert(logPath, identifier + " Sticky Has Reset Warning", Alert.AlertType.kWarning);
        otherWarningAlert = new Alert(logPath, identifier + " Sticky Other Warning", Alert.AlertType.kWarning);
        overcurrentWarningAlert =
                new Alert(logPath, identifier + " Sticky Overcurrent Warning", Alert.AlertType.kWarning);
        sensorWarningAlert = new Alert(logPath, identifier + " Sticky Sensor Warning", Alert.AlertType.kWarning);
        stallWarningAlert = new Alert(logPath, identifier + " Sticky Stall Warning", Alert.AlertType.kWarning);
    }

    /**
     * Creates a new SparkStickyAlertLogger for the specified SPARK MAX or SPARK Flex motor controller. The alerts will
     * be logged to the specified path. Messages will be prefixed with the identifier [SPARK (device ID)].
     *
     * @param spark The SPARK MAX or SPARK Flex motor controller to log alerts for.
     * @param logPath The path to log the alerts to.
     */
    public SparkStickyAlertLogger(SparkBase spark, String logPath) {
        this(spark, logPath, "[SPARK " + spark.getDeviceId() + "]");
    }

    /**
     * Creates a new SparkStickyAlertLogger for the specified SPARK MAX or SPARK Flex motor controller. The alerts will
     * be logged to the "Alerts/[device ID]" path. Messages will be prefixed with the identifier [SPARK (device ID)].
     *
     * @param spark The SPARK MAX or SPARK Flex motor controller to log alerts for.
     */
    public SparkStickyAlertLogger(SparkBase spark) {
        this(spark, "Alerts/" + spark.getDeviceId());
    }

    /**
     * Updates the alerts based on the current sticky faults and warnings of the SPARK MAX or SPARK Flex motor
     * controller. This method should be called periodically to ensure that the alerts are up-to-date.
     */
    @Override
    public void update() {
        super.update();

        canFaultAlert.set(spark.getStickyFaults().can);
        sparkEepromFaultAlert.set(spark.getStickyFaults().escEeprom);
        firmwareFaultAlert.set(spark.getStickyFaults().firmware);
        gateDriverFaultAlert.set(spark.getStickyFaults().gateDriver);
        motorTypeFaultAlert.set(spark.getStickyFaults().motorType);
        otherFaultAlert.set(spark.getStickyFaults().other);
        sensorFaultAlert.set(spark.getStickyFaults().sensor);
        temperatureFaultAlert.set(spark.getStickyFaults().temperature);

        brownoutWarningAlert.set(spark.getStickyWarnings().brownout);
        sparkEepromWarningAlert.set(spark.getStickyWarnings().escEeprom);
        externalEepromWarningAlert.set(spark.getStickyWarnings().extEeprom);
        hasResetWarningAlert.set(spark.getStickyWarnings().hasReset);
        otherWarningAlert.set(spark.getStickyWarnings().other);
        overcurrentWarningAlert.set(spark.getStickyWarnings().overcurrent);
        sensorWarningAlert.set(spark.getStickyWarnings().sensor);
        stallWarningAlert.set(spark.getStickyWarnings().stall);
    }
}
