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
 * Class to log persistent alerts based on the faults and warnings of a SPARK MAX or SPARK Flex motor controller. For
 * full functionality, ensure that the update method is called periodically.
 */
public class SparkAlertLogger {
    final SparkBase spark;

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
     * Creates a new SparkAlertLogger for the specified SPARK MAX or SPARK Flex motor controller. The alerts will be
     * logged to the specified path. Messages will be prefixed with the specified identifier.
     *
     * @param spark The SPARK MAX or SPARK Flex motor controller to log alerts for.
     * @param logPath The path to log the alerts to.
     * @param identifier The identifier to prefix the alert messages with.
     */
    public SparkAlertLogger(SparkBase spark, String logPath, String identifier) {
        this.spark = spark;

        canFaultAlert = new Alert(logPath, identifier + " CAN Fault", Alert.AlertType.kError);
        sparkEepromFaultAlert = new Alert(logPath, identifier + " SPARK EEPROM Fault", Alert.AlertType.kError);
        firmwareFaultAlert = new Alert(logPath, identifier + " Firmware Fault", Alert.AlertType.kError);
        gateDriverFaultAlert = new Alert(logPath, identifier + " Gate Driver Fault", Alert.AlertType.kError);
        motorTypeFaultAlert = new Alert(logPath, identifier + " Motor Type Fault", Alert.AlertType.kError);
        otherFaultAlert = new Alert(logPath, identifier + " Other Fault", Alert.AlertType.kError);
        sensorFaultAlert = new Alert(logPath, identifier + " Sensor Fault", Alert.AlertType.kError);
        temperatureFaultAlert = new Alert(logPath, identifier + " Temperature Fault", Alert.AlertType.kError);

        brownoutWarningAlert = new Alert(logPath, identifier + " Brownout Warning", Alert.AlertType.kWarning);
        sparkEepromWarningAlert = new Alert(logPath, identifier + " SPARK EEPROM Warning", Alert.AlertType.kWarning);
        externalEepromWarningAlert =
                new Alert(logPath, identifier + " External EEPROM Warning", Alert.AlertType.kWarning);
        hasResetWarningAlert = new Alert(logPath, identifier + " Has Reset Warning", Alert.AlertType.kWarning);
        otherWarningAlert = new Alert(logPath, identifier + " Other Warning", Alert.AlertType.kWarning);
        overcurrentWarningAlert = new Alert(logPath, identifier + " Overcurrent Warning", Alert.AlertType.kWarning);
        sensorWarningAlert = new Alert(logPath, identifier + " Sensor Warning", Alert.AlertType.kWarning);
        stallWarningAlert = new Alert(logPath, identifier + " Stall Warning", Alert.AlertType.kWarning);
    }

    /**
     * Creates a new SparkAlertLogger for the specified SPARK MAX or SPARK Flex motor controller. The alerts will be
     * logged to the specified path. Messages will be prefixed with the identifier [SPARK (device ID)].
     *
     * @param spark The SPARK MAX or SPARK Flex motor controller to log alerts for.
     * @param logPath The path to log the alerts to.
     */
    public SparkAlertLogger(SparkBase spark, String logPath) {
        this(spark, logPath, "[SPARK " + spark.getDeviceId() + "]");
    }

    /**
     * Creates a new SparkAlertLogger for the specified SPARK MAX or SPARK Flex motor controller. The alerts will be
     * logged to the "Alerts/[device ID]" path. Messages will be prefixed with the identifier [SPARK (device ID)].
     *
     * @param spark The SPARK MAX or SPARK Flex motor controller to log alerts for.
     */
    public SparkAlertLogger(SparkBase spark) {
        this(spark, "Alerts/" + spark.getDeviceId());
    }

    /**
     * Updates the alerts based on the current faults and warnings of the SPARK MAX or SPARK Flex motor controller. This
     * method should be called periodically to ensure that the alerts are up-to-date.
     */
    public void update() {
        canFaultAlert.set(spark.getFaults().can);
        sparkEepromFaultAlert.set(spark.getFaults().escEeprom);
        firmwareFaultAlert.set(spark.getFaults().firmware);
        gateDriverFaultAlert.set(spark.getFaults().gateDriver);
        motorTypeFaultAlert.set(spark.getFaults().motorType);
        otherFaultAlert.set(spark.getFaults().other);
        sensorFaultAlert.set(spark.getFaults().sensor);
        temperatureFaultAlert.set(spark.getFaults().temperature);

        brownoutWarningAlert.set(spark.getWarnings().brownout);
        sparkEepromWarningAlert.set(spark.getWarnings().escEeprom);
        externalEepromWarningAlert.set(spark.getWarnings().extEeprom);
        hasResetWarningAlert.set(spark.getWarnings().hasReset);
        otherWarningAlert.set(spark.getWarnings().other);
        overcurrentWarningAlert.set(spark.getWarnings().overcurrent);
        sensorWarningAlert.set(spark.getWarnings().sensor);
        stallWarningAlert.set(spark.getWarnings().stall);
    }
}
