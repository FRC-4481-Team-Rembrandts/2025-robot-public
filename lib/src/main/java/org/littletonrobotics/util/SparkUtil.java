/*
 * Copyright 2021-2024 FRC 6328
 * http://github.com/Mechanical-Advantage
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 *
 * We're borrowing this one 🧡.
 */
package org.littletonrobotics.util;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SparkUtil {
    /** Stores whether any error was has been detected by other utility methods. */
    public static boolean sparkStickyFault = false;

    /** Processes a value from a Spark only if the value is valid.
     * @param spark The SPARK motor controller to check for errors.
     * @param supplier The supplier of the value to process.
     * @param consumer The consumer of the value to process in case the SPARK returns OK.
     */
    public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
        double value = supplier.getAsDouble();
        if (spark.getLastError() == REVLibError.kOk) {
            consumer.accept(value);
        } else {
            sparkStickyFault = true;
        }
    }

    /** Processes a value from a Spark only if the value is valid.
     * @param spark The SPARK motor controller to check for errors.
     * @param suppliers The suppliers of the values to process.
     * @param consumers The consumers of the values to process in case the SPARK returns OK.
     */
    public static void ifOk(SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumers) {
        double[] values = new double[suppliers.length];
        for (int i = 0; i < suppliers.length; i++) {
            values[i] = suppliers[i].getAsDouble();
            if (spark.getLastError() != REVLibError.kOk) {
                sparkStickyFault = true;
                return;
            }
        }
        consumers.accept(values);
    }

    /** Attempts to run the command until no error is produced.
     * @param spark The SPARK motor controller to check for errors.
     * @param maxAttempts The maximum number of attempts to run the command.
     * @param command The command to run.
     */
    public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error == REVLibError.kOk) {
                break;
            } else {
                sparkStickyFault = true;
            }
        }
    }
}
