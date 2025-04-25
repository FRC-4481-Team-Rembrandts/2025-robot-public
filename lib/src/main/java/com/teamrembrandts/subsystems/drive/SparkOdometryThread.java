/*
 * Copyright (c) 2021-2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.subsystems.drive;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version includes an overload for Spark signals, which checks for errors to ensure that all measurements in
 * the sample are valid.
 */
public class SparkOdometryThread {
    private final List<SparkBase> sparks = new ArrayList<>();
    private final List<DoubleSupplier> sparkSignals = new ArrayList<>();
    private final List<DoubleSupplier> genericSignals = new ArrayList<>();
    private final List<Queue<Double>> sparkQueues = new ArrayList<>();
    private final List<Queue<Double>> genericQueues = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();

    private static SparkOdometryThread instance = null;
    private final Notifier notifier = new Notifier(this::run);

    /**
     * Returns the singleton instance of the SparkOdometryThread.
     *
     * @return The singleton instance of the SparkOdometryThread.
     */
    public static SparkOdometryThread getInstance() {
        if (instance == null) {
            instance = new SparkOdometryThread();
        }
        return instance;
    }

    /** Creates a new SparkOdometryThread. */
    private SparkOdometryThread() {
        notifier.setName("OdometryThread");
    }

    /**
     * Starts the thread with the specified loop period.
     *
     * @param looperDt The loop period in seconds.
     */
    public void start(double looperDt) {
        if (!timestampQueues.isEmpty()) {
            notifier.startPeriodic(looperDt);
        }
    }

    /**
     * Registers a Spark signal to be read from the thread.
     *
     * @param spark The Spark motor controller to read from.
     * @param signal The signal to read from the Spark.
     * @return A queue containing the signal values.
     */
    public Queue<Double> registerSignal(SparkBase spark, DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drive.ODOMETRY_LOCK.lock();
        try {
            sparks.add(spark);
            sparkSignals.add(signal);
            sparkQueues.add(queue);
        } finally {
            Drive.ODOMETRY_LOCK.unlock();
        }
        return queue;
    }

    /**
     * Registers a generic signal to be read from the thread.
     *
     * @param signal The signal to read.
     * @return A queue containing the signal values.
     */
    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drive.ODOMETRY_LOCK.lock();
        try {
            genericSignals.add(signal);
            genericQueues.add(queue);
        } finally {
            Drive.ODOMETRY_LOCK.unlock();
        }
        return queue;
    }

    /**
     * Returns a new queue that returns timestamp values for each sample.
     *
     * @return A queue containing the timestamp values.
     */
    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drive.ODOMETRY_LOCK.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            Drive.ODOMETRY_LOCK.unlock();
        }
        return queue;
    }

    /** Runs the thread, reading all signals and saving them to the queues. */
    private void run() {
        // Save new data to queues
        Drive.ODOMETRY_LOCK.lock();
        try {
            // Get sample timestamp
            double timestamp = RobotController.getFPGATime() / 1e6;

            // Read Spark values, mark invalid in case of error
            double[] sparkValues = new double[sparkSignals.size()];
            boolean isValid = true;
            for (int i = 0; i < sparkSignals.size(); i++) {
                sparkValues[i] = sparkSignals.get(i).getAsDouble();
                if (sparks.get(i).getLastError() != REVLibError.kOk) {
                    isValid = false;
                }
            }

            // If valid, add values to queues
            if (isValid) {
                for (int i = 0; i < sparkSignals.size(); i++) {
                    sparkQueues.get(i).offer(sparkValues[i]);
                }
                for (int i = 0; i < genericSignals.size(); i++) {
                    genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
                }
                for (Queue<Double> timestampQueue : timestampQueues) {
                    timestampQueue.offer(timestamp);
                }
            }
        } finally {
            Drive.ODOMETRY_LOCK.unlock();
        }
    }
}
