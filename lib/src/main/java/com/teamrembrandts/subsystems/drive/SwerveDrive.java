/*
 * Copyright (c) 2024-2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.subsystems.drive;

import com.teamrembrandts.math.kinematics.*;
import com.teamrembrandts.subsystems.vision.VisionMeasurement;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

/** A subsystem for controlling a swerve drivetrain. */
public class SwerveDrive extends Drive {

    private final SwerveModule[] modules;
    private final NthOrderSwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final double trackWidthX;
    private final double trackWidthY;
    private final double maxVelocity;
    private final double maxAcceleration;
    private final double maxAngularVelocity;

    private final SwerveModulePosition[] previousPositions;
    private final SwerveModulePosition[] adjustedPositions;
    private final SwerveModulePosition[] previousAdjustedPositions;
    private final SwerveModulePosition[] modulePositions;
    private ChassisState currentTargetState = new ChassisState();
    private Rotation2d rawGyroRotation = new Rotation2d();

    public final Supplier<Pose2d> robotPoseSupplier;
    public final Consumer<VisionMeasurement> visionMeasurementConsumer;
    private double lastResetTime = -1.0;

    /**
     * Creates a new swerve drive subsystem.
     *
     * @param trackWidthX The horizontal distance between the wheels.
     * @param trackWidthY The vertical distance between the wheels.
     * @param maxVelocity The maximum linear velocity of the drivetrain.
     * @param maxAcceleration The maximum linear acceleration of the drivetrain.
     * @param maxAngularVelocity The maximum angular velocity of the drivetrain.
     * @param odometryThreadFrequency The frequency of the odometry thread.
     * @param imuIO The input/output that measures the robot angle.
     * @param moduleIOs The input/outputs for the individual swerve modules.
     */
    public SwerveDrive(
            double trackWidthX,
            double trackWidthY,
            double maxVelocity,
            double maxAcceleration,
            double maxAngularVelocity,
            double odometryThreadFrequency,
            IMUIO imuIO,
            SwerveModuleIO... moduleIOs) {
        super(imuIO, odometryThreadFrequency);

        this.trackWidthX = trackWidthX;
        this.trackWidthY = trackWidthY;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxAngularVelocity = maxAngularVelocity;

        modules = new SwerveModule[moduleIOs.length];
        modulePositions = new SwerveModulePosition[moduleIOs.length];
        previousPositions = new SwerveModulePosition[moduleIOs.length];
        previousAdjustedPositions = new SwerveModulePosition[moduleIOs.length];
        adjustedPositions = new SwerveModulePosition[moduleIOs.length];

        for (int i = 0; i < modules.length; i++) {
            modules[i] = new SwerveModule(moduleIOs[i], i);
            modulePositions[i] = new SwerveModulePosition();
            previousPositions[i] = new SwerveModulePosition();
            previousAdjustedPositions[i] = new SwerveModulePosition();
            adjustedPositions[i] = new SwerveModulePosition();
        }

        kinematics = new SecondOrderSwerveDriveKinematics(getModuleTranslations());
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, modulePositions, new Pose2d());

        // Define the consumers and suppliers
        robotPoseSupplier = poseEstimator::getEstimatedPosition;
        visionMeasurementConsumer = (measurement) -> poseEstimator.addVisionMeasurement(
                measurement.robotPose(), measurement.timestamp(), measurement.stdDevs());
    }

    @Override
    public void periodic() {
        // Lock the mutex to update the IOs which use the odometry thread
        ODOMETRY_LOCK.lock();
        super.periodic();
        for (SwerveModule module : modules) {
            module.periodic();
        }
        ODOMETRY_LOCK.unlock();

        Logger.processInputs("Swerve/IMU", imuInputs);

        updateOdometry();

        // Log empty data on disable
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }
    }

    /**
     * Update the pose estimator using the multiple results. These results are gathered by the notifier threads reading
     * the encoders
     */
    private void updateOdometry() {
        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[modules.length];
            for (int j = 0; j < modules.length; j++) {
                modulePositions[j] = modules[j].getOdometryPositions()[i];
                moduleDeltas[j] = new SwerveModulePosition(
                        modulePositions[j].distanceMeters - previousPositions[j].distanceMeters,
                        modulePositions[j].angle);
                // Calculate if velocity and acceleration are in line
                double direction = normalizedDotProduct(
                        currentTargetState.getSpeeds().vxMetersPerSecond,
                        currentTargetState.getSpeeds().vyMetersPerSecond,
                        currentTargetState.getAccelerations().ax,
                        currentTargetState.getAccelerations().ay);
                double accelerationMagnitude =
                        Math.hypot(currentTargetState.getAccelerations().ax, currentTargetState.getAccelerations().ay);
                double slipFactor = direction * calculateSlipFactor(accelerationMagnitude);
                Logger.recordOutput("Swerve/SlipFactor", slipFactor);
                adjustedPositions[j] = new SwerveModulePosition(
                        previousAdjustedPositions[j].distanceMeters + moduleDeltas[j].distanceMeters * (1 - slipFactor),
                        modulePositions[j].angle);
                previousAdjustedPositions[j] = adjustedPositions[j];
                previousPositions[j] = modulePositions[j];
            }
            // Update gyro angle
            if (imuInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = imuInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }
            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, adjustedPositions);
        }
        // Check if the wheels are not fighting each other
        Logger.recordOutput("SwerveStates/Kinematic Error", kinematics.getKinematicError(getSwerveModuleStates()));
    }

    /**
     * Sets the target speeds for the swerve drivetrain. Speeds should be robot relative.
     *
     * @param targetSpeeds The target speeds for the swerve drivetrain.
     */
    @Override
    protected void setTargetSpeeds(ChassisSpeeds targetSpeeds) {
        setTargetState(new ChassisState(targetSpeeds, new ChassisAccelerations()));
    }

    /**
     * Sets the target speeds and acceleration for the swerve drivetrain. ChassisState should be robot relative.
     *
     * @param targetState The target speeds for the swerve drivetrain.
     */
    @Override
    protected void setTargetState(ChassisState targetState) {
        currentTargetState = targetState;

        // Calculate module setpoints
        NthOrderSwerveModuleState[] setpointStates = kinematics.toDesaturatedSwerveModuleStates(
                targetState, getPose().getRotation(), getMaxVelocity());

        SwerveModuleState[] logOptimizedStates = new SwerveModuleState[modules.length];

        // Send data to individual modules
        for (int i = 0; i < modules.length; i++) {
            setpointStates[i].optimize(modules[i].getState().angle);
            modules[i].setTargetState(setpointStates[i]);
            logOptimizedStates[i] = setpointStates[i].toSwerveModuleState();
            Logger.recordOutput(
                    "SwerveStates/Second Order/Acceleration/" + i,
                    setpointStates[i].acceleration.in(Units.MetersPerSecondPerSecond));
            Logger.recordOutput(
                    "SwerveStates/Second Order/AngularVelocity/" + i,
                    setpointStates[i].angularVelocity.in(Units.RadiansPerSecond));
        }
        Logger.recordOutput("SwerveStates/SetpointsOptimized", logOptimizedStates);
    }

    /**
     * Gets the field relative speeds of the swerve drivetrain.
     *
     * @return The current speeds of the swerve drivetrain.
     */
    @Override
    protected ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                getRobotRelativeSpeeds(), poseEstimator.getEstimatedPosition().getRotation());
    }

    /**
     * Gets the robot relative speeds of the swerve drivetrain.
     *
     * @return The current speeds of the swerve drivetrain.
     */
    @Override
    protected ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    /**
     * Gets the current pose of the swerve drivetrain.
     *
     * @return The current pose of the swerve drivetrain.
     */
    @AutoLogOutput(key = "Odometry/Robot")
    @Override
    protected Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Gets the current states of the swerve modules, which is the velocity and the angle.
     *
     * @return The current states of the individual models.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Retrieve the maximum velocity of the drivetrain.
     *
     * @return The maximum velocity.
     */
    protected double getMaxVelocity() {
        return maxVelocity;
    }

    /**
     * Retrieve the maximum angular velocity of the drivetrain.
     *
     * @return The maximum angular velocity.
     */
    protected double getMaxAngularVelocity() {
        return maxAngularVelocity;
    }

    /**
     * Retrieve the maximum acceleration of the drivetrain.
     *
     * @return The maximum acceleration.
     */
    protected double getMaxAcceleration() {
        return maxAcceleration;
    }

    /**
     * Retrieve the maximum angular acceleration of the drivetrain.
     *
     * @return The maximum angular acceleration.
     */
    protected double getMaxAngularAcceleration() {
        // Approximation of the radius of the drivetrain
        // Correct if drivetrain is a square
        double driveRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);

        // Convert m/s to rad/s
        return getMaxAcceleration() / driveRadius;
    }

    /**
     * Gets the individual translations of the swerve modules.
     *
     * @return The individual translations of the swerve modules.
     */
    private Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(trackWidthX / 2, trackWidthY / 2),
            new Translation2d(trackWidthX / 2, -trackWidthY / 2),
            new Translation2d(-trackWidthX / 2, trackWidthY / 2),
            new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
        };
    }

    /**
     * Gets the current position for the swerve modules.
     *
     * @return The current positions for the swerve modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    @Override
    public Supplier<Pose2d> getRobotPoseSupplier() {
        return this::getPose;
    }

    @Override
    public Consumer<VisionMeasurement> getVisionMeasurementConsumer() {
        return (measurement) -> {
            if (measurement.timestamp() > lastResetTime)
                poseEstimator.addVisionMeasurement(
                        measurement.robotPose(), measurement.timestamp(), measurement.stdDevs());
        };
    }

    @Override
    public Supplier<ChassisSpeeds> getFieldRelativeSpeedsSupplier() {
        return this::getFieldRelativeSpeeds;
    }

    @Override
    public Supplier<ChassisSpeeds> getRobotRelativeSpeedsSupplier() {
        return this::getRobotRelativeSpeeds;
    }

    @Override
    public Command joystickDrive(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, double deadband) {
        return Commands.run(
                () -> {
                    // Apply deadband
                    double linearMagnitude = MathUtil.applyDeadband(
                            Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), deadband);
                    Rotation2d linearDirection = linearMagnitude > 0
                            ? new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble())
                            : new Rotation2d();
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), deadband);

                    // Square values
                    linearMagnitude = linearMagnitude * linearMagnitude;
                    omega = Math.copySign(omega * omega, omega);

                    // Calculate new linear velocity
                    Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
                            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                            .getTranslation();

                    // Convert to field relative speeds & send command
                    boolean isFlipped = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
                    this.setTargetSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                            linearVelocity.getX() * this.getMaxVelocity(),
                            linearVelocity.getY() * this.getMaxVelocity(),
                            omega * this.getMaxAngularVelocity(),
                            isFlipped
                                    ? this.getPose().getRotation().plus(new Rotation2d(Math.PI))
                                    : this.getPose().getRotation()));
                },
                this);
    }

    @Override
    public Command setRobotPose(Supplier<Pose2d> poseSupplier) {
        return Commands.runOnce(() -> {
            // Wait 6 code cycles before accepting poses again, this has been verified by means of simulation
            lastResetTime = Logger.getTimestamp() / 1e6 + LoggedRobot.defaultPeriodSecs * 6;
            poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), poseSupplier.get());
        });
    }

    @Override
    protected void runCharacterization(double voltage) {
        for (SwerveModule module : modules) {
            module.setDriveOpenLoop(voltage);
            module.setTurnAngle(new Rotation2d());
        }
    }

    @Override
    protected double getFFCharacterizationVelocity() {
        double averageVelocity = 0.0;
        for (SwerveModule module : modules) {
            averageVelocity += module.getDriveVelocity();
        }
        return averageVelocity / modules.length;
    }

    @Override
    protected List<Double> getWheelRadiusCharacterizationPositions() {
        List<Double> positions = new LinkedList<>();
        for (SwerveModule module : modules) {
            positions.add(module.getDriveDistance());
        }
        return positions;
    }

    /**
     * Calculate the slip factor based on the robot acceleration.
     *
     * @param robotAcceleration The robot acceleration.
     * @return The slip factor.
     */
    private double calculateSlipFactor(double robotAcceleration) {
        double maxTheoreticalAcceleration = 10;
        double curveSlope = 2.2; // Higher slope means lower slip
        double epsilon = 1e-3;
        // Make sure the input to the arctanh function is between -1 and 1
        double scaledAccel = MathUtil.clamp(robotAcceleration / maxTheoreticalAcceleration, -1 + epsilon, 1 - epsilon);
        // Calculate the fraction of the wheel velocity that is caused by slip using the arctanh function
        double slipFactor = 1 / curveSlope * Math.log((1 + scaledAccel) / (1 - scaledAccel)) / 2;
        return MathUtil.clamp(slipFactor, -1, 1);
    }

    /**
     * Calculate the normalized dot product of two vectors based on their individual components.
     *
     * @param x1 The x component of the first vector.
     * @param y1 The y component of the first vector.
     * @param x2 The x component of the second vector.
     * @param y2 The y component of the second vector.
     * @return The normalized dot product of the two vectors.
     */
    private double normalizedDotProduct(double x1, double y1, double x2, double y2) {
        double mag1 = Math.hypot(x1, y1);
        double mag2 = Math.hypot(x2, y2);
        if (mag1 == 0 || mag2 == 0) {
            return 0;
        }
        return (x1 * x2 + y1 * y2) / (mag1 * mag2);
    }
}
