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

import static edu.wpi.first.units.Units.*;

import com.teamrembrandts.math.kinematics.ChassisState;
import com.teamrembrandts.math.trajectory.HolonomicTrajectory;
import com.teamrembrandts.math.trajectory.TrajectoryFollower;
import com.teamrembrandts.subsystems.vision.VisionMeasurement;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * A subsystem that represents a drivetrain. Can be used to create commands that control the drivetrain without needing
 * to know the specifics.
 */
public abstract class Drive extends SubsystemBase {
    static final Lock ODOMETRY_LOCK = new ReentrantLock();

    private final IMUIO imuIO;
    final IMUInputsAutoLogged imuInputs = new IMUInputsAutoLogged();
    private final Alert imuDisconnectedWarning =
            new Alert("Alerts/Drive", "[IMU] Disconnected, using odometry as fallback", Alert.AlertType.kWarning);

    /**
     * Create a new drive
     *
     * @param imuIO The IO for the IMU of the drive
     * @param odometryLooperFrequency The looper periodic time in seconds of the odometry thread
     */
    public Drive(IMUIO imuIO, double odometryLooperFrequency) {
        super();
        this.imuIO = imuIO;
        // Start threads (no-op if no signals have been created)
        SparkOdometryThread.getInstance().start(1.0 / odometryLooperFrequency);
    }

    @Override
    public void periodic() {
        imuIO.updateInputs(imuInputs);

        SmartDashboard.putBoolean("Connection Status/IMU", imuInputs.connected);
        imuDisconnectedWarning.set(imuInputs.connected);
    }

    /**
     * Sets the target speeds for the drivetrain. Speeds should be robot relative.
     *
     * @param targetSpeeds The target speeds for the drivetrain.
     */
    protected abstract void setTargetSpeeds(ChassisSpeeds targetSpeeds);

    /**
     * Sets the target chassis state (velocity plus acceleration) for the drivetrain. The ChassisState should be
     * robotrelative
     *
     * @param targetState The target state for the drivetrain.
     */
    protected abstract void setTargetState(ChassisState targetState);

    /**
     * Gets the robot relative speeds of the drivetrain.
     *
     * @return The robot relative speeds of the drivetrain.
     */
    protected abstract ChassisSpeeds getRobotRelativeSpeeds();

    /**
     * Gets the field relative speeds of the drivetrain.
     *
     * @return The field relative speeds of the drivetrain.
     */
    protected abstract ChassisSpeeds getFieldRelativeSpeeds();

    /**
     * Gets the pose of the drivetrain.
     *
     * @return The pose of the drivetrain.
     */
    protected abstract Pose2d getPose();

    /**
     * Gets the maximum velocity of the drivetrain.
     *
     * @return The maximum velocity of the drivetrain.
     */
    protected abstract double getMaxVelocity();

    /**
     * Gets the maximum angular velocity of the drivetrain.
     *
     * @return The maximum angular velocity of the drivetrain.
     */
    protected abstract double getMaxAngularVelocity();

    /**
     * Gets the maximum acceleration of the drivetrain.
     *
     * @return The maximum acceleration of the drivetrain.
     */
    protected abstract double getMaxAcceleration();

    /**
     * Gets the maximum angular acceleration of the drivetrain.
     *
     * @return The maximum angular acceleration of the drivetrain.
     */
    protected abstract double getMaxAngularAcceleration();

    /**
     * Get the supplier for the current robot pose
     *
     * @return The supplier for the current robot pose
     */
    public abstract Supplier<Pose2d> getRobotPoseSupplier();

    /**
     * Get the consumer for the vision measurements, this is used by the vision subsystem to pass measurements to the
     * drive subsystem
     *
     * @return The consumer for the vision measurements
     */
    public abstract Consumer<VisionMeasurement> getVisionMeasurementConsumer();

    /**
     * Get the supplier for the field relative speeds
     *
     * @return The supplier for the field relative speeds
     */
    public abstract Supplier<ChassisSpeeds> getFieldRelativeSpeedsSupplier();

    /**
     * Get the supplier for the robot relative speeds
     *
     * @return The supplier for the robot relative speeds
     */
    public abstract Supplier<ChassisSpeeds> getRobotRelativeSpeedsSupplier();

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     *
     * @param xSupplier Supplier for the x velocity of the robot (forward)
     * @param ySupplier Supplier for the y velocity of the robot (sideways, left positive)
     * @param omegaSupplier Supplier for the angular velocity of the robot (counter-clockwise positive)
     * @param deadband The deadband for the joysticks
     * @return Command that drives the robot using joystick inputs
     */
    public abstract Command joystickDrive(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, double deadband);

    /**
     * Sets the robot to a specified pose.
     *
     * @param poseSupplier The pose to reset to.
     * @return Command that resets the robot to a specified pose.
     */
    public abstract Command setRobotPose(Supplier<Pose2d> poseSupplier);

    /**
     * sets the targetSpeeds for a drivetrain to follow a trajectory.
     *
     * @param trajectory Supplier for the trajectory to follow.
     * @param translationStaticKp Translation static kP.
     * @param translationDynamicKp Translation dybnamic kP.
     * @param rotationKp Rotatoin kP
     * @param distanceTolerance The linear tolerance to the target in meters.
     * @param angularTolerance The angular tolerance to the target in radians.
     * @return a command that follows a given trajectory and interrupts when the target is reached.
     */
    public Command followTrajectory(
            Supplier<HolonomicTrajectory> trajectory,
            double translationStaticKp,
            double translationDynamicKp,
            double rotationKp,
            double distanceTolerance,
            double angularTolerance) {
        TrajectoryFollower follower = new TrajectoryFollower(
                trajectory,
                getRobotPoseSupplier(),
                this::getRobotRelativeSpeeds,
                distanceTolerance,
                angularTolerance,
                translationStaticKp,
                translationDynamicKp,
                rotationKp);
        return Commands.sequence(
                Commands.runOnce(follower::start),
                setTargetStateCommand(follower::getTargetState).until(new Trigger(follower::isFinished)),
                Commands.runOnce(() -> setTargetSpeeds(new ChassisSpeeds(0, 0, 0))));
    }

    /**
     * sets the targetSpeeds for a drivetrain to follow a trajectory.
     *
     * @param trajectory Supplier for the trajectory to follow.
     * @param translationStaticKp Translation static kP.
     * @param translationDynamicKp Translation dybnamic kP.
     * @param rotationKp Rotatoin kP
     * @param distanceTolerance The linear tolerance to the target in meters.
     * @param angularTolerance The angular tolerance to the target in radians.
     * @param timeout the timeout to finish after the profile is finished
     * @return a command that follows a given trajectory and interrupts when the target is reached.
     */
    public Command followTrajectory(
            Supplier<HolonomicTrajectory> trajectory,
            double translationStaticKp,
            double translationDynamicKp,
            double rotationKp,
            double distanceTolerance,
            double angularTolerance,
            double timeout) {
        TrajectoryFollower follower = new TrajectoryFollower(
                trajectory,
                getRobotPoseSupplier(),
                this::getRobotRelativeSpeeds,
                distanceTolerance,
                angularTolerance,
                translationStaticKp,
                translationDynamicKp,
                rotationKp);
        return Commands.sequence(
                Commands.runOnce(follower::start),
                setTargetStateCommand(follower::getTargetState)
                        .until(new Trigger(follower::isFinished)
                                .or(new Trigger(follower::isProfileFinished).debounce(timeout))),
                Commands.runOnce(() -> setTargetSpeeds(new ChassisSpeeds(0, 0, 0))));
    }

    /**
     * sets the targetSpeeds for a drivetrain to follow a trajectory.
     *
     * @param trajectory The trajectory to follow.
     * @param translationStaticKp Translation static kP.
     * @param translationDynamicKp Translation dybnamic kP.
     * @param rotationKp Rotation kP
     * @param distanceTolerance The linear tolerance to the target in meters.
     * @param angularTolerance The angular tolerance to the target in radians.
     * @return a command that follows a given trajectory and interrupts when the target is reached.
     */
    public Command followTrajectory(
            HolonomicTrajectory trajectory,
            double translationStaticKp,
            double translationDynamicKp,
            double rotationKp,
            double distanceTolerance,
            double angularTolerance) {
        return followTrajectory(
                () -> trajectory,
                translationStaticKp,
                translationDynamicKp,
                rotationKp,
                distanceTolerance,
                angularTolerance);
    }

    /**
     * sets the targetSpeeds for a drivetrain to follow a trajectory.
     *
     * @param trajectory The trajectory to follow.
     * @param translationStaticKp Translation static kP.
     * @param translationDynamicKp Translation dybnamic kP.
     * @param rotationKp Rotation kP
     * @param distanceTolerance The linear tolerance to the target in meters.
     * @param angularTolerance The angular tolerance to the target in radians.
     * @param timeout the timeout to finish after the profile is finished
     * @return a command that follows a given trajectory and interrupts when the target is reached.
     */
    public Command followTrajectory(
            HolonomicTrajectory trajectory,
            double translationStaticKp,
            double translationDynamicKp,
            double rotationKp,
            double distanceTolerance,
            double angularTolerance,
            double timeout) {
        return followTrajectory(
                () -> trajectory,
                translationStaticKp,
                translationDynamicKp,
                rotationKp,
                distanceTolerance,
                angularTolerance,
                timeout);
    }

    /**
     * sets the targetSpeeds for a drivetrain to follow a trajectory.
     *
     * @param trajectory The supplier for the trajectory to follow.
     * @param translationStaticKp Translation static kP.
     * @param translationDynamicKp Translation dybnamic kP.
     * @param rotationKp Rotation kP
     * @param distanceTolerance The linear tolerance to the target in meters.
     * @param angularTolerance The angular tolerance to the target in radians.
     * @param resetToStartPosition Whether the position of the robot should be reset to the starting position of the
     *     trajectory. This should only be done for the first trajectory in autonomous when there is no vision.
     * @return a command that follows a given trajectory and interrupts when the target is reached.
     */
    public Command followTrajectory(
            Supplier<HolonomicTrajectory> trajectory,
            double translationStaticKp,
            double translationDynamicKp,
            double rotationKp,
            double distanceTolerance,
            double angularTolerance,
            boolean resetToStartPosition) {
        if (!resetToStartPosition) {
            return followTrajectory(
                    trajectory,
                    translationStaticKp,
                    translationDynamicKp,
                    rotationKp,
                    distanceTolerance,
                    angularTolerance);
        }
        return Commands.sequence(
                setRobotPose(trajectory.get()::getInitialPose),
                followTrajectory(
                        trajectory,
                        translationStaticKp,
                        translationDynamicKp,
                        rotationKp,
                        distanceTolerance,
                        angularTolerance));
    }

    /**
     * sets the targetSpeeds for a drivetrain to follow a trajectory.
     *
     * @param trajectory The supplier for the trajectory to follow.
     * @param translationStaticKp Translation static kP.
     * @param translationDynamicKp Translation dybnamic kP.
     * @param rotationKp Rotation kP
     * @param distanceTolerance The linear tolerance to the target in meters.
     * @param angularTolerance The angular tolerance to the target in radians.
     * @param timeout the timeout to finish after the profile is finished
     * @param resetToStartPosition Whether the position of the robot should be reset to the starting position of the
     *     trajectory. This should only be done for the first trajectory in autonomous when there is no vision.
     * @return a command that follows a given trajectory and interrupts when the target is reached.
     */
    public Command followTrajectory(
            Supplier<HolonomicTrajectory> trajectory,
            double translationStaticKp,
            double translationDynamicKp,
            double rotationKp,
            double distanceTolerance,
            double angularTolerance,
            double timeout,
            boolean resetToStartPosition) {
        if (!resetToStartPosition) {
            return followTrajectory(
                    trajectory,
                    translationStaticKp,
                    translationDynamicKp,
                    rotationKp,
                    distanceTolerance,
                    angularTolerance,
                    timeout);
        }
        return Commands.sequence(
                setRobotPose(trajectory.get()::getInitialPose),
                followTrajectory(
                        trajectory,
                        translationStaticKp,
                        translationDynamicKp,
                        rotationKp,
                        distanceTolerance,
                        angularTolerance,
                        timeout));
    }

    /**
     * sets the targetSpeeds for a drivetrain to follow a trajectory.
     *
     * @param trajectory The trajectory to follow.
     * @param translationStaticKp Translation static kP.
     * @param translationDynamicKp Translation dybnamic kP.
     * @param rotationKp Rotation kP.
     * @param distanceTolerance The linear tolerance to the target in meters.
     * @param angularTolerance The angular tolerance to the target in radians.
     * @param resetToStartPosition Whether the position of the robot should be reset to the starting position of the
     *     trajectory. This should only be done for the first trajectory in autonomous when there is no vision.
     * @return a command that follows a given trajectory and interrupts when the target is reached.
     */
    public Command followTrajectory(
            HolonomicTrajectory trajectory,
            double translationStaticKp,
            double translationDynamicKp,
            double rotationKp,
            double distanceTolerance,
            double angularTolerance,
            boolean resetToStartPosition) {
        return followTrajectory(
                () -> trajectory,
                translationStaticKp,
                translationDynamicKp,
                rotationKp,
                distanceTolerance,
                angularTolerance,
                resetToStartPosition);
    }

    /**
     * sets the targetSpeeds for a drivetrain to follow a trajectory.
     *
     * @param trajectory The trajectory to follow.
     * @param translationStaticKp Translation static kP.
     * @param translationDynamicKp Translation dybnamic kP.
     * @param rotationKp Rotation kP.
     * @param distanceTolerance The linear tolerance to the target in meters.
     * @param angularTolerance The angular tolerance to the target in radians.
     * @param timeout the timeout to finish after the profile is finished
     * @param resetToStartPosition Whether the position of the robot should be reset to the starting position of the
     *     trajectory. This should only be done for the first trajectory in autonomous when there is no vision.
     * @return a command that follows a given trajectory and interrupts when the target is reached.
     */
    public Command followTrajectory(
            HolonomicTrajectory trajectory,
            double translationStaticKp,
            double translationDynamicKp,
            double rotationKp,
            double distanceTolerance,
            double angularTolerance,
            double timeout,
            boolean resetToStartPosition) {
        return followTrajectory(
                () -> trajectory,
                translationStaticKp,
                translationDynamicKp,
                rotationKp,
                distanceTolerance,
                angularTolerance,
                timeout,
                resetToStartPosition);
    }

    /**
     * Continuously sets the target speeds for the drivetrain. Speeds should be robot relative.
     *
     * @param chassisSpeeds Supplier of robot relative ChassisSpeed
     * @return Command that sets the target speeds for the drivetrain
     */
    private Command setTargetSpeedsCommand(Supplier<ChassisSpeeds> chassisSpeeds) {
        return Commands.run(() -> setTargetSpeeds(chassisSpeeds.get()), this);
    }

    /**
     * Continuously sets the target speeds for the drivetrain. Speeds should be robot relative.
     *
     * @param xSpeed Supplier of the target speed in the x direction.
     * @param ySpeed Supplier of the target speed in the y direction.
     * @param angularSpeed Supplier of the target angular speed.
     * @return Command that sets the target speeds for the drivetrain
     */
    private Command setTargetSpeedsCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier angularSpeed) {
        return Commands.run(
                () -> setTargetSpeeds(
                        new ChassisSpeeds(xSpeed.getAsDouble(), ySpeed.getAsDouble(), angularSpeed.getAsDouble())),
                this);
    }

    /**
     * Continuously sets the target speeds for the drivetrain. Speeds should be robot relative. To be used for
     * non-holonomic drivetrains.
     *
     * @param xSpeed Supplier of the target speed in the x direction.
     * @param angularSpeed Supplier of the target angular speed.
     * @return Command that sets the target speeds for the drivetrain
     */
    private Command setTargetSpeedsCommand(DoubleSupplier xSpeed, DoubleSupplier angularSpeed) {
        return Commands.run(
                () -> setTargetSpeeds(new ChassisSpeeds(xSpeed.getAsDouble(), 0, angularSpeed.getAsDouble())), this);
    }

    /**
     * Continuously sets the target state for the drivetrain. Speeds should be robot relative.
     *
     * @param targetState Supplier of the target state for the drivetrain including accelerations
     * @return Command that sets the target state for the drivetrain
     */
    private Command setTargetStateCommand(Supplier<ChassisState> targetState) {
        return Commands.run(() -> setTargetState(targetState.get()), this);
    }

    /**
     * Checks if the current position is within a certain margin
     *
     * @param translation2d The target position
     * @param positionMargin The margin for the position
     * @return Trigger for whether the drivetrain is near the target position
     */
    public Trigger onPosition(Translation2d translation2d, DoubleSupplier positionMargin) {
        return new Trigger(() -> MathUtil.isNear(
                0, getPose().getTranslation().getDistance(translation2d), positionMargin.getAsDouble()));
    }

    /**
     * Checks if the current position is within a certain margin
     *
     * @param translation2d The target position
     * @param positionMargin The margin for the position
     * @return Trigger for whether the drivetrain is near the target position
     */
    public Trigger onPosition(Translation2d translation2d, double positionMargin) {
        return onPosition(translation2d, () -> positionMargin);
    }

    /**
     * Checks if the current position is within a certain margin
     *
     * @param pose2d The target position
     * @param positionMargin The margin for the position
     * @return Trigger for whether the drivetrain is near the target position
     */
    public Trigger onPosition(Pose2d pose2d, DoubleSupplier positionMargin) {
        return onPosition(pose2d.getTranslation(), positionMargin);
    }

    /**
     * Checks if the current position is within a certain margin
     *
     * @param pose2d The target position
     * @param positionMargin The margin for the position
     * @return Trigger for whether the drivetrain is near the target position
     */
    public Trigger onPosition(Pose2d pose2d, double positionMargin) {
        return onPosition(pose2d.getTranslation(), () -> positionMargin);
    }

    /**
     * Checks if the current rotation is within a certain margin
     *
     * @param targetRotation2d The target rotation
     * @param rotationMargin The margin for the rotation
     * @return Trigger for whether the drivetrain is near the target rotation
     */
    public Trigger onRotation(Rotation2d targetRotation2d, Angle rotationMargin) {
        return new Trigger(() -> MathUtil.isNear(
                targetRotation2d.getDegrees(), getPose().getRotation().getDegrees(), rotationMargin.in(Degrees)));
    }

    /**
     * Checks if the current velocity is below a certain threshold
     *
     * @param velocityMargin The margin for the velocity
     * @return Trigger for whether the drivetrain is (near) stationary
     */
    public Trigger isStationary(DoubleSupplier velocityMargin) {
        return new Trigger(() -> MathUtil.isNear(
                0,
                Math.hypot(
                        Math.hypot(
                                getFieldRelativeSpeeds().vxMetersPerSecond, getFieldRelativeSpeeds().vyMetersPerSecond),
                        getFieldRelativeSpeeds().omegaRadiansPerSecond),
                velocityMargin.getAsDouble()));
    }

    /**
     * Checks if the current velocity is below a certain threshold
     *
     * @param velocityMargin The margin for the velocity
     * @return Trigger for whether the drivetrain is stationary
     */
    public Trigger isStationary(double velocityMargin) {
        return isStationary(() -> velocityMargin);
    }

    /**
     * Checks if the robot is parallel to floor withing margin
     *
     * @param angleMargin The margin
     * @return trigger for whether both pitch and roll are parallel to floor with margin
     */
    public Trigger isParallelToFloor(Rotation2d angleMargin) {
        return new Trigger(() -> MathUtil.isNear(0, imuInputs.pitchPosition.getRadians(), angleMargin.getRadians())
                && MathUtil.isNear(0, imuInputs.rollPosition.getRadians(), angleMargin.getRadians()));
    }

    /**
     * Measure the feedforward constants of the drive subsystem.
     *
     * @param rampRate The rate at which the voltage is increased in V/s
     * @return A command that characterizes the feedforward constants of the drive subsystem.
     */
    public Command feedforwardCharacterization(double rampRate) {
        //        double FF_RAMP_RATE = 0.1;
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

                // Allow modules to orient
                Commands.run(() -> runCharacterization(0.0), this).withTimeout(2.0),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(
                                () -> {
                                    double voltage = timer.get() * rampRate;
                                    runCharacterization(voltage);
                                    velocitySamples.add(getFFCharacterizationVelocity());
                                    voltageSamples.add(voltage);
                                },
                                this)

                        // When cancelled, calculate and print results
                        .finallyDo(() -> {
                            int n = velocitySamples.size();
                            double sumX = 0.0;
                            double sumY = 0.0;
                            double sumXY = 0.0;
                            double sumX2 = 0.0;
                            for (int i = 0; i < n; i++) {
                                sumX += velocitySamples.get(i);
                                sumY += voltageSamples.get(i);
                                sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                            }
                            double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                            double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                            NumberFormat formatter = new DecimalFormat("#0.00000");
                            System.out.println("********** Drive FF Characterization Results **********");
                            System.out.println("\tkS: " + formatter.format(kS));
                            System.out.println("\tkV: " + formatter.format(kV));
                        }));
    }

    /**
     * Measures the wheel radius of the drive subsystem.
     *
     * @param wheelRadiusRampRate The rate at which the angular velocity of the robot is increased in rad/s^2
     * @param wheelRadiusMaxVelocity The maximum angular velocity of the robot during characterization in rad/s
     * @param driveBaseRadius The radius of the drive base in meters
     * @param currentWheelRadius The current wheel radius that is used in constants in meters
     * @return A command that characterizes the wheel radius of the drive subsystem.
     */
    public Command wheelRadiusCharacterization(
            double wheelRadiusRampRate,
            double wheelRadiusMaxVelocity,
            double driveBaseRadius,
            double currentWheelRadius) {
        //        double WHEEL_RADIUS_RAMP_RATE = 0.05;
        //        double WHEEL_RADIUS_MAX_VELOCITY = 0.25;
        SlewRateLimiter limiter = new SlewRateLimiter(wheelRadiusRampRate);
        List<Double> startWheelPositions = new LinkedList<>();
        List<Rotation2d> robotAngles = new LinkedList<>();

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        // Reset acceleration limiter
                        Commands.runOnce(() -> limiter.reset(0.0)),

                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(wheelRadiusMaxVelocity);
                                    setTargetSpeeds(new ChassisSpeeds(0.0, 0.0, speed));
                                },
                                this)),

                // Measurement sequence
                Commands.sequence(
                        // Wait for modules to fully orient before starting measurement
                        Commands.waitSeconds(1.0),

                        // Record starting measurement
                        Commands.runOnce(() -> {
                            startWheelPositions.addAll(getWheelRadiusCharacterizationPositions());
                            robotAngles.add(getPose().getRotation());
                        }),

                        // Update gyro delta
                        Commands.run(() -> robotAngles.add(getPose().getRotation()))

                                // When cancelled, calculate and print results
                                .finallyDo(() -> {
                                    List<Double> finalWheelPositions =
                                            new LinkedList<>(getWheelRadiusCharacterizationPositions());
                                    double wheelDeltaMeters = 0.0;
                                    for (int i = 0; i < finalWheelPositions.size(); i++) {
                                        wheelDeltaMeters +=
                                                Math.abs(finalWheelPositions.get(i) - startWheelPositions.get(i))
                                                        / finalWheelPositions.size();
                                    }
                                    // The wheel delta is in meters, because we already take into account a wheel radius
                                    // in the IOs.
                                    // Convert back to radians of the wheel using this 'theoretical' wheel radius
                                    double wheelDelta = wheelDeltaMeters / currentWheelRadius;

                                    // Since the angles of the gyro loops from -180 to 180, we need to sum over all
                                    // differences
                                    double gyroDeltaRad = 0.0;
                                    for (int i = 1; i < robotAngles.size(); i++) {
                                        gyroDeltaRad += robotAngles
                                                .get(i)
                                                .minus(robotAngles.get(i - 1))
                                                .getRadians();
                                    }
                                    gyroDeltaRad = Math.abs(gyroDeltaRad);
                                    double optimizedWheelRadius = (gyroDeltaRad * driveBaseRadius) / wheelDelta;

                                    NumberFormat formatter = new DecimalFormat("#0.000");
                                    System.out.println("********** Wheel Radius Characterization Results **********");
                                    System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                                    System.out.println("\tGyro Delta: " + formatter.format(gyroDeltaRad) + " radians");
                                    System.out.println("\tWheel Radius: "
                                            + formatter.format(optimizedWheelRadius)
                                            + " meters, "
                                            + formatter.format(Units.metersToInches(optimizedWheelRadius))
                                            + " inches");
                                })));
    }

    /**
     * Returns a command to run a quasistatic test in the specified direction. This applies a slowly increasing voltage
     * to the drivetrain
     *
     * @param rampRate The rate at which the voltage is increased in V/s
     * @param direction The direction of the quasistatic test (forward or backward)
     * @return A command that characterizes the feedforward constants of the drive subsystem.
     */
    public Command sysIdQuasistatic(double rampRate, SysIdRoutine.Direction direction) {
        SysIdRoutine sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(rampRate).per(Second),
                        null,
                        null,
                        (state) -> Logger.recordOutput("SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    /**
     * Returns a command to run a dynamic test in the specified direction. This applies a step voltage to the drivetrain
     *
     * @param voltageStep The voltage step to apply to the drivetrain
     * @param direction The direction of the dynamic test (forward or backward)
     * @return A command that characterizes the feedforward constants of the drive subsystem.
     */
    public Command sysIdDynamic(double voltageStep, SysIdRoutine.Direction direction) {
        SysIdRoutine sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        Volt.of(voltageStep),
                        null,
                        (state) -> Logger.recordOutput("SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /**
     * Set the target of the motors in the drivetrain using the input voltage to characterize the system. To which
     * motors the voltage is set depends on which constants are being characterized.
     *
     * @param voltage The target voltage for characterization
     */
    protected abstract void runCharacterization(double voltage);

    /**
     * Return the average velocity of the characterized motors in the desired units.
     *
     * @return The average velocity in the units of the characterized feedforward constant.
     */
    protected abstract double getFFCharacterizationVelocity();

    /**
     * Get the positions of the drive wheels for the wheel radius characterization. The position is expressed in the
     * distance covered by the wheels in meters.
     *
     * @return List of the positions of the drive wheels
     */
    protected abstract List<Double> getWheelRadiusCharacterizationPositions();
}
