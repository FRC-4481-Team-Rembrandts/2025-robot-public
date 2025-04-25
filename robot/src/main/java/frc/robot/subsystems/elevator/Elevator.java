/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ElevatorConstants.*;
import static frc.robot.util.TimeUtil.getDeltaTime;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.GameVisualizer;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

/** The Elevator subsystem. */
public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    @AutoLogOutput
    private double activeSetpoint = GROUND_TO_PIVOT_AXLE_DISTANCE_LOWEST_POSITION;

    @AutoLogOutput
    private double savedScoringSetpoint = TARGET_HEIGHT_L4;

    @AutoLogOutput
    private boolean isCalibrated = false;

    private final TrapezoidProfile profile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(MOTION_PROFILE_MAX_VELOCITY, M_PROFILE_MAX_ACCEL));
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State profileSetpoint = null;

    private double prevTargetVelocity = 0;

    private final GameVisualizer visualizer;

    /**
     * Creates a new Elevator subsystem
     *
     * @param io The IO of the Elevator.
     */
    public Elevator(ElevatorIO io) {
        this.io = io;

        visualizer = GameVisualizer.getInstance();
    }

    /**
     * Command to go to calibrate elevator sensors
     *
     * @return Command to go neutral position.
     */
    public Command calibrate() {
        return Commands.sequence(
                runMotorWithPowerCommand(() -> POWER_CALIBRATE)
                        .until(isReachingCurrentLimitTrigger().or(bottomLimitSwitchTrigger())),
                runMotorWithPowerCommand(() -> 0).withTimeout(0.2),
                resetEncodersCommand());
    }

    /**
     * sets relative position to absolute position
     *
     * @return Command to reset the encoders
     */
    public Command resetEncodersCommand() {
        return Commands.runOnce(() -> {
            io.setRelativeEncoderPosition(inputs.absoluteEncoderPosition);
            profileSetpoint = new TrapezoidProfile.State(inputs.absoluteEncoderPosition, 0);
            isCalibrated = true;
        });
    }

    /**
     * Command to run motors with certain power.
     *
     * @param motorPower The power from -1 to 1 to run the motor.
     * @return Command to run motor slowly towards calibration position.
     */
    private Command runMotorWithPowerCommand(DoubleSupplier motorPower) {
        return Commands.run(() -> io.setPower(motorPower.getAsDouble()), this);
    }

    /**
     * Command to go to neutral position
     *
     * @return Command to go neutral position.
     */
    public Command goToStowedCommand() {
        return goToPositionOrCalibrate(() -> STOWED_HEIGHT);
    }

    /**
     * Command to check if the Elevator is on the desired position The elevator can only be on position once it is
     * calibrated
     *
     * @return Command to check if on target position.
     */
    public Trigger onPositionTrigger() {
        return new Trigger(() -> MathUtil.isNear(activeSetpoint, inputs.leaderPosition, POSITION_MARGIN_METERS)
                && isCalibratedTrigger().getAsBoolean());
    }

    /**
     * Command to get the elevator to the position L1 position
     *
     * @return Command that goes to the position of the elevator to level 1 of the reef.
     */
    public Command goToL1Command() {
        return goToPositionOrCalibrate(() -> TARGET_HEIGHT_L1);
    }

    /**
     * Command to get the elevator to the position L2 position
     *
     * @return Command that goes to the position of the elevator to level 2 of the reef.
     */
    public Command goToL2Command() {
        return goToPositionOrCalibrate(() -> TARGET_HEIGHT_L2);
    }

    /**
     * Command to get the elevator to the position L3 position
     *
     * @return Command that goes to the position of the elevator to level 3 of the reef.
     */
    public Command goToL3Command() {
        return goToPositionOrCalibrate(() -> TARGET_HEIGHT_L3);
    }

    /**
     * Command to get the elevator to the position L4 position
     *
     * @return Command that goes to the position of the elevator to level 4 of the reef.
     */
    public Command goToL4Command() {
        return goToPositionOrCalibrate(() -> TARGET_HEIGHT_L4);
    }

    /**
     * Command to set the position of the elevator to the algae L3
     *
     * @return Command to set the position of the elevator to the algae L3
     */
    public Command goToAlgaeL3Command() {
        return goToPositionOrCalibrate(() -> TARGET_HEIGHT_ALGAE_L3);
    }

    /**
     * Command to set the position of the elevator to the algae L2
     *
     * @return Command to set the position of the elevator to the algae L2
     */
    public Command goToAlgaeL2Command() {
        return goToPositionOrCalibrate(() -> TARGET_HEIGHT_ALGAE_L2);
    }

    /**
     * Command to set the position of the elevator to the processor
     *
     * @return Command to set the position of the elevator to the processor
     */
    public Command goToProcessorCommand() {
        return goToPositionOrCalibrate(() -> TARGET_HEIGHT_PROCESSOR);
    }

    public Command goToBargeCommand() {
        return goToPositionOrCalibrate(() -> TARGET_HEIGHT_BARGE);
    }

    public Command goToClimbCommand() {
        return goToPositionOrCalibrate(() -> TARGET_HEIGHT_CLIMB);
    }

    /**
     * Command to set the position of the elevator to retrieve a coral from the Coral station
     *
     * @return Command to set the position of the elevator to retrieve a coral from the Coral station
     */
    public Command goToCoralStationCommand() {
        return goToPositionOrCalibrate(() -> TARGET_HEIGHT_CORAL_INTAKE);
    }

    public Command goToCoralStationAutonomousCommand() {
        return goToPositionOrCalibrate(() -> TARGET_HEIGHT_CORAL_INTAKE_AUTON);
    }

    /**
     * Command to go to the desired position
     *
     * @return Command to go to a certain position.
     */
    public Command goToScoringPositionCommand() {
        return goToPositionOrCalibrate(() -> savedScoringSetpoint);
    }

    /**
     * Command to go to the desired position if the elevator is calibrated. Otherwise, first calibrate the elevator and
     * then go to the desired position.
     *
     * @param targetPosition the position for the elevator to target.
     * @return Command to go to a certain target position.
     */
    private Command goToPositionOrCalibrate(DoubleSupplier targetPosition) {
        return new ConditionalCommand(
                goToPosition(targetPosition),
                Commands.sequence(calibrate(), goToPosition(targetPosition)),
                isCalibratedTrigger());
    }

    /**
     * Command to go to the desired position
     *
     * @param targetPosition the position for the elevator to target.
     * @return Command to go to a certain target position.
     */
    private Command goToPosition(DoubleSupplier targetPosition) {
        return Commands.run(
                        () -> {
                            activeSetpoint = targetPosition.getAsDouble();
                            goal = new TrapezoidProfile.State(targetPosition.getAsDouble(), 0);
                            profileSetpoint = profile.calculate(LoggedRobot.defaultPeriodSecs, profileSetpoint, goal);
                            double targetAcceleration =
                                    (profileSetpoint.velocity - prevTargetVelocity) / getDeltaTime();
                            Logger.recordOutput("Elevator/profile position", profileSetpoint.position);
                            Logger.recordOutput("Elevator/profile velocity", profileSetpoint.velocity);
                            io.setTarget(profileSetpoint.position, profileSetpoint.velocity, targetAcceleration);
                            prevTargetVelocity = profileSetpoint.velocity;
                        },
                        this)
                .beforeStarting(() -> {
                    activeSetpoint = targetPosition.getAsDouble();
                    profileSetpoint = new TrapezoidProfile.State(inputs.leaderPosition, inputs.leaderVelocity);
                })
                .finallyDo(() -> {
                    if (profileSetpoint.position <= DISABLE_HEIGHT && activeSetpoint != TARGET_HEIGHT_CLIMB) {
                        io.setPower(0.0);
                    } else if (onPositionTrigger().getAsBoolean()) {
                        io.setTarget(activeSetpoint);
                    } else {
                        io.setTarget(profileSetpoint.position);
                    }
                });
    }

    /**
     * Command to set the velocity of the elevator
     *
     * @param velocity the velocity of the elevator in m/s
     * @return Command to set the velocity of the elevator.
     */
    public Command setVelocityCommand(DoubleSupplier velocity) {
        return goToPositionOrCalibrate(() -> activeSetpoint + velocity.getAsDouble() * getDeltaTime());
    }

    /**
     * Measure the feedforward constants of the elevator subsystem.
     *
     * @param rampRate The rate at which the voltage is increased in V/s
     * @return A command that characterizes the feedforward constants of the drive subsystem.
     */
    public Command feedforwardCharacterization(double rampRate) {
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
                Commands.run(() -> io.setPower(0.0), this).withTimeout(2.0),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(
                                () -> {
                                    double voltage = timer.get() * rampRate;
                                    io.setPower(voltage / 12.0);
                                    velocitySamples.add(inputs.leaderVelocity);
                                    voltageSamples.add(voltage);
                                },
                                this)

                        // When cancelled, calculate and print results
                        .finallyDo(() -> {
                            io.setPower(0.0);

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
                            System.out.println("********** Elevator FF Characterization Results **********");
                            System.out.println("\tkS + kG: " + formatter.format(kS));
                            System.out.println("\tkV: " + formatter.format(kV));
                        }));
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
                        Seconds.of(20),
                        (state) -> Logger.recordOutput("SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> io.setPower(voltage.in(Volts) / 12), null, this));
        return run(() -> io.setPower(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
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
                        Seconds.of(1.0),
                        (state) -> Logger.recordOutput("SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> io.setPower(voltage.in(Volts) / 12), null, this));
        return run(() -> io.setPower(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /**
     * Trigger that checks if the elevator is above a certain position.
     *
     * @return Trigger that checks if the elevator is above a certain position.
     */
    public Trigger isElevatorUpTrigger() {
        return new Trigger(() -> inputs.leaderPosition > ELEVATOR_TRIGGER_UP);
    }

    /**
     * Trigger that checks if the elevator is reaching the current limit in one of both motors
     *
     * @return Whether the motors are near the current limit
     */
    public Trigger isReachingCurrentLimitTrigger() {
        return new Trigger(() -> inputs.leaderCurrentDraw >= CURRENT_LIMIT * CURRENT_LIMIT_DETECTION_FACTOR
                || inputs.followerCurrentDraw >= CURRENT_LIMIT * CURRENT_LIMIT_DETECTION_FACTOR);
    }

    /**
     * Trigger that checks if the elevator is on the bottom limit switch
     *
     * @return Whether the bottom limit switch is triggered
     */
    public Trigger bottomLimitSwitchTrigger() {
        return new Trigger(() -> inputs.bottomLimitSwitchTriggered);
    }

    /**
     * Trigger that checks if the elevator is calibrated
     *
     * @return Whether the elevator is calibrated
     */
    public Trigger isCalibratedTrigger() {
        return new Trigger(() -> isCalibrated);
    }

    /**
     * Disable the motors
     *
     * @return command that disables the motors
     */
    public Command disableCommand() {
        return Commands.runOnce(() -> io.setPower(0), this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        SmartDashboard.putBoolean("Connection Status/Elevator Calibrated", isCalibrated);
        visualizer.setElevatorPosition(inputs.leaderPosition);
    }
}
