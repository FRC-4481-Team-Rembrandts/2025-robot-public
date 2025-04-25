/*
 * Copyright (c) 2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.pivot;

import static frc.robot.constants.PivotConstants.*;
import static frc.robot.util.TimeUtil.getDeltaTime;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GameVisualizer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
    private Rotation2d angleGoal = new Rotation2d();
    private Rotation2d savedScoringSetpoint = TARGET_ANGLE_L4;

    private final PivotIO io;
    private final PivotInputsAutoLogged inputs = new PivotInputsAutoLogged();
    private final TrapezoidProfile profile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(M_PROFILE_MAX_VEL, M_PROFILE_MAX_ACCEL));
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State profileSetpoint = null;

    private double prevTargetVelocity = 0;

    private Angle aAngle = null;
    private double aDouble = 0.0;

    private final GameVisualizer visualizer;

    public Pivot(PivotIO io) {
        this.io = io;
        visualizer = GameVisualizer.getInstance();
    }

    /**
     * Checks if the pivot is at the setpoint
     *
     * @return Trigger that checks if the pivot is at his setpoint
     */
    public Trigger onPositionTrigger() {
        return new Trigger(() ->
                MathUtil.isNear(angleGoal.getDegrees(), inputs.angle.getDegrees(), POSITION_MARGIN_DEGREES, -180, 180));
    }

    public Command goToHorizontalCommand() {
        return goToTargetAngle(() -> TARGET_ANGLE_HORIZONTAL_CORAL);
    }

    public Command goToBargePrepCommand() {
        return goToTargetAngle(() -> TARGET_ANGLE_BARGE_PREP);
    }

    /**
     * Command to set the angle of the pivot for intaking a coral at the coral station
     *
     * @return Command to set the angle of the pivot for intaking a coral at the coral station
     */
    public Command goToCoralStationCommand() {
        return goToTargetAngle(() -> TARGET_ANGLE_CORAL_INTAKE);
    }

    public Command goToCoralStationAutonomousCommand() {
        return goToTargetAngle(() -> TARGET_ANGLE_CORAL_INTAKE_AUTON);
    }

    /**
     * Command to set the angle of the pivot for the neutral position This is vertical
     *
     * @return Command to set the angle of the pivot for the neutral position
     */
    public Command goToStowedCommand() {
        return goToTargetAngle(() -> STOWED_ANGLE);
    }

    public Command goToClimbCommand() {
        return goToTargetAngle(() -> TARGET_CLIMBER);
    }

    /**
     * Command to go to the angle for the scoring position on level l1
     *
     * @return Command to go to the angle of the pivot for the scoring position on level l1
     */
    public Command goToL1Command() {
        return goToTargetAngle(() -> TARGET_ANGLE_L1);
    }

    /**
     * Command to go to the angle for the scoring position on level l2
     *
     * @return Command to go to the angle of the pivot for the scoring position on level l2
     */
    public Command goToL2Command() {
        return goToTargetAngle(() -> TARGET_ANGLE_L2);
    }

    /**
     * Command to go to the angle for the scoring position on level l3
     *
     * @return Command to go to the angle of the pivot for the scoring position on level l3
     */
    public Command goToL3Command() {
        return goToTargetAngle(() -> TARGET_ANGLE_L3);
    }

    /**
     * Command to go to the angle for the scoring position on level l4
     *
     * @return Command to go to the angle of the pivot for the scoring position on level l4
     */
    public Command goToL4Command() {
        return goToTargetAngle(() -> TARGET_ANGLE_L4);
    }

    /**
     * Command to go to the angle for the scoring position on the algae on level l3
     *
     * @return Command to go to the angle of the pivot for the scoring position on the algae on level l3
     */
    public Command goToAlgaeL3Command() {
        return goToTargetAngle(() -> ALGAE_L3);
    }

    /**
     * Command to go to the angle for the scoring position on the algae on level l2
     *
     * @return Command to go to the angle of the pivot for the scoring position on the algae on level l2
     */
    public Command goToAlgaeL2Command() {
        return goToTargetAngle(() -> ALGAE_L2);
    }

    /**
     * Command to go to the angle for the scoring position on the processor
     *
     * @return Command to go to the angle of the pivot for the scoring position on the processor
     */
    public Command goToProcessorCommand() {
        return goToTargetAngle(() -> TARGET_ANGLE_PROCESSOR);
    }

    public Command goToBargeCommand() {
        return goToTargetAngle(() -> TARGET_ANGLE_BARGE);
    }

    /**
     * Command to start the movement of the pivot
     *
     * @return Command to start the movement of the pivot
     */
    public Command goToScoringPositionCommand() {
        return goToTargetAngle(() -> savedScoringSetpoint); // .until(onPositionTrigger());
    }

    /**
     * Command to set the position of the pivot with a motion profile
     *
     * @param angle the desired angle position of the pivot
     * @return Command to set the position of the pivot with a motion profile
     */
    private Command goToTargetAngle(Supplier<Rotation2d> angle) {
        return Commands.run(
                        () -> {
                            angleGoal = angle.get();
                            goal = new TrapezoidProfile.State(angle.get().getRadians(), 0);
                            profileSetpoint = profile.calculate(LoggedRobot.defaultPeriodSecs, profileSetpoint, goal);
                            double targetAcceleration =
                                    (profileSetpoint.velocity - prevTargetVelocity) / getDeltaTime();
                            Logger.recordOutput("Pivot/profile position", profileSetpoint.position);
                            Logger.recordOutput("Pivot/profile velocity", profileSetpoint.velocity);
                            io.setTarget(
                                    new Rotation2d(profileSetpoint.position),
                                    profileSetpoint.velocity,
                                    targetAcceleration);
                            prevTargetVelocity = profileSetpoint.velocity;
                        },
                        this)
                .beforeStarting(() -> {
                    angleGoal = angle.get();
                    profileSetpoint = new TrapezoidProfile.State(inputs.angle.getRadians(), inputs.velocityRadPerSec);
                })
                .finallyDo(() -> {
                    if (onPositionTrigger().getAsBoolean()) {
                        io.setTarget(angleGoal);
                    } else {
                        io.setTarget(new Rotation2d(profileSetpoint.position));
                    }
                });
    }

    /**
     * Disable motors
     *
     * @return command that disables the motors
     */
    public Command disableCommand() {
        return Commands.runOnce(() -> io.setPower(0), this);
    }

    /**
     * Command to set the angularVelocity of the pivot
     *
     * @param angularVelocitySupplier the velocity of the pivot
     * @return Command to set the velocity of the pivot.
     */
    public Command setVelocity(Supplier<AngularVelocity> angularVelocitySupplier) {
        return Commands.run(
                () -> {
                    aAngle = angularVelocitySupplier.get().times(Time.ofBaseUnits(getDeltaTime(), Units.Seconds));
                    aDouble = aAngle.in(Units.Radians);
                    goToTargetAngle(() -> savedScoringSetpoint.plus(Rotation2d.fromRadians(aDouble)));
                },
                this);
    }

    public Trigger isPivotDown() {
        return new Trigger(() -> inputs.angle.getDegrees() <= PIVOT_IS_DOWN);
    }

    public Trigger isPivotReadyToShootBarge() {
        return new Trigger(() -> inputs.angle.getDegrees() >= PIVOT_READY_TO_SCORE_IN_BARGE);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);

        visualizer.setPivotAngle(inputs.angle);

        Logger.recordOutput("Pivot/OnPosition", onPositionTrigger().getAsBoolean());
        Logger.recordOutput("Pivot/pivot under -70", isPivotDown().getAsBoolean());
        Logger.recordOutput("Pivot ready for barge", isPivotReadyToShootBarge().getAsBoolean());
    }
}
