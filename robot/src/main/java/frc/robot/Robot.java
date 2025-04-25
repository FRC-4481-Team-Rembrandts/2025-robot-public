/*
 * Copyright (c) 2024-2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot;

import static frc.robot.util.TimeUtil.updateDeltaTime;

import com.teamrembrandts.simulation.VisionEnvironmentSimulator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Autos;
import frc.robot.constants.BuildConstants;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    /** This function is run when the robot is first started up and should be used for any initialization code. */
    @Override
    public void robotInit() {

        Logger.recordMetadata("Library/ProjectName", com.teamrembrandts.BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("Library/BuildDate", com.teamrembrandts.BuildConstants.BUILD_DATE);
        Logger.recordMetadata("Library/GitSHA", com.teamrembrandts.BuildConstants.GIT_SHA);
        Logger.recordMetadata("Library/GitDate", com.teamrembrandts.BuildConstants.GIT_DATE);
        Logger.recordMetadata("Library/GitBranch", com.teamrembrandts.BuildConstants.GIT_BRANCH);
        switch (com.teamrembrandts.BuildConstants.DIRTY) {
            case 0 -> Logger.recordMetadata("Library/GitDirty", "All changes committed");
            case 1 -> Logger.recordMetadata("Library/GitDirty", "Uncomitted changes");
            default -> Logger.recordMetadata("Library/GitDirty", "Unknown");
        }

        Logger.recordMetadata("Robot/ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("Robot/BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("Robot/GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("Robot/GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("Robot/GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0 -> Logger.recordMetadata("Robot/GitDirty", "All changes committed");
            case 1 -> Logger.recordMetadata("Robot/GitDirty", "Uncomitted changes");
            default -> Logger.recordMetadata("Robot/GitDirty", "Unknown");
        }

        AutoLogOutputManager.addPackage("com.teamrembrandts");

        SmartDashboard.putData(CommandScheduler.getInstance());

        switch (Constants.MODE) {
            case REAL -> {
                Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")

                if (!DriverStation.isFMSAttached()) {
                    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
                }

                LoggedPowerDistribution.getInstance(
                        Constants.PDH_ID, PowerDistribution.ModuleType.kRev); // Enables power distribution logging
                Logger.registerURCL(URCL.startExternal());
            }
            case SIM -> {
                Logger.addDataReceiver(new WPILOGWriter()); // Log to the logs folder in the project
                Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            }
            case REPLAY -> {
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or
                // prompt the user)
                Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
                Logger.addDataReceiver(
                        new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
            }
        }

        // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the
        // "Understanding Data Flow" page
        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata
        // values may be added.

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        Map<String, Integer> commandCounts = new HashMap<>();
        BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
            String name = command.getName();
            int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
            commandCounts.put(name, count);
            Logger.recordOutput("CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
            Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
        CommandScheduler.getInstance()
                .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
        CommandScheduler.getInstance().onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
        CommandScheduler.getInstance()
                .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

        Autos.warmupChoreoTrajectoryFollower().schedule();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
     * during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard integrated
     * updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        updateDeltaTime();

        SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
        try {
            Logger.recordOutput(
                    "MatchTimeNWT", SmartDashboard.getEntry("MatchTime").getDouble(-44.81));
        } catch (Exception ignored) {
        }
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        VisionEnvironmentSimulator.getInstance().update();
    }
}
