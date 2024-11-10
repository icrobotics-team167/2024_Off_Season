// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.cotc.drive.Swerve;
import frc.cotc.drive.SwerveIO;
import frc.cotc.drive.SwerveIOPhoenix;
import frc.cotc.vision.VisionPoseEstimatorIO;
import frc.cotc.vision.VisionPoseEstimatorIOPhoton;
import org.littletonrobotics.junction.*;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  public static final String CANIVORE_NAME = "Canivore";

  private final Autos autos;

  @SuppressWarnings({"DataFlowIssue", "UnreachableCode"})
  public Robot() {
    Logger.recordMetadata("Project", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("Git branch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("Git commit date", BuildConstants.GIT_DATE);
    Logger.recordMetadata("Git SHA", BuildConstants.GIT_SHA);
    //noinspection ConstantValue
    Logger.recordMetadata("Uncommited changes", BuildConstants.DIRTY == 1 ? "True" : "False");
    Logger.recordMetadata("Compile date", BuildConstants.BUILD_DATE);

    String mode = Robot.isReal() ? "REAL" : "SIM";
    //    String mode = "REPLAY";

    switch (mode) {
      case "REAL" -> {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        //noinspection resource
        new PowerDistribution(
            1, PowerDistribution.ModuleType.kRev); // Enables power distribution logging
        SignalLogger.start(); // Start logging Phoenix CAN signals
      }
      case "SIM" -> {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to the project's logs folder
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

        SignalLogger.start(); // Start logging Phoenix CAN signals
      }
      case "REPLAY" -> {
        setUseTiming(false); // Run as fast as possible
        String logPath =
            LogFileUtil
                .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_replay"))); // Save outputs to a new log
      }
    }

    Logger.start();

    Swerve swerve = getSwerve(mode);

    CommandJoystick primaryLeft = new CommandJoystick(0);
    CommandJoystick primaryRight = new CommandJoystick(1);

    // Robot wants +X fwd, +Y left
    // Sticks are +X right +Y back
    swerve.setDefaultCommand(
        swerve.teleopDrive(
            () -> MathUtil.applyDeadband(-primaryLeft.getY(), .01),
            () -> MathUtil.applyDeadband(-primaryLeft.getX(), .01),
            () -> MathUtil.applyDeadband(-primaryRight.getX(), .01)));
    RobotModeTriggers.disabled().or(primaryLeft.button(3)).whileTrue(swerve.stopInX());

    autos = new Autos(swerve);
  }

  private Swerve getSwerve(String mode) {
    SwerveIO swerveIO;
    VisionPoseEstimatorIO poseEstimatorIO;

    switch (mode) {
      case "REAL" -> {
        swerveIO = new SwerveIOPhoenix();
        poseEstimatorIO = new VisionPoseEstimatorIOPhoton();
      }
      case "SIM" -> {
        swerveIO = new SwerveIOPhoenix();
        poseEstimatorIO = new VisionPoseEstimatorIO() {};
      }
      default -> {
        swerveIO = new SwerveIO() {};
        poseEstimatorIO = new VisionPoseEstimatorIO() {};
      }
    }

    return new Swerve(swerveIO, poseEstimatorIO);
  }

  private Command autoCommand;

  @Override
  public void autonomousInit() {
    autoCommand = autos.getAuto();
    autoCommand.schedule();
  }

  @Override
  public void autonomousExit() {
    autoCommand.cancel();
  }

  @Override
  public void robotPeriodic() {
    // Updates Choreo's auto selector.
    autos.update();
    // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled commands,
    // running already-scheduled commands, removing finished or interrupted commands, and running
    // subsystem periodic() methods. This must be called from the robot's periodic block in order
    // for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  public static volatile double simVoltage = 12;

  @Override
  public void simulationPeriodic() {
    if (!Logger.hasReplaySource()) {
      RoboRioSim.setVInVoltage(simVoltage);
    }
  }

  public static boolean isOnRed() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red;
  }
}
