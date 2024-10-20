// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.cotc.drive.Swerve;
import frc.cotc.drive.SwerveIO;
import frc.cotc.drive.SwerveIOPhoenix;
import frc.cotc.vision.VisionPoseEstimatorIO;
import frc.cotc.vision.VisionPoseEstimatorIOPhoton;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  // Prevent IntelliJ from yelling at me about the real/sim/log replay switching code
  @SuppressWarnings("RedundantSuppression,DataFlowIssue,UnreachableCode,DuplicateBranchesInSwitch")
  public Robot() {
    Logger.recordMetadata("Project", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("Git branch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("Git commit date", BuildConstants.GIT_DATE);
    Logger.recordMetadata("Git SHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("Compile date", BuildConstants.BUILD_DATE);

    String mode = Robot.isReal() ? "REAL" : "SIM";
    // String mode = "REPLAY";

    switch (mode) {
      case "REAL", "SIM" -> {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        //noinspection resource
        new PowerDistribution(
            1, PowerDistribution.ModuleType.kRev); // Enables power distribution logging
      }
      case "REPLAY" -> {
        setUseTiming(false); // Run as fast as possible
        String logPath =
            LogFileUtil
                .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
      }
    }

    SignalLogger.start();
    Logger.start();

    Swerve swerve = getSwerve(mode);

    CommandXboxController primaryController = new CommandXboxController(0);

    // Robot wants +X fwd, +Y left
    // Sticks are +X right +Y back
    swerve.setDefaultCommand(
        swerve.teleopDrive(
            () -> MathUtil.applyDeadband(-primaryController.getLeftY(), .01),
            () -> MathUtil.applyDeadband(-primaryController.getLeftX(), .01),
            () -> MathUtil.applyDeadband(-primaryController.getRightX(), .01)));
    RobotModeTriggers.disabled().or(primaryController.povDown()).whileTrue(swerve.stopInX());

    // primaryController.rightBumper().onTrue(swerve.driveCharacterization());
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

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled commands,
    // running already-scheduled commands, removing finished or interrupted commands, and running
    // subsystem periodic() methods. This must be called from the robot's periodic block in order
    // for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }
}
