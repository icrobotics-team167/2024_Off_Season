// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import static frc.cotc.drive.SwerveSetpointGenerator.SwerveSetpoint;
import static java.lang.Math.PI;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.Robot;
import frc.cotc.vision.VisionPoseEstimatorIO;
import frc.cotc.vision.VisionPoseEstimatorIO.VisionPoseEstimatorIOInputs;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final SwerveIO swerveIO;

  private final SwerveIO.SwerveIOInputs swerveInputs;

  private final SwerveSetpointGenerator setpointGenerator;
  private final SwerveSetpoint stopInXSetpoint;
  private final double[] EMPTY_FORCES = new double[4];
  private SwerveSetpoint lastSetpoint;

  private final double maxLinearSpeedMetersPerSec, drivebaseRadius, angularSpeedFudgeFactor;

  private final SwervePoseEstimator poseEstimator;

  private final VisionPoseEstimatorIO[] visionIOs;
  private final VisionPoseEstimatorIOInputs[] visionInputs;

  private final PIDController xController, yController, yawController;

  public Swerve(SwerveIO driveIO, VisionPoseEstimatorIO[] visionIOs) {
    this.swerveIO = driveIO;
    var CONSTANTS = driveIO.getConstants();
    swerveInputs = new SwerveIO.SwerveIOInputs();
    driveIO.updateInputs(swerveInputs);
    Logger.processInputs("Swerve/Constants", CONSTANTS);

    maxLinearSpeedMetersPerSec =
        CONSTANTS.DRIVE_MOTOR.freeSpeedRadPerSec * (CONSTANTS.WHEEL_DIAMETER_METERS / 2);
    drivebaseRadius =
        Math.hypot(CONSTANTS.TRACK_WIDTH_METERS / 2, CONSTANTS.TRACK_LENGTH_METERS / 2);
    angularSpeedFudgeFactor = CONSTANTS.ANGULAR_SPEED_FUDGING;

    Logger.recordOutput("Swerve/Max Linear Speed", maxLinearSpeedMetersPerSec);
    Logger.recordOutput("Swerve/Max Angular Speed", maxLinearSpeedMetersPerSec / drivebaseRadius);

    setpointGenerator =
        new SwerveSetpointGenerator(
            new Translation2d[] {
              new Translation2d(
                  CONSTANTS.TRACK_LENGTH_METERS / 2, CONSTANTS.TRACK_WIDTH_METERS / 2),
              new Translation2d(
                  CONSTANTS.TRACK_LENGTH_METERS / 2, -CONSTANTS.TRACK_WIDTH_METERS / 2),
              new Translation2d(
                  -CONSTANTS.TRACK_LENGTH_METERS / 2, CONSTANTS.TRACK_WIDTH_METERS / 2),
              new Translation2d(
                  -CONSTANTS.TRACK_LENGTH_METERS / 2, -CONSTANTS.TRACK_WIDTH_METERS / 2)
            },
            CONSTANTS.DRIVE_MOTOR,
            CONSTANTS.DRIVE_MOTOR_CURRENT_LIMIT_AMPS,
            CONSTANTS.MAX_STEER_SPEED_RAD_PER_SEC,
            CONSTANTS.MASS_KG,
            CONSTANTS.MOI_KG_METERS_SQUARED,
            CONSTANTS.WHEEL_DIAMETER_METERS,
            CONSTANTS.WHEEL_COF);
    stopInXSetpoint =
        new SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[] {
              new SwerveModuleState(
                  0,
                  new Rotation2d(
                      Math.atan2(
                          CONSTANTS.TRACK_WIDTH_METERS / 2, CONSTANTS.TRACK_LENGTH_METERS / 2))),
              new SwerveModuleState(
                  0,
                  new Rotation2d(
                      Math.atan2(
                          -CONSTANTS.TRACK_WIDTH_METERS / 2, CONSTANTS.TRACK_LENGTH_METERS / 2))),
              new SwerveModuleState(
                  0,
                  new Rotation2d(
                      Math.atan2(
                          CONSTANTS.TRACK_WIDTH_METERS / 2, -CONSTANTS.TRACK_LENGTH_METERS / 2))),
              new SwerveModuleState(
                  0,
                  new Rotation2d(
                      Math.atan2(
                          -CONSTANTS.TRACK_WIDTH_METERS / 2, -CONSTANTS.TRACK_LENGTH_METERS / 2)))
            },
            EMPTY_FORCES);
    lastSetpoint = new SwerveSetpoint(new ChassisSpeeds(), swerveInputs.moduleStates, EMPTY_FORCES);

    poseEstimator =
        new SwervePoseEstimator(
            setpointGenerator.getKinematics(),
            new Rotation2d(),
            Arrays.copyOfRange(
                swerveInputs.odometryPositions,
                swerveInputs.odometryPositions.length - 4,
                swerveInputs.odometryPositions.length),
            new Pose2d());
    this.visionIOs = visionIOs;
    visionInputs = new VisionPoseEstimatorIOInputs[visionIOs.length];
    for (int i = 0; i < visionIOs.length; i++) {
      visionInputs[i] = new VisionPoseEstimatorIOInputs();
    }

    xController = new PIDController(5, 0, 0);
    yController = new PIDController(5, 0, 0);
    yawController = new PIDController(5, 0, 0);
    yawController.enableContinuousInput(-PI, PI);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Swerve/Commanded speeds", lastSetpoint.chassisSpeeds());
    Logger.recordOutput("Swerve/Drive setpoint", lastSetpoint.moduleStates());

    swerveIO.updateInputs(swerveInputs);
    Logger.processInputs("Swerve", swerveInputs);
    Logger.recordOutput("Swerve/Actual Speed", getRobotChassisSpeeds());

    var driveStdDevs = getDriveStdDevs();
    Logger.recordOutput("Swerve/Odometry/Drive Std Devs", driveStdDevs);
    poseEstimator.setDriveMeasurementStdDevs(driveStdDevs);

    for (int i = 0; i < swerveInputs.odometryTimestamps.length; i++) {
      poseEstimator.updateWithTime(
          swerveInputs.odometryTimestamps[i],
          swerveInputs.odometryYaws[i],
          Arrays.copyOfRange(swerveInputs.odometryPositions, i * 4, i * 4 + 4));
    }

    var tagPoses = new ArrayList<Pose3d>();
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i].updateInputs(visionInputs[i]);
      Logger.processInputs("Vision/" + i, visionInputs[i]);

      for (var poseEstimate : visionInputs[i].poseEstimates) {
        if (!MathUtil.isNear(0, poseEstimate.estimatedPose().getZ(), .005)) {
          continue;
        }

        poseEstimator.addVisionMeasurement(
            poseEstimate.estimatedPose().toPose2d(),
            poseEstimate.timestamp(),
            new double[] {.1, .1, .1});
        tagPoses.addAll(Arrays.asList(poseEstimate.tagsUsed()));
      }
    }
    Logger.recordOutput("Swerve/Odometry/Vision tags used", tagPoses.toArray(new Pose3d[0]));

    Logger.recordOutput("Swerve/Odometry/Final Position", getPose());
  }

  /**
   * Estimate drive wheel slippage by comparing the actual wheel velocities to the idealized wheel
   * velocities. If there is a significant deviation, then a wheel(s) is slipping, and we should
   * raise the estimated standard deviation of the drivebase odometry to trust the wheel encoders
   * less.
   *
   * <p>Algorithm from <a href="https://youtu.be/N6ogT5DjGOk">1690 Orbit's 2nd software session</a>
   *
   * @return An array of length 3, containing the estimated standard deviations in each axis (x, y,
   *     yaw)
   */
  private double[] getDriveStdDevs() {
    var speeds = getRobotChassisSpeeds();
    // Get idealized states from the current robot velocity.
    var idealStates = setpointGenerator.getKinematics().toSwerveModuleStates(speeds);

    double squaredSum = 0;
    for (int i = 0; i < 4; i++) {
      var measuredVector =
          new Translation2d(
              swerveInputs.moduleStates[i].speedMetersPerSecond,
              swerveInputs.moduleStates[i].angle);
      var idealVector =
          new Translation2d(idealStates[i].speedMetersPerSecond, idealStates[i].angle);

      // Compare the state vectors and get the delta between them.
      var delta = measuredVector.getDistance(idealVector);

      // Square the delta and add it to a sum
      squaredSum += delta * delta;
    }

    // Sqrt of avg of squared deltas = standard deviation
    double linearStdDevs = Math.sqrt(squaredSum / 4);

    // Get the % speeds of each axis
    // Minimum of 1% to avoid divide by 0 errors
    var fieldSpeeds = toFieldRelative(speeds);
    var xSpeed =
        Math.max(Math.abs(fieldSpeeds.vxMetersPerSecond) / maxLinearSpeedMetersPerSec, .01);
    var ySpeed =
        Math.max(Math.abs(fieldSpeeds.vyMetersPerSecond) / maxLinearSpeedMetersPerSec, .01);
    var yawSpeed =
        Math.max(
            Math.abs(fieldSpeeds.omegaRadiansPerSecond)
                / (maxLinearSpeedMetersPerSec / drivebaseRadius),
            .01);

    // Normalize
    var magnitude = Math.max(xSpeed, Math.max(ySpeed, yawSpeed));
    xSpeed /= magnitude;
    ySpeed /= magnitude;
    yawSpeed /= magnitude;

    // Scale each axis by the normalized % speed of each axis
    // Assume a minimum of 5 mm deviation, due to mechanical slop
    // Also prevents divide by 0 errors
    var minimum = .005;
    return new double[] {
      linearStdDevs * xSpeed + minimum,
      linearStdDevs * ySpeed + minimum,
      (linearStdDevs / drivebaseRadius) * yawSpeed + (minimum / drivebaseRadius)
    };
  }

  public Command teleopDrive(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return run(
        () ->
            teleopDrive(
                new ChassisSpeeds(
                    xSupplier.getAsDouble() * maxLinearSpeedMetersPerSec,
                    ySupplier.getAsDouble() * maxLinearSpeedMetersPerSec,
                    omegaSupplier.getAsDouble() * maxLinearSpeedMetersPerSec / drivebaseRadius)));
  }

  public Command stopInX() {
    return run(
        () -> {
          swerveIO.drive(stopInXSetpoint, EMPTY_FORCES);
          lastSetpoint = stopInXSetpoint;
        });
  }

  public Command resetGyro() {
    return runOnce(() -> swerveIO.resetGyro(poseEstimator.getEstimatedPosition().getRotation()));
  }

  private void autoDrive(ChassisSpeeds speeds, double[] forceFeedforwards) {
    // teleopDrive uses the current drive state for more responsiveness, autoDrive uses the
    // previous generated setpoint for more consistency
    drive(speeds, lastSetpoint, forceFeedforwards);
  }

  private void teleopDrive(ChassisSpeeds speeds) {
    drive(
        toRobotRelative(speeds),
        new SwerveSetpoint(
            setpointGenerator.getKinematics().toChassisSpeeds(swerveInputs.moduleStates),
            swerveInputs.moduleStates,
            EMPTY_FORCES),
        EMPTY_FORCES);
  }

  private void drive(
      ChassisSpeeds speeds, SwerveSetpoint lastSetpoint, double[] forceFeedforwards) {
    var translationalMagnitude = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    if (translationalMagnitude > maxLinearSpeedMetersPerSec) {
      speeds.vxMetersPerSecond *= maxLinearSpeedMetersPerSec / translationalMagnitude;
      speeds.vyMetersPerSecond *= maxLinearSpeedMetersPerSec / translationalMagnitude;

      translationalMagnitude = maxLinearSpeedMetersPerSec;
    }
    speeds.omegaRadiansPerSecond *=
        MathUtil.interpolate(
            1,
            angularSpeedFudgeFactor,
            MathUtil.inverseInterpolate(0, maxLinearSpeedMetersPerSec, translationalMagnitude));

    var setpoint =
        setpointGenerator.generateSetpoint(
            lastSetpoint, speeds, RobotController.getBatteryVoltage());
    swerveIO.drive(setpoint, forceFeedforwards);
    this.lastSetpoint = setpoint;
  }

  private void fieldOrientedDrive(ChassisSpeeds speeds, double[] forceFeedforwards) {
    autoDrive(toRobotRelative(speeds), forceFeedforwards);
  }

  public void followTrajectory(Pose2d currentPose, SwerveSample sample) {
    var feedforward = new ChassisSpeeds(sample.vx, sample.vy, sample.omega);

    var targetPose = new Pose2d(sample.x, sample.y, new Rotation2d(sample.heading));
    var feedback =
        new ChassisSpeeds(
            xController.calculate(currentPose.getX(), targetPose.getX()),
            yController.calculate(currentPose.getY(), targetPose.getY()),
            yawController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians()));

    Logger.recordOutput("Choreo/Error", targetPose.minus(currentPose));

    var forceVectors = new double[4];
    for (int i = 0; i < 4; i++) {
      forceVectors[i] =
          new Translation2d(sample.moduleForcesX()[i], sample.moduleForcesY()[i])
              .rotateBy(new Rotation2d(-sample.heading))
              .getNorm();
    }

    Logger.recordOutput("Choreo/Target Pose", targetPose);
    Logger.recordOutput("Choreo/Feedforward", toRobotRelative(feedforward));
    Logger.recordOutput("Choreo/Feedback", toRobotRelative(feedback));

    var output = feedforward.plus(feedback);
    fieldOrientedDrive(output, forceVectors);
    Logger.recordOutput("Choreo/Output", toRobotRelative(output));
  }

  private ChassisSpeeds getRobotChassisSpeeds() {
    return setpointGenerator.getKinematics().toChassisSpeeds(swerveInputs.moduleStates);
  }

  private ChassisSpeeds toRobotRelative(ChassisSpeeds fieldRelative) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelative, swerveInputs.gyroYaw);
  }

  private ChassisSpeeds toFieldRelative(ChassisSpeeds robotRelative) {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, swerveInputs.gyroYaw);
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetForAuto(Pose2d pose) {
    if (Robot.isSimulation()
        && !Logger.hasReplaySource()
        && swerveIO instanceof SwerveIOPhoenix phoenix) {
      phoenix.resetGroundTruth(pose);
    }
    swerveIO.resetGyro(pose.getRotation());
    xController.reset();
    yController.reset();
    yawController.reset();
    poseEstimator.resetPosition(
        pose.getRotation(),
        Arrays.copyOfRange(
            swerveInputs.odometryPositions,
            swerveInputs.odometryPositions.length - 4,
            swerveInputs.odometryPositions.length),
        pose);
  }
}
