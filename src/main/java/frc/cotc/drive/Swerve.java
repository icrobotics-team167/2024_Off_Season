// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.cotc.drive.SwerveSetpointGenerator.SwerveSetpoint;
import static java.lang.Math.PI;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.cotc.Robot;
import frc.cotc.vision.FiducialPoseEstimatorIO;
import frc.cotc.vision.FiducialPoseEstimatorIO.FiducialPoseEstimatorIOInputs;
import frc.cotc.vision.FiducialPoseEstimatorIOPhoton;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final SwerveIO swerveIO;

  private final SwerveIO.SwerveIOInputs swerveInputs;

  private final SwerveSetpointGenerator setpointGenerator;
  private final SwerveSetpoint stopInXSetpoint;
  private final double[] EMPTY_FEEDFORWARDS = new double[4];
  private SwerveSetpoint lastSetpoint;

  private final double maxLinearSpeedMetersPerSec;
  private final double maxAngularSpeedRadPerSec;
  private final double angularSpeedFudgeFactor;

  private final SwervePoseEstimator poseEstimator;

  private final FiducialPoseEstimatorIO[] visionIOs;
  private final FiducialPoseEstimatorIOInputs[] visionInputs;

  private final PIDController xController, yController, yawController;

  public Swerve(SwerveIO driveIO, FiducialPoseEstimatorIO[] visionIOs) {
    this.swerveIO = driveIO;
    var CONSTANTS = driveIO.getConstants();
    swerveInputs = new SwerveIO.SwerveIOInputs();
    driveIO.updateInputs(swerveInputs);
    Logger.processInputs("Swerve", swerveInputs);
    Logger.processInputs("Swerve/Constants", CONSTANTS);

    maxLinearSpeedMetersPerSec =
        CONSTANTS.DRIVE_MOTOR.freeSpeedRadPerSec * (CONSTANTS.WHEEL_DIAMETER_METERS / 2);
    maxAngularSpeedRadPerSec =
        maxLinearSpeedMetersPerSec
            / Math.hypot(CONSTANTS.TRACK_WIDTH_METERS / 2, CONSTANTS.TRACK_LENGTH_METERS / 2);
    angularSpeedFudgeFactor = CONSTANTS.ANGULAR_SPEED_FUDGING;

    Logger.recordOutput("Swerve/Max Linear Speed", maxLinearSpeedMetersPerSec);
    Logger.recordOutput("Swerve/Max Angular Speed", maxAngularSpeedRadPerSec);

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
            EMPTY_FEEDFORWARDS);
    lastSetpoint =
        new SwerveSetpoint(new ChassisSpeeds(), swerveInputs.moduleStates, EMPTY_FEEDFORWARDS);

    poseEstimator =
        new SwervePoseEstimator(
            setpointGenerator.getKinematics(),
            new Rotation2d(),
            getLatestModulePositions(),
            new Pose2d());
    this.visionIOs = visionIOs;
    visionInputs = new FiducialPoseEstimatorIOInputs[visionIOs.length];
    for (int i = 0; i < visionIOs.length; i++) {
      visionInputs[i] = new FiducialPoseEstimatorIOInputs();
    }

    xController = new PIDController(5, 0, 0);
    yController = new PIDController(5, 0, 0);
    yawController = new PIDController(5, 0, 0);
    yawController.enableContinuousInput(-PI, PI);
  }

  private final SwerveModuleState[] lastModuleForces =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();

  @Override
  public void periodic() {
    Logger.recordOutput("Swerve/Commanded speeds", lastSetpoint.chassisSpeeds());
    Logger.recordOutput("Swerve/Drive setpoint", lastSetpoint.moduleStates());
    Logger.recordOutput("Swerve/Force feedforwards", lastModuleForces);

    swerveIO.updateInputs(swerveInputs);
    Logger.processInputs("Swerve", swerveInputs);
    robotRelativeSpeeds = getRobotChassisSpeeds();
    Logger.recordOutput("Swerve/Actual Speed", robotRelativeSpeeds);

    // .toFieldRelative is a mutator method, so in order for robotRelativeSpeeds to stay intact, a
    // copy needs to be instantiated.
    fieldRelativeSpeeds =
        new ChassisSpeeds(
            robotRelativeSpeeds.vxMetersPerSecond,
            robotRelativeSpeeds.vyMetersPerSecond,
            robotRelativeSpeeds.omegaRadiansPerSecond);
    fieldRelativeSpeeds.toFieldRelativeSpeeds(swerveInputs.gyroYaw);

    var driveStdDevs = getDriveStdDevs();
    Logger.recordOutput("Swerve/Odometry/Drive Std Devs", driveStdDevs);
    poseEstimator.setDriveMeasurementStdDevs(driveStdDevs);

    for (var frame : swerveInputs.odometryFrames) {
      if (frame.timestamp() < 0) {
        break;
      }
      poseEstimator.updateWithTime(frame.timestamp(), frame.gyroYaw(), frame.positions());
    }

    if (Robot.isSimulation() && !Logger.hasReplaySource()) {
      FiducialPoseEstimatorIOPhoton.VisionSim.getInstance().update();
    }
    var tagPoses = new ArrayList<Pose3d>();
    var poseEstimates = new ArrayList<Pose3d>();
    var poseEstimateYaws = new ArrayList<Rotation2d>();
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i].updateInputs(visionInputs[i]);
      Logger.processInputs("Vision/" + i, visionInputs[i]);

      if (!visionInputs[i].hasNewData) {
        // If there's no new data, save some CPU
        continue;
      }

      for (int j = 0; j < visionInputs[i].poseEstimates.length; j++) {
        var poseEstimate = visionInputs[i].poseEstimates[j];

        // Discard if the pose is too far above/below the ground
        if (!MathUtil.isNear(0, poseEstimate.estimatedPose().getZ(), .05)) {
          continue;
        }

        var stdDevs = getVisionStdDevs(poseEstimate);
        Logger.recordOutput("Vision/Std Devs/" + i + "/" + j, stdDevs);

        poseEstimator.addVisionMeasurement(
            poseEstimate.estimatedPose().toPose2d(), poseEstimate.timestamp(), stdDevs);

        poseEstimates.add(poseEstimate.estimatedPose());
        poseEstimateYaws.add(new Rotation2d(poseEstimate.estimatedPose().getRotation().getZ()));
        tagPoses.addAll(Arrays.asList(poseEstimate.tagsUsed()));
      }
    }
    Logger.recordOutput("Vision/All pose estimates", poseEstimates.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/All pose estimates/yaws", poseEstimateYaws.toArray(new Rotation2d[0]));
    Logger.recordOutput("Vision/All tags used", tagPoses.toArray(new Pose3d[0]));

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
    // Get idealized states from the current robot velocity.
    var idealStates = setpointGenerator.getKinematics().toSwerveModuleStates(robotRelativeSpeeds);

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
    // Minimum of 25% to avoid divide by 0 errors and to prevent the scaled std dev from being
    // too large
    var xSpeed =
        Math.max(Math.abs(fieldRelativeSpeeds.vxMetersPerSecond) / maxLinearSpeedMetersPerSec, .25);
    var ySpeed =
        Math.max(Math.abs(fieldRelativeSpeeds.vyMetersPerSecond) / maxLinearSpeedMetersPerSec, .25);

    // Normalize the smaller one to 100% and scale the other to match
    var minMagnitude = Math.min(xSpeed, ySpeed);
    xSpeed /= minMagnitude;
    ySpeed /= minMagnitude;

    // Scale each axis by the normalized (?) % speed of each axis
    // Add a minimum to account for mechanical slop and to prevent divide by 0 errors
    return new double[] {(linearStdDevs + .005) * xSpeed, (linearStdDevs + .005) * ySpeed, .001};
  }

  private double[] getVisionStdDevs(FiducialPoseEstimatorIO.PoseEstimate poseEstimate) {
    double translationalScoreSum = 0;
    double rotationalScoreSum = 0;
    for (var distanceMeters : poseEstimate.tagDistances()) {
      translationalScoreSum += .125 * distanceMeters * distanceMeters;
      rotationalScoreSum += .0125 * distanceMeters * distanceMeters;
    }

    var translationalDivisor = Math.pow(poseEstimate.tagDistances().length, 5);
    var rotationalDivisor = Math.pow(poseEstimate.tagDistances().length, 2);

    var translationalVelMagnitude =
        Math.hypot(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond)
            / maxLinearSpeedMetersPerSec;
    var angularVelMagnitude =
        Math.abs(fieldRelativeSpeeds.omegaRadiansPerSecond) / maxAngularSpeedRadPerSec;

    var translationalScalar = MathUtil.interpolate(1, 5, translationalVelMagnitude);
    var rotationalScalar = MathUtil.interpolate(1, 5, angularVelMagnitude);

    return new double[] {
      translationalScoreSum * translationalScalar / translationalDivisor,
      translationalScoreSum * translationalScalar / translationalDivisor,
      rotationalScoreSum * rotationalScalar / rotationalDivisor
    };
  }

  private final ChassisSpeeds teleopDriveSpeeds = new ChassisSpeeds();
  private final Translation2d[] EMPTY_FORCES =
      new Translation2d[] {
        Translation2d.kZero, Translation2d.kZero, Translation2d.kZero, Translation2d.kZero
      };

  public Command teleopDrive(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return run(
        () -> {
          teleopDriveSpeeds.vxMetersPerSecond =
              xSupplier.getAsDouble() * maxLinearSpeedMetersPerSec;
          teleopDriveSpeeds.vyMetersPerSecond =
              ySupplier.getAsDouble() * maxLinearSpeedMetersPerSec;
          teleopDriveSpeeds.omegaRadiansPerSecond =
              omegaSupplier.getAsDouble() * maxAngularSpeedRadPerSec;

          teleopDriveSpeeds.toRobotRelativeSpeeds(swerveInputs.gyroYaw);
          drive(teleopDriveSpeeds, lastSetpoint, EMPTY_FORCES);
        });
  }

  public Command stopInX() {
    return run(() -> {
          swerveIO.drive(stopInXSetpoint, EMPTY_FEEDFORWARDS);
          lastSetpoint = stopInXSetpoint;
        })
        .ignoringDisable(true);
  }

  public Command resetGyro() {
    return runOnce(
        () -> {
          var pose = poseEstimator.getEstimatedPosition();
          swerveIO.resetGyro(poseEstimator.getEstimatedPosition().getRotation());
          poseEstimator.resetPosition(pose.getRotation(), getLatestModulePositions(), pose);
        });
  }

  private void autoDrive(ChassisSpeeds speeds, Translation2d[] forceFeedforwards) {
    // teleopDrive uses the current drive state for more responsiveness, autoDrive uses the
    // previous generated setpoint for more consistency
    drive(speeds, lastSetpoint, forceFeedforwards);
  }

  private void drive(
      ChassisSpeeds speeds, SwerveSetpoint lastSetpoint, Translation2d[] forceVectors) {
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

    double[] forceFeedforwards = new double[4];
    for (int i = 0; i < 4; i++) {
      if (MathUtil.isNear(0, forceVectors[i].getNorm(), 1e-6)) {
        forceFeedforwards[i] = 0;
        continue;
      }
      var scalar = forceVectors[i].getAngle().minus(setpoint.moduleStates()[i].angle).getCos();
      forceFeedforwards[i] = forceVectors[i].getNorm() * scalar;
      lastModuleForces[i] =
          new SwerveModuleState(
              forceVectors[i].getNorm() * Math.abs(scalar) / 20, forceVectors[i].getAngle());
    }

    swerveIO.drive(setpoint, forceFeedforwards);
    this.lastSetpoint = setpoint;
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

    var forceVectors = new Translation2d[4];
    for (int i = 0; i < 4; i++) {
      forceVectors[i] =
          new Translation2d(sample.moduleForcesX()[i], sample.moduleForcesY()[i])
              .rotateBy(new Rotation2d(-sample.heading));
    }

    feedforward.toRobotRelativeSpeeds(swerveInputs.gyroYaw);
    feedback.toRobotRelativeSpeeds(swerveInputs.gyroYaw);

    Logger.recordOutput("Choreo/Target Pose", targetPose);
    Logger.recordOutput("Choreo/Feedforward", feedforward);
    Logger.recordOutput("Choreo/Feedback", feedback);

    var output = feedforward.plus(feedback);
    autoDrive(output, forceVectors);
    Logger.recordOutput("Choreo/Output", output);
  }

  public Command steerCharacterize() {
    var sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(6),
                Seconds.of(12),
                state -> SignalLogger.writeString("SysIDState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> swerveIO.steerCharacterization(voltage.baseUnitMagnitude()),
                null,
                this));

    return sequence(
        runOnce(swerveIO::initSysId),
        sysId.quasistatic(SysIdRoutine.Direction.kForward),
        waitSeconds(1),
        sysId.quasistatic(SysIdRoutine.Direction.kReverse),
        waitSeconds(1),
        sysId.dynamic(SysIdRoutine.Direction.kForward),
        waitSeconds(1),
        sysId.dynamic(SysIdRoutine.Direction.kReverse));
  }

  private ChassisSpeeds getRobotChassisSpeeds() {
    return setpointGenerator.getKinematics().toChassisSpeeds(swerveInputs.moduleStates);
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public SwerveModulePosition[] getLatestModulePositions() {
    if (swerveInputs.odometryFrames.length == 0) {
      throw new IndexOutOfBoundsException(
          "swerveInputs.odometryFrames.length was 0! This should not be possible.");
    }
    return swerveInputs.odometryFrames[swerveInputs.odometryFrames.length - 1].positions();
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
    poseEstimator.resetPosition(pose.getRotation(), getLatestModulePositions(), pose);
  }
}
