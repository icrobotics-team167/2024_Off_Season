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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.vision.VisionPoseEstimatorIO;
import frc.cotc.vision.VisionPoseEstimatorIO.VisionPoseEstimatorIOInputs;
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

  private final double maxLinearSpeedMetersPerSec;
  private final double drivebaseRadius;
  private final double angularSpeedFudgeFactor;

  private final SwervePoseEstimator poseEstimator;

  private final VisionPoseEstimatorIO[] visionIOs;
  private final VisionPoseEstimatorIOInputs[] visionInputs;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController yawController;

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
            Arrays.copyOfRange(
                swerveInputs.odometryPositions,
                swerveInputs.odometryPositions.length - 4,
                swerveInputs.odometryPositions.length),
            swerveInputs.gyroYaw,
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
    Logger.recordOutput("Swerve/Odometry/Drive Std Devs/Translational", driveStdDevs.get(0));
    Logger.recordOutput("Swerve/Odometry/Drive Std Devs/Rotational", driveStdDevs.get(2));
    poseEstimator.setDriveMeasurementStdDevs(driveStdDevs);

    var drivePoseUpdates = new Pose2d[swerveInputs.odometryTimestamps.length];
    for (int i = 0; i < swerveInputs.odometryTimestamps.length; i++) {
      drivePoseUpdates[i] =
          poseEstimator.updateWithTime(
              swerveInputs.odometryTimestamps[i],
              swerveInputs.odometryYaws[i],
              Arrays.copyOfRange(swerveInputs.odometryPositions, i * 4, i * 4 + 4));
    }
    Logger.recordOutput("Swerve/Odometry/Drive pose updates", drivePoseUpdates);

    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i].updateInputs(visionInputs[i]);
      Logger.processInputs("Vision/" + i, visionInputs[i]);
    }

    Logger.recordOutput("Swerve/Odometry/Final Position", getPose());
  }

  private Vector<N3> getDriveStdDevs() {
    var idealStates =
        setpointGenerator.getKinematics().toSwerveModuleStates(getRobotChassisSpeeds());

    double squaredSum = 0;
    for (int i = 0; i < 4; i++) {
      var measuredVector =
          new Translation2d(
              swerveInputs.moduleStates[i].speedMetersPerSecond,
              swerveInputs.moduleStates[i].angle);
      var idealVector =
          new Translation2d(idealStates[i].speedMetersPerSecond, idealStates[i].angle);

      var delta = measuredVector.getDistance(idealVector);

      squaredSum += delta * delta;
    }

    // Sqrt of avg = standard deviation
    double linearStdDevs = Math.sqrt(squaredSum / 4);
    double angularStdDevs = linearStdDevs / drivebaseRadius;

    return VecBuilder.fill(linearStdDevs, linearStdDevs, angularStdDevs);
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

  private Command stop() {
    return run(
        () -> {
          lastSetpoint =
              new SwerveSetpoint(
                  new ChassisSpeeds(),
                  new SwerveModuleState[] {
                    new SwerveModuleState(0, swerveInputs.moduleStates[0].angle),
                    new SwerveModuleState(0, swerveInputs.moduleStates[1].angle),
                    new SwerveModuleState(0, swerveInputs.moduleStates[2].angle),
                    new SwerveModuleState(0, swerveInputs.moduleStates[3].angle)
                  },
                  EMPTY_FORCES);
          swerveIO.drive(lastSetpoint, EMPTY_FORCES);
        });
  }

  private void autoDrive(ChassisSpeeds speeds, double[] forceFeedforwards) {
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
    lastSetpoint = setpoint;
  }

  private void teleopDrive(ChassisSpeeds speeds) {
    speeds = toRobotRelative(speeds);

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
            new SwerveSetpoint(
                setpointGenerator.getKinematics().toChassisSpeeds(swerveInputs.moduleStates),
                swerveInputs.moduleStates,
                EMPTY_FORCES),
            speeds,
            RobotController.getBatteryVoltage());
    swerveIO.drive(setpoint, EMPTY_FORCES);
    lastSetpoint = setpoint;
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

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetForAuto(Pose2d pose) {
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
