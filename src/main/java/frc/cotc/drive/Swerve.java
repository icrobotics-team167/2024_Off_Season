// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.cotc.drive.SwerveSetpointGenerator.SwerveSetpoint;
import static java.lang.Math.PI;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.cotc.Robot;
import frc.cotc.vision.VisionPoseEstimator;
import frc.cotc.vision.VisionPoseEstimatorIO;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final SwerveIO swerveIO;

  private final SwerveIOInputsAutoLogged swerveInputs;

  private final SwerveSetpointGenerator setpointGenerator;
  private final SwerveSetpoint stopInXSetpoint;
  private final double[] EMPTY_FORCES = new double[4];
  private SwerveSetpoint lastSetpoint;

  private final double maxLinearSpeedMetersPerSec;
  private final double maxAngularSpeedRadiansPerSec;

  private final SwerveDrivePoseEstimator poseEstimator;
  private final VisionPoseEstimator visionPoseEstimator;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController yawController;

  public Swerve(SwerveIO driveIO, VisionPoseEstimatorIO poseEstimatorIO) {
    this.swerveIO = driveIO;
    var CONSTANTS = driveIO.getConstants();
    swerveInputs = new SwerveIOInputsAutoLogged();
    driveIO.updateInputs(swerveInputs);
    Logger.processInputs("Swerve/Constants", CONSTANTS);

    maxLinearSpeedMetersPerSec =
        (CONSTANTS.DRIVE_MOTOR_MAX_SPEED / CONSTANTS.DRIVE_GEAR_RATIO)
            * (CONSTANTS.WHEEL_DIAMETER / 2);
    maxAngularSpeedRadiansPerSec =
        maxLinearSpeedMetersPerSec
            / Math.hypot(CONSTANTS.TRACK_WIDTH / 2, CONSTANTS.TRACK_LENGTH / 2);

    Logger.recordOutput("Swerve/Max Linear Speed", maxLinearSpeedMetersPerSec);
    Logger.recordOutput("Swerve/Max Angular Speed", maxAngularSpeedRadiansPerSec);

    setpointGenerator =
        new SwerveSetpointGenerator(
            new Translation2d[] {
              new Translation2d(CONSTANTS.TRACK_LENGTH / 2, CONSTANTS.TRACK_WIDTH / 2),
              new Translation2d(CONSTANTS.TRACK_LENGTH / 2, -CONSTANTS.TRACK_WIDTH / 2),
              new Translation2d(-CONSTANTS.TRACK_LENGTH / 2, CONSTANTS.TRACK_WIDTH / 2),
              new Translation2d(-CONSTANTS.TRACK_LENGTH / 2, -CONSTANTS.TRACK_WIDTH / 2)
            },
            new SwerveSetpointGenerator.ModuleLimits(
                maxLinearSpeedMetersPerSec,
                CONSTANTS.MAX_ACCELERATION,
                CONSTANTS.STEER_MOTOR_MAX_SPEED / CONSTANTS.STEER_GEAR_RATIO));
    stopInXSetpoint =
        new SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[] {
              new SwerveModuleState(
                  0,
                  new Rotation2d(
                      Math.atan2(CONSTANTS.TRACK_WIDTH / 2, CONSTANTS.TRACK_LENGTH / 2))),
              new SwerveModuleState(
                  0,
                  new Rotation2d(
                      Math.atan2(-CONSTANTS.TRACK_WIDTH / 2, CONSTANTS.TRACK_LENGTH / 2))),
              new SwerveModuleState(
                  0,
                  new Rotation2d(
                      Math.atan2(CONSTANTS.TRACK_WIDTH / 2, -CONSTANTS.TRACK_LENGTH / 2))),
              new SwerveModuleState(
                  0,
                  new Rotation2d(
                      Math.atan2(-CONSTANTS.TRACK_WIDTH / 2, -CONSTANTS.TRACK_LENGTH / 2)))
            },
            EMPTY_FORCES);
    lastSetpoint = new SwerveSetpoint(new ChassisSpeeds(), swerveInputs.moduleStates, EMPTY_FORCES);

    poseEstimator =
        new SwerveDrivePoseEstimator(
            setpointGenerator.getKinematics(),
            swerveInputs.gyroYaw,
            Arrays.copyOfRange(
                swerveInputs.odometryPositions,
                swerveInputs.odometryPositions.length - 4,
                swerveInputs.odometryPositions.length),
            new Pose2d());
    visionPoseEstimator =
        new VisionPoseEstimator(
            poseEstimatorIO,
            this::getRobotChassisSpeeds,
            (pose, timestamp, translationalStDevs, angularStDevs) -> {
              poseEstimator.addVisionMeasurement(
                  pose,
                  timestamp,
                  VecBuilder.fill(translationalStDevs, translationalStDevs, angularStDevs));
            });

    xController = new PIDController(5, 0, 0);
    yController = new PIDController(5, 0, 0);
    yawController = new PIDController(5, 0, 0);
    yawController.enableContinuousInput(-PI, PI);
  }

  @Override
  public void periodic() {
    swerveIO.updateInputs(swerveInputs);
    Logger.processInputs("Swerve", swerveInputs);
    for (int i = 0; i < swerveInputs.odometryTimestamps.length; i++) {
      poseEstimator.updateWithTime(
          swerveInputs.odometryTimestamps[i],
          swerveInputs.odometryYaws[i],
          Arrays.copyOfRange(swerveInputs.odometryPositions, i * 4, i * 4 + 4));
    }
    visionPoseEstimator.poll();
    Logger.recordOutput("Swerve/Actual Speed", getRobotChassisSpeeds());
    Logger.recordOutput("Swerve/Odometry Position", getPose());
  }

  public Command teleopDrive(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return run(
        () ->
            fieldOrientedDrive(
                new ChassisSpeeds(
                    xSupplier.getAsDouble() * maxLinearSpeedMetersPerSec,
                    ySupplier.getAsDouble() * maxLinearSpeedMetersPerSec,
                    omegaSupplier.getAsDouble() * maxAngularSpeedRadiansPerSec)));
  }

  public Command stopInX() {
    return run(
        () -> {
          swerveIO.drive(stopInXSetpoint, EMPTY_FORCES);
          lastSetpoint = stopInXSetpoint;
          Logger.recordOutput("Swerve/Commanded speeds", stopInXSetpoint.chassisSpeeds());
          Logger.recordOutput("Swerve/Drive setpoint", stopInXSetpoint.moduleStates());
        });
  }

  public Command driveCharacterization() {
    SysIdRoutine characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(4).per(Second),
                Volts.of(4),
                Seconds.of(4),
                (state) -> SignalLogger.writeString("SysIDState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> swerveIO.driveCharacterization(voltage.baseUnitMagnitude()),
                null,
                this));
    return sequence(
        characterizationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        stop().withTimeout(2),
        characterizationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        stop().withTimeout(2),
        characterizationRoutine.dynamic(SysIdRoutine.Direction.kForward),
        stop().withTimeout(2),
        characterizationRoutine.dynamic(SysIdRoutine.Direction.kReverse),
        stop().withTimeout(2));
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

  private void drive(ChassisSpeeds speeds, double[] forceFeedforwards) {
    var setpoint =
        setpointGenerator.generateSetpoint(lastSetpoint, speeds, Robot.defaultPeriodSecs);
    Logger.recordOutput("Swerve/Commanded speeds", setpoint.chassisSpeeds());
    Logger.recordOutput("Swerve/Drive setpoint", setpoint.moduleStates());
    swerveIO.drive(setpoint, forceFeedforwards);
    lastSetpoint = setpoint;
  }

  private void fieldOrientedDrive(ChassisSpeeds speeds) {
    fieldOrientedDrive(speeds, EMPTY_FORCES);
  }

  private void fieldOrientedDrive(ChassisSpeeds speeds, double[] forceFeedforwards) {
    drive(toRobotRelative(speeds), forceFeedforwards);
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
