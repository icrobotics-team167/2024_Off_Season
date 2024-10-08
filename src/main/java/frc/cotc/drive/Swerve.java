// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import static frc.cotc.drive.SwerveSetpointGenerator.ModuleLimits;
import static frc.cotc.drive.SwerveSetpointGenerator.SwerveSetpoint;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private SwerveSetpoint lastSetpoint;
  private final SwerveSetpoint stopInXSetpoint;

  private final double maxLinearSpeedMetersPerSec;
  private final double maxAngularSpeedRadiansPerSec;

  private final SwerveDrivePoseEstimator poseEstimator;
  private final VisionPoseEstimator visionPoseEstimator;

  public Swerve(SwerveIO driveIO, VisionPoseEstimatorIO poseEstimatorIO) {
    this.swerveIO = driveIO;
    var CONSTANTS = driveIO.getConstants();
    swerveInputs = new SwerveIOInputsAutoLogged();
    driveIO.updateInputs(swerveInputs);
    Logger.processInputs("Swerve/Constants", CONSTANTS);

    maxLinearSpeedMetersPerSec =
        (CONSTANTS.DRIVE_MAX_ROTOR_VELOCITY / CONSTANTS.DRIVE_GEAR_RATIO)
            * (CONSTANTS.WHEEL_DIAMETER / 2);
    maxAngularSpeedRadiansPerSec =
        maxLinearSpeedMetersPerSec
            / Math.hypot(CONSTANTS.TRACK_WIDTH / 2, CONSTANTS.TRACK_LENGTH / 2);

    setpointGenerator =
        new SwerveSetpointGenerator(
            new Translation2d[] {
              new Translation2d(CONSTANTS.TRACK_WIDTH / 2, CONSTANTS.TRACK_LENGTH / 2),
              new Translation2d(CONSTANTS.TRACK_WIDTH / 2, -CONSTANTS.TRACK_LENGTH / 2),
              new Translation2d(-CONSTANTS.TRACK_WIDTH / 2, CONSTANTS.TRACK_LENGTH / 2),
              new Translation2d(-CONSTANTS.TRACK_WIDTH / 2, -CONSTANTS.TRACK_LENGTH / 2),
            },
            new ModuleLimits(
                maxLinearSpeedMetersPerSec,
                CONSTANTS.MAX_ACCEL,
                CONSTANTS.STEER_MAX_ROTOR_VELOCITY / CONSTANTS.STEER_GEAR_RATIO));
    lastSetpoint = new SwerveSetpoint(new ChassisSpeeds(), swerveInputs.moduleStates);
    stopInXSetpoint =
        new SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[] {
              new SwerveModuleState(
                  0,
                  new Rotation2d(
                      Math.atan2(CONSTANTS.TRACK_LENGTH / 2, CONSTANTS.TRACK_WIDTH / 2))),
              new SwerveModuleState(
                  0,
                  new Rotation2d(
                      Math.atan2(CONSTANTS.TRACK_LENGTH / 2, CONSTANTS.TRACK_WIDTH / 2))),
              new SwerveModuleState(
                  0,
                  new Rotation2d(
                      Math.atan2(CONSTANTS.TRACK_LENGTH / 2, CONSTANTS.TRACK_WIDTH / 2))),
              new SwerveModuleState(
                  0,
                  new Rotation2d(Math.atan2(CONSTANTS.TRACK_LENGTH / 2, CONSTANTS.TRACK_WIDTH / 2)))
            });

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
            this::getChassisSpeeds,
            (pose, timestamp, translationalStDevs, angularStDevs) -> {
              poseEstimator.addVisionMeasurement(
                  pose,
                  timestamp,
                  VecBuilder.fill(translationalStDevs, translationalStDevs, angularStDevs));
            });
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
          swerveIO.drive(stopInXSetpoint.moduleStates());
          lastSetpoint = stopInXSetpoint;
        });
  }

  private void drive(ChassisSpeeds speeds) {
    var setpoint =
        setpointGenerator.generateSetpoint(lastSetpoint, speeds, Robot.defaultPeriodSecs);
    swerveIO.drive(setpoint.moduleStates());
    lastSetpoint = setpoint;
  }

  private void fieldOrientedDrive(ChassisSpeeds speeds) {
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, swerveInputs.gyroYaw));
  }

  private ChassisSpeeds getChassisSpeeds() {
    return setpointGenerator.getKinematics().toChassisSpeeds(swerveInputs.moduleStates);
  }
}
