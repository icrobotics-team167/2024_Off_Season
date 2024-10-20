// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.cotc.vision.VisionPoseEstimator;
import frc.cotc.vision.VisionPoseEstimatorIO;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final SwerveIO swerveIO;

  private final SwerveIOInputsAutoLogged swerveInputs;

  private final SwerveDriveKinematics kinematics;
  private final SwerveModuleState[] stopInXSetpoint;

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
        (CONSTANTS.DRIVE_MOTOR_MAX_SPEED / CONSTANTS.DRIVE_GEAR_RATIO)
            * (CONSTANTS.WHEEL_DIAMETER / 2);
    maxAngularSpeedRadiansPerSec =
        maxLinearSpeedMetersPerSec
            / Math.hypot(CONSTANTS.TRACK_WIDTH / 2, CONSTANTS.TRACK_LENGTH / 2);

    Logger.recordOutput("Swerve/Max Linear Speed", maxLinearSpeedMetersPerSec);
    Logger.recordOutput("Swerve/Max Angular Speed", maxAngularSpeedRadiansPerSec);

    kinematics =
        new SwerveDriveKinematics(
            new Translation2d(CONSTANTS.TRACK_LENGTH / 2, CONSTANTS.TRACK_WIDTH / 2),
            new Translation2d(CONSTANTS.TRACK_LENGTH / 2, -CONSTANTS.TRACK_WIDTH / 2),
            new Translation2d(-CONSTANTS.TRACK_LENGTH / 2, CONSTANTS.TRACK_WIDTH / 2),
            new Translation2d(-CONSTANTS.TRACK_LENGTH / 2, -CONSTANTS.TRACK_WIDTH / 2));
    stopInXSetpoint =
        new SwerveModuleState[] {
          new SwerveModuleState(
              0, new Rotation2d(Math.atan2(CONSTANTS.TRACK_WIDTH / 2, CONSTANTS.TRACK_LENGTH / 2))),
          new SwerveModuleState(
              0,
              new Rotation2d(Math.atan2(-CONSTANTS.TRACK_WIDTH / 2, CONSTANTS.TRACK_LENGTH / 2))),
          new SwerveModuleState(
              0,
              new Rotation2d(Math.atan2(CONSTANTS.TRACK_WIDTH / 2, -CONSTANTS.TRACK_LENGTH / 2))),
          new SwerveModuleState(
              0,
              new Rotation2d(Math.atan2(-CONSTANTS.TRACK_WIDTH / 2, -CONSTANTS.TRACK_LENGTH / 2)))
        };

    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
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
    Logger.recordOutput(
        "Swerve/Actual Speed", kinematics.toChassisSpeeds(swerveInputs.moduleStates));
    Logger.recordOutput("Swerve/Odometry Position", poseEstimator.getEstimatedPosition());
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
    return run(() -> swerveIO.drive(stopInXSetpoint));
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
        () ->
            swerveIO.drive(
                new SwerveModuleState[] {
                  new SwerveModuleState(),
                  new SwerveModuleState(),
                  new SwerveModuleState(),
                  new SwerveModuleState()
                }));
  }

  private void drive(ChassisSpeeds speeds) {
    var setpoint = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpoint, maxLinearSpeedMetersPerSec);
    for (int i = 0; i < 4; i++) {
      setpoint[i] = SwerveModuleState.optimize(setpoint[i], swerveInputs.moduleStates[i].angle);
      setpoint[i].speedMetersPerSecond *=
          MathUtil.clamp(
              Math.cos(setpoint[i].angle.minus(swerveInputs.moduleStates[i].angle).getRadians()),
              0,
              1);
    }
    Logger.recordOutput("Swerve/Commanded speeds", kinematics.toChassisSpeeds(setpoint));
    Logger.recordOutput("Swerve/Drive setpoint", setpoint);
    swerveIO.drive(setpoint);
  }

  private void fieldOrientedDrive(ChassisSpeeds speeds) {
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, swerveInputs.gyroYaw));
  }

  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(swerveInputs.moduleStates);
  }
}
