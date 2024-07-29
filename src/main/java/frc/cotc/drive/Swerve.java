// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.Robot;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final SwerveIO io;
  private final SwerveIOInputsAutoLogged inputs;
  private final SwerveIOConstantsAutoLogged CONSTANTS;

  private SwerveSetpointGenerator.SwerveSetpoint lastSetpoint;
  private final SwerveSetpointGenerator setpointGenerator;

  private final double TOP_SPEED;
  private final double MAX_OMEGA;

  public Swerve(SwerveIO io) {
    this.io = io;
    inputs = new SwerveIOInputsAutoLogged();
    io.updateInputs(inputs);

    // Slightly cursed way to get stats specific to each hardware implementation
    CONSTANTS = io.getConstants();
    Logger.processInputs("Swerve/Constants", CONSTANTS);

    setpointGenerator =
        new SwerveSetpointGenerator(
            new Translation2d[] {
              new Translation2d(CONSTANTS.TRACK_WIDTH / 2, CONSTANTS.TRACK_LENGTH / 2),
              new Translation2d(CONSTANTS.TRACK_WIDTH / 2, -CONSTANTS.TRACK_LENGTH / 2),
              new Translation2d(-CONSTANTS.TRACK_WIDTH / 2, CONSTANTS.TRACK_LENGTH / 2),
              new Translation2d(-CONSTANTS.TRACK_WIDTH / 2, -CONSTANTS.TRACK_LENGTH / 2)
            });

    lastSetpoint =
        new SwerveSetpointGenerator.SwerveSetpoint(
            new ChassisSpeeds(), inputs.moduleStates, new double[] {0, 0, 0, 0});

    TOP_SPEED =
        (CONSTANTS.MAX_ROTOR_VELOCITY / CONSTANTS.DRIVE_GEAR_RATIO)
            * (CONSTANTS.WHEEL_DIAMETER / 2.0);

    MAX_OMEGA = TOP_SPEED / Math.hypot(CONSTANTS.TRACK_WIDTH / 2, CONSTANTS.TRACK_LENGTH / 2);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve", inputs);
  }

  public void drive(ChassisSpeeds speed) {
    var limits =
        new SwerveSetpointGenerator.ModuleLimits(
            TOP_SPEED,
            CONSTANTS.MAX_ACCEL,
            CONSTANTS.MAX_ROTOR_VELOCITY / CONSTANTS.STEER_GEAR_RATIO);

    var setpoint =
        setpointGenerator.generateSetpoint(limits, lastSetpoint, speed, Robot.defaultPeriodSecs);

    Logger.recordOutput("Swerve/Drive Setpoint", setpoint.moduleStates());
    io.drive(setpoint.moduleStates(), setpoint.steerFeedforward());

    lastSetpoint = setpoint;
  }

  public void stopInX() {
    var setpoint =
        new Rotation2d[] {
          new Rotation2d(CONSTANTS.TRACK_WIDTH / 2, CONSTANTS.TRACK_LENGTH / 2),
          new Rotation2d(-CONSTANTS.TRACK_WIDTH / 2, CONSTANTS.TRACK_LENGTH / 2),
          new Rotation2d(CONSTANTS.TRACK_WIDTH / 2, -CONSTANTS.TRACK_LENGTH / 2),
          new Rotation2d(-CONSTANTS.TRACK_WIDTH / 2, -CONSTANTS.TRACK_LENGTH / 2),
        };
    io.stopWithAngles(setpoint);

    lastSetpoint =
        new SwerveSetpointGenerator.SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[] {
              new SwerveModuleState(0, setpoint[0]),
              new SwerveModuleState(0, setpoint[1]),
              new SwerveModuleState(0, setpoint[2]),
              new SwerveModuleState(0, setpoint[3]),
            },
            new double[4]);
  }

  /** Command for controlling the drivebase from driver controls. */
  public Command teleopDrive(
      DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier rotInput) {
    return run(() -> {
          double xVel = xInput.getAsDouble() * TOP_SPEED;
          double yVel = yInput.getAsDouble() * TOP_SPEED;
          double omega = rotInput.getAsDouble() * MAX_OMEGA;

          double translationalVel = Math.hypot(xVel, yVel);
          if (translationalVel > TOP_SPEED) {
            xVel /= (translationalVel / TOP_SPEED);
            yVel /= (translationalVel / TOP_SPEED);
          }

          drive(
              ChassisSpeeds.discretize(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      new ChassisSpeeds(xVel, yVel, omega), inputs.gyroYaw),
                  Robot.defaultPeriodSecs));
        })
        .withName("Swerve teleop drive");
  }
}
