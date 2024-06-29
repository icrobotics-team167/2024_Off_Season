// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import edu.wpi.first.math.MathUtil;
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
        new SwerveSetpointGenerator.SwerveSetpoint(new ChassisSpeeds(), inputs.moduleStates);

    TOP_SPEED =
        (CONSTANTS.MAX_ROTOR_SPEED / CONSTANTS.DRIVE_GEAR_RATIO) * (CONSTANTS.WHEEL_DIAMETER / 2.0);

    MAX_OMEGA = TOP_SPEED / Math.hypot(CONSTANTS.TRACK_WIDTH / 2, CONSTANTS.TRACK_LENGTH / 2);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve", inputs);
  }

  public void drive(ChassisSpeeds speed) {
    double maxAccel = getMaxAccel();
    SwerveSetpointGenerator.ModuleLimits limits =
        new SwerveSetpointGenerator.ModuleLimits(
            TOP_SPEED, maxAccel, CONSTANTS.MAX_ROTOR_SPEED / CONSTANTS.STEER_GEAR_RATIO);

    SwerveSetpointGenerator.SwerveSetpoint setpoint =
        setpointGenerator.generateSetpoint(limits, lastSetpoint, speed, Robot.defaultPeriodSecs);

    Logger.recordOutput("Swerve/Drive accel limit", maxAccel);
    Logger.recordOutput("Swerve/Drive Setpoint", setpoint.moduleStates());
    io.drive(setpoint.moduleStates());

    lastSetpoint = setpoint;
  }

  /**
   * Calculates a max acceleration value from the last setpoints' module speeds
   *
   * <p>A very rough approximation of motor dynamics, but I can't be bothered to figure out the math
   * to do it for real.
   *
   * <p>Maps wheel speeds from a meters per sec speed to a 0-1 value, and then lerps between a max
   * accel at 0 and a min accel at 1.
   */
  private double getMaxAccel() {
    // Get the highest wheel speed
    double highestSpeed = 0;
    for (SwerveModuleState state : lastSetpoint.moduleStates()) {
      if (Math.abs(state.speedMetersPerSecond) > highestSpeed) {
        highestSpeed = Math.abs(state.speedMetersPerSecond);
      }
    }
    return MathUtil.interpolate(
        CONSTANTS.MAX_ACCEL,
        CONSTANTS.MIN_ACCEL,
        MathUtil.inverseInterpolate(0, TOP_SPEED, highestSpeed));
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

          drive(new ChassisSpeeds(xVel, yVel, omega));
        })
        .withName("Swerve teleop drive");
  }
}
