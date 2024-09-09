// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {
  @AutoLog
  class SwerveIOInputs {
    SwerveModuleState[] moduleStates =
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
        };
    Rotation2d gyroYaw = new Rotation2d();

    double[] odometryTimestamps = new double[0];
    // I wanted this to be a 2D array, so that one of the dimensions can be data ID and the other
    // dimension can be the module ID, but AK doesn't support 2D arrays
    // So the data is packed into a 1D array
    SwerveModulePosition[] odometryPositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };
    Rotation2d[] odometryYaws = new Rotation2d[0];
  }

  @SuppressWarnings("CanBeFinal")
  @AutoLog
  class SwerveIOConstants {
    // Choreo defaults
    // Meters
    double TRACK_LENGTH = Units.inchesToMeters(22.75);
    double TRACK_WIDTH = Units.inchesToMeters(22.75);
    double WHEEL_DIAMETER = Units.inchesToMeters(4);

    // Reductions
    double DRIVE_GEAR_RATIO = 6.75;
    double STEER_GEAR_RATIO = 150.0 / 7.0;

    // Inversions
    boolean[] DRIVE_MOTOR_INVERSIONS = new boolean[] {false, false, false, false};
    boolean STEER_MOTOR_INVERTED = true;

    // Rad/sec
    double MAX_ROTOR_VELOCITY = Units.rotationsPerMinuteToRadiansPerSecond(6000);

    /*
     * Acceleration limits shouldn't be used to limit current draw, that's the purpose of current
     * limits and linear accel limits feel terrible on controls
     *
     * Instead, use this to prevent wheel slip, and limit acceleration using current limits
     */
    // Meters/sec^2
    double MAX_ACCEL = 8;
  }

  /**
   * Gets the drive constants.
   *
   * @return A SwerveIOConstantsAutoLogged object that contains constants.
   */
  default SwerveIOConstantsAutoLogged getConstants() {
    return new SwerveIOConstantsAutoLogged();
  }

  /**
   * Updates the {@link SwerveIOInputs} to feed in new data from the drivebase.
   *
   * @param inputs The SwerveIOInputs object. Will be mutated.
   */
  default void updateInputs(SwerveIOInputs inputs) {}

  /**
   * Drives the drivebase.
   *
   * @param setpoint The drive setpoint.
   * @param steerFeedforward The feedforward for the steering. Rad/sec.
   */
  default void drive(SwerveModuleState[] setpoint, double[] steerFeedforward) {
    drive(setpoint, steerFeedforward, new double[] {0, 0, 0, 0});
  }

  /**
   * Drives the drivebase.
   *
   * @param setpoint The drive setpoint.
   * @param steerFeedforward The feedforward for the steering. Rad/sec.
   * @param torqueFeedforward The feedforward for the drive motor. Newtons.
   */
  default void drive(
      SwerveModuleState[] setpoint, double[] steerFeedforward, double[] torqueFeedforward) {}

  default void steerCharacterization(double volts) {}

  default void driveCharacterization(double volts) {}

  default void stop() {}

  default void stopWithAngles(Rotation2d[] angles) {}
}
