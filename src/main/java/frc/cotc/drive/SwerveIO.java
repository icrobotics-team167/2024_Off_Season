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
    double[] odometryTimestamps = new double[0];
  }

  @SuppressWarnings("CanBeFinal")
  @AutoLog
  class SwerveModuleConstants {
    // Meters
    double TRACK_WIDTH;
    double TRACK_LENGTH;
    double WHEEL_DIAMETER;

    // Reductions
    double DRIVE_GEAR_RATIO;

    // Rad/sec
    double DRIVE_MOTOR_MAX_SPEED;
  }

  /**
   * Gets the drive constants.
   *
   * @return A SwerveIOConstantsAutoLogged object that contains constants.
   */
  default SwerveModuleConstantsAutoLogged getConstants() {
    return new SwerveModuleConstantsAutoLogged();
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
   */
  default void drive(SwerveModuleState[] setpoint) {
    drive(setpoint, new double[4]);
  }

  /**
   * Drives the drivebase.
   *
   * @param setpoint The drive setpoint.
   * @param torqueFeedforward The feedforward for the drive motor. Newtons.
   */
  default void drive(SwerveModuleState[] setpoint, double[] torqueFeedforward) {}

  default void steerCharacterization(double volts) {}

  default void driveCharacterization(double volts) {}
}
