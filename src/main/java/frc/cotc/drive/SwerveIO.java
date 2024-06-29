// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

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
  }

  @AutoLog
  class SwerveIOConstants {
    // Choreo defaults
    // Meters
    double TRACK_LENGTH = Units.inchesToMeters(22.75);
    double TRACK_WIDTH = Units.inchesToMeters(22.75);
    double WHEEL_DIAMETER = Units.inchesToMeters(4);

    // Reductions
    double DRIVE_GEAR_RATIO = 6.75;
    double STEER_GEAR_RATIO = 150.0 / 70.0;

    // Rad/sec
    double MAX_ROTOR_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(5800);

    // Meters/sec^2
    double MAX_ACCEL = 8;
    double MIN_ACCEL = 4;
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
   * @param setpoint The drive setpoint. Meters per second.
   */
  default void drive(SwerveModuleState[] setpoint) {}
}
