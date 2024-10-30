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
import frc.cotc.util.MotorCurrentDraws;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SwerveIO {
  class SwerveIOInputs implements LoggableInputs {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    Rotation2d gyroYaw;

    // I wanted this to be a 2D array, so that one of the dimensions can be data ID and the other
    // dimension can be the module ID, but AK doesn't support 2D arrays
    // So the data is packed into a 1D array
    SwerveModulePosition[] odometryPositions;
    Rotation2d[] odometryYaws;
    double[] odometryTimestamps;

    MotorCurrentDraws[] driveMotorCurrents = new MotorCurrentDraws[4];
    MotorCurrentDraws[] steerMotorCurrents = new MotorCurrentDraws[4];

    @Override
    public void toLog(LogTable table) {
      table.put("ModuleStates", moduleStates);
      table.put("GyroYaw", gyroYaw);
      table.put("OdometryPositions", odometryPositions);
      table.put("OdometryYaws", odometryYaws);
      table.put("OdometryTimestamps", odometryTimestamps);
      table.put("DriveMotorCurrents", MotorCurrentDraws.struct, driveMotorCurrents);
      table.put("SteerMotorCurrents", MotorCurrentDraws.struct, steerMotorCurrents);
    }

    @Override
    public void fromLog(LogTable table) {
      moduleStates = table.get("ModuleStates", moduleStates);
      gyroYaw = table.get("GyroYaw", gyroYaw);
      odometryPositions = table.get("OdometryPositions", odometryPositions);
      odometryYaws = table.get("OdometryYaws", odometryYaws);
      odometryTimestamps = table.get("OdometryTimestamps", odometryTimestamps);
      driveMotorCurrents =
          table.get("DriveMotorCurrents", MotorCurrentDraws.struct, driveMotorCurrents);
      steerMotorCurrents =
          table.get("SteerMotorCurrents", MotorCurrentDraws.struct, steerMotorCurrents);
    }
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
    double STEER_GEAR_RATIO;

    // Rad/sec
    double DRIVE_MOTOR_MAX_SPEED;
    double STEER_MOTOR_MAX_SPEED;

    // Meters/sec^2
    double MAX_LINEAR_ACCELERATION;

    // Due to kinematic limits, it may not be possible for the bot to stay moving straight when
    // spinning and moving at the same time. This fudge factor slows down the max angular speed
    // when the bot is translating, but doesn't affect the limit when the bot isn't translating.
    // Scalar
    double ANGULAR_SPEED_FUDGING;
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
   * @param forceFeedforward The feedforward for the drive motor. Newtons.
   */
  default void drive(SwerveSetpointGenerator.SwerveSetpoint setpoint, double[] forceFeedforward) {}

  default void resetGyro(Rotation2d newYaw) {}

  default void driveCharacterization(double volts) {}
}
