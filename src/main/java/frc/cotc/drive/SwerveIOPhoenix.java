// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.cotc.RobotConstants;

public class SwerveIOPhoenix implements SwerveIO {
  private final SwerveIOConstantsAutoLogged CONSTANTS;
  private final Module[] modules;

  public SwerveIOPhoenix() {
    CONSTANTS = new SwerveIOConstantsAutoLogged();
    CONSTANTS.DRIVE_GEAR_RATIO = 6.75;
    CONSTANTS.STEER_GEAR_RATIO = 150.0 / 7.0;
    CONSTANTS.WHEEL_DIAMETER = Units.inchesToMeters(4);
    CONSTANTS.TRACK_WIDTH = Units.inchesToMeters(22.75);
    CONSTANTS.TRACK_LENGTH = Units.inchesToMeters(22.75);
    CONSTANTS.MAX_ROTOR_VELOCITY = Units.rotationsPerMinuteToRadiansPerSecond(5800);
    CONSTANTS.DRIVE_MOTOR_INVERTED = false;
    CONSTANTS.STEER_MOTOR_INVERTED = true;
    CONSTANTS.MAX_ACCEL = 8;

    modules =
        new Module[] {
          new Module(0, CONSTANTS),
          new Module(1, CONSTANTS),
          new Module(2, CONSTANTS),
          new Module(3, CONSTANTS)
        };
  }

  @Override
  public SwerveIOConstantsAutoLogged getConstants() {
    return CONSTANTS;
  }

  @Override
  public void updateInputs(SwerveIOInputs inputs) {
    SwerveIO.super.updateInputs(inputs);
  }

  @Override
  public void drive(
      SwerveModuleState[] setpoint, double[] steerFeedforward, double[] forceFeedforward) {
    for (int i = 0; i < 4; i++) {
      modules[i].drive(setpoint[i], steerFeedforward[i], forceFeedforward[i]);
    }
  }

  @Override
  public void stop() {
    for (Module module : modules) {
      module.stop();
    }
  }

  @Override
  public void stopWithAngles(Rotation2d[] angles) {
    for (int i = 0; i < 4; i++) {
      modules[i].stopWithAngle(angles[i]);
    }
  }

  private class Module {
    final TalonFX driveMotor;
    final TalonFX steerMotor;
    final CANcoder steerEncoder;

    final double wheelDiameter;
    final double maxRotorVelocity;
    final double driveGearRatio;
    final double steerGearRatio;

    protected Module(int id, SwerveIOConstantsAutoLogged constants) {
      driveMotor = new TalonFX(id * 3, RobotConstants.CANIVORE_NAME);
      steerMotor = new TalonFX(id * 3 + 1, RobotConstants.CANIVORE_NAME);
      steerEncoder = new CANcoder(id * 3 + 2, RobotConstants.CANIVORE_NAME);

      var driveConfig = new TalonFXConfiguration();
      driveConfig.Feedback.SensorToMechanismRatio = CONSTANTS.DRIVE_GEAR_RATIO;
      driveConfig.MotorOutput.Inverted =
          constants.DRIVE_MOTOR_INVERTED
              ? InvertedValue.Clockwise_Positive
              : InvertedValue.CounterClockwise_Positive;
      driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
      driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = 80;
      driveConfig.Slot0.kS = 0;
      driveConfig.Slot0.kV = 0;
      driveConfig.Slot0.kA = 0; // TODO: SysID these values
      driveConfig.Slot0.kP = 0;
      driveConfig.Slot0.kD = 0;
      driveConfig.Audio.AllowMusicDurDisable = true;
      driveConfig.MotionMagic.MotionMagicAcceleration =
          CONSTANTS.MAX_ACCEL * CONSTANTS.DRIVE_GEAR_RATIO;
      driveMotor.getConfigurator().apply(driveConfig);

      var encoderConfig = new CANcoderConfiguration();
      steerEncoder.getConfigurator().apply(encoderConfig);

      var steerConfig = new TalonFXConfiguration();
      steerConfig.Feedback.SensorToMechanismRatio = CONSTANTS.STEER_GEAR_RATIO;
      steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
      steerConfig.MotorOutput.Inverted =
          constants.STEER_MOTOR_INVERTED
              ? InvertedValue.Clockwise_Positive
              : InvertedValue.CounterClockwise_Positive;
      steerConfig.CurrentLimits.StatorCurrentLimit = 40;
      steerConfig.Slot0.kS = 0;
      steerConfig.Slot0.kP = 0; // TODO: SysID these values
      steerConfig.Slot0.kD = 0;
      steerConfig.Audio.AllowMusicDurDisable = true;
      steerMotor.getConfigurator().apply(steerConfig);

      wheelDiameter = constants.WHEEL_DIAMETER;
      maxRotorVelocity = constants.MAX_ROTOR_VELOCITY;
      driveGearRatio = constants.DRIVE_GEAR_RATIO;
      steerGearRatio = constants.STEER_GEAR_RATIO;
    }

    private final VelocityTorqueCurrentFOC driveControlRequest =
        new VelocityTorqueCurrentFOC(0, 0, 0, 0, true, false, false);
    private final PositionVoltage steerControlRequest = new PositionVoltage(0);

    protected void drive(
        SwerveModuleState setpoint, double steerFeedforward, double forceFeedforward) {
      if (MathUtil.isNear(0, setpoint.speedMetersPerSecond, 1e-3)
          && MathUtil.isNear(0, steerFeedforward, 1e-3)
          && MathUtil.isNear(0, forceFeedforward, 1e-3)) {
        stop();
        return;
      }

      driveMotor.setControl(
          driveControlRequest
              .withVelocity(
                  (setpoint.speedMetersPerSecond / (wheelDiameter * Math.PI)) * driveGearRatio)
              .withFeedForward(
                  // ((Force (N) * radius (m)) / gear ratio (dimensionless)) / kT (Nm/amp) = amps
                  ((forceFeedforward * (wheelDiameter / 2)) / driveGearRatio) / 9.81));
      steerMotor.setControl(
          steerControlRequest
              .withPosition(setpoint.angle.getRotations())
              .withFeedForward(12.0 * (steerFeedforward / (maxRotorVelocity / steerGearRatio))));
    }

    private final StaticBrake brakeControlRequest = new StaticBrake();

    private void stop() {
      driveMotor.setControl(brakeControlRequest);
      steerMotor.setControl(brakeControlRequest);
    }

    protected void stopWithAngle(Rotation2d angle) {
      driveMotor.setControl(brakeControlRequest);
      steerMotor.setControl(
          steerControlRequest.withPosition(angle.getRotations()).withFeedForward(0));
    }
  }
}
