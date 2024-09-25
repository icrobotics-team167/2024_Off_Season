// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.cotc.Robot;
import frc.cotc.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class SwerveIOPhoenix implements SwerveIO {
  private final SwerveIOConstantsAutoLogged CONSTANTS;
  private final PhoenixModule[] modules;

  private final BaseStatusSignal[] signals = new BaseStatusSignal[9];
  private final Pigeon2 gyro;

  private final PhoenixOdometryThread odometryThread;

  public SwerveIOPhoenix() {
    CONSTANTS = new SwerveIOConstantsAutoLogged();
    CONSTANTS.DRIVE_GEAR_RATIO = 6.75;
    CONSTANTS.STEER_GEAR_RATIO = 150.0 / 7.0;
    CONSTANTS.WHEEL_DIAMETER = Units.inchesToMeters(4);
    CONSTANTS.TRACK_WIDTH = Units.inchesToMeters(22.75);
    CONSTANTS.TRACK_LENGTH = Units.inchesToMeters(22.75);
    CONSTANTS.MAX_ROTOR_VELOCITY = Units.rotationsPerMinuteToRadiansPerSecond(5800);
    CONSTANTS.MAX_ACCEL = 8;

    modules =
        new PhoenixModule[] {
          new PhoenixModule(0, CONSTANTS),
          new PhoenixModule(1, CONSTANTS),
          new PhoenixModule(2, CONSTANTS),
          new PhoenixModule(3, CONSTANTS)
        };

    gyro = new Pigeon2(17, RobotConstants.CANIVORE_NAME);

    for (int i = 0; i < 4; i++) {
      signals[i * 2] = modules[i].driveMotor.getVelocity();
      signals[i * 2 + 1] = modules[i].steerEncoder.getAbsolutePosition();
    }
    signals[8] = gyro.getYaw();

    BaseStatusSignal.setUpdateFrequencyForAll(100, signals);

    odometryThread =
        new PhoenixOdometryThread(
            new PhoenixOdometryThread.ModuleSignals[] {
              modules[0].getModuleSignals(),
              modules[1].getModuleSignals(),
              modules[2].getModuleSignals(),
              modules[3].getModuleSignals()
            },
            gyro,
            CONSTANTS.WHEEL_DIAMETER);

    ParentDevice[] devices = new ParentDevice[13];
    for (int i = 0; i < 4; i++) {
      devices[i * 3] = modules[i].driveMotor;
      devices[i * 3 + 1] = modules[i].steerMotor;
      devices[i * 3 + 2] = modules[i].steerEncoder;
    }
    devices[12] = gyro;
    ParentDevice.optimizeBusUtilizationForAll(devices);

    if (Robot.isSimulation()) {
      new PhoenixDriveSim(
              new TalonFX[] {
                modules[0].driveMotor,
                modules[1].driveMotor,
                modules[2].driveMotor,
                modules[3].driveMotor
              },
              new TalonFX[] {
                modules[0].steerMotor,
                modules[1].steerMotor,
                modules[2].steerMotor,
                modules[3].steerMotor
              },
              new CANcoder[] {
                modules[0].steerEncoder,
                modules[1].steerEncoder,
                modules[2].steerEncoder,
                modules[3].steerEncoder
              },
              CONSTANTS.DRIVE_GEAR_RATIO,
              CONSTANTS.STEER_GEAR_RATIO,
              CONSTANTS.DRIVE_MOTOR_INVERSIONS,
              CONSTANTS.STEER_MOTOR_INVERTED)
          .start(500);
    }
    odometryThread.start();
  }

  @Override
  public SwerveIOConstantsAutoLogged getConstants() {
    return CONSTANTS;
  }

  @Override
  public void updateInputs(SwerveIOInputs inputs) {
    BaseStatusSignal.refreshAll(signals);
    inputs.gyroYaw = Rotation2d.fromDegrees(signals[8].getValueAsDouble());
    inputs.moduleStates =
        new SwerveModuleState[] {
          new SwerveModuleState(
              signals[0].getValueAsDouble() * CONSTANTS.WHEEL_DIAMETER * Math.PI,
              Rotation2d.fromRotations(signals[1].getValueAsDouble())),
          new SwerveModuleState(
              signals[2].getValueAsDouble() * CONSTANTS.WHEEL_DIAMETER * Math.PI,
              Rotation2d.fromRotations(signals[3].getValueAsDouble())),
          new SwerveModuleState(
              signals[4].getValueAsDouble() * CONSTANTS.WHEEL_DIAMETER * Math.PI,
              Rotation2d.fromRotations(signals[5].getValueAsDouble())),
          new SwerveModuleState(
              signals[6].getValueAsDouble() * CONSTANTS.WHEEL_DIAMETER * Math.PI,
              Rotation2d.fromRotations(signals[7].getValueAsDouble())),
        };

    PhoenixOdometryThread.OdometryFrame data = odometryThread.getData();

    if (data.timestamps().length > 0) {
      inputs.odometryTimestamps = data.timestamps();
      inputs.odometryPositions = data.modulePositions();
      inputs.odometryYaws = data.yaws();
    } else {
      inputs.odometryTimestamps = new double[] {Logger.getTimestamp() * 1e-6};
      inputs.odometryPositions =
          new SwerveModulePosition[] {
            new SwerveModulePosition(
                modules[0].driveMotor.getPosition().getValueAsDouble()
                    * CONSTANTS.WHEEL_DIAMETER
                    * Math.PI,
                Rotation2d.fromRotations(
                    modules[0].steerEncoder.getAbsolutePosition().getValueAsDouble())),
            new SwerveModulePosition(
                modules[1].driveMotor.getPosition().getValueAsDouble()
                    * CONSTANTS.WHEEL_DIAMETER
                    * Math.PI,
                Rotation2d.fromRotations(
                    modules[1].steerEncoder.getAbsolutePosition().getValueAsDouble())),
            new SwerveModulePosition(
                modules[2].driveMotor.getPosition().getValueAsDouble()
                    * CONSTANTS.WHEEL_DIAMETER
                    * Math.PI,
                Rotation2d.fromRotations(
                    modules[2].steerEncoder.getAbsolutePosition().getValueAsDouble())),
            new SwerveModulePosition(
                modules[3].driveMotor.getPosition().getValueAsDouble()
                    * CONSTANTS.WHEEL_DIAMETER
                    * Math.PI,
                Rotation2d.fromRotations(
                    modules[3].steerEncoder.getAbsolutePosition().getValueAsDouble())),
          };
      inputs.odometryYaws =
          new Rotation2d[] {Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble())};
    }
  }

  @Override
  public void drive(SwerveModuleState[] setpoint, double[] forceFeedforward) {
    for (int i = 0; i < 4; i++) {
      modules[i].drive(setpoint[i], forceFeedforward[i]);
    }
  }

  boolean characterizationInitialized = false;

  @Override
  public void steerCharacterization(double volts) {
    if (!characterizationInitialized) {
      BaseStatusSignal.setUpdateFrequencyForAll(
          500,
          modules[0].getModuleSignals().drivePosition(),
          modules[0].getModuleSignals().driveVelocity(),
          modules[0].getModuleSignals().steerPosition(),
          modules[0].getModuleSignals().steerVelocity(),
          modules[1].getModuleSignals().drivePosition(),
          modules[1].getModuleSignals().driveVelocity(),
          modules[1].getModuleSignals().steerPosition(),
          modules[1].getModuleSignals().steerVelocity(),
          modules[2].getModuleSignals().drivePosition(),
          modules[2].getModuleSignals().driveVelocity(),
          modules[2].getModuleSignals().steerPosition(),
          modules[2].getModuleSignals().steerVelocity(),
          modules[3].getModuleSignals().drivePosition(),
          modules[3].getModuleSignals().driveVelocity(),
          modules[3].getModuleSignals().steerPosition(),
          modules[3].getModuleSignals().steerVelocity());
      characterizationInitialized = true;
    }
    for (int i = 0; i < 4; i++) {
      modules[i].steerMotor.setVoltage(volts);
      modules[i].driveMotor.stopMotor();
    }
  }

  private final PositionVoltage characterizationPos = new PositionVoltage(0);
  private final TorqueCurrentFOC foc = new TorqueCurrentFOC(0);

  @Override
  public void driveCharacterization(double volts) {
    if (!characterizationInitialized) {
      BaseStatusSignal.setUpdateFrequencyForAll(
          500,
          modules[0].getModuleSignals().drivePosition(),
          modules[0].getModuleSignals().driveVelocity(),
          modules[0].getModuleSignals().steerPosition(),
          modules[0].getModuleSignals().steerVelocity(),
          modules[1].getModuleSignals().drivePosition(),
          modules[1].getModuleSignals().driveVelocity(),
          modules[1].getModuleSignals().steerPosition(),
          modules[1].getModuleSignals().steerVelocity(),
          modules[2].getModuleSignals().drivePosition(),
          modules[2].getModuleSignals().driveVelocity(),
          modules[2].getModuleSignals().steerPosition(),
          modules[2].getModuleSignals().steerVelocity(),
          modules[3].getModuleSignals().drivePosition(),
          modules[3].getModuleSignals().driveVelocity(),
          modules[3].getModuleSignals().steerPosition(),
          modules[3].getModuleSignals().steerVelocity());
      modules[0].driveMotor.optimizeBusUtilization(50, 1);
      modules[1].driveMotor.optimizeBusUtilization(50, 1);
      modules[2].driveMotor.optimizeBusUtilization(50, 1);
      modules[3].driveMotor.optimizeBusUtilization(50, 1);
      characterizationInitialized = true;
    }
    SignalLogger.writeDouble("SysIdInput", volts);
    for (int i = 0; i < 4; i++) {
      modules[i].steerMotor.setControl(characterizationPos);
      modules[i].driveMotor.setControl(foc.withOutput(volts));
    }
  }

  @Override
  public void stop() {
    for (PhoenixModule module : modules) {
      module.stop();
    }
  }

  @Override
  public void stopWithAngles(Rotation2d[] angles) {
    for (int i = 0; i < 4; i++) {
      modules[i].stopWithAngle(angles[i]);
    }
  }

  private class PhoenixModule {
    final TalonFX driveMotor;
    final TalonFX steerMotor;
    final CANcoder steerEncoder;

    final double wheelDiameter;
    final double maxRotorVelocity;
    final double driveGearRatio;
    final double steerGearRatio;

    protected PhoenixModule(int id, SwerveIOConstantsAutoLogged constants) {
      driveMotor = new TalonFX(id * 3 + 2, RobotConstants.CANIVORE_NAME);
      steerMotor = new TalonFX(id * 3 + 3, RobotConstants.CANIVORE_NAME);
      steerEncoder = new CANcoder(id * 3 + 4, RobotConstants.CANIVORE_NAME);

      var driveConfig = new TalonFXConfiguration();
      driveConfig.Feedback.SensorToMechanismRatio = CONSTANTS.DRIVE_GEAR_RATIO;
      driveConfig.MotorOutput.Inverted =
          constants.DRIVE_MOTOR_INVERSIONS[id]
              ? InvertedValue.Clockwise_Positive
              : InvertedValue.CounterClockwise_Positive;
      driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
      driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = 80;
      driveConfig.Slot0.kS = 0;
      driveConfig.Slot0.kV = 0;
      driveConfig.Slot0.kA = 1.8;
      driveConfig.Slot0.kP = 6;
      driveConfig.Slot0.kD = 0;
      driveConfig.Audio.AllowMusicDurDisable = true;
      driveConfig.MotionMagic.MotionMagicAcceleration =
          CONSTANTS.MAX_ACCEL * CONSTANTS.DRIVE_GEAR_RATIO;
      driveMotor.getConfigurator().apply(driveConfig);

      var encoderConfig = new CANcoderConfiguration();
      steerEncoder.getConfigurator().apply(encoderConfig);

      var steerConfig = new TalonFXConfiguration();
      steerConfig.Feedback.SensorToMechanismRatio = 1;
      steerConfig.Feedback.RotorToSensorRatio = CONSTANTS.STEER_GEAR_RATIO;
      steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
      steerConfig.Feedback.FeedbackRemoteSensorID = steerEncoder.getDeviceID();
      steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
      steerConfig.MotorOutput.Inverted =
          constants.STEER_MOTOR_INVERTED
              ? InvertedValue.Clockwise_Positive
              : InvertedValue.CounterClockwise_Positive;
      steerConfig.CurrentLimits.StatorCurrentLimit = 40;
      steerConfig.Slot0.kS = 0;
      steerConfig.Slot0.kP = 24;
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

    protected void drive(SwerveModuleState setpoint, double forceFeedforward) {
      if (MathUtil.isNear(0, setpoint.speedMetersPerSecond, 1e-3)
          && MathUtil.isNear(0, forceFeedforward, 1e-3)) {
        stop();
        return;
      }

      driveMotor.setControl(
          driveControlRequest
              .withVelocity(
                  (setpoint.speedMetersPerSecond / (wheelDiameter * Math.PI)) * driveGearRatio)
              .withFeedForward(
                  Math.signum(setpoint.speedMetersPerSecond)
                      *
                      // ((Force (N) * radius (m)) / gear ratio (dimensionless)) / kT (Nm/amp) =
                      // amps
                      ((forceFeedforward * (wheelDiameter / 2)) / driveGearRatio)
                      / 9.81));
      steerMotor.setControl(steerControlRequest.withPosition(setpoint.angle.getRotations()));
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

    protected PhoenixOdometryThread.ModuleSignals getModuleSignals() {
      return new PhoenixOdometryThread.ModuleSignals(
          driveMotor.getPosition(),
          driveMotor.getVelocity(),
          steerEncoder.getAbsolutePosition(),
          steerMotor.getVelocity());
    }
  }
}
