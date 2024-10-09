// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.cotc.Robot;

public class SwerveIOPhoenix implements SwerveIO {
  private static final SwerveIOConstantsAutoLogged CONSTANTS;

  static {
    CONSTANTS = new SwerveIOConstantsAutoLogged();

    CONSTANTS.TRACK_WIDTH = Units.inchesToMeters(22.75);
    CONSTANTS.TRACK_LENGTH = Units.inchesToMeters(22.75);
    CONSTANTS.WHEEL_DIAMETER = Units.inchesToMeters(4);

    CONSTANTS.DRIVE_GEAR_RATIO = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);
    CONSTANTS.STEER_GEAR_RATIO = 150.0 / 7.0;

    CONSTANTS.DRIVE_MOTOR_INVERSIONS = new boolean[] {false, false, false, false};
    CONSTANTS.STEER_MOTOR_INVERTED = true;

    CONSTANTS.DRIVE_MAX_ROTOR_VELOCITY = Units.rotationsPerMinuteToRadiansPerSecond(5800);
    CONSTANTS.STEER_MAX_ROTOR_VELOCITY = Units.rotationsPerMinuteToRadiansPerSecond(6000);

    CONSTANTS.MAX_ACCEL = 12;
  }

  private static class Module {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder encoder;

    public Module(int id) {
      driveMotor = new TalonFX(id * 3, "Croppenheimer");
      steerMotor = new TalonFX(id * 3 + 1, "Croppenheimer");
      encoder = new CANcoder(id * 3 + 2, "Croppenheimer");

      var driveConfig = new TalonFXConfiguration();
      driveConfig.Feedback.SensorToMechanismRatio = CONSTANTS.DRIVE_GEAR_RATIO;
      driveConfig.MotionMagic.MotionMagicAcceleration =
          CONSTANTS.MAX_ACCEL / (CONSTANTS.WHEEL_DIAMETER * Math.PI);
      driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      driveConfig.MotorOutput.Inverted =
          CONSTANTS.DRIVE_MOTOR_INVERSIONS[id]
              ? InvertedValue.Clockwise_Positive
              : InvertedValue.CounterClockwise_Positive;
      driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 100;
      driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = 100;
      driveConfig.Audio.AllowMusicDurDisable = true;

      var steerConfig = new TalonFXConfiguration();
      steerConfig.Feedback.SensorToMechanismRatio = 1;
      steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
      steerConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
      steerConfig.Feedback.RotorToSensorRatio = CONSTANTS.STEER_GEAR_RATIO;
      steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      steerConfig.MotorOutput.Inverted =
          CONSTANTS.STEER_MOTOR_INVERTED
              ? InvertedValue.Clockwise_Positive
              : InvertedValue.CounterClockwise_Positive;
      steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
      steerConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
      steerConfig.CurrentLimits.StatorCurrentLimit = 40;
      steerConfig.Audio.AllowMusicDurDisable = true;

      if (Robot.isReal()) {
        driveConfig.Slot0.kS = 0;
        driveConfig.Slot0.kV = 0;
        driveConfig.Slot0.kA = 0;
        driveConfig.Slot0.kP = 0;

        steerConfig.Slot0.kS = 0;
        steerConfig.Slot0.kP = 48;
      } else {
        driveConfig.Slot0.kS = 0;
        driveConfig.Slot0.kV = 0;
        driveConfig.Slot0.kA = 0;
        driveConfig.Slot0.kP = 0;

        steerConfig.Slot0.kS = 0;
        steerConfig.Slot0.kP = 48;
      }

      var encoderConfig = new CANcoderConfiguration();

      driveMotor.getConfigurator().apply(driveConfig);
      steerMotor.getConfigurator().apply(steerConfig);
      encoder.getConfigurator().apply(encoderConfig);
    }

    private final MotionMagicVelocityTorqueCurrentFOC driveControlRequest =
        new MotionMagicVelocityTorqueCurrentFOC(0);

    public void run(SwerveModuleState state, double forceFeedforward) {
      driveMotor.setControl(driveControlRequest.withVelocity(state.speedMetersPerSecond));
    }
  }

  private static class SimThread extends Thread {
    private static class SimModule {
      private final TalonFXSimState driveMotorSim;
      private final TalonFXSimState steerMotorSim;
      private final CANcoderSimState encoderSim;

      public SimModule(Module module) {
        driveMotorSim = module.driveMotor.getSimState();
        steerMotorSim = module.steerMotor.getSimState();
        encoderSim = module.encoder.getSimState();
      }
    }
  }
}
