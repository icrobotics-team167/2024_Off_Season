// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.cotc.Robot;
import frc.cotc.util.FOCMotorSim;
import org.littletonrobotics.junction.Logger;

public class SwerveIOPhoenix implements SwerveIO {
  private static final SwerveModuleConstantsAutoLogged CONSTANTS;
  private static final double STEER_GEAR_RATIO = 150.0 / 7.0;

  static {
    CONSTANTS = new SwerveModuleConstantsAutoLogged();

    CONSTANTS.TRACK_WIDTH = Units.inchesToMeters(22.75);
    CONSTANTS.TRACK_LENGTH = Units.inchesToMeters(22.75);
    CONSTANTS.WHEEL_DIAMETER = Units.inchesToMeters(4);

    CONSTANTS.DRIVE_GEAR_RATIO = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);

    CONSTANTS.DRIVE_MOTOR_MAX_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(5800);
  }

  private final Module[] modules = new Module[4];

  public SwerveIOPhoenix() {
    for (int i = 0; i < 4; i++) {
      modules[i] = new Module(i);
    }

    if (Robot.isSimulation()) {
      new SimThread(modules).start();
    }
  }

  @Override
  public void updateInputs(SwerveIOInputs inputs) {
    for (int i = 0; i < 4; i++) {
      inputs.moduleStates[i] = new SwerveModuleState();
    }
  }

  @Override
  public void drive(SwerveModuleState[] setpoint) {
    SwerveIO.super.drive(setpoint);
  }

  @Override
  public void steerCharacterization(double volts) {
    SwerveIO.super.steerCharacterization(volts);
  }

  @Override
  public void driveCharacterization(double volts) {
    SwerveIO.super.driveCharacterization(volts);
  }

  @Override
  public SwerveModuleConstantsAutoLogged getConstants() {
    return CONSTANTS;
  }

  private static class Module {
    final TalonFX driveMotor;
    final TalonFX steerMotor;
    final CANcoder encoder;

    public Module(int id) {
      driveMotor = new TalonFX(id * 3, "Croppenheimer");
      steerMotor = new TalonFX(id * 3 + 1, "Croppenheimer");
      encoder = new CANcoder(id * 3 + 2, "Croppenheimer");

      var driveConfig = new TalonFXConfiguration();
      driveConfig.Feedback.SensorToMechanismRatio = CONSTANTS.DRIVE_GEAR_RATIO;
      driveConfig.MotionMagic.MotionMagicAcceleration = 12 / (CONSTANTS.WHEEL_DIAMETER * Math.PI);
      driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 100;
      driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = 100;
      driveConfig.Audio.AllowMusicDurDisable = true;

      var steerConfig = new TalonFXConfiguration();
      steerConfig.Feedback.SensorToMechanismRatio = 1;
      steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
      steerConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
      steerConfig.Feedback.RotorToSensorRatio = STEER_GEAR_RATIO;
      steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      steerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
      steerConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
      steerConfig.CurrentLimits.StatorCurrentLimit = 40;
      steerConfig.Audio.AllowMusicDurDisable = true;

      //noinspection IfStatementWithIdenticalBranches
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

  private static class OdometryThread extends Thread {
    private final ModuleSignals[] moduleSignals = new ModuleSignals[4];

    private final BaseStatusSignal[] signals = new BaseStatusSignal[18];
    private final StatusSignal<Double> yawSignal;
    private final StatusSignal<Double> yawVelSignal;

    private final double FREQUENCY;

    public OdometryThread(Module[] modules, Pigeon2 gyro, double frequency) {
      for (int i = 0; i < 4; i++) {
        moduleSignals[i] = ModuleSignals.fromModule(modules[i]);
      }

      for (int i = 0; i < 4; i++) {
        System.arraycopy(moduleSignals[i].getSignals(), 0, signals, i * 4, 4);
      }
      yawSignal = gyro.getYaw();
      yawVelSignal = gyro.getAngularVelocityZWorld();
      signals[16] = yawSignal;
      signals[17] = yawVelSignal;

      BaseStatusSignal.setUpdateFrequencyForAll(250, signals);

      setDaemon(true);
      Threads.setCurrentThreadPriority(true, 2);

      FREQUENCY = frequency;
    }

    private final CircularBuffer<OdometryFrame> frameBuffer = new CircularBuffer<>(10);

    @Override
    public void run() {
      //noinspection InfiniteLoopStatement
      while (true) {
        BaseStatusSignal.waitForAll(2.0 / FREQUENCY, signals);
        var frame =
            new OdometryFrame(
                new SwerveModulePosition[] {
                  moduleSignals[0].poll(),
                  moduleSignals[1].poll(),
                  moduleSignals[2].poll(),
                  moduleSignals[3].poll()
                },
                Rotation2d.fromDegrees(
                    BaseStatusSignal.getLatencyCompensatedValue(yawSignal, yawVelSignal)),
                Logger.getRealTimestamp() / 1e6);
        synchronized (frameBuffer) {
          frameBuffer.addLast(frame);
        }
      }
    }

    private record OdometryFrame(
        SwerveModulePosition[] positions, Rotation2d gyro, double timestampSeconds) {}

    private record ModuleSignals(
        StatusSignal<Double> drivePos,
        StatusSignal<Double> driveVel,
        StatusSignal<Double> steerPos,
        StatusSignal<Double> steerVel) {
      BaseStatusSignal[] getSignals() {
        return new BaseStatusSignal[] {drivePos, driveVel, steerPos, steerVel};
      }

      SwerveModulePosition poll() {
        return new SwerveModulePosition(
            BaseStatusSignal.getLatencyCompensatedValue(drivePos, driveVel)
                * (CONSTANTS.WHEEL_DIAMETER * Math.PI),
            Rotation2d.fromRotations(
                BaseStatusSignal.getLatencyCompensatedValue(steerPos, steerVel)));
      }

      static ModuleSignals fromModule(Module module) {
        return new ModuleSignals(
            module.driveMotor.getPosition(),
            module.driveMotor.getVelocity(),
            module.encoder.getAbsolutePosition(),
            module.steerMotor.getVelocity());
      }
    }
  }

  private static class SimThread extends Thread {
    private final SimModule[] simModules = new SimModule[4];

    public SimThread(Module[] modules) {
      for (int i = 0; i < 4; i++) {
        simModules[i] = new SimModule(modules[i]);
      }
      setDaemon(true);
      setName("Drive Sim Thread");
      Threads.setCurrentThreadPriority(true, 1);
    }

    @Override
    public void run() {
      double lastTime = Logger.getRealTimestamp() / 1e6;
      for (SimModule module : simModules) {
        module.steerSim.setState((Math.random() * 2 - 1) * Math.PI, 0);
        module.steerMotorSim.setRawRotorPosition(
            module.steerSim.getAngularPositionRotations() * STEER_GEAR_RATIO);
      }
      double currentDrawAmps = 0;
      double baseVoltage = 12;
      double resistanceOhms = .01;
      //noinspection InfiniteLoopStatement
      while (true) {
        double voltage = baseVoltage - (currentDrawAmps * resistanceOhms);
        currentDrawAmps = 0;
        double currentTime = Logger.getRealTimestamp() / 1e6;
        for (SimModule module : simModules) {
          currentDrawAmps += module.run(voltage, currentTime - lastTime);
        }
        lastTime = currentTime;
      }
    }

    private static class SimModule {
      private final TalonFXSimState driveMotorSim;
      private final TalonFXSimState steerMotorSim;
      private final CANcoderSimState encoderSim;

      private final FOCMotorSim driveWheelSim;
      private final DCMotorSim steerSim;

      SimModule(Module module) {
        driveMotorSim = module.driveMotor.getSimState();
        steerMotorSim = module.steerMotor.getSimState();
        encoderSim = module.encoder.getSimState();

        driveWheelSim =
            new FOCMotorSim(DCMotor.getKrakenX60Foc(1), CONSTANTS.DRIVE_GEAR_RATIO, .05);
        steerSim = new DCMotorSim(DCMotor.getKrakenX60(1), STEER_GEAR_RATIO, .005);
      }

      double run(double voltage, double dt) {
        driveMotorSim.setSupplyVoltage(voltage);
        steerMotorSim.setSupplyVoltage(voltage);
        encoderSim.setSupplyVoltage(voltage);

        driveWheelSim.tick(driveMotorSim.getTorqueCurrent(), dt);
        steerSim.setInputVoltage(steerMotorSim.getMotorVoltage());
        steerSim.update(dt);

        driveMotorSim.setRawRotorPosition(driveWheelSim.getPos() * CONSTANTS.DRIVE_GEAR_RATIO);
        driveMotorSim.setRotorVelocity(driveWheelSim.getVel() * CONSTANTS.DRIVE_GEAR_RATIO);
        driveMotorSim.setRotorAcceleration(driveWheelSim.getAccel() * CONSTANTS.DRIVE_GEAR_RATIO);

        steerMotorSim.setRotorVelocity(steerSim.getAngularPositionRotations() * STEER_GEAR_RATIO);
        steerMotorSim.setRotorVelocity(
            (steerSim.getAngularVelocityRPM() / 60.0) * STEER_GEAR_RATIO);

        encoderSim.addPosition(steerSim.getAngularPositionRotations());
        encoderSim.setVelocity(steerSim.getAngularVelocityRPM() / 60.0);

        return driveMotorSim.getSupplyCurrent() + steerMotorSim.getSupplyCurrent();
      }
    }
  }
}
