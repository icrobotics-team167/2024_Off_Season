// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import static frc.cotc.drive.SwerveSetpointGenerator.SwerveSetpoint;
import static java.lang.Math.PI;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.cotc.Robot;
import frc.cotc.RobotConstants;
import frc.cotc.util.FOCMotorSim;
import org.littletonrobotics.junction.Logger;

public class SwerveIOPhoenix implements SwerveIO {
  private static final SwerveModuleConstantsAutoLogged CONSTANTS;
  private static final double WHEEL_CIRCUMFERENCE;

  static {
    CONSTANTS = new SwerveModuleConstantsAutoLogged();

    CONSTANTS.TRACK_WIDTH = Units.inchesToMeters(22.75);
    CONSTANTS.TRACK_LENGTH = Units.inchesToMeters(22.75);
    CONSTANTS.WHEEL_DIAMETER = Units.inchesToMeters(4);
    WHEEL_CIRCUMFERENCE = CONSTANTS.WHEEL_DIAMETER * PI;

    CONSTANTS.DRIVE_GEAR_RATIO = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);
    CONSTANTS.STEER_GEAR_RATIO = 150.0 / 7.0;

    CONSTANTS.DRIVE_MOTOR_MAX_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(5800);
    CONSTANTS.STEER_MOTOR_MAX_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(5800);

    CONSTANTS.MAX_ACCELERATION = 15;
  }

  private final Module[] modules = new Module[4];
  private final BaseStatusSignal[] signals = new BaseStatusSignal[9];

  private final OdometryThread odometryThread;
  private final Pigeon2 gyro;

  public SwerveIOPhoenix() {
    for (int i = 0; i < 4; i++) {
      modules[i] = new Module(i);
      signals[i * 2] = modules[i].driveMotor.getVelocity();
      signals[i * 2 + 1] = modules[i].encoder.getAbsolutePosition();
    }
    gyro = new Pigeon2(13, RobotConstants.CANIVORE_NAME);
    signals[8] = gyro.getYaw();

    odometryThread = new OdometryThread(modules, gyro, 250);

    if (Robot.isSimulation()) {
      new SimThread(modules, gyro, 500).start();
    }
    odometryThread.start();
  }

  @Override
  public void updateInputs(SwerveIOInputs inputs) {
    BaseStatusSignal.refreshAll(signals);
    for (int i = 0; i < 4; i++) {
      inputs.moduleStates[i] =
          new SwerveModuleState(
              signals[i * 2].getValueAsDouble() * WHEEL_CIRCUMFERENCE,
              Rotation2d.fromRotations(signals[i * 2 + 1].getValueAsDouble()));
    }
    inputs.gyroYaw = Rotation2d.fromDegrees(signals[8].getValueAsDouble());

    var odometryData = odometryThread.poll();

    if (odometryData.length > 0) {
      inputs.odometryTimestamps = new double[odometryData.length];
      inputs.odometryYaws = new Rotation2d[odometryData.length];
      inputs.odometryPositions = new SwerveModulePosition[odometryData.length * 4];
      for (int i = 0; i < odometryData.length; i++) {
        inputs.odometryTimestamps[i] = odometryData[i].timestampSeconds();
        inputs.odometryYaws[i] = odometryData[i].gyroYaw();
        System.arraycopy(odometryData[i].positions, 0, inputs.odometryPositions, i * 4, 4);
      }
    } else {
      // If the odometry thread doesn't have anything, fall back to getting the data directly
      // Slower but guaranteed
      inputs.odometryTimestamps = new double[] {Logger.getRealTimestamp() / 1e6};
      inputs.odometryYaws =
          new Rotation2d[] {Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble())};
      inputs.odometryPositions = new SwerveModulePosition[4];
      for (int i = 0; i < 4; i++) {
        inputs.odometryPositions[i] = modules[i].getPosition();
      }
    }
  }

  @Override
  public void drive(SwerveSetpoint setpoint, double[] forceFeedforward) {
    for (int i = 0; i < 4; i++) {
      modules[i].run(
          setpoint.moduleStates()[i], setpoint.steerFeedforwards()[i], forceFeedforward[i]);
    }
  }

  @Override
  public void resetGyro(Rotation2d newYaw) {
    gyro.setYaw(newYaw.getDegrees());
  }

  @Override
  public void driveCharacterization(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].driveCharacterization(volts);
    }
  }

  @Override
  public SwerveModuleConstantsAutoLogged getConstants() {
    return CONSTANTS;
  }

  private static class Module {
    final TalonFX driveMotor;
    final TalonFX steerMotor;
    final CANcoder encoder;

    final StatusSignal<AngularVelocity> driveVelocity;

    @SuppressWarnings("DuplicateBranchesInSwitch")
    public Module(int id) {
      driveMotor = new TalonFX(id * 3, RobotConstants.CANIVORE_NAME);
      steerMotor = new TalonFX(id * 3 + 1, RobotConstants.CANIVORE_NAME);
      encoder = new CANcoder(id * 3 + 2, RobotConstants.CANIVORE_NAME);

      var driveConfig = new TalonFXConfiguration();
      driveConfig.Feedback.SensorToMechanismRatio = CONSTANTS.DRIVE_GEAR_RATIO;
      driveConfig.MotionMagic.MotionMagicAcceleration =
          // Slight fudge factor to allow for control headroom
          1.1 * CONSTANTS.MAX_ACCELERATION / WHEEL_CIRCUMFERENCE;
      driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 100;
      driveConfig.TorqueCurrent.PeakReverseTorqueCurrent =
          driveConfig.TorqueCurrent.PeakForwardTorqueCurrent;
      driveConfig.Audio.AllowMusicDurDisable = true;

      var steerConfig = new TalonFXConfiguration();
      steerConfig.Feedback.SensorToMechanismRatio = 1;
      steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
      steerConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
      steerConfig.Feedback.RotorToSensorRatio = CONSTANTS.STEER_GEAR_RATIO;
      steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      steerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
      steerConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
      steerConfig.CurrentLimits.StatorCurrentLimit = 20;
      steerConfig.Audio.AllowMusicDurDisable = true;

      var encoderConfig = new CANcoderConfiguration();

      if (Robot.isReal()) {
        driveConfig.Slot0.kS = 0;
        driveConfig.Slot0.kV = 0;
        driveConfig.Slot0.kA = 0;
        driveConfig.Slot0.kP = 0;

        switch (id) {
          case 0 -> encoderConfig.MagnetSensor.MagnetOffset = 0;
          case 1 -> encoderConfig.MagnetSensor.MagnetOffset = 0;
          case 2 -> encoderConfig.MagnetSensor.MagnetOffset = 0;
          case 3 -> encoderConfig.MagnetSensor.MagnetOffset = 0;
        }
      } else {
        driveConfig.Slot0.kV = 0;
        driveConfig.Slot0.kA = 2.75;
        driveConfig.Slot0.kP = driveConfig.TorqueCurrent.PeakForwardTorqueCurrent * 2;

        steerConfig.Slot0.kP = 72;
        steerConfig.Slot0.kD = .125;
      }

      driveMotor.getConfigurator().apply(driveConfig);
      steerMotor.getConfigurator().apply(steerConfig);
      encoder.getConfigurator().apply(encoderConfig);

      driveVelocity = driveMotor.getVelocity();
    }

    private final MotionMagicVelocityTorqueCurrentFOC driveControlRequest =
        new MotionMagicVelocityTorqueCurrentFOC(0).withOverrideCoastDurNeutral(true);
    private final PositionVoltage steerControlRequest = new PositionVoltage(0).withEnableFOC(false);
    private final StaticBrake brakeControlRequest = new StaticBrake();

    private final double drive_kT = DCMotor.getKrakenX60Foc(1).KtNMPerAmp;
    private final double steer_kV =
        12 / (CONSTANTS.STEER_MOTOR_MAX_SPEED / CONSTANTS.STEER_GEAR_RATIO);

    public void run(SwerveModuleState state, double steerFeedforward, double forceFeedforward) {
      if (MathUtil.isNear(0, state.speedMetersPerSecond, 1e-3)
          && MathUtil.isNear(0, forceFeedforward, 1e-3)
          && MathUtil.isNear(
              0, driveVelocity.refresh().getValueAsDouble() * WHEEL_CIRCUMFERENCE, 1e-3)) {
        driveMotor.setControl(brakeControlRequest);
      } else {
        driveMotor.setControl(
            driveControlRequest
                .withVelocity(state.speedMetersPerSecond / WHEEL_CIRCUMFERENCE)
                .withFeedForward(
                    forceFeedforward
                        * (CONSTANTS.WHEEL_DIAMETER / 2)
                        / CONSTANTS.DRIVE_GEAR_RATIO
                        * drive_kT));
      }
      steerMotor.setControl(
          steerControlRequest
              .withPosition(state.angle.getRotations())
              .withFeedForward(steerFeedforward * steer_kV));
    }

    private final TorqueCurrentFOC characterizationControlRequest = new TorqueCurrentFOC(0);

    public void driveCharacterization(double amps) {
      steerMotor.setControl(steerControlRequest.withPosition(0));
      driveMotor.setControl(characterizationControlRequest.withOutput(amps));
    }

    public SwerveModulePosition getPosition() {
      return new SwerveModulePosition(
          driveMotor.getPosition().getValueAsDouble() * WHEEL_CIRCUMFERENCE,
          Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble()));
    }
  }

  private static class OdometryThread extends Thread {
    final ModuleSignals[] moduleSignals = new ModuleSignals[4];

    final BaseStatusSignal[] signals = new BaseStatusSignal[18];

    final CircularBuffer<OdometryFrame> frameBuffer;
    final double FREQUENCY;

    OdometryThread(Module[] modules, Pigeon2 gyro, double frequency) {
      for (int i = 0; i < 4; i++) {
        moduleSignals[i] = ModuleSignals.fromModule(modules[i]);
      }

      for (int i = 0; i < 4; i++) {
        System.arraycopy(moduleSignals[i].getSignals(), 0, signals, i * 4, 4);
      }
      signals[16] = gyro.getYaw();
      signals[17] = gyro.getAngularVelocityZWorld();

      BaseStatusSignal.setUpdateFrequencyForAll(frequency, signals);

      setDaemon(true);
      Threads.setCurrentThreadPriority(true, 2);

      FREQUENCY = frequency;
      frameBuffer = new CircularBuffer<>((int) Math.round(2 * FREQUENCY * Robot.defaultPeriodSecs));
    }

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
                    BaseStatusSignal.getLatencyCompensatedValueAsDouble(signals[16], signals[17])),
                Logger.getRealTimestamp() / 1e6);
        synchronized (frameBuffer) {
          frameBuffer.addLast(frame);
        }
      }
    }

    OdometryFrame[] poll() {
      OdometryFrame[] retFrames;
      synchronized (frameBuffer) {
        retFrames = new OdometryFrame[frameBuffer.size()];
        for (int i = 0; i < frameBuffer.size(); i++) {
          retFrames[i] = frameBuffer.get(i);
        }
      }
      return retFrames;
    }

    record OdometryFrame(
        SwerveModulePosition[] positions, Rotation2d gyroYaw, double timestampSeconds) {}

    record ModuleSignals(
        BaseStatusSignal drivePos,
        BaseStatusSignal driveVel,
        BaseStatusSignal steerPos,
        BaseStatusSignal steerVel) {
      BaseStatusSignal[] getSignals() {
        return new BaseStatusSignal[] {drivePos, driveVel, steerPos, steerVel};
      }

      SwerveModulePosition poll() {
        return new SwerveModulePosition(
            BaseStatusSignal.getLatencyCompensatedValueAsDouble(drivePos, driveVel)
                * WHEEL_CIRCUMFERENCE,
            Rotation2d.fromRotations(
                BaseStatusSignal.getLatencyCompensatedValueAsDouble(steerPos, steerVel)));
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

  private static class SimThread {
    final SimModule[] simModules = new SimModule[4];
    final Pigeon2SimState gyroSimState;
    final Notifier simNotifier;
    final double frequency;

    final Watchdog watchdog;

    SimThread(Module[] modules, Pigeon2 gyro, double frequency) {
      for (int i = 0; i < 4; i++) {
        simModules[i] = new SimModule(modules[i]);
      }
      gyroSimState = gyro.getSimState();

      simNotifier = new Notifier(this::run);
      this.frequency = frequency;

      watchdog = new Watchdog(1.0 / frequency, () -> {});
    }

    double lastTime;

    public void start() {
      lastTime = Logger.getRealTimestamp() / 1e6;
      for (SimModule module : simModules) {
        module.steerSim.setState((Math.random() * 2 - 1) * PI, 0);
        module.steerMotorSim.setRawRotorPosition(
            module.steerSim.getAngularPositionRotations() * CONSTANTS.STEER_GEAR_RATIO);
        module.encoderSim.setRawPosition(module.steerSim.getAngularPositionRotations());
      }
      simNotifier.startPeriodic(1.0 / frequency);
    }

    private final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            new Translation2d(CONSTANTS.TRACK_LENGTH / 2, CONSTANTS.TRACK_WIDTH / 2),
            new Translation2d(CONSTANTS.TRACK_LENGTH / 2, -CONSTANTS.TRACK_WIDTH / 2),
            new Translation2d(-CONSTANTS.TRACK_LENGTH / 2, CONSTANTS.TRACK_WIDTH / 2),
            new Translation2d(-CONSTANTS.TRACK_LENGTH / 2, -CONSTANTS.TRACK_WIDTH / 2));
    private double yawDeg = 0;

    public void run() {
      watchdog.reset();
      double currentTime = Logger.getRealTimestamp() / 1e6;
      double dt = currentTime - lastTime;
      SwerveModuleState[] moduleStates = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        simModules[i].run(dt);
        moduleStates[i] = simModules[i].getModuleState();
        watchdog.addEpoch("Module " + i + " tick");
      }

      yawDeg +=
          Units.radiansToDegrees(
              kinematics.toChassisSpeeds(moduleStates).omegaRadiansPerSecond * dt);
      gyroSimState.setRawYaw(yawDeg);

      watchdog.addEpoch("Gyro calculations");

      lastTime = currentTime;
      watchdog.disable();
      if (watchdog.isExpired()) {
        printTimingWarning();
      }
    }

    private void printTimingWarning() {
      DriverStation.reportWarning(
          "Swerve sim thread went over time! Expected "
              + 1000.0 / frequency
              + " ms, got "
              + watchdog.getTime() * 1000.0
              + " ms",
          false);
      watchdog.printEpochs();
    }

    private static class SimModule {
      final TalonFXSimState driveMotorSim;
      final TalonFXSimState steerMotorSim;
      final CANcoderSimState encoderSim;

      final FOCMotorSim driveWheelSim;
      final DCMotorSim steerSim;

      SimModule(Module module) {
        driveMotorSim = module.driveMotor.getSimState();
        steerMotorSim = module.steerMotor.getSimState();
        encoderSim = module.encoder.getSimState();

        driveMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;
        steerMotorSim.Orientation = ChassisReference.Clockwise_Positive;
        encoderSim.Orientation = ChassisReference.CounterClockwise_Positive;

        driveWheelSim =
            new FOCMotorSim(DCMotor.getKrakenX60Foc(1), CONSTANTS.DRIVE_GEAR_RATIO, .05);
        steerSim =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60(1), .005, CONSTANTS.STEER_GEAR_RATIO),
                DCMotor.getKrakenX60(1));
      }

      private double lastSteerRotorVel = 0;

      void run(double dt) {
        // Update drive sim
        driveWheelSim.tick(driveMotorSim.getTorqueCurrent(), dt);

        driveMotorSim.setRawRotorPosition(
            Units.radiansToRotations(driveWheelSim.getPos()) * CONSTANTS.DRIVE_GEAR_RATIO);
        driveMotorSim.setRotorVelocity(
            Units.radiansToRotations(driveWheelSim.getVel()) * CONSTANTS.DRIVE_GEAR_RATIO);
        driveMotorSim.setRotorAcceleration(
            Units.radiansToRotations(driveWheelSim.getAccel()) * CONSTANTS.DRIVE_GEAR_RATIO);

        // Update steer sim
        steerSim.setInputVoltage(steerMotorSim.getMotorVoltage());
        steerSim.update(dt);

        double steerRotorVel = steerSim.getAngularVelocityRPM() / 60.0 * CONSTANTS.STEER_GEAR_RATIO;

        steerMotorSim.setRawRotorPosition(
            steerSim.getAngularPositionRotations() * CONSTANTS.STEER_GEAR_RATIO);
        steerMotorSim.setRotorVelocity(steerRotorVel);
        encoderSim.setRawPosition(steerSim.getAngularPositionRotations());
        encoderSim.setVelocity(steerSim.getAngularVelocityRPM() / 60.0);

        steerMotorSim.setRotorAcceleration((lastSteerRotorVel - steerRotorVel) / dt);

        lastSteerRotorVel = steerRotorVel;
      }

      SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            driveWheelSim.getVel() * CONSTANTS.WHEEL_DIAMETER / 2,
            new Rotation2d(steerSim.getAngularPositionRad()));
      }
    }
  }
}
